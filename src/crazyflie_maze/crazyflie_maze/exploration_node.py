import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, Int8

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        
        # --- SUSCRIPCIONES ---
        # 1. Escuchamos la señal del Jefe / SLAM (0 = Parar, 1 = Explorar)
        self.sub_estado_mision = self.create_subscription(
            Int8, '/crazyflie/estado_mision', self.mision_callback, 10)
        
        # 2. Escuchamos los sensores láser (Frente, Atrás, Izq, Der, Arriba)
        self.sub_sensores = self.create_subscription(
            Float32MultiArray, '/crazyflie/sensores', self.sensores_callback, 10)
            
        # 3. Escuchamos la Pose para saber nuestro Yaw exacto y clavar los giros
        self.sub_pose = self.create_subscription(
            PoseStamped, '/crazyflie/pose', self.pose_callback, 10)
            
        # --- PUBLICADORES ---
        # Enviamos velocidades a la Persona 1
        self.pub_cmd_vel = self.create_publisher(Twist, '/crazyflie/cmd_vel', 10)
        
        # --- VARIABLES INTERNAS DE ESTADO ---
        self.activo = False  # Empieza hibernando hasta recibir un 1
        self.estado_vuelo = "AVANZANDO" # Puede ser AVANZANDO o GIRANDO
        
        self.dist_frente = 0.0
        self.dist_izq = 0.0
        self.dist_der = 0.0
        
        self.yaw_actual = 0.0
        self.yaw_objetivo = 0.0
        
        # Parámetros del laberinto y control (en milímetros)
        self.DISTANCIA_PARED_FRONTAL = 300.0 # Frenar a 30 cm
        self.HUECO_MINIMO = 400.0            # Si hay más de 40 cm, hay un camino
        
        # Bucle principal a 10 Hz
        self.timer = self.create_timer(0.1, self.logica_exploracion)
        self.get_logger().info("Nodo Explorador (Persona 3) hibernando. Esperando señal 1...")

    # --- CALLBACKS (Actualización de datos) ---
    def mision_callback(self, msg):
        if msg.data == 1 and not self.activo:
            self.activo = True
            self.get_logger().info("¡Señal de inicio! Empezando exploración...")
        elif msg.data == 0 and self.activo:
            self.activo = False
            self.get_logger().info("¡Señal de SLAM! Laberinto completado. Hibernando...")
            # Mandamos un Twist vacío para clavar frenos
            self.pub_cmd_vel.publish(Twist())

    def sensores_callback(self, msg):
        # Suponiendo orden: Frente, Atrás, Izquierda, Derecha, Arriba
        self.dist_frente = msg.data[0]
        self.dist_izq = msg.data[2]
        self.dist_der = msg.data[3]

    def pose_callback(self, msg):
        # Extraemos el Yaw (guiñada) a partir del cuaternión
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw_actual = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # --- FUNCIONES TÁCTICAS ---
    def calcular_diferencia_angulos(self, angulo1, angulo2):
        """Calcula la diferencia más corta entre dos ángulos (-180 a 180)"""
        dif = (angulo1 - angulo2) % 360
        if dif > 180:
            dif -= 360
        return dif

    def iniciar_giro(self, grados):
        self.estado_vuelo = "GIRANDO"
        self.yaw_objetivo = (self.yaw_actual + grados) % 360
        if self.yaw_objetivo > 180:
            self.yaw_objetivo -= 360
        self.get_logger().info(f"Muro de frente. Girando {grados}º (Objetivo: {self.yaw_objetivo:.1f}º)")

    # --- CEREBRO PRINCIPAL ---
    def logica_exploracion(self):
        if not self.activo:
            return # Si estamos en 0, no hacemos nada

        cmd = Twist()

        if self.estado_vuelo == "AVANZANDO":
            # 1. Evaluar cruces y muros
            if self.dist_frente < self.DISTANCIA_PARED_FRONTAL:
                # Toca elegir camino (Regla de la mano derecha)
                if self.dist_der > self.HUECO_MINIMO:
                    self.iniciar_giro(-90) # Derecha
                elif self.dist_izq > self.HUECO_MINIMO:
                    self.iniciar_giro(90)  # Izquierda
                else:
                    self.iniciar_giro(180) # Callejón sin salida, media vuelta
                
                # Publicar el comando a 0 para frenar antes de girar
                self.pub_cmd_vel.publish(cmd)
                return

            # 2. Orden de avance pura (La Persona 1 se encargará de que no se tuerza)
            cmd.linear.x = 0.15 
            cmd.angular.z = 0.0 # Tú no corriges la trayectoria, de eso se encarga el 1

            self.pub_cmd_vel.publish(cmd)

        elif self.estado_vuelo == "GIRANDO":
            error_yaw = self.calcular_diferencia_angulos(self.yaw_objetivo, self.yaw_actual)
            
            # Si el error es menor a 5 grados, damos el giro por bueno
            if abs(error_yaw) < 5.0:
                self.estado_vuelo = "AVANZANDO"
                self.get_logger().info("Giro completado. Avanzando...")
            else:
                # Girar hacia el objetivo. Limitamos la velocidad angular para no desestabilizarlo.
                # Si el error es positivo, gira izq. Si es negativo, gira der.
                velocidad_giro = 0.5 if error_yaw > 0 else -0.5
                cmd.angular.z = velocidad_giro
            
            self.pub_cmd_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    nodo = ExplorationNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()