#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_control_node.py - Persona 1: Control de vuelo y estabilización (Crazyflie 2.1)
Incluye:
- Despegue y aterrizaje suaves (HighLevelCommander)
- Publicación de Pose precisa por Filtro de Kalman (/crazyflie/pose)
- Control híbrido: Velocidades (/crazyflie/cmd_vel) y Posición (/crazyflie/goal_pose)
- Estabilización activa de pasillo: Centrado lateral + Corrección de Yaw (error trigonométrico/coseno)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control')
        
        # --- PARÁMETROS ---
        self.declare_parameter('uri', 'radio://0/80/2M/E7E7E7E7AA')
        self.uri = self.get_parameter('uri').value
        self.ALTURA_VUELO = 0.50      # Altura constante de 50 cm
        self.HUECO_MINIMO = 0.40      # Más de 40 cm se considera cruce/abierto, no pared
        self.DIST_MUESTREO = 0.10     # Cada 10 cm de avance calculamos el ángulo con la pared
        
        # --- ESTADO INTERNO ---
        self.scf = None
        self.conectado = False
        self.volando = False
        self.modo_control = "HOLD"    # HOLD, VEL (Nodo 3), GOTO (Nodo 5)
        
        # Telemetría interna (Kalman)
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.z_actual = 0.0
        self.yaw_actual = 0.0
        
        # Láseres (en metros)
        self.dist_frente = 9.9
        self.dist_izq = 9.9
        self.dist_der = 9.9
        
        # Comandos recibidos del Nodo 3
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz_deg = 0.0
        
        # Memoria para el cálculo de paralelismo con la pared (Error del coseno/arco tangente)
        self.x_ref = 0.0
        self.y_ref = 0.0
        self.izq_ref = 9.9
        self.der_ref = 9.9
        self.correccion_yaw_deg = 0.0
        
        # --- COMUNICACIONES ROS 2 ---
        # 1. Publicador de Pose (Kalman) para SLAM (Nodo 4), Exploración (Nodo 3) y Planificación (Nodo 5)
        self.pub_pose = self.create_publisher(PoseStamped, '/crazyflie/pose', 10)
        
        # 2. Suscriptores de navegación y sensores
        self.sub_cmd_vel = self.create_subscription(Twist, '/crazyflie/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/crazyflie/goal_pose', self.goal_callback, 10)
        self.sub_sensores = self.create_subscription(Float32MultiArray, '/crazyflie/sensores', self.sensores_callback, 10)
        
        # 3. Servicios de despegue y aterrizaje
        self.create_service(Trigger, '~/takeoff', self.takeoff_callback)
        self.create_service(Trigger, '~/land', self.land_callback)
        
        # --- INICIO DE CONEXIÓN Y BUCLE DE ESTABILIZACIÓN ---
        self.conectar_dron()
        self.timer_control = self.create_timer(0.05, self.bucle_estabilizacion) # 20 Hz

    def conectar_dron(self):
        cflib.crtp.init_drivers()
        self.get_logger().info(f'Conectando a {self.uri}...')
        try:
            self.scf = SyncCrazyflie(self.uri, cf=Crazyflie())
            self.scf.open_link()
            self.conectado = True
            
            # Activamos HighLevelCommander y reseteamos el Filtro de Kalman para máxima precisión
            self.scf.cf.param.set_value('commander.enHighLevel', '1')
            self.scf.cf.param.set_value('kalman.resetEstimation', '1')
            self.scf.cf.param.set_value('kalman.resetEstimation', '0')
            
            self.configurar_telemetria()
            self.get_logger().info('¡Dron conectado y Filtro de Kalman listo!')
        except Exception as e:
            self.get_logger().error(f'Error al conectar: {e}')

    def configurar_telemetria(self):
        logconf = LogConfig(name='KalmanPose', period_in_ms=50) # 20 Hz
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stateEstimate.roll', 'float')
        logconf.add_variable('stateEstimate.pitch', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        
        self.scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self.publicar_pose)
        logconf.start()

    def euler_a_cuaternion(self, roll, pitch, yaw):
        r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
        cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
        cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
        cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        )

    def publicar_pose(self, timestamp, data, logconf):
        self.x_actual = data['stateEstimate.x']
        self.y_actual = data['stateEstimate.y']
        self.z_actual = data['stateEstimate.z']
        self.yaw_actual = data['stateEstimate.yaw']
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = self.x_actual
        msg.pose.position.y = self.y_actual
        msg.pose.position.z = self.z_actual
        
        qx, qy, qz, qw = self.euler_a_cuaternion(
            data['stateEstimate.roll'], data['stateEstimate.pitch'], self.yaw_actual
        )
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = qx, qy, qz, qw
        self.pub_pose.publish(msg)

    # --- CALLBACKS DE SENSORES Y ÓRDENES ---
    def sensores_callback(self, msg: Float32MultiArray):
        # Convertimos de milímetros a metros
        self.dist_frente = msg.data[0] / 1000.0
        self.dist_izq = msg.data[2] / 1000.0
        self.dist_der = msg.data[3] / 1000.0

    def cmd_vel_callback(self, msg: Twist):
        """Recibe velocidades del Nodo 3 (Exploración) y activa el modo de estabilización de pasillo."""
        if not self.volando:
            return
        self.modo_control = "VEL"
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz_deg = math.degrees(msg.angular.z)

    def goal_callback(self, msg: PoseStamped):
        """Recibe coordenadas finales del Nodo 5 (Planificación A*) y usa el HighLevelCommander."""
        if not self.conectado or not self.volando:
            return
            
        if self.modo_control == "VEL":
            # Liberamos el control de bajo nivel antes de pasar a GoTo
            self.scf.cf.commander.send_notify_setpoint_stop()
            
        self.modo_control = "GOTO"
        px, py, pz = msg.pose.position.x, msg.pose.position.y, self.ALTURA_VUELO
        
        q = msg.pose.orientation
        yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Calculamos el tiempo necesario para ir suave a 0.25 m/s
        dist = math.hypot(px - self.x_actual, py - self.y_actual)
        duracion = max(1.5, dist / 0.25)
        
        self.get_logger().info(f"GoTo -> X:{px:.2f} Y:{py:.2f} (en {duracion:.1f}s)")
        self.scf.cf.high_level_commander.go_to(px, py, pz, yaw_rad, duracion)

    # --- BUCLE DE ESTABILIZACIÓN Y CORRECCIÓN EN MOVIMIENTO (20 Hz) ---
    def guardar_muestra_pared(self):
        self.x_ref = self.x_actual
        self.y_ref = self.y_actual
        self.izq_ref = self.dist_izq
        self.der_ref = self.dist_der

    def bucle_estabilizacion(self):
        """
        Ejecuta el control de altura constante, centrado de pasillo y corrección
        de paralelismo (Yaw) por trigonometría cuando el dron avanza en modo VEL.
        """
        if not self.conectado or not self.volando or self.modo_control != "VEL":
            return

        vx_final = self.cmd_vx
        vy_final = self.cmd_vy
        wz_final = self.cmd_wz_deg

        # Solo aplicamos corrección de pasillo cuando estamos avanzando recto (sin girar en esquinas)
        if abs(self.cmd_wz_deg) < 1.0 and self.cmd_vx > 0.01:
            
            # 1. CENTRADO LATERAL EN EL PASILLO (Eje Y)
            if self.dist_izq < self.HUECO_MINIMO and self.dist_der < self.HUECO_MINIMO:
                error_centro = self.dist_izq - self.dist_der
                vy_final += max(-0.1, min(0.1, error_centro * 0.4)) # Empujón lateral suave hacia el centro

            # 2. CORRECCIÓN DE PARALELISMO / YAW (Trigonometría / Error del coseno)
            avance = math.hypot(self.x_actual - self.x_ref, self.y_actual - self.y_ref)
            if avance >= self.DIST_MUESTREO:
                # Si tenemos pared continua a la derecha, calculamos el ángulo de desvío
                if self.dist_der < self.HUECO_MINIMO and self.der_ref < self.HUECO_MINIMO:
                    delta_pared = self.dist_der - self.der_ref
                    angulo_desvio = math.degrees(math.atan2(delta_pared, avance))
                    self.correccion_yaw_deg = -angulo_desvio * 0.6 # Ganancia proporcional de guiñada
                # Si no hay derecha pero tenemos pared izquierda continua
                elif self.dist_izq < self.HUECO_MINIMO and self.izq_ref < self.HUECO_MINIMO:
                    delta_pared = self.dist_izq - self.izq_ref
                    angulo_desvio = math.degrees(math.atan2(delta_pared, avance))
                    self.correccion_yaw_deg = angulo_desvio * 0.6
                else:
                    self.correccion_yaw_deg = 0.0
                
                self.guardar_muestra_pared()

            wz_final += max(-25.0, min(25.0, self.correccion_yaw_deg))
        else:
            # Si el Nodo 3 está ordenando un giro de 90º, reseteamos las referencias de pared
            self.correccion_yaw_deg = 0.0
            self.guardar_muestra_pared()

        # Enviamos el comando estabilizado manteniendo la altura clavada en ALTURA_VUELO
        self.scf.cf.commander.send_hover_setpoint(vx_final, vy_final, wz_final, self.ALTURA_VUELO)

    # --- SERVICIOS DE VUELO BÁSICO ---
    def takeoff_callback(self, request, response):
        if not self.conectado:
            response.success, response.message = False, "No conectado."
            return response
        self.get_logger().info("Despegue suave...")
        self.scf.cf.high_level_commander.takeoff(self.ALTURA_VUELO, 2.0)
        self.volando = True
        self.modo_control = "HOLD"
        self.guardar_muestra_pared()
        response.success, response.message = True, "Despegando a 0.5m"
        return response

    def land_callback(self, request, response):
        if not self.conectado:
            response.success, response.message = False, "No conectado."
            return response
        self.get_logger().info("Aterrizaje suave...")
        if self.modo_control == "VEL":
            self.scf.cf.commander.send_notify_setpoint_stop()
        self.modo_control = "HOLD"
        self.volando = False
        self.scf.cf.high_level_commander.land(0.0, 2.0)
        response.success, response.message = True, "Aterrizando"
        return response

    def cerrar_seguro(self):
        if self.conectado:
            self.scf.cf.commander.send_stop_setpoint()
            self.scf.close_link()


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = FlightControlNode()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.cerrar_seguro()
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()