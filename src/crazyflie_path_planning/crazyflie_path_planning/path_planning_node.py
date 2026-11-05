#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Importar mensajes estándar de ROS2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # Variable de estado para controlar si el nodo está "despierto"
        self.is_active = False

        # Suscripción al nodo jefe
        self.boss_sub = self.create_subscription(
            Bool, '/comando_jefe', self.boss_callback, 10)
        
        # 1. SUSCRIPCIONES (Datos de otros nodos)
        # Suscripción al Mapa (Persona 4)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/mapa_laberinto', self.map_callback, 10)
            
        # Suscripción a la Telemetría (Persona 2)
        self.telemetry_sub = self.create_subscription(
            Odometry, '/telemetria_dron', self.telemetry_callback, 10)
            
        # Suscripción al Estado de Vuelo (Persona 1)
        self.flight_state_sub = self.create_subscription(
            String, '/estado_vuelo', self.flight_state_callback, 10)

        # 2. PUBLICACIONES
        # Publicador del Objetivo de Vuelo (Para Persona 1)
        self.target_pub = self.create_publisher(
            PoseStamped, '/objetivo_vuelo', 10)

        # Variables internas para almacenar el estado y la ruta
        self.current_map = None
        self.current_pose = None
        self.current_flight_state = "IDLE"
        
        # Lista para almacenar los puntos de la ruta calculada por A*
        self.current_route = []

        self.get_logger().info('Nodo de planificación iniciado. Esperando señal de despertar del jefe...')

    def boss_callback(self, msg):
        """
        Recibe la orden del nodo jefe para despertar (True) o entrar en reposo (False).
        """
        if msg.data and not self.is_active:
            self.is_active = True
            self.get_logger().info("¡Nodo despertado! Iniciando procesamiento de rutas.")
        elif not msg.data and self.is_active:
            self.is_active = False
            self.get_logger().info("Nodo en reposo. Ignorando peticiones.")

    def map_callback(self, msg):
        """
        Recibe el mapa de la Persona 4.
        """
        if not self.is_active:
            return  # Ignorar actualizaciones si el nodo está dormido

        self.current_map = msg
        # TODO: Implementar aquí el "inflado de obstáculos" para respetar el tamaño del dron y el margen de seguridad.

    def telemetry_callback(self, msg):
        """
        Recibe telemetría de la Persona 2.
        """
        if not self.is_active:
            return
            
        self.current_pose = msg.pose.pose
        # TODO: Verificar progreso y desviación geométrica.

    def flight_state_callback(self, msg):
        """
        Recibe estado de vuelo de la Persona 1 y gestiona el envío de la ruta punto a punto.
        """
        if not self.is_active:
            return

        self.current_flight_state = msg.data
        
        if self.current_flight_state == "HOVERING":
            # El dron está quieto en el aire, podemos enviarle el siguiente punto de la ruta
            if len(self.current_route) > 0:
                self.get_logger().info("El dron está estático. Enviando siguiente punto de la ruta...")
                next_cell = self.current_route.pop(0) # Sacamos el primer punto de la lista
                self.send_navigation_point(next_cell)
            else:
                self.get_logger().debug("Dron en HOVERING pero no hay ruta pendiente.", throttle_duration_sec=5.0)
                
        elif self.current_flight_state == "EMERGENCY":
            self.get_logger().warn("¡Dron en emergencia! Abortando y borrando ruta pendiente.")
            self.current_route = []  # Vaciamos la lista de puntos para que no intente seguir
            
        elif self.current_flight_state == "MOVING":
            # El dron está desplazándose al punto que le enviamos. Simplemente esperamos a que vuelva a HOVERING.
            pass

    def calculate_a_star(self, start_cell, goal_cell):
        """
        Implementación del algoritmo A*.
        """
        if not self.is_active:
            self.get_logger().warn("Petición de cálculo rechazada: el nodo está en reposo.")
            return []

        if self.current_map is None:
            self.get_logger().error("Aún no hay mapa disponible.")
            return []

        # TODO: Desarrollar algoritmo A* aquí, excluyendo celdas ocupadas y desconocidas.
        # Guardar la ruta resultante en self.current_route
        # self.current_route = [celda1, celda2, celda3...]
        
        return self.current_route

    def send_navigation_point(self, cell):
        """
        Convierte una celda de la ruta en un punto de navegación espacial y lo publica.
        """
        target_msg = PoseStamped()
        
        # OBLIGATORIO: Metadatos temporales y marco de referencia global
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'map' 
        
        # TODO: Convertir coordenadas de celda (cell) a x, y reales según resolución y origen del mapa
        # target_msg.pose.position.x = coordenada_x_real
        # target_msg.pose.position.y = coordenada_y_real
        # target_msg.pose.position.z = 0.4  # Altura de vuelo habitual
        
        # OBLIGATORIO: Cuaternión neutro válido para evitar errores de cálculo de 'yaw' en el nodo de control
        target_msg.pose.orientation.w = 1.0 
        target_msg.pose.orientation.x = 0.0
        target_msg.pose.orientation.y = 0.0
        target_msg.pose.orientation.z = 0.0
        
        self.target_pub.publish(target_msg)
        self.get_logger().info('Nuevo objetivo enviado al controlador de vuelo.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()