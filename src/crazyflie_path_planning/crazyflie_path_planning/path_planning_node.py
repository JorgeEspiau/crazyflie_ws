#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Importar mensajes estándar de ROS2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Bool  # Añadido Bool para la señal de despertar

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # --- NUEVO: Variable de estado para controlar si el nodo está "despierto" ---
        self.is_active = False

        # --- NUEVO: Suscripción al nodo jefe ---
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

        # Variables internas para almacenar el estado
        self.current_map = None
        self.current_pose = None
        self.current_flight_state = "completado"

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
        # TODO: Implementar aquí el "inflado de obstáculos" para respetar el tamaño del dron.

    def telemetry_callback(self, msg):
        """
        Recibe telemetría de la Persona 2.
        """
        if not self.is_active:
            return
            
        self.current_pose = msg.pose.pose
        # TODO: Verificar progreso y desviación.

    def flight_state_callback(self, msg):
        """
        Recibe estado de vuelo de la Persona 1.
        """
        if not self.is_active:
            return

        self.current_flight_state = msg.data
        if self.current_flight_state == "bloqueado":
            self.get_logger().warn("Ruta bloqueada. Es necesario recalcular.")
            # TODO: Lógica para detenerse y recalcular ruta.

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

        route_cells = []
        # TODO: Desarrollar algoritmo A* aquí.
        return route_cells

    def send_navigation_points(self, route_cells):
        """
        Convierte las celdas de la ruta en puntos de navegación espaciales.
        """
        if not self.is_active:
            return

        for cell in route_cells:
            # TODO: Convertir coordenadas y publicar
            
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.target_pub.publish(target_msg)
            self.get_logger().info('Publicando nuevo punto de navegación.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()