#!/usr/bin/env python3
import heapq
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool


class PathPlanningNode(Node):

    def __init__(self):
        super().__init__('path_planning_node')

        # Parametros configurables del objetivo y margen de seguridad del dron
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('robot_radius', 0.16)  # Radio Crazyflie + margen (metros)
        self.declare_parameter('map_topic', '/map')

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        map_topic = self.get_parameter('map_topic').value

        self.is_active = False
        self.route_computed = False

        # Suscripciones
        self.create_subscription(Bool, '/comando_jefe', self.boss_callback, 10)
        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 10)
        self.create_subscription(PoseStamped, '/crazyflie/pose', self.pose_callback, 10)
        self.create_subscription(String, '/estado_vuelo', self.flight_state_callback, 10)

        # Publicadores
        self.target_pub = self.create_publisher(PoseStamped, '/objetivo_vuelo', 10)
        self.mission_done_pub = self.create_publisher(Bool, '/crazyflie/mision_completada', 10)

        self.current_map = None
        self.inflated_grid = None
        self.current_pose = None
        self.current_flight_state = "IDLE"
        self.current_route = []

        self.timer = self.create_timer(0.5, self.plan_and_dispatch_loop)
        self.get_logger().info('Nodo de planificacion A* iniciado. Esperando orden del maestro...')

    def boss_callback(self, msg):
        if msg.data and not self.is_active:
            self.is_active = True
            self.route_computed = False
            self.current_route = []
            self.get_logger().info("Nodo despertado. Preparando calculo de ruta optima A*...")
        elif not msg.data and self.is_active:
            self.is_active = False
            self.current_route = []
            self.get_logger().info("Nodo en reposo.")

    def map_callback(self, msg):
        self.current_map = msg
        if self.is_active and not self.route_computed:
            self.inflated_grid = self.inflate_obstacles(msg)

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def flight_state_callback(self, msg):
        self.current_flight_state = msg.data
        if not self.is_active:
            return

        if self.current_flight_state == "EMERGENCY":
            self.get_logger().warn("Dron en emergencia. Abortando ruta.")
            self.current_route = []
            self.is_active = False

    def plan_and_dispatch_loop(self):
        if not self.is_active:
            return

        if self.current_map is None or self.current_pose is None:
            self.get_logger().info("Esperando mapa y pose para calcular A*...", throttle_duration_sec=2.0)
            return

        # 1. Calcular ruta una vez al despertar
        if not self.route_computed:
            if self.inflated_grid is None:
                self.inflated_grid = self.inflate_obstacles(self.current_map)

            start_cell = self.world_to_grid(self.current_pose.position.x, self.current_pose.position.y)
            goal_cell = self.world_to_grid(self.goal_x, self.goal_y)

            if start_cell is None or goal_cell is None:
                self.get_logger().error("Inicio o meta fuera de los limites del mapa.")
                return

            raw_path = self.calculate_a_star(start_cell, goal_cell)
            if not raw_path:
                self.get_logger().error("A* no encontro ninguna ruta libre hacia la meta.")
                self.is_active = False
                return

            # Simplificar ruta para enviar solo esquinas y destino
            self.current_route = self.simplify_path(raw_path)
            self.route_computed = True
            self.get_logger().info(f"Ruta A* calculada: {len(self.current_route)} waypoints clave.")

        # 2. Despachar siguiente waypoint cuando el dron este en HOVERING
        if self.current_flight_state == "HOVERING":
            if len(self.current_route) > 0:
                next_cell = self.current_route.pop(0)
                self.send_navigation_point(next_cell)
            else:
                self.get_logger().info("Todos los waypoints de A* completados.")
                self.mission_done_pub.publish(Bool(data=True))
                self.is_active = False

    def inflate_obstacles(self, grid_msg: OccupancyGrid):
        width = grid_msg.info.width
        height = grid_msg.info.height
        res = grid_msg.info.resolution
        data = grid_msg.data

        inflation_cells = max(1, int(math.ceil(self.robot_radius / res)))
        # 0 = libre, 1 = ocupado/desconocido
        grid = [[0] * width for _ in range(height)]

        obstacles = []
        for r in range(height):
            row_offset = r * width
            for c in range(width):
                val = data[row_offset + c]
                # Tratar celdas ocupadas (>50) o desconocidas (-1) como no transitables
                if val > 50 or val < 0:
                    grid[r][c] = 1
                    if val > 50:
                        obstacles.append((r, c))

        # Inflar unicamente los obstaculos reales detectados
        for r_obs, c_obs in obstacles:
            r_min = max(0, r_obs - inflation_cells)
            r_max = min(height - 1, r_obs + inflation_cells)
            c_min = max(0, c_obs - inflation_cells)
            c_max = min(width - 1, c_obs + inflation_cells)
            for r in range(r_min, r_max + 1):
                for c in range(c_min, c_max + 1):
                    if math.hypot(r - r_obs, c - c_obs) <= inflation_cells:
                        grid[r][c] = 1

        return grid

    def world_to_grid(self, x, y):
        info = self.current_map.info
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        if 0 <= r < info.height and 0 <= c < info.width:
            return (r, c)
        return None

    def grid_to_world(self, cell):
        r, c = cell
        info = self.current_map.info
        x = info.origin.position.x + (c + 0.5) * info.resolution
        y = info.origin.position.y + (r + 0.5) * info.resolution
        return (x, y)

    def find_nearest_free_cell(self, cell):
        r0, c0 = cell
        height = len(self.inflated_grid)
        width = len(self.inflated_grid[0])
        if self.inflated_grid[r0][c0] == 0:
            return cell
        for radius in range(1, 10):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    nr, nc = r0 + dr, c0 + dc
                    if 0 <= nr < height and 0 <= nc < width and self.inflated_grid[nr][nc] == 0:
                        return (nr, nc)
        return None

    def calculate_a_star(self, start_cell, goal_cell):
        start = self.find_nearest_free_cell(start_cell)
        goal = self.find_nearest_free_cell(goal_cell)
        if start is None or goal is None:
            return []

        height = len(self.inflated_grid)
        width = len(self.inflated_grid[0])

        # 8 direcciones (ortogonales y diagonales)
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path[1:]  # Excluir celda inicial actual

            cr, cc = current
            for dr, dc, cost in neighbors:
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < height and 0 <= nc < width and self.inflated_grid[nr][nc] == 0:
                    # Evitar cortar esquinas en movimientos diagonales
                    if dr != 0 and dc != 0:
                        if self.inflated_grid[cr + dr][cc] == 1 or self.inflated_grid[cr][cc + dc] == 1:
                            continue

                    tentative_g = g_score[current] + cost
                    neighbor = (nr, nc)
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + math.hypot(goal[0] - nr, goal[1] - nc)
                        heapq.heappush(open_set, (f_score, neighbor))

        return []

    def simplify_path(self, path):
        if len(path) <= 2:
            return path
        simplified = []
        prev_dir = None
        for i in range(len(path) - 1):
            curr_dir = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            if curr_dir != prev_dir:
                simplified.append(path[i])
                prev_dir = curr_dir
        simplified.append(path[-1])
        return simplified

    def send_navigation_point(self, cell):
        x, y = self.grid_to_world(cell)
        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'odom'
        target_msg.pose.position.x = x
        target_msg.pose.position.y = y
        target_msg.pose.position.z = 0.5
        target_msg.pose.orientation.w = 1.0

        self.target_pub.publish(target_msg)
        self.get_logger().info(f'Waypoint enviado: ({x:.2f}, {y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()