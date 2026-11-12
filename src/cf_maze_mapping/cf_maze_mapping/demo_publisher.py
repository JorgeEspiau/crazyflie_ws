"""Publicador ficticio de odometría + Multi-ranger (4 rayos) para ROS 2 Humble.

No utiliza hardware. El entorno es una sala de 4 x 4 m con un obstáculo central.
Interfaz deliberadamente provisional para integrar posteriormente Crazyswarm2.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

ODOM_TOPIC = '/placeholder/odom'
SCAN_TOPIC = '/placeholder/scan'
ODOM_FRAME = 'placeholder_odom'
BASE_FRAME = 'placeholder_base_link'

# Segmentos de paredes: (x1, y1, x2, y2), metros.
WALLS = [
    (0.0, 0.0, 4.0, 0.0), (4.0, 0.0, 4.0, 4.0),
    (4.0, 4.0, 0.0, 4.0), (0.0, 4.0, 0.0, 0.0),
    (1.6, 1.6, 2.4, 1.6), (2.4, 1.6, 2.4, 2.4),
    (2.4, 2.4, 1.6, 2.4), (1.6, 2.4, 1.6, 1.6),
]

# Recorrido de ensayo sin cruzar paredes, a 0.7 m de los muros exteriores.
WAYPOINTS = [(0.7, 0.7), (3.3, 0.7), (3.3, 3.3), (0.7, 3.3)]
SPEED_M_S = 0.20
RANGE_MIN = 0.06
RANGE_MAX = 3.5
PERIOD_S = 0.10


def cross(ax, ay, bx, by):
    return ax * by - ay * bx


def ray_distance(x, y, angle):
    """Distancia a la primera pared intersectada por un rayo."""
    dx, dy = math.cos(angle), math.sin(angle)
    closest = float('inf')
    for x1, y1, x2, y2 in WALLS:
        sx, sy = x2 - x1, y2 - y1
        den = cross(dx, dy, sx, sy)
        if abs(den) < 1e-10:
            continue
        qx, qy = x1 - x, y1 - y
        distance = cross(qx, qy, sx, sy) / den
        segment_fraction = cross(qx, qy, dx, dy) / den
        if distance >= RANGE_MIN and 0.0 <= segment_fraction <= 1.0:
            closest = min(closest, distance)
    return closest if closest <= RANGE_MAX else float('inf')


def scripted_pose(distance_travelled):
    """Recorrido rectangular periódico; orientación tangente a la trayectoria."""
    edges = []
    for i, (x1, y1) in enumerate(WAYPOINTS):
        x2, y2 = WAYPOINTS[(i + 1) % len(WAYPOINTS)]
        length = math.hypot(x2 - x1, y2 - y1)
        edges.append((x1, y1, x2, y2, length))
    distance_travelled %= sum(edge[4] for edge in edges)
    for x1, y1, x2, y2, length in edges:
        if distance_travelled <= length:
            a = distance_travelled / length
            return (x1 + (x2 - x1) * a,
                    y1 + (y2 - y1) * a,
                    math.atan2(y2 - y1, x2 - x1))
        distance_travelled -= length
    return 0.7, 0.7, 0.0


class DemoPublisher(Node):
    def __init__(self):
        super().__init__('cf_maze_demo_publisher')
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.scan_pub = self.create_publisher(LaserScan, SCAN_TOPIC, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.elapsed = 0.0
        self.timer = self.create_timer(PERIOD_S, self.tick)
        self.get_logger().info('Demo sin Crazyflie: odometría y 4 rayos a 10 Hz')

    def tick(self):
        self.elapsed += PERIOD_S
        x, y, yaw = scripted_pose(self.elapsed * SPEED_M_S)
        # En esta primera demo el sensor coincide con el origen del dron.
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = ODOM_FRAME
        transform.child_frame_id = BASE_FRAME
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = BASE_FRAME
        # Orden: detrás, derecha, delante, izquierda (ángulos locales).
        scan.angle_min = -math.pi
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / 2.0
        scan.time_increment = 0.0
        scan.scan_time = PERIOD_S
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = [
            ray_distance(x, y, yaw + a)
            for a in (-math.pi, -math.pi / 2.0, 0.0, math.pi / 2.0)
        ]
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DemoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
