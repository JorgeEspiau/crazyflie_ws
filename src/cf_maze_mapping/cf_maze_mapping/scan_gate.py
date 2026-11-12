import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ScanGate(Node):

    def __init__(self):

        super().__init__('cf_mapping_scan_gate')

        # Parametros de comunicacion
        self.declare_parameter(
            'input_scan_topic',
            '/cf231/scan'
        )

        self.declare_parameter(
            'output_scan_topic',
            '/mapping/scan'
        )

        self.declare_parameter(
            'ready_topic',
            '/mapping/ready'
        )

        self.declare_parameter(
            'start_enabled',
            False
        )

        # Obtener parametros
        input_topic = self.get_parameter(
            'input_scan_topic'
        ).value

        output_topic = self.get_parameter(
            'output_scan_topic'
        ).value

        ready_topic = self.get_parameter(
            'ready_topic'
        ).value

        self.enabled = bool(
            self.get_parameter('start_enabled').value
        )

        if input_topic == output_topic:
            raise ValueError(
                'Los topicos de entrada y salida deben ser diferentes'
            )

        # Suscripcion a los sensores reales
        self.create_subscription(
            LaserScan,
            input_topic,
            self.on_scan,
            qos_profile_sensor_data
        )

        # Suscripcion al estado del controlador
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(
            Bool,
            ready_topic,
            self.on_ready,
            status_qos
        )

        # Publicador de mediciones hacia SLAM Toolbox
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            output_topic,
            scan_qos
        )

        # Publicador del estado del mapeado
        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.active_pub = self.create_publisher(
            Bool,
            '/mapping/active',
            active_qos
        )

        self.publish_active()

        self.get_logger().info(
            f'Entrada: {input_topic} | '
            f'Salida: {output_topic} | '
            f'Estado: {ready_topic}'
        )

    def publish_active(self):

        msg = Bool()
        msg.data = self.enabled

        self.active_pub.publish(msg)

    def on_ready(self, msg):

        requested = bool(msg.data)

        if requested == self.enabled:
            return

        self.enabled = requested

        self.publish_active()

        if self.enabled:

            self.get_logger().info(
                'Mapeo habilitado'
            )

        else:

            self.get_logger().info(
                'Mapeo pausado'
            )

    def on_scan(self, msg):

        if self.enabled:

            # Reenviar sin modificar las mediciones
            self.scan_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ScanGate()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()