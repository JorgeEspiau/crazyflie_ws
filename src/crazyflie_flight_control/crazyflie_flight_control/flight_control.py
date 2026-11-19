
import math
import time
import threading

import rclpy
from rclpy.node import Node

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16MultiArray, Float32MultiArray, Bool
from std_srvs.srv import Trigger


# ============================================================
# CONFIGURACION
# ============================================================

URI = "radio://0/80/2M/E7E7E7E7AA"

ALTURA_DESPEGUE = 0.4
VELOCIDAD_DESPEGUE = 0.2

FRECUENCIA_LOG_POSE = 50
FRECUENCIA_LOG_SENSORES = 100

# Durante la prueba inicial no se permite movimiento autonomo.
ENABLE_AUTONOMOUS_MOTION = False


class FlightControlNode(Node):

    def __init__(self):
        super().__init__("flight_control_node")

        # ----------------------------------------------------
        # PUBLICADORES
        # ----------------------------------------------------

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/crazyflie/pose",
            10
        )

        self.sensores_pub = self.create_publisher(
            UInt16MultiArray,
            "/crazyflie/sensores_raw",
            10
        )

        self.goal_reached_pub = self.create_publisher(
            Bool,
            "/crazyflie/goal_reached",
            10
        )

        # ----------------------------------------------------
        # SUSCRIPTORES
        # ----------------------------------------------------

        self.create_subscription(
            Float32MultiArray,
            "/crazyflie/sensores",
            self.sensores_callback,
            10
        )

        # Durante la validacion inicial no se ejecutan
        # ordenes de movimiento de P3 ni objetivos de P5.

        # ----------------------------------------------------
        # SERVICIOS
        # ----------------------------------------------------

        self.create_service(
            Trigger,
            "~/takeoff",
            self.takeoff_callback
        )

        self.create_service(
            Trigger,
            "~/land",
            self.land_callback
        )

        # ----------------------------------------------------
        # ESTADO
        # ----------------------------------------------------

        self.scf = None
        self.cf = None
        self.mc = None

        self.conectado = False
        self.volando = False
        self.aterrizando = False

        self.flight_lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.sensores = [9900] * 5

        # ----------------------------------------------------
        # CONEXION
        # ----------------------------------------------------

        self.conectar_dron()

        # ----------------------------------------------------
        # TIMER
        # ----------------------------------------------------

        self.timer = self.create_timer(
            0.05,
            self.control_callback
        )

    # ========================================================
    # CONEXION CON CRAZYFLIE
    # ========================================================

    def conectar_dron(self):

        try:
            cflib.crtp.init_drivers()

            self.get_logger().info(
                f"Conectando con Crazyflie: {URI}"
            )

            self.scf = SyncCrazyflie(
                URI,
                cf=Crazyflie()
            )

            self.scf.open_link()

            self.cf = self.scf.cf

            # Una unica instancia de MotionCommander
            # asociada a la conexion ya establecida.

            self.mc = MotionCommander(self.scf)

            # ------------------------------------------------
            # LOG DE POSE
            # ------------------------------------------------

            self.log_pose = LogConfig(
                name="Pose",
                period_in_ms=FRECUENCIA_LOG_POSE
            )

            for variable in (
                "stateEstimate.x",
                "stateEstimate.y",
                "stateEstimate.z",
                "stateEstimate.roll",
                "stateEstimate.pitch",
                "stateEstimate.yaw"
            ):
                self.log_pose.add_variable(
                    variable,
                    "float"
                )

            self.log_pose.data_received_cb.add_callback(
                self.pose_callback
            )

            self.cf.log.add_config(self.log_pose)
            self.log_pose.start()

            # ------------------------------------------------
            # LOG DE SENSORES
            # ------------------------------------------------

            self.log_ranges = LogConfig(
                name="Sensores",
                period_in_ms=FRECUENCIA_LOG_SENSORES
            )

            for variable in (
                "range.front",
                "range.back",
                "range.left",
                "range.right",
                "range.up"
            ):
                self.log_ranges.add_variable(
                    variable,
                    "uint16_t"
                )

            self.log_ranges.data_received_cb.add_callback(
                self.range_callback
            )

            self.cf.log.add_config(self.log_ranges)
            self.log_ranges.start()

            self.conectado = True

            self.get_logger().info(
                "Crazyflie conectado correctamente."
            )

            self.get_logger().info(
                "MotionCommander inicializado."
            )

            self.get_logger().info(
                "Telemetria de pose y sensores iniciada."
            )

        except Exception as e:

            self.get_logger().error(
                f"Error de conexion o inicializacion: {e}"
            )

            self.conectado = False

            if self.scf is not None:
                try:
                    self.scf.close_link()
                except Exception:
                    pass

    # ========================================================
    # TELEMETRIA DE POSE
    # ========================================================

    def pose_callback(self, timestamp, data, logconf):

        self.x = data["stateEstimate.x"]
        self.y = data["stateEstimate.y"]
        self.z = data["stateEstimate.z"]

        self.roll = data["stateEstimate.roll"]
        self.pitch = data["stateEstimate.pitch"]
        self.yaw = data["stateEstimate.yaw"]

        r = math.radians(self.roll) / 2.0
        p = math.radians(self.pitch) / 2.0
        a = math.radians(self.yaw) / 2.0

        pose = PoseStamped()

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.header.frame_id = "world"

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z

        q = pose.pose.orientation

        q.x = (
            math.sin(r) * math.cos(p) * math.cos(a)
            - math.cos(r) * math.sin(p) * math.sin(a)
        )

        q.y = (
            math.cos(r) * math.sin(p) * math.cos(a)
            + math.sin(r) * math.cos(p) * math.sin(a)
        )

        q.z = (
            math.cos(r) * math.cos(p) * math.sin(a)
            - math.sin(r) * math.sin(p) * math.cos(a)
        )

        q.w = (
            math.cos(r) * math.cos(p) * math.cos(a)
            + math.sin(r) * math.sin(p) * math.sin(a)
        )

        self.pose_pub.publish(pose)

    # ========================================================
    # TELEMETRIA DE SENSORES
    # ========================================================

    def range_callback(self, timestamp, data, logconf):

        msg = UInt16MultiArray()

        msg.data = [
            data["range.front"],
            data["range.back"],
            data["range.left"],
            data["range.right"],
            data["range.up"]
        ]

        self.sensores_pub.publish(msg)

    def sensores_callback(self, msg):

        if len(msg.data) < 5:
            return

        self.sensores = [
            float(v) if math.isfinite(v) and v > 0 else 9900.0
            for v in msg.data[:5]
        ]

    # ========================================================
    # DESPEGUE
    # ========================================================

    def takeoff_callback(self, request, response):

        with self.flight_lock:

            if not self.conectado:
                response.success = False
                response.message = "Crazyflie no conectado."
                return response

            if self.volando:
                response.success = False
                response.message = "El dron ya esta volando."
                return response

            try:

                self.get_logger().info(
                    "Solicitando despegue mediante MotionCommander..."
                )

                self.mc.take_off(
                    height=ALTURA_DESPEGUE,
                    velocity=VELOCIDAD_DESPEGUE
                )

                self.volando = True
                self.aterrizando = False

                response.success = True
                response.message = (
                    "MotionCommander ha completado la orden de despegue."
                )

                self.get_logger().info(
                    "Orden de despegue completada."
                )

            except Exception as e:

                self.volando = False

                response.success = False
                response.message = str(e)

                self.get_logger().error(
                    f"Error durante el despegue: {e}"
                )

        return response

    # ========================================================
    # ATERRIZAJE
    # ========================================================

    def land_callback(self, request, response):

        with self.flight_lock:

            if not self.conectado:
                response.success = False
                response.message = "Crazyflie no conectado."
                return response

            if not self.volando:
                response.success = False
                response.message = "El dron no esta volando."
                return response

            try:

                self.get_logger().info(
                    "Solicitando aterrizaje mediante MotionCommander..."
                )

                self.mc.land()

                self.volando = False
                self.aterrizando = False

                response.success = True
                response.message = (
                    "MotionCommander ha completado el aterrizaje."
                )

                self.get_logger().info(
                    "Aterrizaje completado."
                )

            except Exception as e:

                response.success = False
                response.message = str(e)

                self.get_logger().error(
                    f"Error durante el aterrizaje: {e}"
                )

        return response

    # ========================================================
    # CONTROL PERIODICO
    # ========================================================

    def control_callback(self):

        # Durante esta primera prueba no enviamos
        # consignas adicionales de movimiento.

        return

    # ========================================================
    # CIERRE
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Cerrando FlightControlNode..."
        )

        if self.conectado and self.volando:

            try:

                self.get_logger().warning(
                    "El dron sigue en vuelo. Solicitando aterrizaje..."
                )

                with self.flight_lock:
                    self.mc.land()
                    self.volando = False

            except Exception as e:

                self.get_logger().error(
                    f"No se pudo completar el aterrizaje: {e}"
                )

        if self.conectado:

            try:
                self.log_pose.stop()
                self.log_ranges.stop()
            except Exception as e:
                self.get_logger().warning(
                    f"Error deteniendo telemetria: {e}"
                )

            try:
                self.scf.close_link()
            except Exception as e:
                self.get_logger().warning(
                    f"Error cerrando radio: {e}"
                )

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = FlightControlNode()

        if not node.conectado:
            raise RuntimeError(
                "No se ha podido conectar con Crazyflie."
            )

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Error en FlightControlNode: {e}")

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
