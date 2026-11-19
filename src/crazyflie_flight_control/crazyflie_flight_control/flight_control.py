
import math
import time
import threading

import rclpy
from rclpy.node import Node

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, Bool
from std_srvs.srv import Trigger


# CONFIGURACION
URI = "radio://0/80/2M/E7E7E7E7AA"

ALTURA = 0.5
FRECUENCIA = 20.0
VELOCIDAD_MAX = 0.25
TIMEOUT_CMD = 0.5

TOLERANCIA_OBJETIVO = 0.10
MUESTRAS_LLEGADA = 5

# Activar solo despues de comprobar los movimientos basicos.
CORRECCION_PASILLO = False
DISTANCIA_PARED = 0.40
DISTANCIA_MUESTREO = 0.10


class FlightControlNode(Node):

    def __init__(self):
        super().__init__("flight_control_node")

        self.pose_pub = self.create_publisher(
            PoseStamped, "/crazyflie/pose", 10)
        self.sensores_pub = self.create_publisher(
            UInt16MultiArray, "/crazyflie/sensores_raw", 10)
        self.goal_reached_pub = self.create_publisher(
            Bool, "/crazyflie/goal_reached", 10)

        self.create_subscription(
            Twist, "/crazyflie/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(
            PoseStamped, "/crazyflie/goal_pose", self.goal_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/crazyflie/sensores",
            self.sensores_callback, 10)

        self.create_service(Trigger, "~/takeoff", self.takeoff_callback)
        self.create_service(Trigger, "~/land", self.land_callback)

        self.scf = None
        self.cf = None
        self.conectado = False
        self.volando = False
        self.aterrizando = False
        self.fin_aterrizaje = 0.0
        self.fin_despegue = 0.0
        self.modo = "HOLD"

        self.x = self.y = self.z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.lock = threading.Lock()

        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.ultimo_cmd = 0.0

        # Orden: frente, atras, izquierda, derecha, arriba.
        self.sensores = [9.9] * 5
        self.ultimo_sensor = 0.0

        self.ref_x = self.ref_y = self.ref_pared = None
        self.ref_lado = None

        self.goal_x = self.goal_y = self.goal_yaw = 0.0
        self.muestras_llegada = 0

        self.conectar_dron()

        self.timer = self.create_timer(
            1.0 / FRECUENCIA, self.control_callback)

    def conectar_dron(self):
        try:
            cflib.crtp.init_drivers()
            self.scf = SyncCrazyflie(URI, cf=Crazyflie())
            self.scf.open_link()
            self.cf = self.scf.cf

            self.cf.param.set_value("commander.enHighLevel", "1")
            time.sleep(0.2)

            self.cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            self.cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(1.0)

            self.log_pose = LogConfig(name="Pose", period_in_ms=50)
            for variable in (
                "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                "stateEstimate.roll", "stateEstimate.pitch",
                "stateEstimate.yaw"
            ):
                self.log_pose.add_variable(variable, "float")

            self.log_pose.data_received_cb.add_callback(self.pose_callback)
            self.cf.log.add_config(self.log_pose)
            self.log_pose.start()

            self.log_ranges = LogConfig(name="Sensores", period_in_ms=100)
            for variable in (
                "range.front", "range.back", "range.left",
                "range.right", "range.up"
            ):
                self.log_ranges.add_variable(variable, "uint16_t")

            self.log_ranges.data_received_cb.add_callback(self.range_callback)
            self.cf.log.add_config(self.log_ranges)
            self.log_ranges.start()

            self.conectado = True
            self.get_logger().info("Crazyflie conectado y logging iniciado")

        except Exception as e:
            self.get_logger().error(f"Error de conexion: {e}")
            if self.scf is not None:
                try:
                    self.scf.close_link()
                except Exception:
                    pass

    def pose_callback(self, timestamp, data, logconf):
        with self.lock:
            self.x = data["stateEstimate.x"]
            self.y = data["stateEstimate.y"]
            self.z = data["stateEstimate.z"]
            self.roll = data["stateEstimate.roll"]
            self.pitch = data["stateEstimate.pitch"]
            self.yaw = data["stateEstimate.yaw"]

            x, y, z = self.x, self.y, self.z
            roll, pitch, yaw = self.roll, self.pitch, self.yaw

        r = math.radians(roll) / 2.0
        p = math.radians(pitch) / 2.0
        a = math.radians(yaw) / 2.0

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = pose.pose.orientation
        q.x = math.sin(r) * math.cos(p) * math.cos(a) - math.cos(r) * math.sin(p) * math.sin(a)
        q.y = math.cos(r) * math.sin(p) * math.cos(a) + math.sin(r) * math.cos(p) * math.sin(a)
        q.z = math.cos(r) * math.cos(p) * math.sin(a) - math.sin(r) * math.sin(p) * math.cos(a)
        q.w = math.cos(r) * math.cos(p) * math.cos(a) + math.sin(r) * math.sin(p) * math.sin(a)

        self.pose_pub.publish(pose)

    def range_callback(self, timestamp, data, logconf):
        msg = UInt16MultiArray()
        msg.data = [
            data["range.front"], data["range.back"],
            data["range.left"], data["range.right"],
            data["range.up"]
        ]
        self.sensores_pub.publish(msg)

    def sensores_callback(self, msg):
        if len(msg.data) < 5:
            return

        self.sensores = [
            float(v) / 1000.0 if math.isfinite(v) and v > 0 else 9.9
            for v in msg.data[:5]
        ]
        self.ultimo_sensor = time.monotonic()

    def cmd_vel_callback(self, msg):
        if not self.volando or self.modo == "GOTO":
            return

        if time.monotonic() < self.fin_despegue:
            return

        if self.modo == "HOLD":
            self.cf.commander.send_notify_setpoint_stop()

        self.cmd_vx = max(-VELOCIDAD_MAX, min(VELOCIDAD_MAX, msg.linear.x))
        self.cmd_vy = max(-VELOCIDAD_MAX, min(VELOCIDAD_MAX, msg.linear.y))
        self.cmd_wz = math.degrees(msg.angular.z)
        self.ultimo_cmd = time.monotonic()
        self.modo = "VEL"

    def goal_callback(self, msg):
        if not self.conectado or not self.volando:
            return
        if self.modo == "GOTO":
            self.get_logger().warning("Ya hay un objetivo activo")
            return

        self.goal_reached_pub.publish(Bool(data=False))

        if self.modo == "VEL":
            self.cf.commander.send_notify_setpoint_stop()

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        q = msg.pose.orientation
        self.goal_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        with self.lock:
            x, y = self.x, self.y

        distancia = math.hypot(self.goal_x - x, self.goal_y - y)
        duracion = max(2.0, distancia / VELOCIDAD_MAX + 1.0)

        self.cf.high_level_commander.go_to(
            self.goal_x, self.goal_y, ALTURA,
            self.goal_yaw, duracion
        )

        self.modo = "GOTO"
        self.muestras_llegada = 0
        self.reset_referencia_pared()

    def reset_referencia_pared(self):
        self.ref_x = self.ref_y = self.ref_pared = None
        self.ref_lado = None

    def corregir_pasillo(self, vx, vy, wz):
        if not CORRECCION_PASILLO:
            return vx, vy, wz

        if time.monotonic() - self.ultimo_sensor > 0.5:
            self.reset_referencia_pared()
            return vx, vy, wz

        frente, atras, izquierda, derecha, arriba = self.sensores

        if izquierda < DISTANCIA_PARED and derecha < DISTANCIA_PARED:
            vy += max(-0.1, min(0.1, (izquierda - derecha) * 0.4))

        if vx <= 0.01 or abs(wz) >= 1.0 or abs(vy) >= 0.05:
            self.reset_referencia_pared()
            return vx, vy, wz

        if derecha < DISTANCIA_PARED:
            pared, lado = derecha, "derecha"
        elif izquierda < DISTANCIA_PARED:
            pared, lado = izquierda, "izquierda"
        else:
            self.reset_referencia_pared()
            return vx, vy, wz

        with self.lock:
            x, y = self.x, self.y

        if self.ref_x is None or self.ref_lado != lado:
            self.ref_x, self.ref_y = x, y
            self.ref_pared = pared
            self.ref_lado = lado
            return vx, vy, wz

        avance = math.hypot(x - self.ref_x, y - self.ref_y)

        if avance >= DISTANCIA_MUESTREO:
            delta = pared - self.ref_pared
            angulo = math.degrees(math.atan2(delta, avance))
            correccion = -angulo * 0.6

            if lado == "izquierda":
                correccion = -correccion

            wz += max(-25.0, min(25.0, correccion))
            self.ref_x, self.ref_y = x, y
            self.ref_pared = pared

        return vx, vy, wz

    def control_callback(self):
        if not self.conectado or not self.volando:
            return

        with self.lock:
            x, y = self.x, self.y

        if self.modo == "GOTO":
            distancia = math.hypot(self.goal_x - x, self.goal_y - y)

            if distancia < TOLERANCIA_OBJETIVO:
                self.muestras_llegada += 1
            else:
                self.muestras_llegada = 0

            if self.muestras_llegada >= MUESTRAS_LLEGADA:
                self.modo = "HOLD"
                self.goal_reached_pub.publish(Bool(data=True))
                self.get_logger().info("Objetivo alcanzado")
            return

        if self.modo == "VEL":
            vx, vy, wz = self.cmd_vx, self.cmd_vy, self.cmd_wz

            if time.monotonic() - self.ultimo_cmd > TIMEOUT_CMD:
                vx = vy = wz = 0.0
                self.reset_referencia_pared()
            else:
                vx, vy, wz = self.corregir_pasillo(vx, vy, wz)

            self.cf.commander.send_hover_setpoint(vx, vy, wz, ALTURA)

    def takeoff_callback(self, request, response):
        if not self.conectado or self.volando:
            response.success = False
            response.message = "No conectado o el dron ya esta volando"
            return response

        try:
            self.cf.high_level_commander.takeoff(ALTURA, 2.0)
            self.volando = True
            self.aterrizando = False
            self.fin_despegue = time.monotonic() + 2.5
            self.modo = "HOLD"

            response.success = True
            response.message = "Orden de despegue enviada"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def land_callback(self, request, response):
        if not self.conectado or not self.volando:
            response.success = False
            response.message = "El dron no esta volando"
            return response

        try:
            if self.modo == "VEL":
                self.cf.commander.send_notify_setpoint_stop()

            self.cf.high_level_commander.land(0.0, 2.0)
            self.volando = False
            self.aterrizando = True
            self.fin_aterrizaje = time.monotonic() + 2.5
            self.modo = "HOLD"

            response.success = True
            response.message = "Aterrizaje ordenado"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def destroy_node(self):
        # Si ROS se cierra en vuelo, solicitar aterrizaje antes de cerrar radio.
        if self.conectado and self.volando:
            try:
                if self.modo == "VEL":
                    self.cf.commander.send_notify_setpoint_stop()
                self.cf.high_level_commander.land(0.0, 2.0)
                self.volando = False
                self.aterrizando = True
                self.fin_aterrizaje = time.monotonic() + 2.5
            except Exception as e:
                self.get_logger().error(f"No se pudo ordenar aterrizaje: {e}")

        if self.aterrizando:
            espera = max(0.0, self.fin_aterrizaje - time.monotonic())
            time.sleep(espera)

        if self.conectado:
            try:
                self.log_pose.stop()
                self.log_ranges.stop()
                self.scf.close_link()
            except Exception as e:
                self.get_logger().warning(f"Error al cerrar: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlightControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
