
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Twist


class ExplorationNode(Node):

    def __init__(self):
        super().__init__("exploration_node")

        # Parametros de exploracion
        self.DISTANCIA_FRONTAL = 300.0
        self.DISTANCIA_LATERAL = 400.0

        self.VELOCIDAD_AVANCE = 0.15
        self.VELOCIDAD_GIRO = 0.5
        self.TOLERANCIA_GIRO = 5.0

        self.activo = False
        self.estado = "AVANZANDO"

        self.dist_frente = None
        self.dist_derecha = None
        self.dist_izquierda = None

        self.yaw_actual = None
        self.yaw_objetivo = 0.0

        self.ultimo_sensor = 0.0
        self.ultima_pose = 0.0

        self.esperando_cierre_derecha = False

        self.create_subscription(
            Int8, "/crazyflie/estado_mision",
            self.mision_callback, 10)

        self.create_subscription(
            Float32MultiArray, "/crazyflie/sensores",
            self.sensores_callback, 10)

        self.create_subscription(
            PoseStamped, "/crazyflie/pose",
            self.pose_callback, 10)

        self.pub_velocidad = self.create_publisher(
            Twist, "/crazyflie/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.logica_exploracion)

        self.get_logger().info("Nodo de exploracion iniciado")

    def mision_callback(self, msg):
        if msg.data == 1:
            if not self.activo:
                self.activo = True
                self.estado = "AVANZANDO"
                self.esperando_cierre_derecha = False
                self.get_logger().info("Exploracion activada")

        elif msg.data == 0:
            self.activo = False
            self.estado = "AVANZANDO"
            self.publicar_velocidad(0.0, 0.0)
            self.get_logger().info("Exploracion detenida")

    def sensores_callback(self, msg):
        if len(msg.data) < 5:
            return

        # Orden: frente, atras, izquierda, derecha, arriba.
        valores = [
            float(v) if math.isfinite(v) and v > 0 else 9900.0
            for v in msg.data[:5]
        ]

        self.dist_frente = valores[0]
        self.dist_izquierda = valores[2]
        self.dist_derecha = valores[3]
        self.ultimo_sensor = time.monotonic()

    def pose_callback(self, msg):
        q = msg.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw_actual = math.degrees(
            math.atan2(siny_cosp, cosy_cosp))

        self.ultima_pose = time.monotonic()

    def publicar_velocidad(self, velocidad_lineal, velocidad_angular):
        msg = Twist()
        msg.linear.x = velocidad_lineal
        msg.angular.z = velocidad_angular
        self.pub_velocidad.publish(msg)

    def normalizar_angulo(self, angulo):
        return (angulo + 180.0) % 360.0 - 180.0

    def diferencia_angulos(self, objetivo, actual):
        return self.normalizar_angulo(objetivo - actual)

    def iniciar_giro(self, grados):
        self.yaw_objetivo = self.normalizar_angulo(
            self.yaw_actual + grados)

        self.estado = "GIRANDO"
        self.publicar_velocidad(0.0, 0.0)

    def logica_exploracion(self):
        if not self.activo:
            return

        # No moverse si faltan sensores o pose actualizados.
        ahora = time.monotonic()

        if (self.dist_frente is None or
                self.dist_derecha is None or
                self.dist_izquierda is None or
                self.yaw_actual is None or
                ahora - self.ultimo_sensor > 0.5 or
                ahora - self.ultima_pose > 0.5):

            self.publicar_velocidad(0.0, 0.0)
            return

        if self.estado == "AVANZANDO":

            # Rearmar la deteccion al volver a encontrar pared derecha.
            if self.dist_derecha <= self.DISTANCIA_LATERAL:
                self.esperando_cierre_derecha = False

            derecha_libre = (
                self.dist_derecha > self.DISTANCIA_LATERAL)

            frente_libre = (
                self.dist_frente >= self.DISTANCIA_FRONTAL)

            izquierda_libre = (
                self.dist_izquierda > self.DISTANCIA_LATERAL)

            # Regla de la mano derecha.
            if derecha_libre and not self.esperando_cierre_derecha:
                self.get_logger().info("Abertura a la derecha")
                self.esperando_cierre_derecha = True
                self.iniciar_giro(-90.0)

            elif frente_libre:
                self.publicar_velocidad(
                    self.VELOCIDAD_AVANCE, 0.0)

            elif izquierda_libre:
                self.get_logger().info("Giro a la izquierda")
                self.iniciar_giro(90.0)

            else:
                self.get_logger().info("Callejon sin salida")
                self.iniciar_giro(180.0)

        elif self.estado == "GIRANDO":

            error = self.diferencia_angulos(
                self.yaw_objetivo, self.yaw_actual)

            if abs(error) <= self.TOLERANCIA_GIRO:
                self.publicar_velocidad(0.0, 0.0)
                self.estado = "AVANZANDO"
                self.get_logger().info("Giro completado")

            else:
                velocidad_angular = (
                    self.VELOCIDAD_GIRO if error > 0
                    else -self.VELOCIDAD_GIRO
                )

                self.publicar_velocidad(
                    0.0, velocidad_angular)

    def destroy_node(self):
        self.publicar_velocidad(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = ExplorationNode()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.publicar_velocidad(0.0, 0.0)
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
