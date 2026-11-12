
import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray, Float32MultiArray, Int8, String, Bool
from std_srvs.srv import Trigger


SERVICIO_TAKEOFF = "/flight_control_node/takeoff"
SERVICIO_LAND = "/flight_control_node/land"


class NodoMaestro(Node):

    def __init__(self):
        super().__init__("nodo_maestro")

        self.create_subscription(
            UInt16MultiArray, "/crazyflie/sensores_raw",
            self.sensores_callback, 10)

        self.pub_sensores = self.create_publisher(
            Float32MultiArray, "/crazyflie/sensores", 10)
        self.pub_estado = self.create_publisher(
            Int8, "/crazyflie/estado_mision", 10)
        self.pub_orden = self.create_publisher(
            String, "/crazyflie/orden_nodos", 10)

        self.create_subscription(
            Bool, "/crazyflie/mapa_listo", self.mapa_callback, 10)
        self.create_subscription(
            Bool, "/crazyflie/mision_completada", self.mision_callback, 10)

        self.takeoff_client = self.create_client(Trigger, SERVICIO_TAKEOFF)
        self.land_client = self.create_client(Trigger, SERVICIO_LAND)

        self.volando = False
        self.aterrizando = False
        self.despegue_pendiente = False
        self.mapa_generado = False
        self.planificacion_activa = False
        self.timer_inicio = None

        self.get_logger().info("Nodo maestro iniciado")

    def sensores_callback(self, msg):
        if len(msg.data) < 5:
            return

        salida = Float32MultiArray()
        # Las lecturas cero se consideran sin medida valida.
        salida.data = [
            float(v) if v > 0 else 9900.0
            for v in msg.data[:5]
        ]
        self.pub_sensores.publish(salida)

    def mandar_orden(self, orden):
        msg = String()
        msg.data = orden
        self.pub_orden.publish(msg)
        self.get_logger().info(f"Orden enviada: {orden}")

    def cambiar_estado_exploracion(self, estado):
        msg = Int8()
        msg.data = estado
        self.pub_estado.publish(msg)

    def despegar(self):
        if self.volando or self.despegue_pendiente:
            self.get_logger().warning("El dron ya esta volando o despegando")
            return

        if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Servicio de despegue no disponible")
            return

        self.despegue_pendiente = True
        future = self.takeoff_client.call_async(Trigger.Request())
        future.add_done_callback(self.takeoff_resultado)

        self.get_logger().info("Solicitando despegue...")

    def takeoff_resultado(self, future):
        self.despegue_pendiente = False

        try:
            respuesta = future.result()
        except Exception as e:
            self.get_logger().error(f"Error en despegue: {e}")
            return

        if not respuesta.success:
            self.get_logger().error(f"Despegue rechazado: {respuesta.message}")
            return

        self.volando = True
        self.mapa_generado = False
        self.planificacion_activa = False

        self.get_logger().info(
            "Despegue aceptado. Esperando a que termine la maniobra...")

        self.timer_inicio = self.create_timer(
            2.5, self.iniciar_exploracion)

    def iniciar_exploracion(self):
        if self.timer_inicio is not None:
            self.timer_inicio.cancel()
            self.destroy_timer(self.timer_inicio)
            self.timer_inicio = None

        if not self.volando:
            return

        self.mandar_orden("INICIAR_SLAM")
        self.cambiar_estado_exploracion(1)
        self.get_logger().info("Exploracion activada")

    def detener_todos(self):
        self.cambiar_estado_exploracion(0)
        self.mandar_orden("DETENER_SLAM")
        self.mandar_orden("DETENER_PLANIFICACION")

    def aterrizar(self):
        if not self.volando or self.aterrizando:
            return

        self.detener_todos()

        if not self.land_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Servicio de aterrizaje no disponible")
            return

        self.aterrizando = True
        future = self.land_client.call_async(Trigger.Request())
        future.add_done_callback(self.land_resultado)
        self.get_logger().info("Solicitando aterrizaje...")

    def land_resultado(self, future):
        try:
            respuesta = future.result()
        except Exception as e:
            self.get_logger().error(f"Error en aterrizaje: {e}")
            self.aterrizando = False
            return

        if respuesta.success:
            self.get_logger().info("Aterrizaje ordenado por P1")
            self.volando = False
        else:
            self.get_logger().error(
                f"Aterrizaje rechazado: {respuesta.message}")

        self.aterrizando = False

    def emergencia(self):
        self.get_logger().warning(
            "Parada solicitada: deteniendo exploracion y aterrizando")
        self.cambiar_estado_exploracion(0)
        self.aterrizar()

    def mapa_callback(self, msg):
        if not msg.data or not self.volando or self.mapa_generado:
            return

        self.mapa_generado = True
        self.get_logger().info("Mapa recibido: iniciando planificacion")

        self.cambiar_estado_exploracion(0)
        self.mandar_orden("DETENER_SLAM")
        self.mandar_orden("INICIAR_PLANIFICACION")
        self.planificacion_activa = True

    def mision_callback(self, msg):
        if msg.data and self.planificacion_activa:
            self.get_logger().info("Mision completada")
            self.aterrizar()

    def parada_por_teclado(self):
        self.cambiar_estado_exploracion(0)
        self.mandar_orden("DETENER_SLAM")
        self.mandar_orden("DETENER_PLANIFICACION")

        if self.volando:
            self.aterrizar()


def leer_tecla(settings, timeout=0.05):
    # cbreak conserva la señal Ctrl+C del terminal.
    tty.setcbreak(sys.stdin.fileno())
    try:
        lista, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if lista else ""
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoMaestro()

    settings = None
    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin)

    menu = """
========== NODO MAESTRO ==========

ESPACIO : Despegar e iniciar exploracion
L       : Aterrizar
0       : Detener exploracion y SLAM
Q       : Aterrizaje controlado
CTRL+C  : Parada y aterrizaje controlado

==================================
"""
    print(menu)

    try:
        while rclpy.ok():
            tecla = ""

            if settings is not None:
                tecla = leer_tecla(settings, 0.05).lower()

            if tecla == " ":
                nodo.despegar()
            elif tecla == "l":
                nodo.aterrizar()
            elif tecla == "0":
                nodo.parada_por_teclado()
            elif tecla == "q":
                nodo.emergencia()

            rclpy.spin_once(nodo, timeout_sec=0.01)

    except KeyboardInterrupt:
        print("\nCTRL+C: solicitando aterrizaje controlado...")
        nodo.cambiar_estado_exploracion(0)
        nodo.mandar_orden("DETENER_SLAM")
        nodo.mandar_orden("DETENER_PLANIFICACION")

        if nodo.volando and not nodo.aterrizando:
            nodo.aterrizar()
            # Dar tiempo a que P1 reciba y ejecute la orden.
            limite = time.monotonic() + 2.5
            while rclpy.ok() and nodo.aterrizando and time.monotonic() < limite:
                rclpy.spin_once(nodo, timeout_sec=0.05)

    finally:
        if settings is not None:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, settings)

        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
