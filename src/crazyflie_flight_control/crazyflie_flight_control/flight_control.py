#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_control.py  -  Persona 1: control de vuelo y estabilización (Crazyflie 2.1)
Compatible con el Nodo 3 (Exploración) y el Nodo 5 (Planificación de rutas).

INTERFAZ ROS 2
--------------
Suscripciones:
  /crazyflie/cmd_vel     Twist         <- Nodo 3 (velocidades en ejes del cuerpo)
  /objetivo_vuelo        PoseStamped   <- Nodo 5 (waypoints absolutos, marco del Kalman)
  /crazyflie/sensores    Float32MultiArray (mm): [frente, atrás, izq, der, arriba]
  /comando_vuelo         String        TAKEOFF | LAND | EMERGENCY  (para el nodo jefe)
Publicaciones:
  /estado_vuelo          String        IDLE|TAKING_OFF|HOVERING|MOVING|LANDING|EMERGENCY
Servicios (Trigger): ~/takeoff, ~/land, ~/emergency
Parámetros PID en caliente: ros2 param set /flight_control pid.velCtlPid.vxKp 22.0

GESTIÓN DE LOS SENSORES (lo que tiene que hacer ESTE nodo, no el explorador)
----------------------------------------------------------------------------
Los láseres del Multi-ranger solo miden bien si apuntan perpendiculares a la pared
y el del suelo (Flow Deck) si el dron está nivelado. Con viento, ninguna de las dos
cosas se mantiene sola, así que en modo velocidad este nodo:

 1. MANTIENE EL RUMBO (heading hold). El modo hover del firmware solo controla la
    velocidad de giro (yawrate=0), no el ángulo: una racha de viento gira el dron y
    el error se queda acumulado. Aquí se guarda un rumbo de referencia y se corrige
    con un P sobre `stabilizer.yaw`. La referencia se recaptura al terminar cada giro
    ordenado por el Nodo 3 (opcionalmente se "ajusta" al múltiplo de 90° más cercano).
 2. CENTRA EL DRON en el pasillo moviéndolo LATERALMENTE (vy), no girándolo: así
    el rumbo (y por tanto el láser lateral) no se toca.
 3. NIVELA: si roll/pitch superan un umbral (el láser del suelo y los laterales ya
    no apuntan bien), se anulan vx/vy hasta que el firmware recupere la horizontal.
    Con timeout, para no quedarse bloqueado si el viento exige inclinación sostenida.

ARBITRAJE ENTRE NODO 3 Y NODO 5
-------------------------------
Gana el que empezó primero: un goal no interrumpe una velocidad "viva" y un Twist no
interrumpe un go_to en curso. Los goals que llegan durante un go_to se ENCOLAN (en
orden) en lugar de pisar al anterior, así no se pierde ningún waypoint aunque el
Nodo 5 envíe dos seguidos.
"""

import math
import threading
import time
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


# --------------------------------------------------------------------------
# Estados y modos
# --------------------------------------------------------------------------
class EstadoVuelo(str, Enum):
    IDLE = 'IDLE'
    TAKING_OFF = 'TAKING_OFF'
    HOVERING = 'HOVERING'
    MOVING = 'MOVING'
    LANDING = 'LANDING'
    EMERGENCY = 'EMERGENCY'


MODO_HOLD = 'HOLD'   # el commander de alto nivel mantiene el último setpoint
MODO_GOTO = 'GOTO'   # trayectoria go_to en curso (alto nivel)
MODO_VEL = 'VEL'     # streaming de setpoints hover (bajo nivel)


# --------------------------------------------------------------------------
# Capa de enlace con el dron (todo lo que toca cflib vive aquí)
# --------------------------------------------------------------------------
class CrazyflieLink:
    def __init__(self, uri, logger):
        self.uri = uri
        self.log = logger
        self.scf = None
        self.cf = None
        self.enlace_perdido = threading.Event()
        self._lock = threading.Lock()
        self._telem = {}

    def conectar(self):
        cflib.crtp.init_drivers()
        self.log.info(f'Conectando a {self.uri} ...')
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache'))
        self.scf.open_link()
        self.scf.wait_for_params()
        self.cf = self.scf.cf
        self.cf.disconnected.add_callback(self._on_desconectado)
        self.cf.connection_lost.add_callback(self._on_desconectado_msg)

        self.cf.param.set_value('commander.enHighLevel', '1')
        self._iniciar_telemetria()
        self._resetear_estimador()
        self.log.info('Enlace listo.')

    def _on_desconectado(self, uri):
        self.log.error(f'Enlace perdido con {uri}')
        self.enlace_perdido.set()

    def _on_desconectado_msg(self, uri, msg):
        self.log.error(f'Conexión perdida: {msg}')
        self.enlace_perdido.set()

    def _resetear_estimador(self):
        self.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2.0)

    def _iniciar_telemetria(self):
        # Un paquete de log admite 26 bytes como máximo (6 floats) -> dos configs.
        grupos = {
            'FC_Posicion': ['kalman.stateX', 'kalman.stateY', 'kalman.stateZ', 'stabilizer.yaw'],
            'FC_Actitud': ['stabilizer.roll', 'stabilizer.pitch', 'pm.vbat'],
        }
        for nombre, variables in grupos.items():
            cfg = LogConfig(name=nombre, period_in_ms=100)
            for v in variables:
                cfg.add_variable(v, 'float')
            self.cf.log.add_config(cfg)
            cfg.data_received_cb.add_callback(self._on_log)
            cfg.error_cb.add_callback(
                lambda lc, msg: self.log.error(f'Error de log {lc.name}: {msg}'))
            cfg.start()

    def _on_log(self, timestamp, data, logconf):
        with self._lock:
            self._telem.update(data)

    def snapshot(self):
        with self._lock:
            return dict(self._telem)

    def armar(self):
        try:
            self.cf.platform.send_arming_request(True)
            time.sleep(0.1)
        except Exception as e:
            self.log.warn(f'Armado no disponible/necesario: {e}')

    def despegar(self, altura, duracion):
        self.cf.high_level_commander.takeoff(altura, duracion)

    def aterrizar(self, altura, duracion):
        self.cf.high_level_commander.land(altura, duracion)

    def ir_a(self, x, y, z, yaw_rad, duracion):
        self.cf.high_level_commander.go_to(x, y, z, yaw_rad, duracion)

    def hover(self, vx, vy, yaw_rate_deg, z):
        # vx, vy: ejes del cuerpo (m/s) | yaw_rate: grados/s (+ = izquierda) | z: altura (m)
        self.cf.commander.send_hover_setpoint(vx, vy, yaw_rate_deg, z)

    def liberar_bajo_nivel(self):
        self.cf.commander.send_notify_setpoint_stop()

    def parar_todo(self):
        try:
            self.cf.high_level_commander.stop()
            self.cf.commander.send_stop_setpoint()
        except Exception as e:
            self.log.error(f'Fallo al parar motores: {e}')

    def emergencia(self):
        self.cf.loc.send_emergency_stop()

    def cerrar(self):
        if self.scf is not None:
            try:
                self.scf.close_link()
            except Exception:
                pass


# --------------------------------------------------------------------------
# Gestor de parámetros PID
# --------------------------------------------------------------------------
class PidTuner:
    NOMBRES = (
        [f'pid_attitude.{e}_{g}' for e in ('roll', 'pitch', 'yaw') for g in ('kp', 'ki', 'kd')]
        + [f'pid_rate.{e}_{g}' for e in ('roll', 'pitch', 'yaw') for g in ('kp', 'ki', 'kd')]
        + [f'posCtlPid.{e}{g}' for e in ('x', 'y', 'z') for g in ('Kp', 'Ki', 'Kd')]
        + [f'velCtlPid.v{e}{g}' for e in ('x', 'y', 'z') for g in ('Kp', 'Ki', 'Kd')]
    )

    def __init__(self, cf, logger):
        self.cf = cf
        self.log = logger

    def leer(self, nombre):
        return float(self.cf.param.get_value(nombre))

    def leer_todos(self):
        valores = {}
        for n in self.NOMBRES:
            try:
                valores[n] = self.leer(n)
            except Exception as e:
                self.log.warn(f'No se pudo leer {n}: {e}')
        return valores

    def escribir(self, nombre, valor):
        self.cf.param.set_value(nombre, str(float(valor)))
        self.log.info(f'PID {nombre} <- {valor}')


# --------------------------------------------------------------------------
# Nodo ROS 2
# --------------------------------------------------------------------------
class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control')
        self._declarar_parametros()

        # --- máquina de estados ---
        self.estado = EstadoVuelo.IDLE
        self.modo = MODO_HOLD
        self._deadline = 0.0
        self._z_objetivo = 0.0
        self._bateria_baja_cnt = 0

        # --- órdenes de velocidad (Nodo 3) ---
        self._vel_cmd = (0.0, 0.0, 0.0, 0.0)     # vx, vy, vz, wz
        self._ultimo_twist = 0.0                 # último Twist (nulo o no)
        self._ultimo_twist_no_nulo = 0.0         # último Twist con movimiento real

        # --- sensores laterales (mm) ---
        self._dist_izq = 1000.0
        self._dist_der = 1000.0
        self._t_sensores = 0.0

        # --- mantenimiento de rumbo ---
        self._yaw_ref = None                     # grados; None = pendiente de capturar
        self._t_fin_giro = 0.0

        # --- nivelación (sensor del suelo / láseres) ---
        self._nivelando = False
        self._nivel_agotado = False
        self._t_nivelando = 0.0

        # --- cola de waypoints (Nodo 5) ---
        self._goal_queue = deque()

        # --- conexión (bloqueante, solo al arrancar) ---
        self.link = CrazyflieLink(self._p('uri'), self.get_logger())
        self.link.conectar()

        # --- PID en caliente ---
        self.pid = PidTuner(self.link.cf, self.get_logger())
        self._pid_base = self.pid.leer_todos()
        for n, v in self._pid_base.items():
            self.declare_parameter('pid.' + n, v)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # --- ROS: publicadores, suscriptores, servicios ---
        self.pub_estado = self.create_publisher(String, '/estado_vuelo', 10)
        self.create_subscription(Twist, self._p('topic_cmd_vel'), self._cb_cmd_vel, 10)
        self.create_subscription(PoseStamped, self._p('topic_goal_pose'), self._cb_goal_pose, 10)
        self.create_subscription(Float32MultiArray, self._p('topic_sensores'),
                                 self._cb_sensores, 10)
        self.create_subscription(String, self._p('topic_comando'), self._cb_comando, 10)
        self.create_service(Trigger, '~/takeoff', self._srv_takeoff)
        self.create_service(Trigger, '~/land', self._srv_land)
        self.create_service(Trigger, '~/emergency', self._srv_emergency)

        self._dt = 1.0 / self._p('control_rate_hz')
        self.create_timer(self._dt, self._bucle_control)
        self.create_timer(0.2, self._publicar_estado)
        self._publicar_estado()
        self.get_logger().info('flight_control listo (estado IDLE).')

    # ------------------------------------------------------------------
    # Parámetros
    # ------------------------------------------------------------------
    def _declarar_parametros(self):
        d = self.declare_parameter
        # Interfaz
        d('uri', 'radio://0/80/2M/E7E7E7E7AA')
        d('topic_cmd_vel', '/crazyflie/cmd_vel')
        d('topic_goal_pose', '/objetivo_vuelo')
        d('topic_sensores', '/crazyflie/sensores')
        d('topic_comando', '/comando_vuelo')
        d('control_rate_hz', 20.0)
        # Vuelo
        d('takeoff_height', 0.5)
        d('takeoff_duration', 2.5)
        d('min_height', 0.15)
        d('max_height', 1.2)
        d('land_speed', 0.15)
        d('max_speed_goto', 0.3)
        d('max_vel_xy', 0.5)
        d('max_vel_z', 0.3)
        d('max_yaw_rate_deg', 90.0)
        d('cmd_vel_timeout', 0.5)
        # Seguridad
        d('battery_min_takeoff', 3.7)
        d('battery_land', 3.3)
        d('max_tilt_deg', 60.0)           # emergencia
        d('pid_max_factor', 3.0)
        # Waypoints (Nodo 5)
        d('goal_queue_max', 8)
        d('map_origin_x', 0.0)   # posición del origen del Kalman expresada en el marco 'map'
        d('map_origin_y', 0.0)
        # Centrado en pasillo (vy)
        d('centrado_activo', True)
        d('centrado_dist_max_mm', 800.0)
        d('centrado_kp', 1.0)             # (m/s) por cada metro de offset
        d('centrado_vy_max', 0.15)
        d('sensor_timeout', 0.3)          # s sin datos de sensores -> no centrar
        # Mantenimiento de rumbo
        d('yaw_hold_activo', True)
        d('yaw_hold_kp', 1.5)             # (°/s) por cada grado de error
        d('yaw_hold_max_dps', 40.0)
        d('yaw_hold_deadband_deg', 1.0)
        d('yaw_hold_settle', 0.4)         # s de espera tras un giro antes de fijar el rumbo
        d('yaw_hold_sign', 1.0)           # pon -1.0 si al probar el dron corrige al revés
        d('yaw_snap_90', False)           # ajustar el rumbo al múltiplo de 90° más cercano
        d('yaw_snap_max_err_deg', 20.0)
        # Nivelación
        d('tilt_nivelar_deg', 8.0)        # por encima: se anulan vx, vy
        d('tilt_ok_deg', 4.0)             # por debajo: se reanuda
        d('tilt_nivelar_timeout', 3.0)    # s máximos esperando a nivelar

    def _p(self, nombre):
        return self.get_parameter(nombre).value

    def _on_set_parameters(self, params):
        cambios = []
        for p in params:
            if not p.name.startswith('pid.'):
                continue
            nombre_cf = p.name[len('pid.'):]
            if nombre_cf not in self._pid_base:
                return SetParametersResult(successful=False, reason='Parámetro PID desconocido')
            try:
                valor = float(p.value)
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason='Valor no numérico')
            tope = max(self._pid_base[nombre_cf] * self._p('pid_max_factor'), 1.0)
            if not (0.0 <= valor <= tope):
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name}: valor fuera de [0, {tope:.2f}] (límite de seguridad)')
            cambios.append((nombre_cf, valor))
        for nombre_cf, valor in cambios:
            self.pid.escribir(nombre_cf, valor)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _norm_ang(grados):
        """Normaliza a [-180, 180)."""
        return (grados + 180.0) % 360.0 - 180.0

    @classmethod
    def _dif_ang(cls, a, b):
        """Diferencia más corta a - b en grados."""
        return cls._norm_ang(a - b)

    @staticmethod
    def _yaw_de_cuaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _cuaternion_neutro(q):
        return abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6 and abs(q.w - 1.0) < 1e-6

    def _set_estado(self, nuevo):
        if nuevo != self.estado:
            self.get_logger().info(f'Estado: {self.estado.value} -> {nuevo.value}')
            self.estado = nuevo
            self._publicar_estado()

    def _publicar_estado(self):
        self.pub_estado.publish(String(data=self.estado.value))

    def _puede_moverse(self):
        return self.estado in (EstadoVuelo.HOVERING, EstadoVuelo.MOVING)

    def _vel_ocupado(self, ahora):
        """True si el Nodo 3 está mandando velocidades reales ahora mismo."""
        return (ahora - self._ultimo_twist_no_nulo) < self._p('cmd_vel_timeout')

    def _reiniciar_control_vel(self):
        """Estado limpio al (re)entrar en modo velocidad."""
        self._yaw_ref = None
        self._t_fin_giro = 0.0
        self._nivelando = False
        self._nivel_agotado = False

    # ------------------------------------------------------------------
    # Acciones de alto nivel (compartidas por servicios y topic de comandos)
    # ------------------------------------------------------------------
    def _hacer_despegue(self):
        if self.estado != EstadoVuelo.IDLE:
            return False, f'No se puede despegar en estado {self.estado.value}'
        snap = self.link.snapshot()
        if not snap:
            return False, 'Sin telemetría todavía'
        vbat = snap.get('pm.vbat', 0.0)
        if vbat < self._p('battery_min_takeoff'):
            return False, f'Batería baja ({vbat:.2f} V)'

        altura = self._clamp(self._p('takeoff_height'), self._p('min_height'),
                             self._p('max_height'))
        dur = self._p('takeoff_duration')
        self._goal_queue.clear()
        self._reiniciar_control_vel()
        self.link.armar()
        self.link.despegar(altura, dur)
        self._z_objetivo = altura
        self.modo = MODO_HOLD
        self._deadline = time.monotonic() + dur + 0.5
        self._set_estado(EstadoVuelo.TAKING_OFF)
        return True, f'Despegando a {altura:.2f} m'

    def _hacer_aterrizaje(self):
        if self.estado in (EstadoVuelo.IDLE, EstadoVuelo.LANDING, EstadoVuelo.EMERGENCY):
            return False, f'Nada que aterrizar en estado {self.estado.value}'
        self._iniciar_aterrizaje('solicitud externa')
        return True, 'Aterrizando'

    def _hacer_emergencia(self):
        self._emergencia('solicitud externa')
        return True, 'Parada de emergencia ejecutada'

    def _iniciar_aterrizaje(self, motivo):
        snap = self.link.snapshot()
        z = snap.get('kalman.stateZ', self._z_objetivo)
        dur = max(1.5, z / self._p('land_speed'))
        self._goal_queue.clear()
        if self.modo == MODO_VEL:
            self.link.liberar_bajo_nivel()
        self.link.aterrizar(0.0, dur)
        self.modo = MODO_HOLD
        self._deadline = time.monotonic() + dur + 1.0
        self.get_logger().warn(f'Aterrizaje ({motivo}), duración {dur:.1f} s')
        self._set_estado(EstadoVuelo.LANDING)

    def _emergencia(self, motivo):
        self.get_logger().error(f'¡EMERGENCIA! Motivo: {motivo}')
        self._goal_queue.clear()
        try:
            self.link.emergencia()
        except Exception as e:
            self.get_logger().error(f'No se pudo enviar emergency stop: {e}')
        self._set_estado(EstadoVuelo.EMERGENCY)

    # ------------------------------------------------------------------
    # Servicios y topic de comandos
    # ------------------------------------------------------------------
    def _srv_takeoff(self, _req, res):
        res.success, res.message = self._hacer_despegue()
        return res

    def _srv_land(self, _req, res):
        res.success, res.message = self._hacer_aterrizaje()
        return res

    def _srv_emergency(self, _req, res):
        res.success, res.message = self._hacer_emergencia()
        return res

    def _cb_comando(self, msg: String):
        cmd = msg.data.strip().upper()
        acciones = {'TAKEOFF': self._hacer_despegue, 'LAND': self._hacer_aterrizaje,
                    'EMERGENCY': self._hacer_emergencia}
        if cmd not in acciones:
            self.get_logger().warn(f'Comando desconocido: "{msg.data}"')
            return
        ok, texto = acciones[cmd]()
        (self.get_logger().info if ok else self.get_logger().warn)(f'{cmd}: {texto}')

    # ------------------------------------------------------------------
    # Callbacks de entrada (cortos y no bloqueantes)
    # ------------------------------------------------------------------
    def _cb_sensores(self, msg: Float32MultiArray):
        # Orden: Frente, Atrás, Izq, Der, Arriba (mm)
        if len(msg.data) >= 4:
            self._dist_izq = msg.data[2]
            self._dist_der = msg.data[3]
            self._t_sensores = time.monotonic()

    def _cb_cmd_vel(self, msg: Twist):
        if not self._puede_moverse():
            self.get_logger().debug(f'cmd_vel ignorado en estado {self.estado.value}',
                                    throttle_duration_sec=2.0)
            return
        ahora = time.monotonic()
        vxy, vz_max = self._p('max_vel_xy'), self._p('max_vel_z')
        vx = self._clamp(msg.linear.x, -vxy, vxy)
        vy = self._clamp(msg.linear.y, -vxy, vxy)
        vz = self._clamp(msg.linear.z, -vz_max, vz_max)
        wz = msg.angular.z
        no_nulo = any(abs(c) > 1e-3 for c in (vx, vy, vz, wz))

        # Arbitraje: un go_to en curso no se interrumpe con velocidades
        if self.modo == MODO_GOTO:
            if no_nulo:
                self.get_logger().warn('cmd_vel ignorado: hay un go_to en curso (Nodo 5)',
                                       throttle_duration_sec=2.0)
            return

        if self.modo != MODO_VEL:
            if not no_nulo:
                return          # un Twist nulo no debe sacar al dron del hover
            snap = self.link.snapshot()
            self._z_objetivo = snap.get('kalman.stateZ', self._z_objetivo)
            self._reiniciar_control_vel()
            self.modo = MODO_VEL

        self._vel_cmd = (vx, vy, vz, wz)
        self._ultimo_twist = ahora
        if no_nulo:
            self._ultimo_twist_no_nulo = ahora
            self._set_estado(EstadoVuelo.MOVING)

    def _cb_goal_pose(self, msg: PoseStamped):
        if not self._puede_moverse():
            self.get_logger().debug(f'goal_pose ignorado en estado {self.estado.value}',
                                    throttle_duration_sec=2.0)
            return
        ahora = time.monotonic()
        p = msg.pose.position
        if not all(math.isfinite(c) for c in (p.x, p.y, p.z)):
            self.get_logger().warn('goal_pose con valores no finitos: ignorado')
            return

        # Arbitraje: no se pisa una velocidad "viva" del Nodo 3
        if self.modo == MODO_VEL and self._vel_ocupado(ahora):
            self.get_logger().warn('goal_pose ignorado: el Nodo 3 está controlando por velocidad',
                                   throttle_duration_sec=2.0)
            return

        # Del marco 'map' al marco del Kalman
        x = p.x - self._p('map_origin_x')
        y = p.y - self._p('map_origin_y')
        # z sin definir (<= 5 cm) -> mantener la altura actual
        z = p.z if p.z > 0.05 else None
        # Cuaternión neutro = "sin yaw especificado" -> mantener el rumbo actual
        yaw = None if self._cuaternion_neutro(msg.pose.orientation) \
            else self._yaw_de_cuaternion(msg.pose.orientation)
        goal = (x, y, z, yaw)

        # Si hay un go_to en curso (o cola pendiente), se encola en orden
        if self.modo == MODO_GOTO and (ahora < self._deadline or self._goal_queue):
            if len(self._goal_queue) < self._p('goal_queue_max'):
                self._goal_queue.append(goal)
                self.get_logger().info(f'Goal encolado ({len(self._goal_queue)} pendientes)')
            else:
                self.get_logger().warn('Cola de goals llena: goal descartado')
            return
        self._ejecutar_goal(*goal)

    def _ejecutar_goal(self, x, y, z, yaw_rad):
        snap = self.link.snapshot()
        z0 = snap.get('kalman.stateZ', self._z_objetivo)
        z = z0 if z is None else z
        z = self._clamp(z, self._p('min_height'), self._p('max_height'))
        if yaw_rad is None:
            yaw_rad = math.radians(snap.get('stabilizer.yaw', 0.0))
        x0, y0 = snap.get('kalman.stateX', 0.0), snap.get('kalman.stateY', 0.0)
        dist = math.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
        dur = max(1.0, dist / self._p('max_speed_goto'))

        if self.modo == MODO_VEL:
            self.link.liberar_bajo_nivel()   # el alto nivel recupera el control
        self.link.ir_a(x, y, z, yaw_rad, dur)
        self.modo = MODO_GOTO
        self._z_objetivo = z
        self._deadline = time.monotonic() + dur + 0.3
        self._set_estado(EstadoVuelo.MOVING)

    # ------------------------------------------------------------------
    # Lazo de control periódico
    # ------------------------------------------------------------------
    def _bucle_control(self):
        ahora = time.monotonic()

        if self.estado == EstadoVuelo.EMERGENCY:
            return
        if self.link.enlace_perdido.is_set():
            self._goal_queue.clear()
            self._set_estado(EstadoVuelo.EMERGENCY)
            return

        en_aire = self.estado in (EstadoVuelo.TAKING_OFF, EstadoVuelo.HOVERING,
                                  EstadoVuelo.MOVING, EstadoVuelo.LANDING)
        if en_aire and self._comprobar_seguridad():
            return

        if self.estado == EstadoVuelo.TAKING_OFF:
            if ahora >= self._deadline:
                self._set_estado(EstadoVuelo.HOVERING)

        elif self.estado == EstadoVuelo.LANDING:
            if ahora >= self._deadline:
                self.link.parar_todo()
                self.modo = MODO_HOLD
                self._set_estado(EstadoVuelo.IDLE)

        elif self.estado in (EstadoVuelo.HOVERING, EstadoVuelo.MOVING):
            if self.modo == MODO_GOTO:
                if ahora >= self._deadline:
                    if self._goal_queue:
                        self._ejecutar_goal(*self._goal_queue.popleft())
                    else:
                        self.modo = MODO_HOLD
                        self._set_estado(EstadoVuelo.HOVERING)

            elif self.modo == MODO_VEL:
                # Watchdog: sin órdenes recientes -> velocidad 0 (se mantiene rumbo y altura)
                if ahora - self._ultimo_twist > self._p('cmd_vel_timeout'):
                    self._vel_cmd = (0.0, 0.0, 0.0, 0.0)
                # Sin velocidades reales vivas, el dron está quieto: avisar al planificador
                if self.estado == EstadoVuelo.MOVING and not self._vel_ocupado(ahora):
                    self._set_estado(EstadoVuelo.HOVERING)
                self._enviar_hover(ahora)

    # ------------------------------------------------------------------
    # Control en modo velocidad: centrado + nivelación + rumbo
    # ------------------------------------------------------------------
    def _enviar_hover(self, ahora):
        snap = self.link.snapshot()
        vx, vy, vz, wz = self._vel_cmd

        # 1) Centrado en pasillo con velocidad LATERAL (no se toca el rumbo)
        if (self._p('centrado_activo') and vx > 0.05
                and abs(wz) < 0.01 and abs(vy) < 0.01):
            vy = self._vy_centrado(ahora, vy)

        # 2) Nivelación: si el dron está muy inclinado, los láseres no apuntan bien
        vx, vy = self._aplicar_nivelacion(snap, ahora, vx, vy)

        # 3) Rumbo: giro ordenado o mantenimiento del rumbo de referencia
        yaw_rate = self._yaw_rate_cmd(snap, ahora, wz)

        # 4) Altura
        self._z_objetivo = self._clamp(self._z_objetivo + vz * self._dt,
                                       self._p('min_height'), self._p('max_height'))
        self.link.hover(vx, vy, yaw_rate, self._z_objetivo)

    def _vy_centrado(self, ahora, vy):
        """Velocidad lateral (+ = izquierda) para centrarse entre las dos paredes."""
        if ahora - self._t_sensores > self._p('sensor_timeout'):
            return vy                          # datos caducados: no corregir a ciegas
        lim = self._p('centrado_dist_max_mm')
        izq, der = self._dist_izq, self._dist_der
        if not (20.0 < izq < lim and 20.0 < der < lim):
            return vy                          # no hay dos paredes válidas (cruce, hueco...)
        # offset > 0: más cerca de la pared derecha -> moverse a la izquierda
        offset_m = (izq - der) / 2000.0
        vmax = self._p('centrado_vy_max')
        return self._clamp(self._p('centrado_kp') * offset_m, -vmax, vmax)

    def _aplicar_nivelacion(self, snap, ahora, vx, vy):
        if 'stabilizer.roll' not in snap or 'stabilizer.pitch' not in snap:
            return vx, vy
        tilt = math.hypot(snap['stabilizer.roll'], snap['stabilizer.pitch'])
        alto, ok = self._p('tilt_nivelar_deg'), self._p('tilt_ok_deg')

        if self._nivel_agotado:
            if tilt < ok:
                self._nivel_agotado = False    # se rearma cuando vuelve a estar nivelado
        elif not self._nivelando:
            if tilt > alto:
                self._nivelando = True
                self._t_nivelando = ahora
                self.get_logger().warn(
                    f'Inclinación {tilt:.1f}° > {alto:.0f}°: se frena para nivelar',
                    throttle_duration_sec=1.0)
        else:
            if tilt < ok:
                self._nivelando = False
                self.get_logger().info('Dron nivelado: se reanuda el avance')
            elif ahora - self._t_nivelando > self._p('tilt_nivelar_timeout'):
                self._nivelando = False
                self._nivel_agotado = True
                self.get_logger().error(
                    f'No se consigue nivelar ({tilt:.1f}°, viento sostenido?): se reanuda igualmente')

        return (0.0, 0.0) if self._nivelando else (vx, vy)

    def _yaw_rate_cmd(self, snap, ahora, wz):
        """Devuelve la velocidad de giro (°/s, + = izquierda) a enviar al dron."""
        max_dps = self._p('max_yaw_rate_deg')

        # Giro ordenado por el Nodo 3: se ejecuta tal cual y se invalida la referencia
        if abs(wz) >= 0.01:
            self._yaw_ref = None
            self._t_fin_giro = ahora
            return self._clamp(math.degrees(wz), -max_dps, max_dps)

        if not self._p('yaw_hold_activo') or 'stabilizer.yaw' not in snap:
            return 0.0
        yaw = snap['stabilizer.yaw']

        # Tras un giro: esperar a que el dron se asiente y fijar el nuevo rumbo
        if self._yaw_ref is None:
            if ahora - self._t_fin_giro < self._p('yaw_hold_settle'):
                return 0.0
            ref = yaw
            if self._p('yaw_snap_90'):
                candidato = round(yaw / 90.0) * 90.0
                if abs(self._dif_ang(candidato, yaw)) <= self._p('yaw_snap_max_err_deg'):
                    ref = candidato
            self._yaw_ref = self._norm_ang(ref)

        err = self._dif_ang(self._yaw_ref, yaw)
        if abs(err) < self._p('yaw_hold_deadband_deg'):
            return 0.0
        lim = min(self._p('yaw_hold_max_dps'), max_dps)
        return self._clamp(self._p('yaw_hold_sign') * self._p('yaw_hold_kp') * err, -lim, lim)

    # ------------------------------------------------------------------
    # Seguridad
    # ------------------------------------------------------------------
    def _comprobar_seguridad(self):
        """Devuelve True si se disparó una acción que cambia el estado."""
        snap = self.link.snapshot()
        if not snap:
            return False

        tilt = max(abs(snap.get('stabilizer.roll', 0.0)), abs(snap.get('stabilizer.pitch', 0.0)))
        if tilt > self._p('max_tilt_deg'):
            self._emergencia(f'inclinación {tilt:.0f}° > {self._p("max_tilt_deg"):.0f}°')
            return True

        vbat = snap.get('pm.vbat', 0.0)
        if (self.estado in (EstadoVuelo.HOVERING, EstadoVuelo.MOVING)
                and 0.0 < vbat < self._p('battery_land')):
            self._bateria_baja_cnt += 1
            if self._bateria_baja_cnt >= int(0.5 / self._dt):
                self._iniciar_aterrizaje(f'batería baja ({vbat:.2f} V)')
                return True
        else:
            self._bateria_baja_cnt = 0
        return False

    # ------------------------------------------------------------------
    # Apagado seguro
    # ------------------------------------------------------------------
    def apagado_seguro(self):
        try:
            if self.estado in (EstadoVuelo.TAKING_OFF, EstadoVuelo.HOVERING, EstadoVuelo.MOVING):
                self.get_logger().warn('Cierre solicitado en vuelo: aterrizando...')
                z = self.link.snapshot().get('kalman.stateZ', self._z_objetivo)
                dur = max(1.5, z / self._p('land_speed'))
                if self.modo == MODO_VEL:
                    self.link.liberar_bajo_nivel()
                self.link.aterrizar(0.0, dur)
                time.sleep(dur + 1.0)     # aquí sí se puede bloquear: el nodo ya no gira
                self.link.parar_todo()
        finally:
            self.link.cerrar()


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
            nodo.apagado_seguro()
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()