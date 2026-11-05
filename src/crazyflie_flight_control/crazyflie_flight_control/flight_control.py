#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_control.py  -  Persona 1: control de vuelo y estabilización (Crazyflie 2.1)
Compatible con Node 3 (Exploración) y Node 5 (Planificación)
"""

import math
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

class EstadoVuelo(str, Enum):
    IDLE = 'IDLE'
    TAKING_OFF = 'TAKING_OFF'
    HOVERING = 'HOVERING'
    MOVING = 'MOVING'
    LANDING = 'LANDING'
    EMERGENCY = 'EMERGENCY'

MODO_HOLD = 'HOLD'
MODO_GOTO = 'GOTO'
MODO_VEL = 'VEL'

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
            cfg.error_cb.add_callback(lambda lc, msg: self.log.error(f'Error de log {lc.name}: {msg}'))
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

class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control')
        self._declarar_parametros()

        self.estado = EstadoVuelo.IDLE
        self.modo = MODO_HOLD
        self._deadline = 0.0
        self._vel_cmd = (0.0, 0.0, 0.0, 0.0)
        self._ultimo_twist = 0.0
        self._z_objetivo = 0.0
        self._bateria_baja_cnt = 0

        self.link = CrazyflieLink(self.get_parameter('uri').value, self.get_logger())
        self.link.conectar()

        self.pid = PidTuner(self.link.cf, self.get_logger())
        self._pid_base = self.pid.leer_todos()
        for n, v in self._pid_base.items():
            self.declare_parameter('pid.' + n, v)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # REAJUSTE DE TÓPICOS PARA NODO 3 Y NODO 5
        self.pub_estado = self.create_publisher(String, '/estado_vuelo', 10)
        self.create_subscription(
            Twist, self.get_parameter('topic_cmd_vel').value, self._cb_cmd_vel, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('topic_goal_pose').value, self._cb_goal_pose, 10)
            
        self.create_service(Trigger, '~/takeoff', self._srv_takeoff)
        self.create_service(Trigger, '~/land', self._srv_land)
        self.create_service(Trigger, '~/emergency', self._srv_emergency)

        hz = self.get_parameter('control_rate_hz').value
        self._dt = 1.0 / hz
        self.create_timer(self._dt, self._bucle_control)
        self.create_timer(0.2, self._publicar_estado)
        self._publicar_estado()
        self.get_logger().info('flight_control listo (estado IDLE).')

    def _declarar_parametros(self):
        d = self.declare_parameter
        d('uri', 'radio://0/80/2M/E7E7E7E7AA')
        d('topic_cmd_vel', '/crazyflie/cmd_vel') # REAJUSTADO PARA EL NODO 3 (Exploración)
        d('topic_goal_pose', '/objetivo_vuelo')  # MANTIENE EL DEL NODO 5 (Planificación)
        d('control_rate_hz', 20.0)
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
        d('battery_min_takeoff', 3.7)
        d('battery_land', 3.3)
        d('max_tilt_deg', 60.0)
        d('pid_max_factor', 3.0)

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

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _yaw_de_cuaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _set_estado(self, nuevo):
        if nuevo != self.estado:
            self.get_logger().info(f'Estado: {self.estado.value} -> {nuevo.value}')
            self.estado = nuevo
            self._publicar_estado()

    def _publicar_estado(self):
        self.pub_estado.publish(String(data=self.estado.value))

    def _puede_moverse(self):
        return self.estado in (EstadoVuelo.HOVERING, EstadoVuelo.MOVING)

    def _srv_takeoff(self, _req, res):
        if self.estado != EstadoVuelo.IDLE:
            res.success, res.message = False, f'No se puede despegar en estado {self.estado.value}'
            return res
        snap = self.link.snapshot()
        vbat = snap.get('pm.vbat', 0.0)
        if not snap:
            res.success, res.message = False, 'Sin telemetría todavía'
            return res
        if vbat < self._p('battery_min_takeoff'):
            res.success, res.message = False, f'Batería baja ({vbat:.2f} V)'
            return res

        altura = self._clamp(self._p('takeoff_height'), self._p('min_height'), self._p('max_height'))
        dur = self._p('takeoff_duration')
        self.link.armar()
        self.link.despegar(altura, dur)
        self._z_objetivo = altura
        self.modo = MODO_HOLD
        self._deadline = time.monotonic() + dur + 0.5
        self._set_estado(EstadoVuelo.TAKING_OFF)
        res.success, res.message = True, f'Despegando a {altura:.2f} m'
        return res

    def _srv_land(self, _req, res):
        if self.estado in (EstadoVuelo.IDLE, EstadoVuelo.LANDING, EstadoVuelo.EMERGENCY):
            res.success, res.message = False, f'Nada que aterrizar en estado {self.estado.value}'
            return res
        self._iniciar_aterrizaje('solicitud de servicio')
        res.success, res.message = True, 'Aterrizando'
        return res

    def _srv_emergency(self, _req, res):
        self._emergencia('solicitud de servicio')
        res.success, res.message = True, 'Parada de emergencia ejecutada'
        return res

    def _iniciar_aterrizaje(self, motivo):
        snap = self.link.snapshot()
        z = snap.get('kalman.stateZ', self._z_objetivo)
        dur = max(1.5, z / self._p('land_speed'))
        if self.modo == MODO_VEL:
            self.link.liberar_bajo_nivel()
        self.link.aterrizar(0.0, dur)
        self.modo = MODO_HOLD
        self._deadline = time.monotonic() + dur + 1.0
        self.get_logger().warn(f'Aterrizaje ({motivo}), duración {dur:.1f} s')
        self._set_estado(EstadoVuelo.LANDING)

    def _emergencia(self, motivo):
        self.get_logger().error(f'¡EMERGENCIA! Motivo: {motivo}')
        try:
            self.link.emergencia()
        except Exception as e:
            self.get_logger().error(f'No se pudo enviar emergency stop: {e}')
        self._set_estado(EstadoVuelo.EMERGENCY)

    def _cb_cmd_vel(self, msg: Twist):
        if not self._puede_moverse():
            self.get_logger().debug(f'cmd_vel ignorado en estado {self.estado.value}',
                                    throttle_duration_sec=2.0)
            return
        vxy, vz_max = self._p('max_vel_xy'), self._p('max_vel_z')
        vx = self._clamp(msg.linear.x, -vxy, vxy)
        vy = self._clamp(msg.linear.y, -vxy, vxy)
        vz = self._clamp(msg.linear.z, -vz_max, vz_max)
        wz = msg.angular.z
        no_nulo = any(abs(c) > 1e-3 for c in (vx, vy, vz, wz))

        if self.modo != MODO_VEL:
            if not no_nulo:
                return
            snap = self.link.snapshot()
            self._z_objetivo = snap.get('kalman.stateZ', self._z_objetivo)
            self.modo = MODO_VEL

        self._vel_cmd = (vx, vy, vz, wz)
        self._ultimo_twist = time.monotonic()
        if no_nulo:
            self._set_estado(EstadoVuelo.MOVING)

    def _cb_goal_pose(self, msg: PoseStamped):
        if not self._puede_moverse():
            self.get_logger().debug(f'goal_pose ignorado en estado {self.estado.value}',
                                    throttle_duration_sec=2.0)
            return
        p = msg.pose.position
        if not all(math.isfinite(c) for c in (p.x, p.y, p.z)):
            self.get_logger().warn('goal_pose con valores no finitos: ignorado')
            return
        z = self._clamp(p.z, self._p('min_height'), self._p('max_height'))
        yaw = self._yaw_de_cuaternion(msg.pose.orientation)

        snap = self.link.snapshot()
        x0, y0, z0 = (snap.get('kalman.stateX', 0.0), snap.get('kalman.stateY', 0.0),
                      snap.get('kalman.stateZ', z))
        dist = math.sqrt((p.x - x0) ** 2 + (p.y - y0) ** 2 + (z - z0) ** 2)
        dur = max(1.0, dist / self._p('max_speed_goto'))

        if self.modo == MODO_VEL:
            self.link.liberar_bajo_nivel()
        self.link.ir_a(p.x, p.y, z, yaw, dur)
        self.modo = MODO_GOTO
        self._z_objetivo = z
        self._deadline = time.monotonic() + dur + 0.3
        self._set_estado(EstadoVuelo.MOVING)

    def _bucle_control(self):
        ahora = time.monotonic()

        if self.estado == EstadoVuelo.EMERGENCY:
            return
        if self.link.enlace_perdido.is_set():
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
                self._set_estado(EstadoVuelo.IDLE)

        elif self.estado in (EstadoVuelo.HOVERING, EstadoVuelo.MOVING):
            if self.modo == MODO_GOTO and ahora >= self._deadline:
                self.modo = MODO_HOLD
                self._set_estado(EstadoVuelo.HOVERING)

            elif self.modo == MODO_VEL:
                if ahora - self._ultimo_twist > self._p('cmd_vel_timeout'):
                    self._vel_cmd = (0.0, 0.0, 0.0, 0.0)
                    self._set_estado(EstadoVuelo.HOVERING)
                vx, vy, vz, wz = self._vel_cmd
                self._z_objetivo = self._clamp(self._z_objetivo + vz * self._dt,
                                               self._p('min_height'), self._p('max_height'))
                yaw_rate = self._clamp(math.degrees(wz), -self._p('max_yaw_rate_deg'),
                                       self._p('max_yaw_rate_deg'))
                self.link.hover(vx, vy, yaw_rate, self._z_objetivo)

    def _comprobar_seguridad(self):
        snap = self.link.snapshot()
        if not snap:
            return False

        tilt = max(abs(snap.get('stabilizer.roll', 0.0)), abs(snap.get('stabilizer.pitch', 0.0)))
        if tilt > self._p('max_tilt_deg'):
            self._emergencia(f'inclinación {tilt:.0f}° > {self._p("max_tilt_deg"):.0f}°')
            return True

        vbat = snap.get('pm.vbat', 0.0)
        if self.estado in (EstadoVuelo.HOVERING, EstadoVuelo.MOVING) and 0.0 < vbat < self._p('battery_land'):
            self._bateria_baja_cnt += 1
            if self._bateria_baja_cnt >= int(0.5 / self._dt):
                self._iniciar_aterrizaje(f'batería baja ({vbat:.2f} V)')
                return True
        else:
            self._bateria_baja_cnt = 0
        return False

    def apagado_seguro(self):
        try:
            if self.estado in (EstadoVuelo.TAKING_OFF, EstadoVuelo.HOVERING, EstadoVuelo.MOVING):
                self.get_logger().warn('Cierre solicitado en vuelo: aterrizando...')
                z = self.link.snapshot().get('kalman.stateZ', self._z_objetivo)
                dur = max(1.5, z / self._p('land_speed'))
                if self.modo == MODO_VEL:
                    self.link.liberar_bajo_nivel()
                self.link.aterrizar(0.0, dur)
                time.sleep(dur + 1.0)
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

