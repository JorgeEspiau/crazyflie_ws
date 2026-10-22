# Importamos herramientas del sistema operativo para poder "secuestrar" el teclado
import sys
import termios
import tty

# NUEVO: librerías estándar para el servidor del panel web (no hay que instalar nada)
import json
import math
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Importamos las librerías base de ROS 2
import rclpy
from rclpy.node import Node

# Importamos el cerebro de Bitcraze (cflib) para hablar por radio con el dron
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig          # NUEVO: telemetría real del dron
from cflib.positioning.motion_commander import MotionCommander

# Esta es la "matrícula" única de tu dron. Si no es esta, no conectará.
URI = 'radio://0/80/2M/E7E7E7E7AA'

# NUEVO: puerto donde el nodo publica los eventos que lee el HTML
PUERTO_PANEL = 8765

MENU_HACKER = """
\033[1;35m🕹️ TERMINAL DE VUELO: MODO VIDEOJUEGO 🕹️\033[0m
-----------------------------------------
[\033[1;32mESPACIO\033[0m] : Despegar / Aterrizar
[\033[1;36mW/S/A/D\033[0m] : Moverse en horizontal (20 cm)
[\033[1;33m↑ / ↓\033[0m]   : Subir / Bajar altura (10 cm)
[\033[1;33m← / →\033[0m]   : Rotar sobre sí mismo (45 grados)
[\033[1;31mQ\033[0m]       : PARADA DE EMERGENCIA (Caída libre)
[CTRL+C]  : Salir y aterrizar
-----------------------------------------
"""


# ======================================================================
# NUEVO: Servidor del panel. Emite eventos "Server-Sent Events" (SSE) en
#   http://localhost:8765/events
# El HTML se conecta ahí y recibe cada comando y la telemetría en vivo.
# ======================================================================
class Panel:
    def __init__(self, puerto):
        self.clientes = []
        self.lock = threading.Lock()
        # Foto del estado actual: se la mandamos a quien se conecte tarde
        self.snapshot = {'type': 'snapshot',
                         'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
                         'state': 'connecting'}
        panel = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # silenciamos el log HTTP en la terminal
                pass

            def do_GET(self):
                if self.path != '/events':
                    self.send_response(404)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                # Permite abrir el HTML con doble clic (file://)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                cola = queue.Queue()
                with panel.lock:
                    panel.clientes.append(cola)
                    inicial = json.dumps(panel.snapshot)
                try:
                    self.wfile.write(f'data: {inicial}\n\n'.encode())
                    self.wfile.flush()
                    while True:
                        try:
                            msg = cola.get(timeout=5)
                            self.wfile.write(f'data: {msg}\n\n'.encode())
                        except queue.Empty:
                            self.wfile.write(b': ping\n\n')  # mantiene viva la conexión
                        self.wfile.flush()
                except Exception:
                    pass
                finally:
                    with panel.lock:
                        if cola in panel.clientes:
                            panel.clientes.remove(cola)

        self.server = ThreadingHTTPServer(('127.0.0.1', puerto), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def publicar(self, **datos):
        # Actualizamos la foto del estado según el tipo de evento
        with self.lock:
            if datos.get('type') == 'cmd':
                self.snapshot['pose'] = dict(datos['to'])
                self.snapshot['state'] = datos['state']
            elif datos.get('type') == 'state':
                self.snapshot['state'] = datos['state']
            elif datos.get('type') == 'emergency':
                self.snapshot['state'] = 'emergency'
            msg = json.dumps(datos)
            for cola in self.clientes:
                cola.put(msg)


def anunciar(panel, pose, key, label, bx=0.0, by=0.0, dz=0.0, dyaw=0.0, dur=1.0, estado='flying'):
    """
    Calcula dónde acabará el dron (dead-reckoning) y se lo cuenta al panel ANTES de moverlo.
    bx/by son metros en ejes del dron (x = delante, y = izquierda); dz en metros; dyaw en grados.
    El panel anima el movimiento durante 'dur' segundos, que es lo que tarda el comando real.
    """
    yaw = math.radians(pose['yaw'])
    pose['x'] += bx * math.cos(yaw) - by * math.sin(yaw)
    pose['y'] += bx * math.sin(yaw) + by * math.cos(yaw)
    pose['z'] = max(0.0, pose['z'] + dz)
    pose['yaw'] += dyaw
    panel.publicar(type='cmd', key=key, label=label, to=dict(pose), dur=dur, state=estado)


def get_key(settings):
    """Lee una tecla al instante, sin esperar a Enter (terminal en modo crudo)."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    # Las flechas envían 3 caracteres: '\x1b' + 2 más
    if key == '\x1b':
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    cflib.crtp.init_drivers()
    settings = termios.tcgetattr(sys.stdin)

    # NUEVO: arrancamos el panel y la pose "estimada" (origen = punto de despegue)
    panel = Panel(PUERTO_PANEL)
    pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    print(f"\n\033[1;36mPanel: abre panel_crazyflie.html (escucha en http://localhost:{PUERTO_PANEL}/events)\033[0m")

    print(f"\n\033[1;33mConectando a {URI}...\033[0m")

    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print("\033[1;32m¡Enlace de radio establecido!\033[0m")
        print(MENU_HACKER)
        panel.publicar(type='state', state='ground')

        # NUEVO: telemetría real a 10 Hz (posición, yaw y batería).
        # Con el Flow deck, stateEstimate usa el mismo origen que nuestro dead-reckoning.
        logconf = None
        try:
            logconf = LogConfig(name='Pose', period_in_ms=100)
            logconf.add_variable('stateEstimate.x', 'float')
            logconf.add_variable('stateEstimate.y', 'float')
            logconf.add_variable('stateEstimate.z', 'float')
            logconf.add_variable('stateEstimate.yaw', 'float')
            logconf.add_variable('pm.vbat', 'float')

            def on_tele(timestamp, data, _cfg):
                panel.publicar(type='tele',
                               x=round(data['stateEstimate.x'], 3),
                               y=round(data['stateEstimate.y'], 3),
                               z=round(data['stateEstimate.z'], 3),
                               yaw=round(data['stateEstimate.yaw'], 1),
                               vbat=round(data['pm.vbat'], 2))

            scf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(on_tele)
            logconf.start()
        except Exception as e:
            print(f"\033[1;33mSin telemetría real ({e}). El panel usará solo los comandos.\033[0m")
            logconf = None

        mc = MotionCommander(scf)
        volando = False

        try:
            while True:
                key = get_key(settings)

                if key == ' ':
                    if not volando:
                        print("\033[1;32m🚀 [ESPACIO] Despegando a 40 cm...\033[0m")
                        anunciar(panel, pose, 'space', 'Despegar a 40 cm',
                                 dz=0.4 - pose['z'], dur=0.4 / 1.0, estado='takeoff')
                        mc.take_off(height=0.4, velocity=1.0)
                        volando = True
                        panel.publicar(type='state', state='flying')
                    else:
                        print("\033[1;33m🛬 [ESPACIO] Aterrizando suavemente...\033[0m")
                        anunciar(panel, pose, 'space', 'Aterrizar',
                                 dz=-pose['z'], dur=max(pose['z'] / 0.2, 0.3), estado='landing')
                        mc.land()
                        volando = False
                        panel.publicar(type='state', state='ground')

                elif key.lower() == 'w' and volando:
                    print("⬆️  Avanzando 20 cm...")
                    anunciar(panel, pose, 'w', 'Avanzar 20 cm', bx=0.2, dur=1.0)
                    mc.forward(0.2, velocity=0.2)

                elif key.lower() == 's' and volando:
                    print("⬇️  Retrocediendo 20 cm...")
                    anunciar(panel, pose, 's', 'Retroceder 20 cm', bx=-0.2, dur=1.0)
                    mc.back(0.2, velocity=0.2)

                elif key.lower() == 'a' and volando:
                    print("⬅️  Desplazamiento izquierdo 20 cm...")
                    anunciar(panel, pose, 'a', 'Izquierda 20 cm', by=0.2, dur=1.0)
                    mc.left(0.2, velocity=0.2)

                elif key.lower() == 'd' and volando:
                    print("➡️  Desplazamiento derecho 20 cm...")
                    anunciar(panel, pose, 'd', 'Derecha 20 cm', by=-0.2, dur=1.0)
                    mc.right(0.2, velocity=0.2)

                elif key == '\x1b[A' and volando:
                    print("🔼 Subiendo 10 cm...")
                    anunciar(panel, pose, 'up', 'Subir 10 cm', dz=0.1, dur=0.5)
                    mc.up(0.1, velocity=0.2)

                elif key == '\x1b[B' and volando:
                    print("🔽 Bajando 10 cm...")
                    anunciar(panel, pose, 'down', 'Bajar 10 cm', dz=-0.1, dur=0.5)
                    mc.down(0.1, velocity=0.2)

                elif key == '\x1b[D' and volando:
                    print("🔄 Rotando 45° a la izquierda...")
                    anunciar(panel, pose, 'left', 'Girar 45° izquierda', dyaw=45, dur=1.0)
                    mc.turn_left(45, rate=45)

                elif key == '\x1b[C' and volando:
                    print("🔄 Rotando 45° a la derecha...")
                    anunciar(panel, pose, 'right', 'Girar 45° derecha', dyaw=-45, dur=1.0)
                    mc.turn_right(45, rate=45)

                elif key.lower() == 'q':
                    print("\033[1;31m💀 [Q] ¡CORTANDO MOTORES! ¡CAÍDA LIBRE!\033[0m")
                    panel.publicar(type='emergency', key='q', label='PARADA DE EMERGENCIA')
                    # El MotionCommander sigue mandando setpoints de hover desde su hilo,
                    # y podrían pisar la orden de parada: lo detenemos primero.
                    try:
                        mc._thread.stop()
                    except Exception:
                        pass
                    scf.cf.commander.send_stop_setpoint()
                    break

                elif key == '\x03':
                    print("\nSaliendo del panel de control...")
                    if volando:
                        anunciar(panel, pose, 'ctrlc', 'Aterrizar (salida)',
                                 dz=-pose['z'], dur=max(pose['z'] / 0.2, 0.3), estado='landing')
                        mc.land()
                        panel.publicar(type='state', state='ground')
                    break

        except Exception as e:
            print(f"\033[1;31mError durante el vuelo: {e}\033[0m")

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            if logconf is not None:
                try:
                    logconf.stop()
                except Exception:
                    pass
            rclpy.shutdown()


if __name__ == '__main__':
    main()