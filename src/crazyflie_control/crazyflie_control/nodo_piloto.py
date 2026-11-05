import sys
import termios
import tty
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E7AA'

MENU_HACKER = """
\033[1;35m🕹️ TERMINAL DE VUELO + RVIZ2 + SENSORES Y ÁNGULOS 🕹️\033[0m
-----------------------------------------
[\033[1;32mESPACIO\033[0m] : Despegar / Aterrizar
[\033[1;36mW/S/A/D\033[0m] : Moverse en horizontal (20 cm)
[\033[1;33m↑ / ↓\033[0m]   : Subir / Bajar altura (10 cm)
[\033[1;33m← / →\033[0m]   : Rotar sobre sí mismo (45 grados)
[\033[1;31mQ\033[0m]       : PARADA DE EMERGENCIA (Caída libre)
[CTRL+C]  : Salir y aterrizar
-----------------------------------------
"""

class NodoPiloto(Node):
    def __init__(self):
        super().__init__('nodo_piloto')
        self.pub_pose = self.create_publisher(PoseStamped, '/crazyflie/pose', 10)
        # Guardamos los ángulos aquí para poder imprimirlos junto con los láseres
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

def euler_a_cuaternion(roll, pitch, yaw):
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)
    
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    
    return qx, qy, qz, qw

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    cflib.crtp.init_drivers()
    
    settings = termios.tcgetattr(sys.stdin)
    nodo = NodoPiloto()
    
    print(f"\n\033[1;33mConectando a {URI}...\033[0m")
    
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print("\033[1;32m¡Enlace de radio establecido!\033[0m")
        print(MENU_HACKER)
        
        # --- 1. CONFIGURACIÓN DE TELEMETRÍA PARA RVIZ2 Y ÁNGULOS ---
        logconf_pose = LogConfig(name='Pose', period_in_ms=100)
        logconf_pose.add_variable('stateEstimate.x', 'float')
        logconf_pose.add_variable('stateEstimate.y', 'float')
        logconf_pose.add_variable('stateEstimate.z', 'float')
        logconf_pose.add_variable('stateEstimate.roll', 'float')
        logconf_pose.add_variable('stateEstimate.pitch', 'float')
        logconf_pose.add_variable('stateEstimate.yaw', 'float')
        
        def on_pose_data(timestamp, data, _cfg):
            # Actualizamos los ángulos internos del nodo para la consola
            nodo.roll = data['stateEstimate.roll']
            nodo.pitch = data['stateEstimate.pitch']
            nodo.yaw = data['stateEstimate.yaw']
            
            msg = PoseStamped()
            msg.header.stamp = nodo.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            
            msg.pose.position.x = data['stateEstimate.x']
            msg.pose.position.y = data['stateEstimate.y']
            msg.pose.position.z = data['stateEstimate.z']
            
            qx, qy, qz, qw = euler_a_cuaternion(nodo.roll, nodo.pitch, nodo.yaw)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            
            nodo.pub_pose.publish(msg)
            
        scf.cf.log.add_config(logconf_pose)
        logconf_pose.data_received_cb.add_callback(on_pose_data)
        logconf_pose.start()
        
        # --- 2. CONFIGURACIÓN DE SENSORES LÁSER (MULTIRANGER) ---
        logconf_sensores = LogConfig(name='Multiranger', period_in_ms=100)
        logconf_sensores.add_variable('range.front', 'uint16_t')
        logconf_sensores.add_variable('range.back', 'uint16_t')
        logconf_sensores.add_variable('range.left', 'uint16_t')
        logconf_sensores.add_variable('range.right', 'uint16_t')
        logconf_sensores.add_variable('range.up', 'uint16_t')
        
        def on_sensores_data(timestamp, data, _cfg):
            frente = data['range.front']
            atras = data['range.back']
            izq = data['range.left']
            der = data['range.right']
            arriba = data['range.up']
            
            # Imprimimos los ángulos del dron y las distancias láser en una sola línea fluida
            sys.stdout.write(f"\r\033[K[Ángulos] R:{nodo.roll:6.1f} P:{nodo.pitch:6.1f} Y:{nodo.yaw:6.1f} | [Láser] F:{frente:4} A:{atras:4} I:{izq:4} D:{der:4} Ar:{arriba:4}")
            sys.stdout.flush()

        scf.cf.log.add_config(logconf_sensores)
        logconf_sensores.data_received_cb.add_callback(on_sensores_data)
        logconf_sensores.start()
        
        mc = MotionCommander(scf)
        volando = False
        
        try:
            while True:
                key = get_key(settings)
                print("") 
                
                if key == ' ':
                    if not volando:
                        print("\033[1;32m🚀 [ESPACIO] Despegando a 40 cm...\033[0m")
                        mc.take_off(height=0.4, velocity=0.2)
                        volando = True
                    else:
                        print("\033[1;33m🛬 [ESPACIO] Aterrizando suavemente...\033[0m")
                        mc.land()
                        volando = False
                        
                elif key.lower() == 'w' and volando:
                    print("⬆️  Avanzando 20 cm...")
                    mc.forward(0.2, velocity=0.2)
                elif key.lower() == 's' and volando:
                    print("⬇️  Retrocediendo 20 cm...")
                    mc.back(0.2, velocity=0.2)
                elif key.lower() == 'a' and volando:
                    print("⬅️  Desplazamiento izquierdo 20 cm...")
                    mc.left(0.2, velocity=0.2)
                elif key.lower() == 'd' and volando:
                    print("➡️  Desplazamiento derecho 20 cm...")
                    mc.right(0.2, velocity=0.2)
                elif key == '\x1b[A' and volando:
                    print("🔼 Subiendo 10 cm...")
                    mc.up(0.1, velocity=0.2)
                elif key == '\x1b[B' and volando:
                    print("🔽 Bajando 10 cm...")
                    mc.down(0.1, velocity=0.2)
                elif key == '\x1b[D' and volando:
                    print("🔄 Rotando 45° a la izquierda...")
                    mc.turn_left(45, rate=45)
                elif key == '\x1b[C' and volando:
                    print("🔄 Rotando 45° a la derecha...")
                    mc.turn_right(45, rate=45)
                elif key.lower() == 'q':
                    print("\033[1;31m💀 [Q] ¡CORTANDO MOTORES! ¡CAÍDA LIBRE!\033[0m")
                    scf.cf.commander.send_stop_setpoint()
                    break
                elif key == '\x03': 
                    print("\nSaliendo...")
                    if volando:
                        mc.land()
                    break
                    
                rclpy.spin_once(nodo, timeout_sec=0)
                
        except Exception as e:
            print(f"\n\033[1;31mError: {e}\033[0m")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            logconf_pose.stop()
            logconf_sensores.stop()
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()