import sys
import termios
import tty

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
\033[1;35m🕹️ TERMINAL DE VUELO + RVIZ2 🕹️\033[0m
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
        # Creamos el publicador para RViz2
        self.pub_pose = self.create_publisher(PoseStamped, '/crazyflie/pose', 10)

def get_key(settings):
    """Secuestra la terminal para leer teclas al instante."""
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
        
        # --- CONFIGURACIÓN DE TELEMETRÍA PARA RVIZ2 ---
        logconf = LogConfig(name='Pose', period_in_ms=100) # 10 Hz
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        
        def on_tele(timestamp, data, _cfg):
            msg = PoseStamped()
            msg.header.stamp = nodo.get_clock().now().to_msg()
            msg.header.frame_id = "world" # Referencia obligatoria para RViz
            
            # Pasamos las coordenadas
            msg.pose.position.x = data['stateEstimate.x']
            msg.pose.position.y = data['stateEstimate.y']
            msg.pose.position.z = data['stateEstimate.z']
            
            # Orientación neutra (para ver el punto en 3D es suficiente)
            msg.pose.orientation.w = 1.0 
            
            nodo.pub_pose.publish(msg)
            
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(on_tele)
        logconf.start()
        # ----------------------------------------------
        
        mc = MotionCommander(scf)
        volando = False
        
        try:
            while True:
                key = get_key(settings)
                
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
                    
                # Procesamos los eventos de ROS 2 para que el publicador no se congele
                rclpy.spin_once(nodo, timeout_sec=0)
                
        except Exception as e:
            print(f"\033[1;31mError: {e}\033[0m")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            logconf.stop()
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()