import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32MultiArray, String

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E7AA'

MENU_MAESTRO = """
\033[1;35m NODO MAESTRO: CONTROL DE SISTEMA \033[0m
-----------------------------------------
[ Vuelo Manual ]
[\033[1;32mESPACIO\033[0m] : Despegar / Aterrizar
[\033[1;31mQ\033[0m]       : PARADA DE EMERGENCIA

[ Orquestación de Otros Nodos ]
[\033[1;36m1\033[0m] : Ordenar "EJECUTAR_EVASION" (Despierta nodo 1)
[\033[1;36m2\033[0m] : Ordenar "EJECUTAR_SEGUIMIENTO" (Despierta nodo 2)
[\033[1;33m0\033[0m] : Ordenar "DETENER_TODOS" (Pone a los nodos en espera)
[CTRL+C]  : Salir y apagar todo
-----------------------------------------
"""

class NodoMaestro(Node):
    def __init__(self):
        super().__init__('nodo_maestro_crazyflie')
        
        # 1. Publicador de sensores (láseres)
        self.pub_sensores = self.create_publisher(Int16MultiArray, 'sensores', 10)
        
        # 2. Publicador de ángulos (Roll, Pitch, Yaw)
        self.pub_angulos = self.create_publisher(Float32MultiArray, 'angulos', 10)
        
        # 3. Publicador de órdenes para otros nodos
        self.pub_orden = self.create_publisher(String, 'orden_nodos', 10)

        self.get_logger().info("Nodo Maestro iniciado. Esperando conexión...")

    def publicar_sensores(self, frente, atras, izq, der, arriba):
        """Empaqueta las lecturas de los láseres y las publica en /sensores"""
        msg = Int16MultiArray()
        # Orden del array: [Frente, Atrás, Izq, Der, Arriba]
        msg.data = [frente, atras, izq, der, arriba]
        self.pub_sensores.publish(msg)

    def publicar_angulos(self, roll, pitch, yaw):
        """Empaqueta los ángulos de actitud y los publica en /angulos"""
        msg = Float32MultiArray()
        # Orden del array: [Roll, Pitch, Yaw]
        msg.data = [float(roll), float(pitch), float(yaw)]
        self.pub_angulos.publish(msg)

    def mandar_orden(self, orden_str):
        """Envía una señal para despertar o detener a otros nodos"""
        msg = String()
        msg.data = orden_str
        self.pub_orden.publish(msg)
        print(f"\n\033[1;36m[MAESTRO] Orden enviada a la red: {orden_str}\033[0m")


def get_key_non_blocking(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    cflib.crtp.init_drivers()
    settings = termios.tcgetattr(sys.stdin)
    
    nodo = NodoMaestro()
    
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print(MENU_MAESTRO)
        
        # --- CONFIGURACIÓN DEL LOG (Sensores + Ángulos) ---
        logconf = LogConfig(name='MultirangerYAngulos', period_in_ms=100) # 10 Hz
        # Variables de distancia (Multiranger)
        logconf.add_variable('range.front', 'uint16_t')
        logconf.add_variable('range.back', 'uint16_t')
        logconf.add_variable('range.left', 'uint16_t')
        logconf.add_variable('range.right', 'uint16_t')
        logconf.add_variable('range.up', 'uint16_t')
        
        # Variables de actitud (Stabilizer)
        logconf.add_variable('stabilizer.roll', 'float')
        logconf.add_variable('stabilizer.pitch', 'float')
        logconf.add_variable('stabilizer.yaw', 'float')
        
        def on_log_data(timestamp, data, _cfg):
            # Extraer sensores
            f = data['range.front']
            a = data['range.back']
            i = data['range.left']
            d = data['range.right']
            up = data['range.up']
            
            # Extraer ángulos
            roll = data['stabilizer.roll']
            pitch = data['stabilizer.pitch']
            yaw = data['stabilizer.yaw']
            
            # Publicar a ROS 2
            nodo.publicar_sensores(f, a, i, d, up)
            nodo.publicar_angulos(roll, pitch, yaw)
            
            # Reflejo en pantalla
            sys.stdout.write(f"\r\033[K📡 Sensores: [F:{f} A:{a} I:{i} D:{d}] | 📐 R:{roll:.1f}° P:{pitch:.1f}° Y:{yaw:.1f}°")
            sys.stdout.flush()

        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(on_log_data)
        logconf.start()
        
        mc = MotionCommander(scf)
        volando = False
        
        try:
            while True:
                key = get_key_non_blocking(settings)
                
                # --- CONTROL DE VUELO BÁSICO ---
                if key == ' ':
                    if not volando:
                        print("\n\033[1;32m🚀 Despegando...\033[0m")
                        
                        volando = True
                    else:
                        print("\n\033[1;33m🛬 Aterrizando...\033[0m")
                        mc.land()
                        volando = False
                elif key.lower() == 'q':
                    scf.cf.commander.send_stop_setpoint()
                    nodo.mandar_orden("EMERGENCIA_DETENER_TODOS")
                    break
                
                # --- ORQUESTACIÓN DE OTROS NODOS ---
                elif key == '1':
                    nodo.mandar_orden("EJECUTAR_EVASION")
                elif key == '2':
                    nodo.mandar_orden("EJECUTAR_SEGUIMIENTO")
                elif key == '0':
                    nodo.mandar_orden("DETENER_TODOS")
                elif key == '\x03': # CTRL+C
                    if volando: mc.land()
                    break

                # Mantener vivo el nodo
                rclpy.spin_once(nodo, timeout_sec=0)
                
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            logconf.stop()
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()