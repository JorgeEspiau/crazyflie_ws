import time
import rclpy
from rclpy.node import Node

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

# Tu matrícula exacta fichada en el laboratorio
URI = 'radio://0/80/2M/E7E7E7E7AA'


class NodoPiloto(Node):
    def __init__(self):
        super().__init__('nodo_piloto')
        cflib.crtp.init_drivers()
        self.get_logger().info(f'Conectando al Crazyflie en {URI} para telemetría...')

        with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
            self.get_logger().info('¡Conectado! Configurando canales de sensores...')

            # Configuramos el paquete de log para leer cada 200 ms (5 veces por segundo)
            log_conf = LogConfig(name='SensoresVuelo', period_in_ms=200)
            log_conf.add_variable('kalman.stateZ', 'float')   # Altura estimada (metros)
            log_conf.add_variable('pm.vbat', 'float')         # Voltaje de la batería
            log_conf.add_variable('stabilizer.roll', 'float') # Inclinación lateral
            log_conf.add_variable('stabilizer.pitch', 'float')# Inclinación frontal
            log_conf.add_variable('stabilizer.yaw', 'float')  # Giro de orientación

            scf.cf.log.add_config(log_conf)
            log_conf.data_received_cb.add_callback(self.sensor_callback)

            # Arrancamos el flujo de datos por radio
            log_conf.start()
            self.get_logger().info('Iniciando secuencia de despegue suave y lectura...')

            with MotionCommander(scf, default_height=0.2) as mc:
                time.sleep(1.0)
                
                self.get_logger().info('--- SUBIENDO DESPACITO (0.1 m/s) ---')
                mc.up(0.3, velocity=0.1)
                time.sleep(2.0)
                
                self.get_logger().info('--- BAJANDO DESPACITO (0.1 m/s) ---')
                mc.down(0.3, velocity=0.1)
                time.sleep(1.5)

            # Detenemos los sensores al terminar el vuelo
            log_conf.stop()
            self.get_logger().info('¡Vuelo y telemetría completados con éxito!')

    def sensor_callback(self, timestamp, data, logconf):
        # Capturamos los valores que van llegando por radio
        altura = data.get('kalman.stateZ', 0.0)
        vbat = data.get('pm.vbat', 0.0)
        roll = data.get('stabilizer.roll', 0.0)
        pitch = data.get('stabilizer.pitch', 0.0)
        yaw = data.get('stabilizer.yaw', 0.0)

        # Imprimimos en la terminal con formato limpio y colores fucsias/verdes a juego con tu setup
        print(
            f"\033[1;35m[TELEMETRÍA]\033[0m "
            f"Alt: \033[1;32m{altura:.3f} m\033[0m | "
            f"Bat: \033[1;33m{vbat:.2f} V\033[0m | "
            f"Roll: {roll:6.2f}° | "
            f"Pitch: {pitch:6.2f}° | "
            f"Yaw: {yaw:6.2f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoPiloto()
    rclpy.shutdown()


if __name__ == '__main__':
    main()