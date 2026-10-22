# Importamos herramientas del sistema operativo para poder "secuestrar" el teclado
import sys
import termios
import tty

# Importamos las librerías base de ROS 2
import rclpy
from rclpy.node import Node

# Importamos el cerebro de Bitcraze (cflib) para hablar por radio con el dron
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Esta es la "matrícula" única de tu dron. Si no es esta, no conectará.
URI = 'radio://0/80/2M/E7E7E7E7AA'

# Un texto bonito con colores (esos \033... son códigos de color para la terminal)
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

def get_key(settings):
    """
    Esta función es magia negra. Normalmente, la terminal espera a que pulses 'Enter' 
    para enviar lo que has escrito. Esto desactiva esa espera para leer tecla a tecla al instante.
    """
    # Ponemos la terminal en "modo crudo" (raw) para que no procese el texto
    tty.setraw(sys.stdin.fileno())
    # Leemos exactamente 1 carácter (tu pulsación)
    key = sys.stdin.read(1)
    
    # Las flechas del teclado envían 3 caracteres ocultos de golpe. Empiezan por '\x1b' (Escape).
    # Si detectamos que es una flecha, leemos los otros 2 caracteres que faltan para no romper nada.
    if key == '\x1b':
        key += sys.stdin.read(2)
        
    # Restauramos la terminal a su estado normal para devolverte el control
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    # Encendemos el motor de ROS 2
    rclpy.init(args=args)
    # Encendemos los drivers de la antena de radio USB
    cflib.crtp.init_drivers()
    
    # Guardamos cómo estaba configurada tu terminal antes de empezar para dejarla igual al salir
    settings = termios.tcgetattr(sys.stdin)
    
    print(f"\n\033[1;33mConectando a {URI}...\033[0m")
    
    # Abrimos la conexión con el dron. El 'with' hace que se desconecte solo si hay un error o acaba.
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print("\033[1;32m¡Enlace de radio establecido!\033[0m")
        print(MENU_HACKER)
        
        # Le damos los mandos al MotionCommander (el piloto automático interno)
        mc = MotionCommander(scf)
        # Una bandera (variable) para saber si estamos en el aire o en el suelo
        volando = False
        
        try:
            # Bucle infinito: el programa se queda dando vueltas aquí atrapado
            while True:
                # Nos quedamos esperando a que pulses una tecla en el teclado
                key = get_key(settings)
                
                # Si pulsas la barra espaciadora...
                if key == ' ':
                    if not volando:
                        print("\033[1;32m🚀 [ESPACIO] Despegando a 40 cm...\033[0m")
                        # OJO AQUÍ: Le has puesto velocity=1.0. ¡Va a subir pegando un salto muy rápido!
                        mc.take_off(height=0.4, velocity=1.0)
                        volando = True # Actualizamos el estado a "estamos en el aire"
                    else:
                        print("\033[1;33m🛬 [ESPACIO] Aterrizando suavemente...\033[0m")
                        mc.land()
                        volando = False # Volvemos a estar en el suelo
                        
                # Si pulsas 'w' (y estamos volando)...
                elif key.lower() == 'w' and volando:
                    print("⬆️  Avanzando 20 cm...")
                    mc.forward(0.2, velocity=0.2)
                    
                # Si pulsas 's' ...
                elif key.lower() == 's' and volando:
                    print("⬇️  Retrocediendo 20 cm...")
                    mc.back(0.2, velocity=0.2)
                    
                # Si pulsas 'a' ...
                elif key.lower() == 'a' and volando:
                    print("⬅️  Desplazamiento izquierdo 20 cm...")
                    mc.left(0.2, velocity=0.2)
                    
                # Si pulsas 'd' ...
                elif key.lower() == 'd' and volando:
                    print("➡️  Desplazamiento derecho 20 cm...")
                    mc.right(0.2, velocity=0.2)
                
                # --- NUEVOS CONTROLES CON FLECHAS ---
                
                # '\x1b[A' es el código oculto de la Flecha Arriba
                elif key == '\x1b[A' and volando:
                    print("🔼 Subiendo 10 cm...")
                    mc.up(0.1, velocity=0.2)
                    
                # '\x1b[B' es el código de la Flecha Abajo
                elif key == '\x1b[B' and volando:
                    print("🔽 Bajando 10 cm...")
                    mc.down(0.1, velocity=0.2)
                    
                # '\x1b[D' es la Flecha Izquierda
                elif key == '\x1b[D' and volando:
                    print("🔄 Rotando 45° a la izquierda...")
                    # Gira 45 grados a una velocidad de 45 grados por segundo
                    mc.turn_left(45, rate=45)
                    
                # '\x1b[C' es la Flecha Derecha
                elif key == '\x1b[C' and volando:
                    print("🔄 Rotando 45° a la derecha...")
                    mc.turn_right(45, rate=45)
                    
                # ------------------------------------
                    
                # Si pulsas la 'q' (botón del pánico)
                elif key.lower() == 'q':
                    print("\033[1;31m💀 [Q] ¡CORTANDO MOTORES! ¡CAÍDA LIBRE!\033[0m")
                    # Esto le manda un 0 absoluto a los motores, saltándose cualquier orden que estuviera haciendo
                    scf.cf.commander.send_stop_setpoint()
                    break # Rompe el bucle infinito y termina el programa
                    
                # '\x03' es el código interno de pulsar Control + C
                elif key == '\x03': 
                    print("\nSaliendo del panel de control...")
                    # Si estabas volando y cierras el programa a la fuerza, aterriza primero por seguridad
                    if volando:
                        mc.land()
                    break # Rompe el bucle
                    
        # Si ocurre un error grave (se corta el Wi-Fi, se apaga el dron, choca, etc.)
        except Exception as e:
            print(f"\033[1;31mError durante el vuelo: {e}\033[0m")
            
        # El bloque 'finally' se ejecuta SIEMPRE al acabar, pase lo que pase (incluso si hay error crítico)
        finally:
            # Restauramos tu terminal. Si no hacemos esto, tu Terminator se quedaría sin mostrar lo que escribes.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            # Apagamos el nodo de ROS 2
            rclpy.shutdown()

# Esto le dice a Python: "Si ejecutas este archivo directamente con ros2 run, arranca la función main()"
if __name__ == '__main__':
    main()