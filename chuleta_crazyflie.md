# 🚁 CHULETA DE VUELO: CRAZYFLIE + ROS 2

## 1. 🌐 ENTORNO Y RED (Crazyswarm2)
> Antes de volar o simular, siempre debes cargar el workspace en cada terminal[cite: 1].
**`source install/setup.bash`** (o `local_setup.bash`[cite: 1])

> **Aislar la red en el laboratorio:** Para evitar interferencias o mover drones de otros grupos, configura tu terminal[cite: 1]:
**`export ROS_LOCALHOST_ONLY=1`**[cite: 1]
**`export ROS_DOMAIN_ID=42`** *(Usa un número único entre 0 y 101)*[cite: 1]

## 2. 🚀 COMANDOS DE CONSOLA
> **Vuelo Real:**
1. Enciende el servidor de radio: **`ros2 launch crazyflie launch.py`**[cite: 1]
2. En otra terminal, lanza tu script: **`ros2 run crazyflie_control nodo_piloto`**[cite: 1]

> **Simulación (Sin dron físico):**
Opción rápida (todo a la vez):
**`ros2 launch crazyflie_examples launch.py script:=nodo_piloto backend:=sim`**[cite: 1]
Opción manual (2 terminales):
1. **`ros2 launch crazyflie launch.py backend:=sim`**[cite: 1]
2. **`ros2 run crazyflie_control nodo_piloto --ros-args -p use_sim_time:=True`**[cite: 1]

## 3. ⚙️ ARCHIVO DE CONFIGURACIÓN (crazyflies.yaml)
*Todo se configura en `crazyflie/config/crazyflies.yaml`*[cite: 1].
- **Matrícula:** Cambia la línea `uri: radio://0/80/2M/E7E7E7E7E7` por la tuya[cite: 1].
- **Activar/Desactivar:** Cambia `enabled: true` a `false` si no quieres que el servidor conecte con ese dron[cite: 1].
- **Telemetría:** En la sección `firmware_logging`, puedes activar topics por defecto (`pose`, `scan`, `odom`, `status`, `imu`) o crear custom topics para leer sensores[cite: 1].

## 4. 🐍 COMANDOS DE VUELO EN PYTHON (MotionCommander)
*Estos comandos se usan dentro del bloque `with MotionCommander(scf) as mc:`*

**Desplazamiento Relativo (Distancias en metros, velocidad en m/s):**
> `mc.up(0.5, velocity=0.2)` : Sube medio metro.
> `mc.down(0.5, velocity=0.2)` : Baja medio metro.
> `mc.forward(1.0, velocity=0.3)` : Avanza 1 metro.
> `mc.back(1.0)` : Retrocede (velocidad por defecto).
> `mc.left(0.5)` / `mc.right(0.5)` : Desplazamiento lateral.

**Rotación (Ángulos en grados, velocidad en grados/s):**
> `mc.turn_left(90, rate=45)` : Gira sobre sí mismo a la izquierda.
> `mc.turn_right(180, rate=45)` : Gira sobre sí mismo a la derecha.

**Parada de Emergencia:**
> `mc.stop()` : Corta el movimiento actual instantáneamente y se queda flotando (hover).