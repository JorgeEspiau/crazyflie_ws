cat > src/crazyflie_sensors/launch/tres_nodos.launch.py <<'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # P1: conexion y control del Crazyflie
    nodo_p1 = Node(
        package='crazyflie_flight_control',
        executable='flight_control',
        name='flight_control_node',
        output='screen',
    )

    # P2: nodo maestro y distribucion de sensores
    nodo_p2 = Node(
        package='crazyflie_sensors',
        executable='nodo_sensores',
        name='nodo_maestro',
        output='screen',
        emulate_tty=True,
    )

    # P3: exploracion con regla de la mano derecha
    nodo_p3 = Node(
        package='crazyflie_maze',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
    )

    return LaunchDescription([
        nodo_p1,
        nodo_p2,
        nodo_p3,
    ])
