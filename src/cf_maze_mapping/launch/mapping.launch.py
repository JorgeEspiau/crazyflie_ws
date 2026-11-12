import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Directorio de nuestro paquete
    mapping_dir = get_package_share_directory('cf_maze_mapping')

    # Configuracion de SLAM Toolbox
    slam_config = os.path.join(
        mapping_dir,
        'config',
        'slam_toolbox.yaml'
    )

    # Directorio oficial de SLAM Toolbox
    slam_dir = get_package_share_directory('slam_toolbox')

    # Lanzamiento oficial con gestion del ciclo de vida
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_dir,
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'false',
        }.items()
    )

    return LaunchDescription([
        slam_launch
    ])