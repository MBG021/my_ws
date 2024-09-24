from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta del archivo de configuración YAML
    config_path = os.path.join(
        get_package_share_directory('pybullet_sim'),
        'config',
        'controller_config.yaml'
    )

    return LaunchDescription([
        #Node(
        #    package='pybullet_sim',
        #    executable='controller_node',
        #    output='screen',
        #    parameters=[config_path],
        #),
         Node(
            package='pybullet_sim',  # Reemplaza con el nombre de tu paquete
            executable='pybullet_node',  # Nombre del archivo sin extensión (x.py)
            output='screen',
            parameters=[{
                # Aquí puedes agregar parámetros si es necesario
            }],
        ),

    ])
