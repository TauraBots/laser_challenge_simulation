import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Caminho para o seu arquivo YAML (ajuste se necessário)
    config = os.path.join(
        os.getcwd(),
        'taura_mavros_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[config],
        )
    ])