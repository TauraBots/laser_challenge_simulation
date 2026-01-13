import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'laser_challenge_simulation'
    pkg_share = get_package_share_directory(pkg_name)
    
    # caminho do mundo (ex: stage_one.world)
    world_path = os.path.join(pkg_share, 'worlds', 'stage_two.world')
    
    # 1. inicia o Gazebo com o mundo q for escolhido
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    # 2. inicia o seu Spawner (Python)
    spawner = ExecuteProcess(
        cmd=['ros2', 'run', pkg_name, 'base_spawner.py', '--ros-args', '-p', 'challenge_stage:=stage_two'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawner
    ])