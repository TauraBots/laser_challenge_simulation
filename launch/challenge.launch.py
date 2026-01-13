import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definição de caminhos do pacote
    pkg_laser = get_package_share_directory('laser_challenge_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. Argumento para escolher o estágio (Default: stage_one)
    stage_arg = DeclareLaunchArgument(
        'stage',
        default_value='stage_one',
        description='Estágio do desafio: stage_one, stage_two, stage_three ou stage_four'
    )

    # Configuração dinâmica do ficheiro de mundo
    stage = LaunchConfiguration('stage')
    world_file = PathJoinSubstitution([
        pkg_laser, 'worlds', [stage, '.world']
    ])

    # 3. AUTOMAÇÃO DO EXPORT (O Pulo do Gato!)
    # Isto substitui o "export GAZEBO_MODEL_PATH" manual no terminal
    models_path = os.path.join(pkg_laser, 'models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', models_path]
    )

    # 4. Lançamento do Gazebo com o mundo selecionado
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 5. Início do nó do Spawner (Python)
    base_spawner = Node(
        package='laser_challenge_simulation',
        executable='base_spawner.py',
        name='base_spawner',
        parameters=[{'challenge_stage': stage}],
        output='screen'
    )

    return LaunchDescription([
        stage_arg,
        set_model_path,
        gazebo,
        base_spawner
    ])