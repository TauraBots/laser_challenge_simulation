import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configurações de Caminhos
    pkg_laser = get_package_share_directory('laser_challenge_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    ardupilot_path = os.path.expanduser('~/ardupilot/Tools/autotest/sim_vehicle.py')
    plugin_path = os.path.expanduser('~/ardupilot_gazebo/build')
    models_path = os.path.join(pkg_laser, 'models')


    # 2. Argumentos de Lançamento
    stage_arg = DeclareLaunchArgument(
        'stage', default_value='stage_one',
        description='Estágio: stage_one, stage_two, etc.'
    )
    stage = LaunchConfiguration('stage')
    world_file = PathJoinSubstitution([pkg_laser, 'worlds', [stage, '.world']])

    # 3. Variáveis de Ambiente (Correções de Performance e Versão)
    set_gz_version = SetEnvironmentVariable(name='GZ_VERSION', value='harmonic')
    set_render_engine = SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb')
    set_plugin_path = SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_path)
    set_gz_resource = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path)
    set_ign_resource = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=models_path)

    # 4. Ações de Execução
    # Inicia o Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Inicia o MAVProxy/SITL (Agora incluído no retorno!)
    mavproxy = ExecuteProcess(
        cmd=[
            'xterm', '-e', 
            ardupilot_path, '-v', 'ArduCopter', '-f', 'gazebo-iris', 
            '--no-rebuild', '-L', 'CBR',
            '--out', '127.0.0.1:14551'
        ],
        output='screen'
    )

    # Bridge de comunicação (Tópicos ROS <-> Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            # Relógio (Gazebo para ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Imagem (Gazebo para ROS)
            '/world/default/model/iris/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Serviço de Spawn (Mapeamento explícito para Harmonic)
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean'
        ],
        remappings=[
            ('/world/default/create', '/spawn_entity')
        ],
        output='screen'
    )

    # Spawner dos modelos da arena e drone
    base_spawner = Node(
        package='laser_challenge_simulation',
        executable='base_spawner.py',
        parameters=[{'challenge_stage': stage}],
        output='screen'
    )

    # Adicione este processo para spawnar o drone sem depender do bridge do ROS
    spawn_drone = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/default/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'sdf_filename: "{os.path.join(models_path, "iris", "model.sdf")}", name: "iris", pose: {{position: {{z: 1.0}}}}'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Variáveis de ambiente fundamentais
        SetEnvironmentVariable(name='GZ_VERSION', value='harmonic'),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_path),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),

        stage_arg,
        gazebo,
        mavproxy,    # <--- ESSENCIAL: Adicionado para o SITL rodar
        bridge,
        base_spawner,
        spawn_drone,
    ])