import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'laser_challenge_simulation'
    pkg_project_simulation = get_package_share_directory(package_name)
    
    # Caminhos das entidades e configurações usando o diretório de instalação
    world_file = os.path.join(pkg_project_simulation, 'worlds', 'stage_one.world')
    models_path = os.path.join(pkg_project_simulation, 'models')
    iris_sdf = os.path.join(models_path, 'iris', 'model.sdf')
    mavros_config = os.path.join(pkg_project_simulation, 'config', 'taura_mavros_params.yaml')

    # Variáveis de ambiente para o Gazebo encontrar os recursos
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=[models_path, ':', os.path.join(pkg_project_simulation, 'worlds')]
    )

    # 1. Abrir o Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    # 2. Nó do MAVROS (Configurado com Namespace e Parâmetros)
   
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros', 
        output='screen',
        parameters=[mavros_config],
    )

    # 3. Bridge entre Gazebo e ROS 2 (Clock e Câmera)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/model/iris/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/world/default/model/iris/link/base_link/sensor/camera/image', '/camera')
        ],
        output='screen'
    )

    # spawn do drone usando timer action (5 segundos de delay)
    spawn_iris = TimerAction(
    period=5.0,
    actions=[
        ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/default/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf_filename: "{iris_sdf}", name: "iris", pose: {{position: {{z: 1}}}}'
            ],
            output='screen',
        )
    ]
)

    return LaunchDescription([
        set_resource_path,
        SetEnvironmentVariable(name='GZ_VERSION', value='harmonic'),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        gazebo,
        mavros_node,
        bridge,
        spawn_iris 
    ])