import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_project_simulation = get_package_share_directory('laser_challenge_simulation')
    
    # Caminhos
    world_file = os.path.join(pkg_project_simulation, 'worlds', 'stage_one.world')
    models_path = os.path.join(pkg_project_simulation, 'models')
    iris_sdf = os.path.join(models_path, 'iris', 'model.sdf')

    # 1. Configurar variáveis de ambiente
    # No Harmonic, precisamos que o Gazebo enxergue tanto os modelos quanto o mundo
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=[models_path, ':', os.path.join(pkg_project_simulation, 'worlds')]
    )

    # 2. Abrir o Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file], # Adicionado -r para rodar o tempo de cara
        output='screen',
    )

    # 3. Spawnar o Iris com ATRASO (O Pulo do Gato)
    # Esperamos 5 segundos para garantir que o mundo carregou
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

    # 4. Bridge (Sua configuração estava ótima, mantive os tópicos)
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

    return LaunchDescription([
        set_resource_path,
        SetEnvironmentVariable(name='GZ_VERSION', value='harmonic'),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        gazebo,
        spawn_iris,
        bridge
    ])