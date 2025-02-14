import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='laser_challenge_simulation').find('laser_challenge_simulation')
    world_gazebo_path = os.path.join(pkg_share, 'worlds/challenge_world.world')
    install_dir = FindPackageShare(package='laser_challenge_simulation').find('laser_challenge_simulation')
    world_models_path = os.path.join(pkg_share, 'models')

    os.environ['GAZEBO_MODEL_PATH'] = world_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))
    
    world_gazebo_arg = DeclareLaunchArgument(name="world", default_value=str(world_gazebo_path), description="starts world for simulation")
    
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
        output='screen'
    )

    return LaunchDescription([
        world_gazebo_arg ,
        gazebo_launch ,
    ])
