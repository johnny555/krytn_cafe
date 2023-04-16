"""
Spawn the cafe world and Krytn
"""
# A bunch of software packages that are needed to launch ROS2
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    pkg_dir = get_package_share_directory('krytn_cafe')

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')

    empty_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    krytn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'diffbot.launch.py')))

    return LaunchDescription([
        empty_gazebo,
        krytn
    ])
