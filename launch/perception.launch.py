from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rplidar_ros"),
            'launch/rplidar.launch.py'))
     )

    #realsense = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #        os.path.join(get_package_share_directory("realsense2_camera"), "launch/rs_launch.py")),
    #    launch_arguments={
    #        'config_file':os.path.join(get_package_share_directory("krytn_cafe","config"), 'perception.yaml')
    ##    }
    # )

    return LaunchDescription([
        lidar
    ])
