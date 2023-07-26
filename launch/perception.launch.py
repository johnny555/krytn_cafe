from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution

import os


def generate_launch_description():
    lidar = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser_link',
                'inverted': False,
                'angle_compensate': True,
            }],
        )

    realsense_params = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([get_package_share_directory("krytn_cafe"), 
            'config', 'perception.yaml']),
        description='Path to the perception parameters file.'
    )

    
    realsense = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory("realsense2_camera"),
             "launch/rs_launch.py"])),
     )

    return LaunchDescription([
        lidar,
        realsense
    ])
