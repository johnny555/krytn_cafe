import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

import subprocess

def generate_launch_description():

    joy = Node(
    package='joy',
    executable='joy_node',
    parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1, 'autorepeat_rate': 20}],
    output='screen'
    )


    nodes = [
        joy 
    ]

    return LaunchConfiguration(nodes)