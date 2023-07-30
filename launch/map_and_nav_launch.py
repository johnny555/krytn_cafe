from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    # ... your existing nodes ...

    # SLAM Toolbox for mapping
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='slam_toolbox_node',
        parameters=[
            get_package_share_directory('krytn_cafe') + '/config/mapper_params.yaml'
        ],
        output='screen'
    )

    # Nav2 bringup for navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'use_sim_time': 'false',
            'params_file': get_package_share_directory('krytn_cafe') + '/config/nav2_params.yaml'
        }.items()
    )

    return LaunchDescription([
        slam_toolbox,
        navigation
    ])
