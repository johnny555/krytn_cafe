from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from os.path import join


def generate_launch_description():
    krytn_config = join(get_package_share_directory("krytn_cafe"), "config")
    krytn_launch = join(get_package_share_directory("krytn_cafe"), "launch")

    # SLAM Toolbox for mapping
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[join(krytn_config, "slam_toolbox_sync.yaml")],
        output="screen",
    )

    # Nav2 bringup for navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(krytn_launch, "navigation_launch.py")]),
        launch_arguments={
            "map_subscribe_transient_local": "true",
            "use_sim_time": "false",
            "params_file": join(krytn_config, "nav2_params.yaml"),
        }.items(),
    )

    return LaunchDescription([slam_toolbox, navigation])
