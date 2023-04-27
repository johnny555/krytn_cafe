"""
Spawn the cafe world and Krytn
"""
# A bunch of software packages that are needed to launch ROS2
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("krytn_cafe")

    # Using a variant of the world with the actors removed.
    world_file_name = os.path.join(pkg_dir, "worlds", "cafe.world")

    pkg_dir = get_package_share_directory("krytn_cafe")

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, "models")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={'world': world_file_name}.items()
    )

    krytn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "spawn_krytn.launch.py")
        )
    )


    return LaunchDescription([gazebo, krytn])
