from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    krytn_launch_dir = join(get_package_share_directory("krytn_cafe"), "launch")

    perception = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [join(krytn_launch_dir, "perception_launch.py")]
            )
        ),
    )

    map_and_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(krytn_launch_dir, "map_and_nav_launch.py")])
    )

    return LaunchDescription([perception, map_and_nav])
