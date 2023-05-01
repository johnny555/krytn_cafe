from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    pkg_dir = get_package_share_directory("krytn_cafe")

    rtabmap_params = DeclareLaunchArgument(
        'rtabmap_params',
        default_value=join(pkg_dir, 'config', 'rtabmap.yaml'),
        description='Path to the RTAB-Map parameters file.'
    )

    rtabmap_params_launch_config = LaunchConfiguration('rtabmap_params')

    return LaunchDescription([
        rtabmap_params,
        Node(
            package="rtabmap_ros",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[rtabmap_params_launch_config],
            remappings=[
                ("rgb/image", "camera1/image_raw"),
                ("depth/image", "camera1/depth/image_raw"),
                ("rgb/camera_info", "camera1/camera_info")
            ]
        )
    ])