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

def generate_robot_description(pkg_dir):
    cmd = ["xacro", os.path.join(pkg_dir, "models/krytn/", "krytn_real.xacro.urdf")]
    process = subprocess.run(cmd, stdout=subprocess.PIPE, text=True)
    return process.stdout

def generate_launch_description():
    pkg_dir = get_package_share_directory("krytn_cafe")

    robot_description = {"robot_description": generate_robot_description(pkg_dir)}

    robot_controllers = {"controller_configuration": os.path.join(pkg_dir, "config", "real_world_diff_drive.yaml")}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("krytn_cafe"),
            "config",
            "real_world_diff_drive.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    joy = Node(
    package='joy',
    executable='joy_node',
    parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1, 'autorepeat_rate': 10.0}],
    output='screen'
    )

    joy_teleop = Node(
    package='joy_teleop',
    executable='joy_teleop',
    name='joy_teleop',
    parameters=[PathJoinSubstitution([get_package_share_directory("krytn_cafe"), 
            'config', 'real_world_diff_drive.yaml'])],
    output='screen'
)


    nodes = [
        joy,
        joy_teleop,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)
