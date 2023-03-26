from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='krytn_cafe',
            namespace='krytn_cafe',
            executable='my_node',
            name='my_node'
        )
    ])