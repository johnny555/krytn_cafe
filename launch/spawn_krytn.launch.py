"""
This spawns krytn. Note it requires that gazebo be already started. 

"""
# A bunch of software packages that are needed to launch ROS2
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
 
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    pkg_dir = get_package_share_directory('krytn_cafe')
 
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
 
    xacro_file = os.path.join(pkg_dir,
                              'models','krytn',
                              'krytn.xacro.urdf')
                              
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    #This spawns a robot based on the robot_description urdf.  
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'krytn'],
                        output='screen')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_base_controller'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher, 
        spawn_entity,
        load_joint_state_controller,
        load_diff_drive_base_controller,  
    ])
