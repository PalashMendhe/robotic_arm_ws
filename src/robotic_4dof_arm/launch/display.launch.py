import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    urdf_path = os.path.join(
        get_package_share_directory('robotic_4dof_arm'),
        'urdf', 'arm.urdf.xacro'
    )

    # Expand xacro directly in Python — bypasses shell quoting issues entirely
    robot_description = xacro.process_file(urdf_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        ),
    ])