import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('robotic_4dof_arm'),
        'urdf', 'arm.urdf.xacro'
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('robotic_4dof_arm'),
        'config', 'controllers.yaml'
    )

    robot_description = xacro.process_file(urdf_path).toxml()
    world_file = PathJoinSubstitution([
        get_package_share_directory('robotic_4dof_arm'),
        'worlds', 'warehouse.sdf'
    ])

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'arm',
                '-topic', 'robot_description',
                '-x', '0.5', '-y', '0', '-z', '0.95'
            ],
            output='screen'
        ),
        # Bridge Gazebo clock to ROS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        ),
        # Spawn controllers after Gazebo has time to load
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['arm_controller', '--param-file', controllers_yaml],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['gripper_controller', '--param-file', controllers_yaml],
                    output='screen'
                ),
            ]
        ),
    ])
