import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Package paths
    pkg_moveit = get_package_share_directory('arm_moveit_config')
    pkg_robot = get_package_share_directory('robotic_4dof_arm')

    urdf_path = os.path.join(pkg_robot, 'urdf', 'arm.urdf.xacro')
    srdf_path = os.path.join(pkg_moveit, 'config', 'arm.srdf')
    kinematics_yaml = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    ompl_yaml = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    controllers_yaml = os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml')
    joint_limits_yaml = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
    rviz_config_path = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    gazebo_launch_path = os.path.join(pkg_robot, 'launch', 'gazebo.launch.py')

    # Load descriptions and configurations
    robot_description = xacro.process_file(urdf_path).toxml()
    robot_description_semantic = open(srdf_path).read()

    with open(kinematics_yaml, 'r') as f:
        kinematics = yaml.safe_load(f)

    with open(ompl_yaml, 'r') as f:
        ompl_config = yaml.safe_load(f)

    with open(controllers_yaml, 'r') as f:
        controllers = yaml.safe_load(f)

    with open(joint_limits_yaml, 'r') as f:
        joint_limits = yaml.safe_load(f)

    # Planning pipeline dictionary
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_config,
    }

    # Launch Configurations
    use_rviz = LaunchConfiguration('use_rviz')
    start_gazebo = LaunchConfiguration('start_gazebo')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'start_gazebo',
            default_value='true',
            description='Whether to start Gazebo simulation with controllers'
        ),

        # Include Gazebo simulation bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(start_gazebo)
        ),

        # Standalone robot_state_publisher (only if NOT running Gazebo bringup)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            condition=UnlessCondition(start_gazebo)
        ),

        # MoveGroup Node (Motion Planning Core)
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'robot_description_kinematics': kinematics},
                {'robot_description_planning': joint_limits},
                planning_pipelines,
                controllers,
                {'use_sim_time': True},
            ],
            output='screen'
        ),

        # Optional RViz2 with MoveIt MotionPlanning plugin
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'robot_description_kinematics': kinematics},
                planning_pipelines,
                {'use_sim_time': True},
            ],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])