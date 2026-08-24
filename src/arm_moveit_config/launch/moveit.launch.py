import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('robotic_4dof_arm'),
        'urdf', 'arm.urdf.xacro'
    )

    srdf_path = os.path.join(
        get_package_share_directory('arm_moveit_config'),
        'config', 'arm.srdf'
    )

    kinematics_yaml = os.path.join(
        get_package_share_directory('arm_moveit_config'),
        'config', 'kinematics.yaml'
    )

    ompl_yaml = os.path.join(
        get_package_share_directory('arm_moveit_config'),
        'config', 'ompl_planning.yaml'
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('arm_moveit_config'),
        'config', 'moveit_controllers.yaml'
    )

    robot_description = xacro.process_file(urdf_path).toxml()
    robot_description_semantic = open(srdf_path).read()

    with open(kinematics_yaml, 'r') as f:
        kinematics = yaml.safe_load(f)

    with open(ompl_yaml, 'r') as f:
        ompl_config = yaml.safe_load(f)

    with open(controllers_yaml, 'r') as f:
        controllers = yaml.safe_load(f)

    # Structure exactly as moveit_configs_builder.py does (ROS2 Lyrical MoveIt2):
    # planning_pipelines dict has the pipeline name as a key containing the pipeline's yaml.
    # See: moveit_configs_utils/moveit_configs_builder.py lines 463-471
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_config,   # ompl_config contains: planning_plugins, request_adapters, response_adapters
    }

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'robot_description_kinematics': kinematics},
                planning_pipelines,
                controllers,
                {'use_sim_time': False},
            ],
            output='screen'
        ),
    ])