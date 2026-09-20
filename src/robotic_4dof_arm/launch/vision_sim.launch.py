#!/usr/bin/env python3
"""
vision_sim.launch.py
--------------------
Launches the vision sorting simulation:
  - Gazebo with vision_sorting.sdf (two tables + D415 camera)
  - Robot state publisher + arm URDF
  - ros2_control controllers
  - gz_ros_bridge (camera topics + clock + joint states)
  - camera_tf_broadcaster (world -> d415_color_optical_frame)
  - Random object spawner (3 red cubes + 3 blue cylinders on Table 2)
  - MoveIt move_group
"""

import os
import math
import random
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import PathJoinSubstitution


# ── Object spawn zone (Table 2 surface, INSIDE the arm's reach envelope) ────
# Table 2 spans x: -0.60 to -1.20, y: -0.40 to +0.40
#
# REACH CONSTRAINT: the analytical IK (_compute_ik) requires
#   sqrt(dr² + dz²) ≤ L1 + L2 = 0.817 m with dz = z + 0.15.
# At grasp height (world z = 1.00 → arm z = 0.05) that gives a maximum
# base radius of R ≈ 0.80 m; use 0.78 m to keep a 2 cm planning margin.
# The spawn band is therefore x ∈ [-0.63, -sqrt(R² - y²)] — objects that
# used to spawn near the far corner (e.g. -0.78, ±0.38 → R = 0.87 m) were
# unreachable and made the arm drive to a clamped, fully-stretched pose.
X_SPAWN_INNER = -0.63   # inner edge of the spawn band (Table 1 ends at -0.50)
R_PICK_MAX    =  0.78   # arm pick radius incl. safety margin (see above)
SPAWN_Y_MIN   = -0.38
SPAWN_Y_MAX   =  0.38
SPAWN_Z       =  1.00   # table surface (0.95) + half object height (0.05)
MIN_SEP       =  0.12   # 12cm centre-to-centre (10cm object + 2cm gap)
N_CUBES       =  2      # sort_zones grid capacity per class
N_CYLINDERS   =  2
MAX_SPAWN_ATTEMPTS = 10000


def _reach_constrained_x_max(y: float) -> float:
    """Outer (most-negative) spawn x allowed at height y by the reach envelope."""
    return -math.sqrt(max(R_PICK_MAX**2 - y * y, 0.0))


def _fallback_positions(n_total: int) -> list:
    """
    Deterministic layout used if random sampling cannot fit all objects:
    a single line inside the reach band with exactly MIN_SEP spacing.
    """
    if n_total > 4:
        raise RuntimeError(
            f'Reach-constrained spawn zone cannot fit {n_total} objects. '
            f'Max is 4 (2 per class).'
        )
    span = 0.60 / max(n_total - 1, 1)
    return [(-0.68, SPAWN_Y_MIN + 0.08 + i * span) for i in range(n_total)]


def _generate_positions(n_total: int) -> list:
    """Generate n_total non-overlapping (x, y) positions on Table 2.

    Positions are sampled inside the arm's reach-constrained band so every
    spawned object is pickable, and separated by at least MIN_SEP.
    """
    positions = []
    for _ in range(MAX_SPAWN_ATTEMPTS):
        if len(positions) >= n_total:
            return positions
        y = random.uniform(SPAWN_Y_MIN, SPAWN_Y_MAX)
        x_outer = _reach_constrained_x_max(y)
        if x_outer >= X_SPAWN_INNER:
            continue  # no valid band at this y (cannot happen inside Y range)
        x = random.uniform(x_outer, X_SPAWN_INNER)
        too_close = any(
            math.hypot(x - px, y - py) < MIN_SEP
            for px, py in positions
        )
        if not too_close:
            positions.append((x, y))

    # Random sampling exhausted — fall back to a guaranteed-valid layout.
    print(
        f'[vision_sim] Random spawn sampling failed after '
        f'{MAX_SPAWN_ATTEMPTS} attempts; using deterministic fallback layout.'
    )
    return _fallback_positions(n_total)


def _make_cube_sdf(name: str) -> str:
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00008333</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00008333</iyy><iyz>0</iyz>
          <izz>0.00008333</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <material>
          <ambient>0.9 0.1 0.1 1</ambient>
          <diffuse>0.9 0.1 0.1 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def _make_cylinder_sdf(name: str) -> str:
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00010417</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00010417</iyy><iyz>0</iyz>
          <izz>0.00006250</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <cylinder><radius>0.05</radius><length>0.10</length></cylinder>
        </geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder><radius>0.05</radius><length>0.10</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.9 1</ambient>
          <diffuse>0.1 0.1 0.9 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def generate_spawn_actions(context):
    """OpaqueFunction: called at launch time to generate random spawn Nodes."""
    positions = _generate_positions(N_CUBES + N_CYLINDERS)
    cube_positions = positions[:N_CUBES]
    cylinder_positions = positions[N_CUBES:]

    spawn_nodes = []
    for i, (x, y) in enumerate(cube_positions):
        name = f'red_cube_{i}'
        spawn_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'vision_sorting',
                    '-name', name,
                    '-string', _make_cube_sdf(name),
                    '-x', f'{x:.4f}',
                    '-y', f'{y:.4f}',
                    '-z', f'{SPAWN_Z:.3f}',
                ],
                output='screen',
            )
        )

    for i, (x, y) in enumerate(cylinder_positions):
        name = f'blue_cylinder_{i}'
        spawn_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'vision_sorting',
                    '-name', name,
                    '-string', _make_cylinder_sdf(name),
                    '-x', f'{x:.4f}',
                    '-y', f'{y:.4f}',
                    '-z', f'{SPAWN_Z:.3f}',
                ],
                output='screen',
            )
        )

    # Spawn all objects after Gazebo has 5 seconds to start
    return [TimerAction(period=5.0, actions=spawn_nodes)]


def generate_launch_description():
    pkg_robot = get_package_share_directory('robotic_4dof_arm')
    pkg_moveit = get_package_share_directory('arm_moveit_config')

    urdf_path = os.path.join(pkg_robot, 'urdf', 'arm.urdf.xacro')
    srdf_path = os.path.join(pkg_moveit, 'config', 'arm.srdf')
    kinematics_yaml = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    ompl_yaml = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    moveit_controllers_yaml = os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml')
    joint_limits_yaml = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
    controllers_yaml = os.path.join(pkg_robot, 'config', 'controllers.yaml')

    robot_description = xacro.process_file(urdf_path).toxml()
    robot_description_semantic = open(srdf_path).read()

    with open(kinematics_yaml, 'r') as f:
        kinematics = yaml.safe_load(f)
    with open(ompl_yaml, 'r') as f:
        ompl_config = yaml.safe_load(f)
    with open(moveit_controllers_yaml, 'r') as f:
        moveit_controllers = yaml.safe_load(f)
    with open(joint_limits_yaml, 'r') as f:
        joint_limits = yaml.safe_load(f)

    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_config,
    }

    world_file = PathJoinSubstitution([pkg_robot, 'worlds', 'vision_sorting.sdf'])

    return LaunchDescription([

        # ── 1. Gazebo with vision_sorting world ───────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # ── 2. Robot State Publisher ──────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }]
        ),

        # ── 3. Spawn arm in Gazebo ────────────────────────────────────────
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'arm',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.95',
            ],
            output='screen'
        ),

        # ── 4. gz_ros_bridge ─────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Clock
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                # Joint states (Gazebo → ROS)
                '/world/vision_sorting/model/arm/joint_state'
                '@sensor_msgs/msg/JointState@gz.msgs.Model',
                # D415 camera topics (Gazebo → ROS)
                '/d415/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/d415/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/d415/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/d415/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                # Jaw contact sensors (Gazebo -> ROS, idea 3a)
                '/world/vision_sorting/model/arm/link/right_prong_link/sensor/right_prong_contact/contact'
                '@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
                '/world/vision_sorting/model/arm/link/left_prong_link/sensor/left_prong_contact/contact'
                '@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
            ],
            remappings=[
                (
                    '/world/vision_sorting/model/arm/joint_state',
                    '/joint_states'
                ),
                (
                    '/world/vision_sorting/model/arm/link/right_prong_link/sensor/right_prong_contact/contact',
                    '/gripper/right_contact'
                ),
                (
                    '/world/vision_sorting/model/arm/link/left_prong_link/sensor/left_prong_contact/contact',
                    '/gripper/left_contact'
                ),
            ],
            output='screen'
        ),

        # ── 5. Camera TF broadcaster (world → d415_color_optical_frame) ──
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robotic_4dof_arm',
                    executable='camera_tf_broadcaster.py',
                    output='screen',
                ),
            ]
        ),

        # ── 6. Controllers ────────────────────────────────────────────────
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster',
                               '--param-file', controllers_yaml],
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
                    arguments=['arm_controller',
                               '--param-file', controllers_yaml],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['gripper_controller',
                               '--param-file', controllers_yaml],
                    output='screen'
                ),
            ]
        ),

        # ── 7. MoveGroup Node ─────────────────────────────────────────────
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'robot_description_kinematics': kinematics},
                {'robot_description_planning': joint_limits},
                planning_pipelines,
                moveit_controllers,
                {'use_sim_time': True},
            ],
            output='screen'
        ),

        # ── 8. Random object spawner ──────────────────────────────────────
        OpaqueFunction(function=generate_spawn_actions),

    ])
