#!/usr/bin/env python3
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume,
)
import yaml
from ament_index_python.packages import get_package_share_directory

# Ensure script directory is in path so we can import PlanningSceneManager
current_dir = os.path.dirname(os.path.realpath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from planning_scene_manager import PlanningSceneManager


class PickAndPlace(Node):
    def __init__(self):
        params_path = os.path.join(get_package_share_directory('arm_moveit_config'), 'config', 'robot_params.yaml')
        with open(params_path, 'r') as f:
            self.params = yaml.safe_load(f)
        super().__init__(
            'pick_and_place_autonomous',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, self.params['robot']['use_sim_time'])
            ]
        )
        # 1. Action client for MoveIt MoveGroup (arm motion planning)
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')

        # 2. Action client for Gripper controller
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )

        self.joint_names = list(self.params['joints']['arm'].keys())
        self.gripper_joint_names = list(self.params['joints']['gripper'].keys())

        # 3. Initialize PlanningSceneManager
        self.scene = PlanningSceneManager(node=self)

        self.get_logger().info('Waiting for MoveGroup and Gripper action servers...')
        self._move_group_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('Action servers connected successfully!')

    def _compute_ik(self, x: float, y: float, z: float) -> list:
        """
        Analytical IK for this arm's specific geometry.
        Returns [theta1..theta6] joint angles for a given TCP (x, y, z).
        Arm parameters: L1=0.425, L2=0.392, d4=0.109, tcp_z_offset=0.170.
        """
        d4 = 0.109
        L1 = 0.425
        L2 = 0.392
        tcp_z_offset = 0.170
        R = math.sqrt(x**2 + y**2)
        theta1 = math.atan2(y, x) + math.asin(d4 / R)
        r_planar = math.sqrt(R**2 - d4**2)
        dr = r_planar
        dz = z + tcp_z_offset - 0.02
        D = math.sqrt(dr**2 + dz**2)
        beta = math.atan2(dz, dr)
        cos_gamma = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
        gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))
        theta2 = math.pi - (beta + gamma)
        cos_theta3 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta3 = math.acos(max(-1.0, min(1.0, cos_theta3)))
        theta4 = theta2 + theta3 + math.pi / 2.0
        while theta4 > math.pi:
            theta4 -= 2 * math.pi
        while theta4 < -math.pi:
            theta4 += 2 * math.pi
        return [theta1, theta2, theta3, theta4, 0.0, 0.0]

    def _is_within_workspace(self, x: float, y: float, z: float) -> bool:
        """
        Checks if the given Cartesian coordinates are within the defined workspace.
        """
        ws = self.params['workspace']
        return (
            ws['min_corner'][0] <= x <= ws['max_corner'][0] and
            ws['min_corner'][1] <= y <= ws['max_corner'][1] and
            ws['min_corner'][2] <= z <= ws['max_corner'][2]
        )

    def _execute_move_group_goal(self, goal: MoveGroup.Goal, label: str = '') -> bool:
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {label}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val == 1:  # 1 = SUCCESS
            self.get_logger().info(f'{label} motion succeeded.')
            time.sleep(0.5)
            return True
        else:
            self.get_logger().error(
                f'Motion failed [code {result.error_code.val}]: {label}'
            )
            return False

    def move_to_pose(self, x: float, y: float, z: float, qx: float = 0.0, qy: float = 0.7071, qz: float = 0.0, qw: float = 0.7071, link_name: str = 'gripper_tcp') -> bool:
        """
        Plans and executes a collision-free path to the target Cartesian pose using MoveIt 2.
        Validates workspace bounds before sending. Retries on failure.
        """
        if not self._is_within_workspace(x, y, z):
            self.get_logger().error(
                f'Target ({x:.2f}, {y:.2f}, {z:.2f}) is OUTSIDE workspace bounds. Refusing goal.'
            )
            return False
        self.get_logger().info(f'Planning collision-aware path to: ({x:.2f}, {y:.2f}, {z:.2f})')

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        m = self.params['motion']
        goal.request.num_planning_attempts = m['num_planning_attempts']
        goal.request.allowed_planning_time = m['allowed_planning_time']
        goal.request.max_velocity_scaling_factor = m['max_velocity_scaling_factor']
        goal.request.max_acceleration_scaling_factor = m['max_acceleration_scaling_factor']
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = m['replan_attempts']
        goal.request.start_state.is_diff = True


        # Workspace parameters
        ws = self.params['workspace']
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.workspace_parameters.min_corner.x = ws['min_corner'][0]
        goal.request.workspace_parameters.min_corner.y = ws['min_corner'][1]
        goal.request.workspace_parameters.min_corner.z = ws['min_corner'][2]
        goal.request.workspace_parameters.max_corner.x = ws['max_corner'][0]
        goal.request.workspace_parameters.max_corner.y = ws['max_corner'][1]
        goal.request.workspace_parameters.max_corner.z = ws['max_corner'][2]

        # Target Pose Constraints
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation.x = float(qx)
        target_pose.orientation.y = float(qy)
        target_pose.orientation.z = float(qz)
        target_pose.orientation.w = float(qw)

        # Position constraint
        tol = self.params['motion']['position_tolerance']
        pcm = PositionConstraint()
        pcm.header.frame_id = 'world'
        pcm.header.stamp = self.get_clock().now().to_msg()
        pcm.link_name = link_name
        bv = BoundingVolume()
        box_prim = SolidPrimitive()
        box_prim.type = SolidPrimitive.BOX
        box_prim.dimensions = [tol, tol, tol]  # tolerance box
        bv.primitives = [box_prim]
        bv.primitive_poses = [target_pose]
        pcm.constraint_region = bv
        pcm.weight = 1.0

        # Orientation constraint
        ocm = OrientationConstraint()
        ocm.header.frame_id = 'world'
        ocm.header.stamp = self.get_clock().now().to_msg()
        ocm.link_name = link_name
        ocm.orientation = target_pose.orientation
        ocm.absolute_x_axis_tolerance = 0.35
        ocm.absolute_y_axis_tolerance = 0.35
        ocm.absolute_z_axis_tolerance = 0.35
        ocm.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pcm]
        constraints.orientation_constraints = [ocm]
        goal.request.goal_constraints = [constraints]

        return self._execute_move_group_goal(goal, label=f'Cartesian ({x:.2f}, {y:.2f}, {z:.2f})')

    def move_to_joints(self, joint_positions: list) -> bool:
        """
        Plans and executes a collision-free motion to target joint positions using MoveIt 2.
        """
        self.get_logger().info(f'Planning collision-aware path to joints: {joint_positions}')

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        m = self.params['motion']
        goal.request.num_planning_attempts = m['num_planning_attempts']
        goal.request.allowed_planning_time = m['allowed_planning_time']
        goal.request.max_velocity_scaling_factor = m['max_velocity_scaling_factor']
        goal.request.max_acceleration_scaling_factor = m['max_acceleration_scaling_factor']
        goal.planning_options.plan_only = False
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        for name, pos in zip(self.joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints = [constraints]

        return self._execute_move_group_goal(goal, label='Joint-space')

    def move_gripper(self, positions: list, duration_sec: float = 1.5):
        """
        Commands the gripper controller to open or close prongs.
        """
        self.get_logger().info(f'Moving gripper to prongs: {positions}')
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
        time.sleep(0.5)

    def preflight_check(self) -> bool:
        self.get_logger().info('Running preflight checks...')
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('PREFLIGHT FAIL: MoveGroup server not available.')
            return False
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('PREFLIGHT FAIL: Gripper server not available.')
            return False
        from sensor_msgs.msg import JointState
        received = [False]
        def _cb(_msg): received[0] = True
        sub = self.create_subscription(JointState, '/joint_states', _cb, 1)
        deadline = time.time() + 3.0
        while not received[0] and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if not received[0]:
            self.get_logger().error('PREFLIGHT FAIL: /joint_states not publishing.')
            return False
        self.get_logger().info('Preflight checks passed ✓')
        return True

    def _abort(self, step: str):
        """
        Called when a mission step fails.
        Attempts a safe return to home, then raises RuntimeError to halt the mission.
        """
        self.get_logger().error(f'══ MISSION ABORT at step: [{step}] ══')
        self.get_logger().warning('Attempting emergency return to home pose...')
        home = list(self.params['joints']['home_position'].values())
        self.move_to_joints(home)
        raise RuntimeError(f'Mission aborted at: {step}')

    def run(self):
        """
        Executes the complete collision-aware pick and place cycle.
        """
        if not self.preflight_check():
            self.get_logger().error('Preflight checks failed. Aborting mission.')
            return
        self.get_logger().info('========================================================')
        self.get_logger().info('   Starting Autonomous Collision-Aware Pick and Place   ')
        self.get_logger().info('========================================================')

        home = list(self.params['joints']['home_position'].values())

        # 1. Initialize MoveIt Planning Scene (Table, Obstacle Divider, Target Box)
        self.get_logger().info('\n[1/11] Initializing Planning Scene Obstacles...')
        self.scene.init_full_scene()
        time.sleep(1.0)

        # 2. Return to Home pose (upright safe stance)
        self.get_logger().info('\n[2/11] Moving to Safe Home Pose...')
        if not self.move_to_joints(home):
            self._abort('Step 2: Home Pose')

        # 3. Open Gripper
        self.get_logger().info('\n[3/11] Opening Gripper...')
        self.move_gripper([0.04, -0.04])

        # 4. Move to Pre-Pick Pose (15 cm above box at x=-0.60, y=0.20)
        self.get_logger().info('\n[4/11] Planning path to Pre-Pick Pose (above box)...')
        if not self.move_to_joints(self._compute_ik(-0.60, 0.20, 0.25)):
            self._abort('Step 4: Pre-Pick Pose')

        # 5. Descend to Grasp Pose (surrounding the target box)
        self.get_logger().info('\n[5/11] Descending to Grasp Height...')
        if not self.move_to_joints(self._compute_ik(-0.60, 0.20, 0.05)):
            self._abort('Step 5: Grasp Height')

        # 6. Close Gripper & Attach Box in MoveIt
        self.get_logger().info('\n[6/11] Closing Gripper & Attaching Box to MoveIt...')
        self.move_gripper([0.028, -0.028])
        self.scene.attach_target_box_to_gripper(link_name='gripper_base_link')
        time.sleep(0.5)

        # 7. Lift Object (Vertical Retreat)
        self.get_logger().info('\n[7/11] Lifting Object vertically...')
        if not self.move_to_joints(self._compute_ik(-0.60, 0.20, 0.35)):
            self._abort('Step 7: Lift')

        # 8. Collision-Aware Transport around Obstacle to Destination Marker
        self.get_logger().info('\n[8/11] Planning Collision-Free Path AROUND Obstacle to Pre-Place...')
        if not self.move_to_joints(self._compute_ik(-0.45, -0.25, 0.30)):
            self._abort('Step 8: Transport')

        # 9. Descend to Place Pose (onto green destination marker)
        self.get_logger().info('\n[9/11] Lowering to Place Pose...')
        if not self.move_to_joints(self._compute_ik(-0.45, -0.25, 0.10)):
            self._abort('Step 9: Place Height')

        # 10. Open Gripper & Detach Object in MoveIt
        self.get_logger().info('\n[10/11] Opening Gripper & Detaching Object...')
        self.move_gripper([0.04, -0.04])
        self.scene.detach_target_box_from_gripper(drop_position=(-0.45, -0.25, 0.05))
        time.sleep(0.5)

        # 11. Retreat Upwards & Return to Home
        self.get_logger().info('\n[11/11] Retreating Upwards & Returning Home...')
        if not self.move_to_joints(self._compute_ik(-0.45, -0.25, 0.30)):
            self._abort('Step 11: Retreat')
        if not self.move_to_joints(home):
            self._abort('Step 11: Return Home')

        self.get_logger().info('========================================================')
        self.get_logger().info('   Pick and Place Mission Completed Successfully! ✓     ')
        self.get_logger().info('========================================================')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        node.run()
    except RuntimeError as e:
        node.get_logger().error(f'Mission failed: {e}')
    except KeyboardInterrupt:
        node.get_logger().info('Pick and place interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()