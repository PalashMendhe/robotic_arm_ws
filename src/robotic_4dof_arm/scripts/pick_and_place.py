#!/usr/bin/env python3
import sys
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
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

# Ensure script directory is in path so we can import PlanningSceneManager
current_dir = os.path.dirname(os.path.realpath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from planning_scene_manager import PlanningSceneManager


class PickAndPlace(Node):
    def __init__(self):
        super().__init__(
            'pick_and_place_autonomous',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )

        # 1. Action client for MoveIt MoveGroup (arm motion planning)
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')

        # 2. Action client for Gripper controller
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'arm_base_joint',
            'upper_arm_joint',
            'forearm_joint',
            'wrist_joint',
            'gripper_baseTOwrist_joint',
            'gripper_base_joint',
        ]
        self.gripper_joint_names = [
            'right_prong_joint',
            'left_prong_joint',
        ]

        # 3. Initialize PlanningSceneManager
        self.scene = PlanningSceneManager(node=self)

        self.get_logger().info('Waiting for MoveGroup and Gripper action servers...')
        self._move_group_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('Action servers connected successfully!')

    def move_to_pose(self, x: float, y: float, z: float, qx: float = 0.0, qy: float = 0.7071, qz: float = 0.0, qw: float = 0.7071, link_name: str = 'gripper_tcp') -> bool:
        """
        Plans and executes a collision-free path to the target Cartesian pose using MoveIt 2.
        """
        self.get_logger().info(f'Planning collision-aware path to: ({x:.2f}, {y:.2f}, {z:.2f})')

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.request.start_state.is_diff = True


        # Workspace parameters
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.workspace_parameters.min_corner.x = -2.0
        goal.request.workspace_parameters.min_corner.y = -2.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 2.0
        goal.request.workspace_parameters.max_corner.y = 2.0
        goal.request.workspace_parameters.max_corner.z = 2.0

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
        pcm = PositionConstraint()
        pcm.header.frame_id = 'world'
        pcm.header.stamp = self.get_clock().now().to_msg()
        pcm.link_name = link_name
        bv = BoundingVolume()
        box_prim = SolidPrimitive()
        box_prim.type = SolidPrimitive.BOX
        box_prim.dimensions = [0.03, 0.03, 0.03]  # 3cm tolerance box
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

        # Send goal to MoveIt
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the motion goal.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:  # 1 = SUCCESS
            self.get_logger().info('MoveIt motion succeeded.')
            time.sleep(0.5)
            return True
        else:
            self.get_logger().error(f'MoveIt execution failed with error code: {result.error_code.val}')
            return False

    def move_to_joints(self, joint_positions: list) -> bool:
        """
        Plans and executes a collision-free motion to target joint positions using MoveIt 2.
        """
        self.get_logger().info(f'Planning collision-aware path to joints: {joint_positions}')

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
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

        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected joint goal.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('Joint trajectory execution succeeded.')
            time.sleep(0.5)
            return True
        else:
            self.get_logger().error(f'MoveIt joint move failed with code: {result.error_code.val}')
            return False

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

    def run(self):
        """
        Executes the complete collision-aware pick and place cycle.
        """
        self.get_logger().info('========================================================')
        self.get_logger().info('   Starting Autonomous Collision-Aware Pick and Place   ')
        self.get_logger().info('========================================================')

        def compute_ik(x, y, z):
            import math
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
            while theta4 > math.pi: theta4 -= 2*math.pi
            while theta4 < -math.pi: theta4 += 2*math.pi
            return [theta1, theta2, theta3, theta4, 0.0, 0.0]

        # 1. Initialize MoveIt Planning Scene (Table, Obstacle Divider, Target Box)
        self.get_logger().info('\n[1/11] Initializing Planning Scene Obstacles...')
        self.scene.init_full_scene()
        time.sleep(1.0)

        # 2. Return to Home pose (upright safe stance)
        self.get_logger().info('\n[2/11] Moving to Safe Home Pose...')
        self.move_to_joints([0.0, 1.57, -1.57, -1.57, 0.0, 0.0])

        # 3. Open Gripper
        self.get_logger().info('\n[3/11] Opening Gripper...')
        self.move_gripper([0.04, -0.04])

        # 4. Move to Pre-Pick Pose (15 cm above box at x=-0.60, y=0.20)
        self.get_logger().info('\n[4/11] Planning path to Pre-Pick Pose (above box)...')
        self.move_to_joints(compute_ik(-0.60, 0.20, 0.25))

        # 5. Descend to Grasp Pose (surrounding the target box)
        self.get_logger().info('\n[5/11] Descending to Grasp Height...')
        self.move_to_joints(compute_ik(-0.60, 0.20, 0.05))

        # 6. Close Gripper & Attach Box in MoveIt
        self.get_logger().info('\n[6/11] Closing Gripper & Attaching Box to MoveIt...')
        self.move_gripper([0.028, -0.028])
        self.scene.attach_target_box_to_gripper(link_name='gripper_base_link')
        time.sleep(0.5)

        # 7. Lift Object (Vertical Retreat)
        self.get_logger().info('\n[7/11] Lifting Object vertically...')
        self.move_to_joints(compute_ik(-0.60, 0.20, 0.35))

        # 8. Collision-Aware Transport around Obstacle to Destination Marker
        self.get_logger().info('\n[8/11] Planning Collision-Free Path AROUND Obstacle to Pre-Place...')
        self.move_to_joints(compute_ik(-0.45, -0.25, 0.30))

        # 9. Descend to Place Pose (onto green destination marker)
        self.get_logger().info('\n[9/11] Lowering to Place Pose...')
        self.move_to_joints(compute_ik(-0.45, -0.25, 0.10))

        # 10. Open Gripper & Detach Object in MoveIt
        self.get_logger().info('\n[10/11] Opening Gripper & Detaching Object...')
        self.move_gripper([0.04, -0.04])
        self.scene.detach_target_box_from_gripper(drop_position=(-0.45, -0.25, 0.05))
        time.sleep(0.5)

        # 11. Retreat Upwards & Return to Home
        self.get_logger().info('\n[11/11] Retreating Upwards & Returning Home...')
        self.move_to_joints(compute_ik(-0.45, -0.25, 0.30))
        self.move_to_joints([0.0, 1.57, -1.57, -1.57, 0.0, 0.0])

        self.get_logger().info('========================================================')
        self.get_logger().info('   Pick and Place Mission Completed Successfully!       ')
        self.get_logger().info('========================================================')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Pick and place interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()