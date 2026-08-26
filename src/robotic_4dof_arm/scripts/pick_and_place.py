#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
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

    def move_arm(self, positions, duration_sec=3.0):
        self.get_logger().info(f'Moving arm to: {positions}')
        self._arm_client.wait_for_server()
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec))
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(1.0)

    def move_gripper(self, positions, duration_sec=1.5):
        self.get_logger().info(f'Moving gripper to: {positions}')
        self._gripper_client.wait_for_server()

        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec))
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(0.5)

def main():
    rclpy.init()
    node = PickAndPlace()

    # Home
    node.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Open gripper
    node.move_gripper([0.04, -0.04])

    # Pick pose — arm reaching down
    node.move_arm([0.0, -1.0, 1.5, 0.0, 0.0, 0.0])

    # Close gripper
    node.move_gripper([0.0, 0.0])

    # Place pose — rotate base, same reach
    node.move_arm([1.5, -1.0, 1.5, 0.0, 0.0, 0.0])

    # Open gripper
    node.move_gripper([0.04, -0.04])

    # Return home
    node.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    node.get_logger().info('Pick and place complete.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()