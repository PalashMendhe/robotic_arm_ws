#!/usr/bin/env python3
"""
camera_tf_broadcaster.py
------------------------
Publishes a single static TF: world → d415_color_optical_frame.

Reads camera pose from camera_params.yaml so the transform can be adjusted
without modifying this script.
"""

import os
import math
import yaml

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert RPY (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class CameraTfBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tf_broadcaster')

        params_path = os.path.join(
            get_package_share_directory('arm_moveit_config'),
            'config', 'camera_params.yaml'
        )
        with open(params_path, 'r') as f:
            cfg = yaml.safe_load(f)['camera']

        xyz = cfg['xyz']
        rpy = cfg['rpy']
        parent = cfg['parent_frame']
        child = cfg['optical_frame_id']

        qx, qy, qz, qw = euler_to_quaternion(*rpy)

        broadcaster = StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = xyz[0]
        tf_msg.transform.translation.y = xyz[1]
        tf_msg.transform.translation.z = xyz[2]
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        broadcaster.sendTransform(tf_msg)
        self.get_logger().info(
            f'Published static TF: {parent} → {child} '
            f'at xyz={xyz}, rpy={rpy}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
