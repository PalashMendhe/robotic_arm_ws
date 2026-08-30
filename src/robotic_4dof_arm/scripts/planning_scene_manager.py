#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, ObjectColor
import yaml
from ament_index_python.packages import get_package_share_directory

class PlanningSceneManager:
    def __init__(self, node: Node = None):
        """
        Allows PlanningSceneManager to either:
        1. Run as its own standalone ROS 2 Node (for testing).
        2. Attach to an existing PickAndPlace node instance.
        """
        # Load parameters from YAML file
        params_path = os.path.join(get_package_share_directory('arm_moveit_config'), 'config', 'robot_params.yaml')
        with open(params_path, 'r') as f:
            self.params = yaml.safe_load(f)
        self.own_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(
                'planning_scene_manager',
                parameter_overrides=[
                    rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, self.params['robot']['use_sim_time'])
                ]
            )
            self.own_node = True
        else:
            self.node = node

        # Publishers for MoveIt planning scene
        self.collision_object_pub = self.node.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        self.attached_object_pub = self.node.create_publisher(
            AttachedCollisionObject, '/attached_collision_object', 10
        )
        self.planning_scene_pub = self.node.create_publisher(
            PlanningScene, '/planning_scene', 10
        )

        # Allow subscribers time to connect
        time.sleep(0.5)

    def _create_header(self, frame_id='world') -> Header:
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def add_box_object(self, name: str, size: tuple, position: tuple, orientation=(0.0, 0.0, 0.0, 1.0), frame_id='world', color: ColorRGBA = None):
        """Creates and publishes a box CollisionObject with optional custom color."""
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])

        obj = CollisionObject()
        obj.header = self._create_header(frame_id)
        obj.id = name
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        # Publish collision object
        self.collision_object_pub.publish(obj)

        # If custom color is provided, publish it via PlanningScene diff
        if color is not None:
            scene_diff = PlanningScene()
            scene_diff.is_diff = True
            obj_color = ObjectColor()
            obj_color.id = name
            obj_color.color = color
            scene_diff.object_colors = [obj_color]
            self.planning_scene_pub.publish(scene_diff)

        self.node.get_logger().info(f"Added collision object '{name}' at {position} in frame '{frame_id}'")
        time.sleep(0.1)

    def remove_object(self, name: str):
        """Removes a CollisionObject by ID."""
        obj = CollisionObject()
        obj.header = self._create_header()
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        self.collision_object_pub.publish(obj)
        self.node.get_logger().info(f"Removed collision object '{name}'")
        time.sleep(0.1)

    def add_static_environment(self):
        """
        Adds static environment objects matching warehouse.sdf relative to robot's base frame (spawned at x=0.5):
        - Table: size [1.5, 1.0, 0.95] at (-0.5, 0.0, -0.475)
        - Obstacle Divider Wall: size [0.5, 0.1, 0.65] at (-0.55, 0.0, 0.05)
        """
        # Table (Neutral Grey, top at z = -0.01 for mounting clearance)
        table_color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.85)
        self.add_box_object(
            name='table',
            size=(1.5, 1.0, 0.95),
            position=(-0.5, 0.0, -0.485),
            color=table_color
        )
        self.add_obstacle()

    def add_obstacle(self):
        """Adds vertical wall obstacle between pick and place locations."""
        obstacle_color = ColorRGBA(r=1.0, g=0.5, b=0.5, a=0.9)
        self.add_box_object(
            name='obstacle',
            size=(0.5, 0.1, 0.65),
            position=(-0.63, -0.04, 0.05),
            color=obstacle_color
        )

    def add_target_box(self, position=(-0.60, 0.20, 0.05), size=(0.1, 0.1, 0.1)):
        """Adds pickable target box at the spawn marker (Red)."""
        box_color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.95)
        self.add_box_object(
            name='target_box',
            size=size,
            position=position,
            color=box_color
        )

    def init_full_scene(self):
        """Initializes the complete planning scene with table, obstacle, and target box."""
        self.node.get_logger().info('Initializing complete MoveIt planning scene...')
        self.add_static_environment()
        self.add_target_box()
        self.node.get_logger().info('Planning scene successfully initialized.')

    def attach_target_box_to_gripper(self, link_name='gripper_base_link', size=(0.1, 0.1, 0.1)):
        """
        Attaches 'target_box' to the robot gripper.
        Sets touch_links to avoid false collision aborts with fingers.
        """
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.header = self._create_header(frame_id=link_name)
        attached_object.object.id = 'target_box'
        attached_object.object.operation = CollisionObject.ADD

        # Define box geometry relative to the gripper link
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        box_pose = Pose()
        box_pose.position.x = 0.0
        box_pose.position.y = 0.0
        box_pose.position.z = 0.075  # Positioned at TCP between the prongs
        box_pose.orientation.w = 1.0

        attached_object.object.primitives = [box]
        attached_object.object.primitive_poses = [box_pose]

        # Allowed contact links (vital to prevent self-collision errors)
        attached_object.touch_links = [
            'right_prong_link',
            'left_prong_link',
            'gripper_base_link',
            'gripper_tcp',
            'wrist_gripper_link'
        ]

        self.attached_object_pub.publish(attached_object)
        self.node.get_logger().info(f"Attached 'target_box' to '{link_name}'")
        time.sleep(0.2)

    def detach_target_box_from_gripper(self, drop_position=(-0.45, -0.25, 0.05), size=(0.1, 0.1, 0.1)):
        """
        Detaches 'target_box' from gripper and re-adds it at the drop-off pose in world frame.
        """
        # 1. Detach from gripper
        detached_object = AttachedCollisionObject()
        detached_object.object.header = self._create_header(frame_id='world')
        detached_object.object.id = 'target_box'
        detached_object.object.operation = CollisionObject.REMOVE
        self.attached_object_pub.publish(detached_object)
        self.node.get_logger().info("Detached 'target_box' from gripper")
        time.sleep(0.1)

        # 2. Re-add as static object at drop location
        box_color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.95)
        self.add_box_object(
            name='target_box',
            size=size,
            position=drop_position,
            color=box_color
        )
        self.node.get_logger().info(f"Re-added 'target_box' at drop position {drop_position}")

    def clear_all_objects(self):
        """Clears all objects from the planning scene."""
        self.remove_object('table')
        self.remove_object('obstacle')
        self.remove_object('target_box')
        self.node.get_logger().info('Cleared all objects from planning scene.')

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    manager = PlanningSceneManager()
    manager.init_full_scene()
    
    try:
        manager.node.get_logger().info('PlanningSceneManager is active with custom colors. Press Ctrl+C to stop.')
        rclpy.spin(manager.node)
    except KeyboardInterrupt:
        pass
    finally:
        if manager.own_node:
            manager.node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
