#!/usr/bin/env python3
"""
shape_detector.py  —  B1 Classical Shape Detection Node
--------------------------------------------------------
Subscribes to the D415 colour + depth streams, detects all objects on
Table 2 via Canny edges + contour fitting, classifies each as
red_cube / blue_cylinder by circularity, back-projects to 3-D world
coordinates via TF2, and publishes vision_msgs/Detection2DArray.

Topics
------
  SUB  /d415/image          sensor_msgs/Image  (bgr8 colour)
  SUB  /d415/depth_image    sensor_msgs/Image  (32FC1 depth, metres)
  SUB  /d415/camera_info    sensor_msgs/CameraInfo  (intrinsics)
  PUB  /vision/detections   vision_msgs/Detection2DArray
  PUB  /vision/debug_image  sensor_msgs/Image   (annotated colour frame)
"""

import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

import message_filters


class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector_subscriber')

        # ── Load tuning parameters from camera_params.yaml ────────────────
        params_path = os.path.join(
            get_package_share_directory('arm_moveit_config'),
            'config', 'camera_params.yaml'
        )
        with open(params_path, 'r') as f:
            cfg = yaml.safe_load(f)

        det = cfg['detection']
        self.canny_low            = det['canny_low']
        self.canny_high           = det['canny_high']
        self.contour_area_min     = det['contour_area_min']
        self.contour_area_max     = det['contour_area_max']
        self.circularity_threshold = det['circularity_threshold']
        self.depth_scale          = det['depth_scale']   # 1.0 for Gazebo float32

        self.depth_min = cfg['depth']['min_m']
        self.depth_max = cfg['depth']['max_m']

        # Camera intrinsics — will be populated on first CameraInfo message
        self.fx = self.fy = self.cx = self.cy = None
        self._camera_info_ready = False

        # ── TF2 buffer + listener ──────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── cv_bridge + publisher ──────────────────────────────────────────
        self.bridge = CvBridge()

        self.pub_detections = self.create_publisher(
            Detection2DArray, '/vision/detections', 10
        )
        self.pub_debug = self.create_publisher(
            Image, '/vision/debug_image', 10
        )

        # ── Camera info subscriber (one-shot to get intrinsics) ────────────
        self.sub_info = self.create_subscription(
            CameraInfo, '/d415/camera_info', self._camera_info_cb, 1
        )

        # ── Synchronised colour + depth subscribers ────────────────────────
        sub_color = message_filters.Subscriber(self, Image, '/d415/image')
        sub_depth = message_filters.Subscriber(self, Image, '/d415/depth_image')
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self.image_callback)

        self.get_logger().info('ShapeDetector ready — waiting for camera feed...')

    # ── Camera intrinsics (populated once) ────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if not self._camera_info_ready:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self._camera_info_ready = True
            self.get_logger().info(
                f'CameraInfo received: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}'
            )

    # ── Image preprocessing ────────────────────────────────────────────────
    def preprocess_image(self, cv_image: np.ndarray) -> np.ndarray:
        gray    = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, self.canny_low, self.canny_high)
        return edges

    # ── Contour detection + classification ────────────────────────────────
    def find_contours(self, edges: np.ndarray):
        """
        Returns list of (contour, class_id, circularity) tuples.
        Both cubes AND cylinders are returned; classification is by circularity.
        """
        raw, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for contour in raw:
            area = cv2.contourArea(contour)
            if not (self.contour_area_min < area < self.contour_area_max):
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            # Classify: circle-like → cylinder, rectangle-like → cube
            if circularity > self.circularity_threshold:
                class_id = 'blue_cylinder'
            else:
                class_id = 'red_cube'
            results.append((contour, class_id, circularity))
        return results

    # ── Pixel centroid from bounding rect ─────────────────────────────────
    def centroid(self, contour) -> tuple:
        """Returns (cx_px, cy_px) from minAreaRect centroid."""
        ((cx_px, cy_px), (w, h), angle) = cv2.minAreaRect(contour)
        return (int(cx_px), int(cy_px))

    # ── Depth lookup with validity check ──────────────────────────────────
    def get_valid_depth(self, depth_image: np.ndarray, cx_px: int, cy_px: int):
        """
        Returns depth in metres, or None if out-of-range.
        Samples a 3×3 patch median for robustness.
        """
        h, w = depth_image.shape[:2]
        x0, x1 = max(0, cx_px - 1), min(w, cx_px + 2)
        y0, y1 = max(0, cy_px - 1), min(h, cy_px + 2)
        patch = depth_image[y0:y1, x0:x1].astype(np.float32)
        valid  = patch[(patch > 0) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        depth_m = float(np.median(valid)) * self.depth_scale
        if not (self.depth_min <= depth_m <= self.depth_max):
            return None
        return depth_m

    # ── Back-project pixel → 3-D camera frame ─────────────────────────────
    def back_project_to_3d(self, cx_px: int, cy_px: int, depth_m: float) -> tuple:
        X_c = (cx_px - self.cx) * depth_m / self.fx
        Y_c = (cy_px - self.cy) * depth_m / self.fy
        Z_c = depth_m
        return (X_c, Y_c, Z_c)

    # ── Transform camera frame → world frame via TF2 ──────────────────────
    def transform_to_world(self, X_c: float, Y_c: float, Z_c: float):
        """
        Uses tf2 to transform a point from d415_color_optical_frame to world.
        Returns (X_w, Y_w, Z_w) or None if the transform is unavailable.
        """
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'world', 'd415_color_optical_frame', Time()
            )
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return None

        t = tf.transform.translation
        r = tf.transform.rotation

        # Quaternion → rotation matrix
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        R = np.array([
            [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
            [  2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qx*qw)],
            [  2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
        ])
        p_c = np.array([X_c, Y_c, Z_c])
        tr  = np.array([t.x, t.y, t.z])
        p_w = R @ p_c + tr
        return (float(p_w[0]), float(p_w[1]), float(p_w[2]))

    # ── Build Detection2DArray message ─────────────────────────────────────
    def detection2darray(self, detections: list, header) -> Detection2DArray:
        """
        detections: list of (cx_px, cy_px, w, h, class_id, circularity, X_w, Y_w, Z_w)
        """
        array_msg = Detection2DArray()
        array_msg.header = header

        for (cx_px, cy_px, bw, bh, class_id, circularity, X_w, Y_w, Z_w) in detections:
            det = Detection2D()
            det.header = header

            # 2-D bounding box (image space)
            det.bbox.center.position.x = float(cx_px)
            det.bbox.center.position.y = float(cy_px)
            det.bbox.size_x = float(bw)
            det.bbox.size_y = float(bh)

            # Hypothesis: class + confidence + 3-D world pose
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_id
            hyp.hypothesis.score    = float(min(circularity, 1.0))
            hyp.pose.pose.position.x = X_w
            hyp.pose.pose.position.y = Y_w
            hyp.pose.pose.position.z = Z_w

            det.results.append(hyp)
            array_msg.detections.append(det)

        return array_msg

    # ── Main callback (colour + depth synchronised) ────────────────────────
    def image_callback(self, color_msg: Image, depth_msg: Image):
        if not self._camera_info_ready:
            return  # wait until intrinsics are known

        cv_image    = self.bridge.imgmsg_to_cv2(color_msg,  desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg,  desired_encoding='passthrough')

        edges    = self.preprocess_image(cv_image)
        contours = self.find_contours(edges)

        debug_img  = cv_image.copy()
        detections = []

        for (contour, class_id, circularity) in contours:
            cx_px, cy_px = self.centroid(contour)
            depth_m = self.get_valid_depth(depth_image, cx_px, cy_px)
            if depth_m is None:
                continue

            world = self.transform_to_world(*self.back_project_to_3d(cx_px, cy_px, depth_m))
            if world is None:
                continue
            X_w, Y_w, Z_w = world

            # Bounding box size (pixels) for the message
            _, (bw, bh), _ = cv2.minAreaRect(contour)

            detections.append((cx_px, cy_px, bw, bh, class_id, circularity, X_w, Y_w, Z_w))

            # ── Debug drawing ──────────────────────────────────────────────
            color = (0, 0, 200) if class_id == 'red_cube' else (200, 0, 0)  # BGR
            cv2.drawContours(debug_img, [contour], -1, color, 2)
            cv2.circle(debug_img, (cx_px, cy_px), 4, (0, 255, 0), -1)
            label = f'{class_id}  c={circularity:.2f}'
            cv2.putText(debug_img, label, (cx_px - 40, cy_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            coord_label = f'({X_w:.2f},{Y_w:.2f},{Z_w:.2f})'
            cv2.putText(debug_img, coord_label, (cx_px - 40, cy_px + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 0), 1)

            self.get_logger().info(
                f'[{class_id}] world=({X_w:.3f}, {Y_w:.3f}, {Z_w:.3f})  '
                f'circ={circularity:.3f}  depth={depth_m:.3f}m'
            )

        # ── Publish results ────────────────────────────────────────────────
        det_array = self.detection2darray(detections, color_msg.header)
        self.pub_detections.publish(det_array)
        self.pub_debug.publish(
            self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        )


def main(args=None):
    rclpy.init(args=args)
    shape_detector_node = ShapeDetector()
    rclpy.spin(shape_detector_node)
    shape_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
