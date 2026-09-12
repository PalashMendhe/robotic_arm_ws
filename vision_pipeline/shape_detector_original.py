from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.time import Time

class shape_detector(Node):
    def __init__(self):
        super().__init__('shape_detector_subscriber')
        # Initialize the shape detection logic here
        self.canny_low = 40 
        self.canny_high = 120
        self.contour_area_min = 400
        self.contour_area_max = 30000
        self.circularity_threshold = 0.88
        self.depth_scale = 1.0
        self.image = None
        self.subscription = self.create_subscription(Image, '/d415/image', self.image_callback, 10)
    def preprocess_image(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        return edges
    def find_contours(self, edges):
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.contour_area_min < area < self.contour_area_max:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * 3.14159 * (area / (perimeter * perimeter))
                    if circularity > self.circularity_threshold:
                        filtered_contours.append(contour)
        return filtered_contours

    def centroid(self, contour):
        ((cx_px, cy_px), (w, h), angle) = cv2.minAreaRect(contour)
        depth_m = depth_image[int(cy_px), int(cx_px)] * self.depth_scale
        if depth_m < 0.30 or depth_m > 2.00:
            self.get_logger().info(f"Depth {depth_m:.2f} m is less than 0.30 m or greater than 2.00 m, ignoring contour.")
            return None
        return (int(cx_px), int(cy_px))

    def back_project_to_3d(self, cx_px, cy_px, depth_m):
        X_c = (cx_px - self.cx) * depth_m / self.fx
        Y_c = (cy_px - self.cy) * depth_m / self.fy
        Z_c = depth_m
        return (X_c, Y_c, Z_c)

    def transform_to_world(self, X_c, Y_c, Z_c):
        tf_buffer.lookup_transform('world', 'd415_color_optical_frame', rclpy.time.Time())
        X_w = self.R[0][0] * X_c + self.R[0][1] * Y_c + self.R[0][2] * Z_c + self.t[0]
        Y_w = self.R[1][0] * X_c + self.R[1][1] * Y_c + self.R[1][2] * Z_c + self.t[1]
        Z_w = self.R[2][0] * X_c + self.R[2][1] * Y_c + self.R[2][2] * Z_c + self.t[2]
        return (X_w, Y_w, Z_w)

    def detection2darray(self, contours):
        #implement this function to convert contours to a 2D array of detections
        pass

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        edges = self.preprocess_image(cv_image)
        contours = self.find_contours(edges)
        for contour in contours:
            centroid = self.centroid(contour)
            if centroid is not None:
                cx_px, cy_px = centroid
                depth_m = depth_image[int(cy_px), int(cx_px)] * self.depth_scale
                X_c, Y_c, Z_c = self.back_project_to_3d(cx_px, cy_px, depth_m)
                X_w, Y_w, Z_w = self.transform_to_world(X_c, Y_c, Z_c)
                self.get_logger().info(f"Detected shape at world coordinates: ({X_w:.2f}, {Y_w:.2f}, {Z_w:.2f})")

def main(args=None):
    rclpy.init(args=args)
    shape_detector_node = shape_detector()
    rclpy.spin(shape_detector_node)
    shape_detector_node.destroy_node()
    rclpy.shutdown()
        