from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  
import cv2

class DetectionTestNode(Node):
    def __init__(self):
        super().__init__('detection_test_node')
        self.model = YOLO("best.pt")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback, 10
        )
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)
        annotated_image = results[0].plot()
        cv2.imshow("Detection Results", annotated_image)
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    node = DetectionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()