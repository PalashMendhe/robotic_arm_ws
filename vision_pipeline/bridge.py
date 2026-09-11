import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import os

from sensor_msgs.msg import Image
class DataCollectorNode(Node): #create data collector node to subscribe to camera images
    def __init__(self):
        super().__init__('data_collector') 
        self.bridge = CvBridge() # initialzie the cv_bridge to convert ROS images to OpenCV format
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.red_count = 0 # count of red boxes in the image
        self.blue_count = 0 # count of blue cylinders in the image
        
        self.red_dir = os.path.join('data_collection', 'red_box')
        self.blue_dir = os.path.join('data_collection', 'blue_cylinders')
        if not os.path.exists(self.red_dir):
            os.makedirs(self.red_dir)
        if not os.path.exists(self.blue_dir):
            os.makedirs(self.blue_dir)

    def image_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv2.imshow('Camera Image', cv_image) # display the image using OpenCV
        key = cv2.waitKey(1) # wait for a key press to update the image display

        if key == ord('r'):
            cv2.imwrite(f'{self.red_dir}/red_box_{self.red_count:03d}.jpg', cv_image)
            self.red_count += 1
            self.get_logger().info(f'Saved red_box_{self.red_count:03d}.jpg')
        elif key == ord('b'):
            cv2.imwrite(f'{self.blue_dir}/blue_cylinder_{self.blue_count:03d}.jpg', cv_image)
            self.blue_count += 1
            self.get_logger().info(f'Saved blue_cylinder_{self.blue_count:03d}.jpg')
        elif key == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollectorNode()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()