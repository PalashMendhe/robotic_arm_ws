from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  
import cv2
from enum import Enum
from geometry_msgs.msg import Twist
class State(Enum):
    SEARCHING = 1
    APPROACHING = 2
    STOPPED = 3
class state_machine(Node):
    
    def __init__(self):
        super().__init__('state_machine_subscriber')
        self.state = State.SEARCHING
        self.target_box = None
        self.ka = 0.002
        self.linear_speed = 0.1
        self.image_width = 640
        self.image_height = 480
        self.last_seen_time = 0.0
        self.timeout_duration = 3.0
        self.visited_threshold = 50
        self.stop_time = None
        self.stop_cooldown = 4.0
        self.model = YOLO("best.pt")
        self.bridge = CvBridge()
        self.locked_target_center = None
        self.lock_threshold = 100
        self.approach_start_time = None
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/model/my_robot/cmd_vel', 10)
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, iou=0.7, conf=0.5)
        annotated_image = results[0].plot()
        cv2.imshow("Detection Results", annotated_image)
        cv2.waitKey(1)
        red_boxes = []
        for box in results[0].boxes:
            cls = int(box.cls[0])
            if cls == 1:  
                red_boxes.append(box)
        if len(red_boxes) > 0:
            if self.state == State.APPROACHING and self.locked_target_center is not None:
                lx, ly = self.locked_target_center
                self.target_box = min(red_boxes, key=lambda b: 
                    abs(float((b.xyxy[0][0]+b.xyxy[0][2])/2) - lx) +
                    abs(float((b.xyxy[0][1]+b.xyxy[0][3])/2) - ly))
            else:
                self.target_box = max(red_boxes, key=lambda b: 
                    float((b.xyxy[0][2]-b.xyxy[0][0]) * (b.xyxy[0][3]-b.xyxy[0][1])))
                cx = float((self.target_box.xyxy[0][0]+self.target_box.xyxy[0][2])/2)
                area = float((self.target_box.xyxy[0][2]-self.target_box.xyxy[0][0]) * 
                            (self.target_box.xyxy[0][3]-self.target_box.xyxy[0][1]))
                self.get_logger().info(f"SEARCHING selected: cx={cx:.0f} area={area:.0f}")
            
            
            self.last_seen_time = self.get_clock().now().nanoseconds / 1e9
        if len(red_boxes) == 0:
            self.target_box = None
        cmd = Twist()
        if self.state == State.SEARCHING:
            self.searching_state(cmd)
        elif self.state == State.APPROACHING:
            self.approaching_state(cmd)
        elif self.state == State.STOPPED:
            self.stopped_state(cmd)
        self.publisher.publish(cmd)
        self.get_logger().info(f"Current State: {self.state.name}, Target Box: {self.target_box}")

    def searching_state(self, cmd):
        if self.target_box is not None:
        
            cx = float((self.target_box.xyxy[0][0]+self.target_box.xyxy[0][2])/2)
            cy = float((self.target_box.xyxy[0][1]+self.target_box.xyxy[0][3])/2)
            self.locked_target_center = (cx, cy)
            self.state = State.APPROACHING
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3

    def approaching_state(self, cmd):
        current_time = float(self.get_clock().now().nanoseconds / 1e9)
    
        # only go to searching after timeout, not immediately on lost detection
        if self.target_box is None:
            if current_time - self.last_seen_time > self.timeout_duration:
                self.locked_target_center = None
                self.approach_start_time = None
                self.state = State.SEARCHING
            return
        
        box_center_x = float(self.target_box.xyxy[0][0] + self.target_box.xyxy[0][2]) / 2
        error_x = float(box_center_x - self.image_width / 2)
        
        box_area = float((self.target_box.xyxy[0][2] - self.target_box.xyxy[0][0]) * 
                        (self.target_box.xyxy[0][3] - self.target_box.xyxy[0][1]))
        image_area = float(self.image_width * self.image_height)
        
        
        cmd.angular.z = float(max(-0.3, min(0.3, -self.ka * error_x)))
        
        
        if abs(error_x) < 80:
            cmd.linear.x = float(self.linear_speed)
        else:
            cmd.linear.x = 0.0
        
        current_time = float(self.get_clock().now().nanoseconds / 1e9)
        if self.approach_start_time is None:
            self.approach_start_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.approach_start_time
        if box_area / image_area > 0.2 and elapsed > 2.0:
            self.state = State.STOPPED
            self.approach_start_time = None
        
        if current_time - self.last_seen_time > self.timeout_duration:
            self.locked_target_center = None
            self.state = State.SEARCHING
        self.get_logger().info(f"error_x={error_x:.0f} angular={cmd.angular.z:.3f} linear={cmd.linear.x:.3f} state={self.state.name}")
    
    def stopped_state(self, cmd):
        cmd.linear.x = 0.0
        cmd.angular.z = 0.3  
        
        if self.stop_time is None:
            self.stop_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("Target reached, rotating away...")
            
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.stop_time > self.stop_cooldown:
            self.stop_time = None
            self.state = State.SEARCHING
            self.get_logger().info("Searching for next target")
            

def main(args=None):
    rclpy.init(args=args)
    node = state_machine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()