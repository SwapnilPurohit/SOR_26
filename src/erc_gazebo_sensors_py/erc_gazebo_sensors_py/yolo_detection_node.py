import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import threading
import numpy as np

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.bridge = CvBridge()
        
        # Load the YOLOv8 model
        self.model = YOLO('yolov8s.pt')
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, 10)
            
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.target_object = None
        self.status = "WAITING FOR TARGET"
        self.current_distance = -1.0
        self.depth_image = None
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.get_target_input, daemon=True)
        self.input_thread.start()

    def get_target_input(self):
        while True:
            target = input("Enter target object (e.g. 'bottle', 'chair'): ").strip()
            if target:
                self.target_object = target
                self.status = "SEARCHING"
                print(f"Target set to: {self.target_object}")

    def depth_callback(self, msg):
        # The depth image is 32FC1
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO inference
        results = self.model(cv_image, verbose=False)
        result = results[0]
        
        target_detected = False
        twist = Twist()
        
        # Plot YOLO boxes on image
        annotated_image = result.plot()
        
        if self.target_object and self.status != "COMPLETED":
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = result.names[class_id]
                
                if class_name.lower() == self.target_object.lower():
                    target_detected = True
                    self.status = "TRACKING"
                    
                    # Bounding box center
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    # Get distance using depth image and median filter
                    if self.depth_image is not None:
                        # Extract a small window around the center
                        w_size = 5
                        # Ensure boundaries
                        min_y = max(0, cy - w_size)
                        max_y = min(self.depth_image.shape[0], cy + w_size + 1)
                        min_x = max(0, cx - w_size)
                        max_x = min(self.depth_image.shape[1], cx + w_size + 1)
                        
                        region = self.depth_image[min_y:max_y, min_x:max_x]
                        
                        # Filter out nans and zeros
                        valid_pixels = region[(~np.isnan(region)) & (region > 0)]
                        if len(valid_pixels) > 0:
                            self.current_distance = np.median(valid_pixels)
                        else:
                            self.current_distance = -1.0
                    
                    # Proportional control
                    if self.current_distance != -1.0:
                        if self.current_distance < 1.0:
                            # Stop!
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.status = "COMPLETED"
                            print("Mission Completed! Enter a new target.")
                        else:
                            # Steer toward center
                            err = annotated_image.shape[1] / 2 - cx
                            # Reduced P-gain to prevent oscillation (snaking)
                            twist.angular.z = float(err) / 300.0
                            # Bound the angular velocity to prevent violent swinging
                            twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
                            twist.linear.x = 0.4 # move forward
                    
                    # Draw center point
                    cv2.circle(annotated_image, (cx, cy), 5, (0, 0, 255), -1)
                    break # Track first matched object
            
            if not target_detected and self.status == "SEARCHING":
                # Spin to search
                twist.angular.z = 0.5
        
        self.cmd_pub.publish(twist)
        
        # Dashboard overlay
        dash_text = [
            f"TARGET: {self.target_object if self.target_object else 'None'}",
            f"STATUS: {self.status}",
            f"DISTANCE: {self.current_distance:.2f}m" if self.current_distance > 0 else "DISTANCE: Unknown",
            f"MODE: {'AUTO' if self.target_object else 'IDLE'}"
        ]
        
        y_offset = 30
        for text in dash_text:
            cv2.putText(annotated_image, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y_offset += 30
            
        cv2.imshow("YOLO Dashboard", annotated_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
