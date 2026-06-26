import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class ChaseTheBall(Node):
    def __init__(self):
        super().__init__('chase_the_ball')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Chaser node started. Looking for a blue ball...")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range for blue color
        lower_blue = (100, 150, 0)
        upper_blue = (140, 255, 255)
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        twist = Twist()
        if contours:
            # Find largest contour
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    
                    # Proportional control for steering
                    err = cv_image.shape[1] / 2 - cx
                    twist.angular.z = float(err) / 100.0
                    twist.linear.x = 0.5 # Move forward
                    
                    # Draw a circle on the detected ball
                    cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 3)
        else:
            # Spin if not found
            twist.angular.z = 0.5

        self.cmd_pub.publish(twist)
        
        # Show what the robot sees
        cv2.imshow("Robot Camera", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ChaseTheBall()
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
