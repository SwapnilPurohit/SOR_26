import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

class PatrollerNode(Node):
    def __init__(self):
        super().__init__('patroller_node')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info('Patroller Node started. Waiting for follow_waypoints action server...')
        self.action_client.wait_for_server()
        self.current_waypoint_index = -1
        self.get_logger().info('Action server found! Sending waypoints...')
        self.send_waypoints()

    def send_waypoints(self):
        goal_msg = FollowWaypoints.Goal()
        
        # Waypoint 1
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.header.stamp = self.get_clock().now().to_msg()
        pose1.pose.position.x = 4.05
        pose1.pose.position.y = 1.69
        pose1.pose.orientation.w = 1.0

        # Waypoint 2
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.header.stamp = self.get_clock().now().to_msg()
        pose2.pose.position.x = 7.33
        pose2.pose.position.y = 0.12
        pose2.pose.orientation.w = 1.0

        # Waypoint 3
        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.header.stamp = self.get_clock().now().to_msg()
        pose3.pose.position.x = 4.02
        pose3.pose.position.y = -1.08
        pose3.pose.orientation.w = 1.0

        # Waypoint 4
        pose4 = PoseStamped()
        pose4.header.frame_id = 'map'
        pose4.header.stamp = self.get_clock().now().to_msg()
        pose4.pose.position.x = 4.21
        pose4.pose.position.y = -4.24
        pose4.pose.orientation.w = 1.0

        # Waypoint 5
        pose5 = PoseStamped()
        pose5.header.frame_id = 'map'
        pose5.header.stamp = self.get_clock().now().to_msg()
        pose5.pose.position.x = 0.84
        pose5.pose.position.y = -7.89
        pose5.pose.orientation.w = 1.0

        # Waypoint 6
        pose6 = PoseStamped()
        pose6.header.frame_id = 'map'
        pose6.header.stamp = self.get_clock().now().to_msg()
        pose6.pose.position.x = -2.72
        pose6.pose.position.y = -5.32
        pose6.pose.orientation.w = 1.0

        # Waypoint 7
        pose7 = PoseStamped()
        pose7.header.frame_id = 'map'
        pose7.header.stamp = self.get_clock().now().to_msg()
        pose7.pose.position.x = -2.28
        pose7.pose.position.y = -0.90
        pose7.pose.orientation.w = 1.0

        # Waypoint 8
        pose8 = PoseStamped()
        pose8.header.frame_id = 'map'
        pose8.header.stamp = self.get_clock().now().to_msg()
        pose8.pose.position.x = -0.14
        pose8.pose.position.y = 0.21
        pose8.pose.orientation.w = 1.0

        goal_msg.poses = [pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8]

        self.get_logger().info('Sending goal with 8 waypoints...')
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted! Executing waypoints...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        if current_waypoint != self.current_waypoint_index:
            if self.current_waypoint_index != -1:
                self.get_logger().info(f'-> Waypoint {self.current_waypoint_index + 1} Reached!')
            self.get_logger().info(f'Navigating to Waypoint {current_waypoint + 1}...')
            self.current_waypoint_index = current_waypoint

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Finished executing all waypoints!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PatrollerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
