#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
import time

def inverse_kinematics(coords, gripper_status, gripper_angle = 0):
    '''
    Calculates the joint angles according to the desired TCP coordinate and gripper angle
    :param coords: list, desired [X, Y, Z] TCP coordinates
    :param gripper_status: string, can be `closed` or `open`
    :param gripper_angle: float, gripper angle in woorld coordinate system (0 = horizontal, pi/2 = vertical)
    :return: list, the list of joint angles, including the 2 gripper fingers
    '''
    # link lengths
    ua_link = 0.2
    fa_link = 0.25
    tcp_link = 0.175
    # z offset (robot arm base height)
    z_offset = 0.075
    # default return list
    angles = [0,0,0,0,0,0]

    # Calculate the shoulder pan angle from x and y coordinates
    j0 = math.atan2(coords[1], coords[0])

    # Re-calculate target coordinated to the wrist joint (x', y', z')
    x = coords[0] - tcp_link * math.cos(j0) * math.cos(gripper_angle)
    y = coords[1] - tcp_link * math.sin(j0) * math.cos(gripper_angle)
    z = coords[2] - z_offset + math.sin(gripper_angle) * tcp_link

    # Solve the problem in 2D using x" and z'
    x = math.sqrt(y*y + x*x)

    # Let's calculate auxiliary lengths and angles
    c = math.sqrt(x*x + z*z)
    alpha = math.asin(z/c)
    beta = math.pi - alpha
    
    # Apply law of cosines
    val = (ua_link*ua_link + c*c - fa_link*fa_link)/(2*c*ua_link)
    val = max(-1.0, min(1.0, val)) # Clamp to avoid domain errors
    gamma = math.acos(val)

    j1 = math.pi/2.0 - alpha - gamma
    
    val2 = (ua_link*ua_link + fa_link*fa_link - c*c)/(2*ua_link*fa_link)
    val2 = max(-1.0, min(1.0, val2)) # Clamp to avoid domain errors
    j2 = math.pi - math.acos(val2) # j2 = 180 - j2'
    
    delta = math.pi - (math.pi - j2) - gamma # delta = 180 - j2' - gamma

    j3 = math.pi + gripper_angle - beta - delta

    angles[0] = j0
    angles[1] = j1
    angles[2] = j2
    angles[3] = j3

    if gripper_status == "open":
        angles[4] = 0.04
        angles[5] = 0.04
    elif gripper_status == "closed":
        # 0.026 leaves a 1mm gap on each side so physics doesn't violently push the object off-center
        angles[4] = 0.026
        angles[5] = 0.026
    else:
        angles[4] = 0.04
        angles[5] = 0.04

    return angles

class IKTester(Node):
    def __init__(self):
        super().__init__('ik_tester_node')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher_ = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.detach_pub = self.create_publisher(Empty, '/green/detach', 10)
        self.attach_pub = self.create_publisher(Empty, '/green/attach', 10)
        
        self.get_logger().info('IK Tester Node started! Waiting for publishers...')
        time.sleep(2.0)
        
        # Detach green cylinder at startup since it is attached by default
        self.detach_pub.publish(Empty())
        self.get_logger().info("Published detach command at startup.")
        
    def move_arm(self, coords, gripper_status, gripper_angle=0):
        angles = inverse_kinematics(coords, gripper_status, gripper_angle)
        
        # Arm Trajectory
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint']
        
        arm_point = JointTrajectoryPoint()
        arm_point.positions = angles[0:4]
        arm_point.time_from_start = Duration(sec=2, nanosec=0)
        
        arm_msg.points = [arm_point]
        self.publisher_.publish(arm_msg)
        self.get_logger().info(f"Published arm trajectory to coords {coords}")
        
        # Gripper Trajectory
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = angles[4:6]
        gripper_point.time_from_start = Duration(sec=2, nanosec=0)
        
        gripper_msg.points = [gripper_point]
        self.gripper_publisher_.publish(gripper_msg)
        self.get_logger().info(f"Published gripper trajectory to {gripper_status}")

def main(args=None):
    rclpy.init(args=args)
    node = IKTester()
    
    node.get_logger().info("Ensuring gripper is detached at startup (sending 10 times)...")
    for _ in range(10):
        node.detach_pub.publish(Empty())
        time.sleep(0.2)
    
    # Target coordinates derived from forward kinematics of the manually
    # found working joint angles (from rqt_joint_trajectory_controller):
    #   shoulder_pan: -0.4684, shoulder_lift: 0.8168,
    #   elbow: 1.6493, wrist: -0.8954
    # FK result: end effector at [0.4257, -0.2154, 0.0168]
    # Gripper angle: 0.0 (horizontal grasp)
    
    GRIPPER_ANGLE = 0.0  # Horizontal approach (gripper points sideways)
    TARGET_X = 0.4257
    TARGET_Y = -0.2154
    GRAB_Z = 0.0168    # FK-derived z for grabbing the cylinder center
    HOVER_Z = 0.20    # Safe hover height above the table
    
    # Calculate horizontal approach positions (pull back by 0.15m)
    dist_target = math.sqrt(TARGET_X**2 + TARGET_Y**2)
    PRE_TARGET_X = TARGET_X - 0.15 * (TARGET_X / dist_target)
    PRE_TARGET_Y = TARGET_Y - 0.15 * (TARGET_Y / dist_target)
    
    DROP_X = 0.0
    DROP_Y = 0.35
    dist_drop = math.sqrt(DROP_X**2 + DROP_Y**2)
    PRE_DROP_X = DROP_X - 0.15 * (DROP_X / dist_drop)
    PRE_DROP_Y = DROP_Y - 0.15 * (DROP_Y / dist_drop)
    
    # Send a sequence of movements
    node.get_logger().info("Moving to point 1 (Hover IN FRONT OF green cylinder)...")
    node.move_arm([PRE_TARGET_X, PRE_TARGET_Y, GRAB_Z], "open", GRIPPER_ANGLE)
    time.sleep(3)
    
    node.get_logger().info("Moving to point 2 (Move forward to grasp)...")
    node.move_arm([TARGET_X, TARGET_Y, GRAB_Z], "open", GRIPPER_ANGLE)
    time.sleep(3)
    
    node.get_logger().info("Closing gripper...")
    node.move_arm([TARGET_X, TARGET_Y, GRAB_Z], "closed", GRIPPER_ANGLE)
    time.sleep(1)
    
    node.get_logger().info("Attaching green cylinder to gripper...")
    node.attach_pub.publish(Empty())
    time.sleep(1)
    
    node.get_logger().info("Moving to point 3 (Lift object)...")
    node.move_arm([TARGET_X, TARGET_Y, HOVER_Z], "closed", GRIPPER_ANGLE)
    time.sleep(3)
    
    node.get_logger().info("Moving to point 4 (Move to drop position)...")
    node.move_arm([DROP_X, DROP_Y, HOVER_Z], "closed", GRIPPER_ANGLE)
    time.sleep(3)

    node.get_logger().info("Moving to point 5 (Lower object)...")
    node.move_arm([DROP_X, DROP_Y, GRAB_Z], "closed", GRIPPER_ANGLE)
    time.sleep(3)
    
    node.get_logger().info("Detaching green cylinder...")
    node.detach_pub.publish(Empty())
    time.sleep(1)
    
    node.get_logger().info("Opening gripper...")
    node.move_arm([DROP_X, DROP_Y, GRAB_Z], "open", GRIPPER_ANGLE)
    time.sleep(1)

    node.get_logger().info("Moving to point 6 (Retract horizontally)...")
    node.move_arm([PRE_DROP_X, PRE_DROP_Y, GRAB_Z], "open", GRIPPER_ANGLE)
    time.sleep(3)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()