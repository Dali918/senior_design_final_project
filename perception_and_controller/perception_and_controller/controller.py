# fill the controller code here

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
import math

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # tunable parameters
        self.linear_velocity = 0.5

        # Stanley controller parameters
        self.k_e = 0.1  # Cross-track error gain
        self.k_heading = 1.0  # Heading error gain
        
        # Subscribe to pose_msg topic from perception
        self.pose_msg_subscriber = self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.pose_msg_callback,
            10
        )

        # Subscribe to pose_info message from simulation
        self.pose_info_subscriber = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_info_callback,
            10
        )

        # Publisher - publish Twist messages to cmd_vel topic for simulation
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Initialize variables to store pose information
        self.target_pose = None
        self.current_pose = None

    def pose_msg_callback(self, msg: PoseStamped):
        self.target_pose = msg.pose
        self.get_logger().info('Received target pose: {}'.format(self.target_pose))
        self.control_loop()

    def pose_info_callback(self, msg: PoseArray):
        if msg.poses:
            self.current_pose = msg.poses[0]
            self.get_logger().info('Received current pose: {}'.format(self.current_pose))
            self.control_loop()

    def control_loop(self):
        if self.target_pose and self.current_pose:
            cmd_vel_msg = Twist()
            
            # Get the current position and orientation of the vehicle
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            
            # Get the target position
            target_x = self.target_pose.position.x
            target_y = self.target_pose.position.y

            # Calculate the difference in position
            dx = target_x - x
            dy = target_y - y

            # Calculate the cross-track error (distance from the vehicle to the desired path)
            cross_track_error = (dy * math.cos(current_yaw)) - (dx * math.sin(current_yaw))

            # Calculate the heading error (difference between the current yaw and the desired heading)
            heading_error = self.angle_difference(math.atan2(dy, dx), current_yaw)

            # Calculate the steering angle using the Stanley controller
            steering_angle = heading_error + math.atan2(self.k_e * cross_track_error, self.linear_velocity)

            # Limit the steering angle to a reasonable range
            steering_angle = max(min(steering_angle, math.radians(30)), math.radians(-30))  # Limit to +/- 30 degrees

            # Calculate the angular velocity based on the steering angle
            angular_velocity = (self.linear_velocity) * math.tan(steering_angle)

            # Set cmd_vel properties
            cmd_vel_msg.linear.x = self.linear_velocity
            cmd_vel_msg.angular.z = angular_velocity
            self.cmd_vel_publisher.publish(cmd_vel_msg)

    def get_yaw_from_pose(self, pose):
        # Extract yaw (rotation around the z-axis) from quaternion
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_difference(self, target_yaw, current_yaw):
        # Calculate the difference between two angles and normalize it
        angle_diff = target_yaw - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
