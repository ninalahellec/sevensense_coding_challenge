#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class DiffToAckermann(Node):
    """
    Converts differential drive commands to Ackermann steering commands.
    This node subscribes to the '/cmd_vel_diff' topic for differential drive commands
    and publishes the converted commands to the '/cmd_vel' topic.
    """
    def __init__(self):
        super().__init__('differential_to_ackermann')

        # distance between front and rear axle
        self.wheelbase = 0.54838
        self.direction = 1  # Direction for pure rotation case

        # Publisher
        self.pub_ackermann = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.sub_twist = self.create_subscription(Twist, '/cmd_vel_diff', self.twist_callback, 10)

        self.get_logger().info(f"DiffToAckermann node started. Publishing to /cmd_vel and subscribing to /cmd_vel_diff.")


    def twist_callback(self, msg):
        ackermann_msg = Twist()

        # Handle normal case: nonzero linear velocity
        if abs(msg.linear.x) > 1e-5:
            ackermann_msg.linear.x = msg.linear.x
            steering_angle = math.atan(self.wheelbase * msg.angular.z / msg.linear.x)

        # Handle pure rotation case: linear velocity is 0 but angular velocity is not 0
        elif abs(msg.angular.z) > 1e-5:
            small_speed = 0.1
            ackermann_msg.linear.x = self.direction * small_speed
            steering_angle = math.atan(self.wheelbase * abs(msg.angular.z) / small_speed)
            # Flip the direction for next time
            self.direction *= -1

        # No movement requested
        else:
            ackermann_msg.linear.x = 0.0
            steering_angle = 0.0

        ackermann_msg.angular.z = steering_angle

        self.get_logger().info(f"Publishing: linear velocity = {ackermann_msg.linear.x:.3f}, steering angle = {steering_angle:.3f}")
        self.pub_ackermann.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
