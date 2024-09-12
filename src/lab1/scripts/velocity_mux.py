#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux')

        # create subscriber for topic /liner/noise
        self.linear_vel_subscriber = self.create_subscription(
            Float64, '/linear/noise', self.linear_vel_sub_callback, 10)

        # create subscriber for topic /angular/noise
        self.angular_vel_subscriber = self.create_subscription(
            Float64, '/angular/noise', self.angular_vel_sub_callback, 10)

        # create publisher for topic cmd_vel
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # set rate parameter
        self.declare_parameter('rate', 5.0)
        self.rate = self.get_parameter(
            'rate').get_parameter_value().double_value

        # additional attrobutes
        self.cmd_vel = Twist()

        # start timer
        self.timer = self.create_timer(1/self.rate, self.timer_callback)

        # start msg
        self.get_logger().info(
            f'Starting {self.get_namespace()} / {self.get_name()}')

    # callback  : set the x-component of linear velocity
    def linear_vel_sub_callback(self, msg: Float64):
        self.cmd_vel.linear.x = msg.data
        self.get_logger().info(f"Received linear noise: {msg.data}")

    # callback  : set the Z-component of angular velocity
    def angular_vel_sub_callback(self, msg: Float64):
        self.cmd_vel.angular.z = msg.data
        self.get_logger().info(f"Received angular noise: {msg.data}")

    # callback timer
    def timer_callback(self):
        self.cmd_publisher.publish(self.cmd_vel)
        self.get_logger().info(
            f"Published cmd_vel: linear.x = {self.cmd_vel.linear.x}, angular.z = {self.cmd_vel.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
