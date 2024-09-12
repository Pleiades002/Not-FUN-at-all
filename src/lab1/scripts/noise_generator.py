#!/usr/bin/python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from lab1_interfaces.srv import SetNoise


class NoiseGenerator(Node):
    def __init__(self):
        # initialize super class node
        super().__init__('noise_generator')
        # create publicsher
        self.noise_publisher = self.create_publisher(Float64, 'noise', 10)

        # set the publisher rate
        self.declare_parameter('rate', 5.0)
        self.rate = self.get_parameter(
            'rate').get_parameter_value().double_value

        # additional attributes
        self.mean = 0.0
        self.variance = 1.0

        # create service server for /set_noise
        self.set_noise_server = self.create_service(
            SetNoise, 'set_noise', self.set_noise_callback)

        # start timer for publisher /noise
        self.timer = self.create_timer(1/self.rate, self.timer_callback)
        self.get_logger().info(
            f'Starting{self.get_namespace()} / {self.get_name()} with the  default parameter. hz: {self.rate} mean: {self.mean}, variance: {self.variance}')

    def set_noise_callback(self, request: SetNoise.Request, response: SetNoise.Response):
        self.mean = request.mean.data
        self.variance = request.variance.data
        return response

    def timer_callback(self):
        msg = Float64()
        msg.data = np.random.normal(self.mean, np.sqrt(self.variance))
        self.noise_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NoiseGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
