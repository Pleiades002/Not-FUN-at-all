#!/usr/bin/python3

import rclpy
import numpy as np

from controller_interfaces.srv import SetParam, SetTarget
from geometry_msgs.msg import Twist, Point, PoseStamped
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.mouse_pose = np.array([0.0, 0.0])
        self.rvis2_pose = np.array([0.0, 0.0])
        self.pizza_pose = np.array([0.0, 0.0])
        self.pizza_flag = 0
        self.kp_linear = 2
        self.kp_angular = 4
        
        # / before = not relative, but no / before = relative to namespace
        self.declare_parameter('frequency', 10.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.rviz2_position_callback, 10)

        self.set_target_server = self.create_service(SetTarget, 'cil', self.set_target_callback)
        self.set_k_param_server = self.create_service(SetParam, 'k_param', self.set_k_param_callback)

        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, 'eat')

        self.create_timer(1 / self.frequency, self.timer_callback)
        self.get_logger().info(f'Starting{self.get_namespace()} /{self.get_name()} with the  default parameter. hz: {self.frequency}')

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def pose_callback(self, msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def mouse_position_callback(self, msg):
        self.mouse_pose[0] = msg.x
        self.mouse_pose[1] = msg.y
        self.spawn_pizza(self.mouse_pose[0], self.mouse_pose[1])

    def rviz2_position_callback(self, msg):
        self.rvis2_pose[0] = msg.pose.position.x + 5
        self.rvis2_pose[1] = msg.pose.position.y + 5
        self.spawn_pizza(self.rvis2_pose[0], self.rvis2_pose[1])

    def spawn_pizza(self, x, y):
        if self.pizza_flag == 0:
            position_request = GivePosition.Request()
            position_request.x = x
            position_request.y = y
            self.pizza_pose[0] = x
            self.pizza_pose[1] = y
            self.spawn_pizza_client.call_async(position_request)
            self.pizza_flag = 1
        else:
            pass

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)
        self.pizza_flag = 0

    def set_target_callback(self, request: SetTarget.Request, response: SetTarget.Response):
        if self.pizza_flag == 0:
            self.x_pos = request.target.x + 5
            self.y_pos = request.target.y + 5
            self.spawn_pizza(self.x_pos, self.y_pos)
            self.pizza_flag = 1
        else:
            pass
        return response

    def set_k_param_callback(self, request: SetParam.Request, response: SetParam.Response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        return response

    def pathFinder(self):
        if self.pizza_flag == 1:
            delta_x = self.pizza_pose[0] - self.robot_pose[0]
            delta_y = self.pizza_pose[1] - self.robot_pose[1]
            distance = np.sqrt((delta_x**2) + (delta_y**2))
            alpha = np.arctan2(delta_y, delta_x)
            temp_turn_theta = alpha - self.robot_pose[2]
            turn_theta = np.arctan2(np.sin(temp_turn_theta), np.cos(temp_turn_theta))
            vx = self.kp_linear * distance
            w = self.kp_angular * turn_theta
            print("Turtle1 pose: ", self.robot_pose)
            print("Pizza1 pose: ", self.pizza_pose)
            print("VX: ", vx, "W: ", w)
            self.cmdvel(vx, w)
            if distance < 0.05:
                self.eat_pizza()
        else:
            self.cmdvel(0.0, 0.0)

    def timer_callback(self):
        self.pathFinder()


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
