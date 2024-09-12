#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.create_timer(0.01, self.timer_callback)

        self.odom1_publisher = self.create_publisher(Odometry, '/odom1', 10)
        self.odom2_publisher = self.create_publisher(Odometry, '/odom2', 10)

        self.create_subscription(Pose, '/turtle1/pose', self.pose1_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose2_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_publisher(self, msg, chaild_frame_id, pub):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = chaild_frame_id

        odom_msg.pose.pose.position.x = msg.x - 5
        odom_msg.pose.pose.position.y = msg.y - 5

        q = quaternion_from_euler(0, 0, msg.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = chaild_frame_id

        t.transform.translation.x = msg.x - 5
        t.transform.translation.y = msg.y - 5
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def pose1_callback(self, msg):
        self.odom_publisher(msg, 'robot1', self.odom1_publisher)

    def pose2_callback(self, msg):
        self.odom_publisher(msg, 'robot2', self.odom2_publisher)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
