#!/usr/bin/env python3
"""Go2W TF 发布节点。
订阅 /utlidar/robot_odom 发布 odom→base_link TF。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TfPublisher(Node):
    def __init__(self):
        super().__init__('go2w_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/utlidar/robot_odom',
                                             self.on_odom, 10)
        self.get_logger().info('TF发布节点就绪: /utlidar/robot_odom → odom→base_link')

    def on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(TfPublisher())


if __name__ == '__main__':
    main()