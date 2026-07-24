"""motion_sdk_mock: sim 替代 Unitree SDK2 (Task3 简化版).

订阅 /cmd_vel + /cmd_vel_nav (双订阅, 契约同 nx_motion_node.py:194-197),
转发 /cmd_vel_nav → /cmd_vel 供 planar_move 消费 (cmd_vel 路径消歧).
SportState(mode/velocity/progress) 回报留完整版.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotionSdkMock(Node):
    def __init__(self):
        super().__init__('motion_sdk_mock')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_nav_cmd_vel, 10)

    def _on_cmd_vel(self, msg):
        pass  # operator cmd_vel; sim 里 planar_move 直接订阅 /cmd_vel

    def _on_nav_cmd_vel(self, msg):
        self.cmd_vel_pub.publish(msg)  # Nav2 自主 → 转发 planar_move


def main():
    rclpy.init()
    node = MotionSdkMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
