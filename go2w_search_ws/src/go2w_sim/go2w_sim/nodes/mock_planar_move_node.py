"""mock_planar_move_node: 订 /cmd_vel 运动学积分发 /odom_planar + TF odom→base_footprint.

绕过 gzserver planar_move (libgazebo_ros_planar_move 在 gzserver 进程内, WSL2 SIGFPE -8
反复崩 → planar_move 死 → 狗不动). 本节点纯 ROS 运动学积分 (vx, wz → x, y, yaw),
不依赖 gzserver, 50Hz 更新.

用户允许"运动模型简化" (四足步态 → planar_move 已是简化, 本节点进一步 ROS 节点化).
真机用 SDK SportGateway; 真仿真 planar_move (b463cl71d/brztink6u) 已验证, 本节点是
WSL2 gzserver 不稳 fallback.

importer: sim_full_bringup.launch.py Node executable=mock_planar_move_node.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


class MockPlanarMove(Node):
    def __init__(self):
        super().__init__('mock_planar_move')
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._wz = 0.0
        self._last_t = self.get_clock().now()
        self._odom_pub = self.create_publisher(Odometry, '/odom_planar', 50)
        self._tf_br = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.02, self._update)  # 50Hz
        self.get_logger().info(
            'mock_planar_move: /cmd_vel → /odom_planar + TF (50Hz 运动学积分)')

    def _on_cmd(self, msg: Twist) -> None:
        self._vx = float(msg.linear.x)
        self._wz = float(msg.angular.z)

    def _update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if not (0.0 < dt <= 1.0):
            dt = 0.02
        self._x += self._vx * dt * math.cos(self._yaw)
        self._y += self._vx * dt * math.sin(self._yaw)
        self._yaw += self._wz * dt
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation.z = math.sin(self._yaw / 2.0)
        t.transform.rotation.w = math.cos(self._yaw / 2.0)
        self._tf_br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = MockPlanarMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
