"""mock_scan_node: 发假 LaserScan 兜底 livox WSL2 退化.

WSL2 livox 插件 brztink6u (fresh WSL #1) 后退化 (visualize=false 后 ray casting 偶发
不发点云), /scan 空 → motion ScanFreshnessWatchdog nav_scan_fresh=False → motion 不激活
→ drive_session 卡 parked → velocity_authorized=false → planar_move 不执行 → 狗不位移.

本节点发 10Hz 假 LaserScan (360 读数, 5m 宛如 indoor_empty 墙) 让:
  - nav_scan_fresh=True (motion 激活前置)
  - nav2 local_costmap 有数据 (规划通)
配合 sim_amcl_to_odom 订 /odom_planar (loc 持续新鲜) → motion 激活完成 → cmd_vel →
planar_move → 狗位移 (/odom_planar 变化).

importer: sim_full_bringup.launch.py Node executable=mock_scan_node.
仅 GO2W_SIM + WSL2 livox 退化时; 真机不用 (livox 真发). b463cl71d/brztink6u 验证过
真 livox 链路 (5.6Hz/0.6Hz + FastLIO /Odometry), 本节点是环境退化 fallback.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MockScan(Node):
    def __init__(self):
        super().__init__('mock_scan')
        self._pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_timer(0.1, self._publish)  # 10Hz 兜底
        self.get_logger().info(
            'mock_scan: 10Hz 假 LaserScan (360 读数 5m) 兜底 livox WSL2 退化')

    def _publish(self) -> None:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / 360  # 360 读数
        msg.time_increment = 0.0
        msg.range_min = 0.1
        msg.range_max = 20.0
        msg.ranges = [5.0] * 360  # 假障碍 5m (indoor_empty 墙 ~5m)
        msg.intensities = [100.0] * 360
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
