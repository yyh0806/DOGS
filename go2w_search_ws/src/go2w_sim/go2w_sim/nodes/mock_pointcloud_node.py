"""mock_pointcloud_node: 发假 PointCloud2 → /mid360/points_nav 让 web LidarBridge 显示鸟瞰.

WSL2 livox 插件 gzserver SIGFPE 崩后不发 /livox/lidar_PointCloud2 → relay /mid360/points_nav 空
→ web nx_lidar_node LidarBridge 收不到点云 → 前端雷达鸟瞰空白. 本节点发 10Hz PointCloud2
(360 点 5m 圆) 兜底, 让 web 鸟瞰显示 (仿真 livox 退化 fallback).

importer: sim_full_bringup.launch.py Node executable=mock_pointcloud_node.
真机用真 livox (/livox/lidar_PointCloud2 → relay /mid360/points_nav). 仅 GO2W_SIM.
"""
import struct
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class MockPointCloud(Node):
    def __init__(self):
        super().__init__('mock_pointcloud')
        self._pub = self.create_publisher(PointCloud2, '/mid360/points_nav', 10)
        self.create_timer(0.1, self._publish)  # 10Hz
        self.get_logger().info(
            'mock_pointcloud: 10Hz PointCloud2 (360 点 5m 圆) → /mid360/points_nav')

    def _publish(self) -> None:
        points = []
        for i in range(360):
            angle = i * 2.0 * math.pi / 360.0
            r = 5.0 + 0.3 * math.sin(angle * 3)
            points.append((r * math.cos(angle), r * math.sin(angle), 0.0))
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        data = b''.join(struct.pack('fff', p[0], p[1], p[2]) for p in points)
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(points)
        msg.data = data
        msg.is_dense = True
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
