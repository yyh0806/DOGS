#!/usr/bin/env python3
"""⚠️ 仅验证用, 勿部署到生产 NX (mock_dog_state_publisher)。

阶段A 端到端验证用的 mock 节点, 模拟 nx_motion_node + nx_sensor_node 发布的话题,
让 nx_web_server.py 在没有真狗 / 没有 nx_sensor_node 时也能跑通 verify_nx_web.sh。

发布话题 (与真节点对齐):
  /dog_state  std_msgs/String(JSON)  2Hz   状态机字符串 + vx/vy/vyaw
  /imu        sensor_msgs/Imu        50Hz  四元数 orientation (yaw 缓慢旋转)
  /scan       sensor_msgs/LaserScan  10Hz  360 点假障碍 (前 2m, 两侧 1m)
  /odom       nav_msgs/Odometry      50Hz  xy 螺旋漂移 (验证 trail 渲染)

QoS: 全部用默认 RELIABLE depth=10 (与 nx_sensor_node.py:104-107 发布端一致,
nx_web_server.py 订阅端也是 RELIABLE depth=10)。

运行 (NX 或任意装了 rclpy 的 Linux):
  source /opt/ros/humble/setup.bash
  python3 web/mock_dog_state_publisher.py
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

import builtin_interfaces.msg


def _stamp(t):
    s = builtin_interfaces.msg.Time()
    sec = int(t)
    s.sec = sec
    s.nanosec = int((t - sec) * 1e9)
    return s


class MockDogNode(Node):
    def __init__(self):
        super().__init__('mock_dog_node')
        self._dog_pub = self.create_publisher(String, '/dog_state', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu', 10)
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 2Hz dog_state, 50Hz imu, 10Hz scan, 50Hz odom
        self.create_timer(0.5, self._pub_dog_state)
        self.create_timer(0.02, self._pub_imu)
        self.create_timer(0.1, self._pub_scan)
        self.create_timer(0.02, self._pub_odom)
        self._t0 = time.time()
        self.get_logger().info("MockDogNode 就绪: 发 /dog_state(2Hz) /imu(50Hz) /scan(10Hz) /odom(50Hz)")

    def _yaw(self):
        # yaw 缓慢旋转 (让前端地图狗箭头转起来)
        t = time.time() - self._t0
        return math.sin(t * 0.3) * 1.5

    def _xy(self):
        t = time.time() - self._t0
        r = min(t * 0.15, 5.0)
        return round(math.cos(t * 0.4) * r, 3), round(math.sin(t * 0.4) * r, 3)

    @staticmethod
    def _yaw_to_quat(yaw):
        # z 轴旋转四元数 (ros_to_json:52-55 反运算)
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _pub_dog_state(self):
        yaw = self._yaw()
        vx = 0.2 * math.cos(yaw)
        vy = 0.2 * math.sin(yaw)
        msg = String()
        # JSON 格式与 nx_motion_node:247-250 发布格式一致
        msg.data = '{"state": "MOVING", "vx": %.3f, "vy": %.3f, "vyaw": 0.000}' % (vx, vy)
        self._dog_pub.publish(msg)

    def _pub_imu(self):
        msg = Imu()
        msg.header.stamp = _stamp(time.time())
        msg.header.frame_id = 'imu_link'
        msg.orientation = self._yaw_to_quat(self._yaw())
        msg.orientation_covariance = [-1.0] + [0.0] * 8
        self._imu_pub.publish(msg)

    def _pub_scan(self):
        msg = LaserScan()
        now = time.time()
        msg.header.stamp = _stamp(now)
        msg.header.frame_id = 'laser'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / 360.0
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        # 假障碍: 前方 2m, 两侧 1m, 其余 5m
        ranges = []
        for i in range(360):
            ang = -math.pi + i * msg.angle_increment
            if -0.3 < ang < 0.3:
                ranges.append(2.0)
            elif abs(ang) < 1.2:
                ranges.append(1.0)
            else:
                ranges.append(5.0)
        msg.ranges = ranges
        self._scan_pub.publish(msg)

    def _pub_odom(self):
        x, y = self._xy()
        msg = Odometry()
        msg.header.stamp = _stamp(time.time())
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = self._yaw_to_quat(self._yaw())
        self._odom_pub.publish(msg)


def main():
    rclpy.init()
    node = MockDogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
