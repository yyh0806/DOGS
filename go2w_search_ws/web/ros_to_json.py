"""ROS2 → JSON 桥接 (运行在 PC 的 Docker Humble 容器内)。

订阅载荷NX发布的狗数据话题, 转成 JSON 写到挂载文件,
让宿主机上的 panel.py (Galactic环境, 无rclpy) 能读到真狗数据。

数据流:
  载荷NX → /imu /scan /odom (DDS) → 本脚本(容器内) → /workspace/web/dog_state.json → panel.py(宿主机)

运行 (容器内):
  source /opt/ros/humble/setup.bash
  python3 /workspace/web/ros_to_json.py
"""
import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry

STATE_FILE = "/workspace/web/dog_state.json"


class RosToJson(Node):
    def __init__(self):
        super().__init__('ros_to_json')
        self._lock = threading.Lock()
        self._state = {
            'yaw': 0.0, 'x': 0.0, 'y': 0.0,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0,  # 角速度
            'imu_count': 0, 'scan_count': 0, 'odom_count': 0,
            'ranges': [], 'trail': [], 'last_t': 0.0,
            'connected': False,
        }
        self._last_trail_t = 0.0

        self.create_subscription(Imu, '/imu', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        # 每 0.15s 写一次文件 (和 panel.py 广播频率一致)
        self.create_timer(0.15, self._write_file)
        self.get_logger().info(f"ros_to_json 桥接就绪, 写 {STATE_FILE}")

    def _on_imu(self, msg):
        with self._lock:
            # 从四元数取 yaw (z轴)
            q = msg.orientation
            # yaw = atan2(2(wz+xy), 1-2(y²+z²))
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self._state['yaw'] = math.atan2(siny_cosp, cosy_cosp)
            self._state['gx'] = msg.angular_velocity.x
            self._state['gy'] = msg.angular_velocity.y
            self._state['gz'] = msg.angular_velocity.z
            self._state['imu_count'] += 1
            self._state['connected'] = True
            self._state['last_t'] = time.time()

    def _on_scan(self, msg):
        with self._lock:
            self._state['ranges'] = [round(r, 2) for r in msg.ranges]
            self._state['scan_count'] += 1
            self._state['last_t'] = time.time()

    def _on_odom(self, msg):
        with self._lock:
            self._state['x'] = msg.pose.pose.position.x
            self._state['y'] = msg.pose.pose.position.y
            self._state['odom_count'] += 1
            # 轨迹采样: 每0.5m记一个点
            now = time.time()
            if now - self._last_trail_t > 0.3:
                self._last_trail_t = now
                self._state['trail'].append([round(self._state['x'], 2), round(self._state['y'], 2)])
                if len(self._state['trail']) > 500:
                    self._state['trail'] = self._state['trail'][-500:]
            self._state['last_t'] = time.time()

    def _write_file(self):
        with self._lock:
            data = dict(self._state)
            data['ranges'] = list(data['ranges'])
            data['trail'] = list(data['trail'])
        # 原子写 (避免 panel.py 读到半写的文件)
        tmp = STATE_FILE + ".tmp"
        try:
            with open(tmp, 'w') as f:
                json.dump(data, f)
            import os
            os.replace(tmp, STATE_FILE)
        except Exception as e:
            self.get_logger().warning(f"写文件失败: {e}")


def main():
    rclpy.init()
    node = RosToJson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
