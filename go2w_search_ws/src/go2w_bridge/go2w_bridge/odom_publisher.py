"""里程计发布: IMU yaw + ICP 平移 → ROS2 Odometry + TF。

从 Go2W DDS 获取 IMU 数据提供 yaw，从 LiDAR ICP 获取平移，
融合为里程计并发布 nav_msgs/Odometry 和 odom→base_link TF。
"""

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomPublisher:
    """融合 IMU yaw + ICP 平移的里程计发布器。"""

    def __init__(self, node):
        """初始化。

        Args:
            node: rclpy Node 实例，用于创建 publisher 和 broadcaster
        """
        self._node = node
        self._lock = threading.Lock()

        # 里程计状态
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0

        # IMU 状态
        self._imu_yaw = 0.0
        self._imu_yaw_offset = None
        self._imu_rpy = [0.0, 0.0, 0.0]

        # 速度指令（从 cmd_vel 获取，用于填充 Odometry.twist）
        self._cmd_vx = 0.0
        self._cmd_vyaw = 0.0

        # ROS2 接口
        self._odom_pub = node.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(node)

        # 上次更新时间
        self._last_time = time.time()

    @property
    def position(self):
        """返回 (x, y, yaw) 当前里程计位置。"""
        with self._lock:
            return self._x, self._y, self._yaw

    def update_imu(self, imu_yaw: float):
        """更新 IMU yaw（在 DDS 回调中调用）。

        Args:
            imu_yaw: IMU 原始 yaw（弧度）
        """
        with self._lock:
            self._imu_yaw = imu_yaw
            if self._imu_yaw_offset is not None:
                self._yaw = imu_yaw - self._imu_yaw_offset

    def calibrate_yaw(self):
        """校准 yaw 偏移：将当前位置视为 yaw=0 的朝向。"""
        with self._lock:
            self._imu_yaw_offset = self._imu_yaw
            self._yaw = 0.0

    def update_icp(self, icp_dx: float, icp_dy: float):
        """更新 ICP 位移（在 LiDAR 处理后调用）。

        Args:
            icp_dx: ICP 算出的 X 方向位移（局部坐标系）
            icp_dy: ICP 算出的 Y 方向位移（局部坐标系）
        """
        with self._lock:
            # 局部位移转到世界坐标
            cos_y = math.cos(self._yaw)
            sin_y = math.sin(self._yaw)
            self._x += cos_y * icp_dx - sin_y * icp_dy
            self._y += sin_y * icp_dx + cos_y * icp_dy

    def set_cmd_vel(self, vx: float, vy: float, vyaw: float):
        """记录当前速度指令（用于填充 Odometry twist）。"""
        with self._lock:
            self._cmd_vx = vx
            self._cmd_vyaw = vyaw

    def reset(self):
        """重置里程计到原点。"""
        with self._lock:
            self._x = 0.0
            self._y = 0.0
            self._yaw = 0.0
            self._imu_yaw_offset = None
            self._vx = 0.0
            self._vy = 0.0
            self._vyaw = 0.0

    def publish(self):
        """发布 Odometry 消息和 TF（在 ROS2 定时器中调用）。"""
        now = self._node.get_clock().now()
        now_sec = time.time()
        dt = now_sec - self._last_time
        self._last_time = now_sec

        with self._lock:
            x, y, yaw = self._x, self._y, self._yaw
            cmd_vx, cmd_vyaw = self._cmd_vx, self._cmd_vyaw

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # yaw → quaternion
        half = yaw / 2.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        # 协方差（ICP 里程计不确定性较大）
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.05   # x
        odom.pose.covariance[7] = 0.05   # y
        odom.pose.covariance[35] = 0.1   # yaw

        # twist（来自 cmd_vel，非测量值）
        odom.twist.twist.linear.x = cmd_vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = cmd_vyaw
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.01

        self._odom_pub.publish(odom)

        # 发布 TF: odom → base_link
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(half)
        tf.transform.rotation.w = math.cos(half)

        self._tf_broadcaster.sendTransform(tf)
