"""Go2W Bridge Node - ROS2 与 Go2W SDK 之间的桥接节点。

职责:
1. 从 Go2W SDK DDS 获取 IMU/LiDAR/视频数据，发布标准 ROS2 话题
2. 订阅 ROS2 /cmd_vel 话题，将速度指令转发给 Go2W SDK
3. 发布 /scan, /odom, /tf, /camera/image_raw 供 SLAM/Nav2 使用
4. 管理 Go2W 的连接、站立/坐下、运动模式切换

节点名: go2w_bridge
话题:
  发布:
    /scan              (sensor_msgs/LaserScan)     - LiDAR 2D 扫描
    /odom              (nav_msgs/Odometry)          - 融合里程计
    /camera/image_raw  (sensor_msgs/Image)          - 摄像头图像
    /tf                (tf2_msgs/TFMessage)         - odom→base_link, base_link→laser_frame
  订阅:
    /cmd_vel           (geometry_msgs/Twist)        - 速度指令
"""

import math
import struct
import time
import logging
import threading
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .sport_client import Go2WSportClient, SDK_AVAILABLE
from .lidar_publisher import LidarPublisher
from .odom_publisher import OdomPublisher

logger = logging.getLogger(__name__)


class Go2WBridgeNode(Node):
    """Go2W SDK-ROS2 桥接节点。

    从 Go2W DDS 读取传感器数据，发布标准 ROS2 消息。
    同时订阅 cmd_vel 转发给 Go2W 运动控制。
    """

    def __init__(self):
        super().__init__('go2w_bridge')

        # 声明参数
        self.declare_parameter('network_interface', '')
        self.declare_parameter('robot_ip', '192.168.123.161')
        self.declare_parameter('use_driving_mode', True)
        self.declare_parameter('stand_on_start', True)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_scan', True)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('publish_camera', True)
        self.declare_parameter('camera_rate', 10.0)

        # 获取参数
        interface = self.get_parameter('network_interface').get_parameter_value().string_value
        self._use_driving = self.get_parameter('use_driving_mode').get_parameter_value().bool_value
        self._stand_on_start = self.get_parameter('stand_on_start').get_parameter_value().bool_value
        self._cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value

        # Go2W SDK 客户端
        self._sport = Go2WSportClient(network_interface=interface)
        self._dds_inited = False
        self._video_client = None
        self._connected = False

        # 子模块
        self._lidar_pub = LidarPublisher()
        self._odom_pub = OdomPublisher(self)

        # LiDAR 数据队列（DDS 回调写入，定时器消费）
        self._pointcloud_queue = []

        # 速度指令状态
        self._last_cmd_time = 0.0
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._cmd_lock = threading.Lock()

        # ROS2 发布器
        scan_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._scan_pub = self.create_publisher(LaserScan, '/scan', scan_qos)

        cmd_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, cmd_qos
        )

        camera_qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._camera_pub = self.create_publisher(Image, '/camera/image_raw', camera_qos)

        # 静态 TF: base_link → laser_frame
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # 定时器
        scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        odom_rate = self.get_parameter('odom_rate').get_parameter_value().double_value
        camera_rate = self.get_parameter('camera_rate').get_parameter_value().double_value

        if scan_rate > 0:
            self._scan_timer = self.create_timer(1.0 / scan_rate, self._process_lidar)
        if odom_rate > 0:
            self._odom_timer = self.create_timer(1.0 / odom_rate, self._publish_odom)
        if camera_rate > 0:
            self._camera_timer = self.create_timer(1.0 / camera_rate, self._publish_camera)

        self._watchdog_timer = self.create_timer(0.1, self._cmd_watchdog)

        # 连接并初始化
        self._connect_and_init()

    def _connect_and_init(self):
        """连接 Go2W，初始化 DDS 订阅和姿态。"""
        self.get_logger().info("正在连接 Go2W...")

        if not SDK_AVAILABLE:
            self.get_logger().warn("unitree_sdk2py 未安装，模拟模式运行")
            self._connected = True
            return

        try:
            from unitree_sdk2py.core.channel import ChannelFactory
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.go2.video.video_client import VideoClient

            factory = ChannelFactory()
            factory.Init(0, self.get_parameter('network_interface')
                         .get_parameter_value().string_value or "")
            self._dds_inited = True

            self._sport._client = SportClient()
            self._sport._client.SetTimeout(10.0)
            self._sport._client.Init()
            self._sport._connected = True

            self._video_client = VideoClient()
            self._video_client.SetTimeout(10.0)
            self._video_client.Init()

            self._subscribe_imu(factory)
            time.sleep(0.3)
            self._subscribe_lidar(factory)

            if self._stand_on_start:
                self._sport.balance_stand()
                self.get_logger().info("Go2W 已站立")
                time.sleep(1.0)

                if self._use_driving:
                    self._sport.switch_to_drive_mode()
                    self.get_logger().info("已切换到轮式驱动模式")

            # 校准 yaw
            time.sleep(0.5)
            self._odom_pub.calibrate_yaw()

            self._connected = True
            self.get_logger().info("Go2W 连接成功! (IMU + LiDAR + Video)")

        except Exception as e:
            self.get_logger().error(f"Go2W 连接失败: {e}")
            self.get_logger().warn("将在模拟模式运行")
            self._connected = True

    def _subscribe_imu(self, factory):
        """订阅 IMU 数据 (DDS rt/lowstate)。"""
        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_

            def on_imu_msg(msg):
                try:
                    imu_yaw = float(msg.imu_state.rpy[2])
                    self._odom_pub.update_imu(imu_yaw)
                except Exception:
                    pass

            ch = factory.CreateRecvChannel('rt/lowstate', LowState_)
            ch.SetReader(handler=on_imu_msg)
            self.get_logger().info("IMU 订阅成功 (rt/lowstate)")
        except Exception as e:
            self.get_logger().warn(f"IMU 订阅失败: {e}")

    def _subscribe_lidar(self, factory):
        """订阅 LiDAR 数据 (DDS rt/utlidar/cloud)。"""
        try:
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_

            def on_lidar_msg(msg):
                try:
                    info = {
                        'data': bytes(msg.data),
                        'point_step': int(msg.point_step),
                        'width': int(msg.width),
                    }
                    self._pointcloud_queue.append(info)
                    if len(self._pointcloud_queue) > 5:
                        self._pointcloud_queue = self._pointcloud_queue[-3:]
                except Exception:
                    pass

            ch = factory.CreateRecvChannel('rt/utlidar/cloud', PointCloud2_)
            ch.SetReader(handler=on_lidar_msg)
            self.get_logger().info("LiDAR 订阅成功 (rt/utlidar/cloud)")
        except Exception as e:
            self.get_logger().warn(f"LiDAR 订阅失败: {e}")

    def _cmd_vel_callback(self, msg: Twist):
        """处理速度指令。"""
        vx, vy, vyaw = msg.linear.x, msg.linear.y, msg.angular.z

        with self._cmd_lock:
            self._cmd_vel = (vx, vy, vyaw)
            self._last_cmd_time = time.time()

        self._sport.set_velocity(vx, vy, vyaw)
        self._odom_pub.set_cmd_vel(vx, vy, vyaw)

    def _cmd_watchdog(self):
        """速度指令看门狗: 超时自动停止。"""
        with self._cmd_lock:
            elapsed = time.time() - self._last_cmd_time

        if elapsed > self._cmd_timeout and self._last_cmd_time > 0:
            self._sport.stop_move()
            with self._cmd_lock:
                self._cmd_vel = (0.0, 0.0, 0.0)
                self._last_cmd_time = 0.0
            self._odom_pub.set_cmd_vel(0.0, 0.0, 0.0)

    def _process_lidar(self):
        """LiDAR 定时器: 处理点云，发布 LaserScan + 更新里程计。"""
        if not self._pointcloud_queue:
            return

        info = self._pointcloud_queue.pop(0)
        _, imu_yaw = self._odom_pub.position[:2], self._odom_pub.position[2] if self._odom_pub.position else 0.0
        imu_yaw = self._odom_pub.position[2]

        laser_scan, icp_dx, icp_dy = self._lidar_pub.process_frame(imu_yaw)

        if laser_scan is not None:
            # 更新时间戳
            laser_scan.header.stamp = self.get_clock().now().to_msg()
            self._scan_pub.publish(laser_scan)

        if abs(icp_dx) > 0.001 or abs(icp_dy) > 0.001:
            self._odom_pub.update_icp(icp_dx, icp_dy)

    def _publish_odom(self):
        """Odometry 定时器: 发布里程计和 TF。"""
        if self._connected:
            self._odom_pub.publish()

    def _publish_camera(self):
        """摄像头定时器: 发布图像。"""
        if not self._connected or self._video_client is None:
            return

        if self._camera_pub.get_subscription_count() == 0:
            return

        try:
            code, data = self._video_client.GetImageSample()
            if code == 0 and data and len(data) > 0:
                raw = bytes(data)
                img_array = np.frombuffer(raw, dtype=np.uint8)

                import cv2
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if frame is not None:
                    h, w = frame.shape[:2]

                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_frame'
                    msg.height = h
                    msg.width = w
                    msg.encoding = 'bgr8'
                    msg.is_bigendian = False
                    msg.step = w * 3
                    msg.data = frame.tobytes()
                    self._camera_pub.publish(msg)
        except Exception:
            pass

    def _publish_static_tf(self):
        """发布静态 TF: base_link → laser_frame。"""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'laser_frame'
        # LiDAR 安装在机器人顶部中央，略偏前
        tf.transform.translation.x = 0.15
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.25
        tf.transform.rotation.w = 1.0

        self._static_tf_broadcaster.sendTransform(tf)

        # base_link → camera_frame
        tf_cam = TransformStamped()
        tf_cam.header.stamp = self.get_clock().now().to_msg()
        tf_cam.header.frame_id = 'base_link'
        tf_cam.child_frame_id = 'camera_frame'
        tf_cam.transform.translation.x = 0.2
        tf_cam.transform.translation.y = 0.0
        tf_cam.transform.translation.z = 0.15
        tf_cam.transform.rotation.w = 1.0

        self._static_tf_broadcaster.sendTransform([tf, tf_cam])

    def destroy_node(self):
        """清理: 停止运动并断开连接。"""
        self.get_logger().info("正在清理...")
        self._sport.stop_move()
        time.sleep(0.2)
        self._sport.sit_down()
        self._sport.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2WBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，正在停止...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
