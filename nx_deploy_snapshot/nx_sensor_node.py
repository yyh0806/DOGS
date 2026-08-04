"""载荷NX 传感器节点 — 读狗数据 → 发 ROS2 话题。

这是 go2w_bridge 演进的第一步:把"PC直连狗SDK"改造成"载荷NX读狗SDK并发布ROS2话题",
让笔记本通过热点DDS订阅, 不再需要网线直连狗。

职责 (只读, 不控狗):
1. 用 unitree_sdk2py 订阅狗的 DDS 话题 (rt/lowstate, rt/utlidar/cloud)
2. 转成标准 ROS2 消息发布:
   - /odom      (nav_msgs/Odometry)  - 用 IMU rpy 做航向 + 死推算位移
   - /imu       (sensor_msgs/Imu)    - IMU 原始数据
   - /scan      (sensor_msgs/LaserScan) - LiDAR 点云投影成 2D 扫描 (costmap 收双源 /scan: nx_sensor + p2l, stamp 同源 lidar time 不 drop)
   - /tf        odom→base_link

不订阅 /cmd_vel, 不控狗 (运动控制是 nx_motion_node 的职责, 阶段C再写)。

运行 (载荷NX上):
  export LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$LD_LIBRARY_PATH
  source /opt/ros/humble/setup.bash
  ros2 run go2w_bridge nx_sensor_node

或在裸 python (不进 ROS2 包):
  python3 src/go2w_bridge/go2w_bridge/nx_sensor_node.py
"""

import math
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster

# unitree SDK (装在 NX 的 ~/.local + ~/CycloneDDS)
try:
    from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_
    SDK_OK = True
except Exception as _e:
    SDK_OK = False
    _SDK_ERR = str(_e)


class NxSensorNode(Node):
    """载荷NX传感器节点: 读狗DDS → 发ROS2。"""

    def __init__(self):
        super().__init__('nx_sensor_node')

        # 参数: 连狗网卡优先用环境变量 (部署时传入), 兜底硬编码默认
        _default_iface = os.environ.get('DOG_INTERFACE', 'enxc8a362616c4c')
        self.declare_parameter('dog_interface', _default_iface)  # 连狗的USB网卡
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_scan', True)
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('wheel_radius', 0.065)  # Go2W 轮径(m), wheel odom 用, 建图后标定

        if not SDK_OK:
            self.get_logger().error(f"unitree_sdk2py 不可用: {_SDK_ERR}")
            self.get_logger().error("确保已装 cyclonedds + unitree_sdk2py, 且 LD_LIBRARY_PATH 含 ~/CycloneDDS/lib")
            return

        iface = self.get_parameter('dog_interface').get_parameter_value().string_value
        self.get_logger().info(f"连接狗主控, 网卡={iface or 'auto'} ...")

        # 初始化 DDS (连狗主控的网卡)
        self._factory = ChannelFactory()
        try:
            self._factory.Init(0, iface)
        except Exception as e:
            self.get_logger().warning(f"网卡 {iface} 失败 {e}, 自动检测")
            self._factory.Init(0, None)

        # 共享状态 (DDS回调线程写, ROS定时器线程读)
        self._lock = threading.Lock()
        self._imu = {'rpy': [0.0, 0.0, 0.0], 'gyro': [0.0, 0.0, 0.0], 'accel': [0.0, 0.0, 0.0],
                     'quat': [1.0, 0.0, 0.0, 0.0], 'count': 0, 't': 0.0}
        self._ranges = []   # LiDAR 360 距离
        self._lidar_count = 0

        # 死推算里程计状态
        # wheel odom 状态 (轮速积分 xy, yaw 用 IMU)
        self._odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'last_t': 0.0, 'v': 0.0}
        self._yaw_offset = None  # 启动时把 yaw 归零 (让初始朝向=0)
        self._wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # 订阅狗的 DDS 话题
        try:
            ChannelSubscriber('rt/lowstate', LowState_).Init(self._on_imu, 1)
            self.get_logger().info("已订阅 rt/lowstate (IMU)")
        except Exception as e:
            self.get_logger().error(f"IMU 订阅失败: {e}")
        try:
            ChannelSubscriber('rt/utlidar/cloud', PointCloud2_).Init(self._on_lidar, 1)
            self.get_logger().info("已订阅 rt/utlidar/cloud (LiDAR)")
        except Exception as e:
            self.get_logger().error(f"LiDAR 订阅失败: {e}")

        # 订阅 /livox/lidar 纯取 header.stamp (全链 lidar time 统一, B' 方案)
        # nx_sensor 的 odom→base_link / /odom / /imu 当前用 wall time, 但 costmap 的 /scan
        # (来自 p2p) 用 lidar time → 跨时钟 drop. 改用 lidar time offset 让全链同源.
        self._lidar_clock_offset_ns = 0  # lidar_stamp_ns - wall_now_ns, 动态更新
        self._lidar_stamp_ok = False
        self._lidar_sub = self.create_subscription(
            Imu, '/livox/imu', self._on_lidar_stamp, 10)

        # H2: 启动后等 /livox/lidar 才发 TF/odom/imu/scan, 避免 wall→lidar 切换跳变
        # (先发 wall time 再切 lidar time, stamp 回跳 1.5s → tf2 extrapolation into the past)
        self._startup_wall_ns = self.get_clock().now().nanoseconds
        self.declare_parameter('lidar_wait_timeout', 30.0)  # 等 lidar 最长 30s
        self._lidar_wait_timeout = self.get_parameter('lidar_wait_timeout').get_parameter_value().double_value
        self._lidar_timeout_logged = False
        # 降级锁存: 超时无 lidar 后保持 wall time, 不因 lidar 恢复而跳变 (H-NEW-2)
        self._lidar_degraded = False

        # ROS2 发布器 (QoS 用默认 RELIABLE, 和大多数订阅端一致, 避免 QoS 不兼容收不到数据)
        self._imu_pub = self.create_publisher(Imu, '/imu', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # /scan: 保留 /scan topic (costmap 收双源: nx_sensor + p2l, 时钟统一后 stamp 同源 lidar time 不 drop)
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # 定时器
        odom_rate = self.get_parameter('odom_rate').get_parameter_value().double_value
        scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        if odom_rate > 0:
            self.create_timer(1.0 / odom_rate, self._publish_odom_imu)
        if scan_rate > 0:
            self.create_timer(1.0 / scan_rate, self._publish_scan)

        self.get_logger().info("nx_sensor_node 就绪: 发 /imu /odom /scan + TF (等 /livox/lidar 才发 TF, 避免跳变)")

    # ---- DDS 回调 (unitree SDK 线程) ----
    def _on_imu(self, msg):
        try:
            with self._lock:
                self._imu['rpy'] = [float(x) for x in msg.imu_state.rpy]
                self._imu['gyro'] = [float(x) for x in msg.imu_state.gyroscope]
                self._imu['accel'] = [float(x) for x in msg.imu_state.accelerometer]
                self._imu['quat'] = [float(x) for x in msg.imu_state.quaternion]
                self._imu['count'] += 1
                self._imu['t'] = time.time()
                # wheel odom: 存 4 轮角速度 (Go2W motor_state[12-15] = 轮子电机, dq=rad/s)
                ms = msg.motor_state
                if len(ms) >= 16:
                    self._imu['wheel_dq'] = [float(ms[i].dq) for i in (12, 13, 14, 15)]
        except Exception:
            pass

    def _on_lidar(self, msg):
        try:
            # PointCloud2 → 360 距离数组 (XY平面投影)
            ranges = [10.0] * 360
            step = int(msg.point_step)
            data = bytes(msg.data)
            n = 0
            for i in range(msg.width):
                off = i * step
                if off + 8 > len(data):
                    break
                x, y = struct.unpack_from('ff', data, off)
                r = math.hypot(x, y)
                if r < 0.1 or r > 10.0:
                    continue
                angle = int((math.atan2(y, x) + math.pi) / (2 * math.pi) * 360) % 360
                if r < ranges[angle]:
                    ranges[angle] = r
                n += 1
            with self._lock:
                self._ranges = ranges
                self._lidar_count += 1
        except Exception:
            pass

    def _on_lidar_stamp(self, msg):
        """纯取 /livox/lidar header.stamp, 算 lidar_clock_offset (lidar - wall).

        EMA 平滑 offset 防 lidar 时钟单帧抖动. offset 用于 _publish_odom_imu / _publish_scan
        发 TF/odom/imu 时 stamp = wall_now + offset ≈ lidar_time, 全链同源.
        """
        if self._lidar_degraded:
            # 已降级锁存: 不更新 offset, 避免恢复时 stamp 跳变 (H-NEW-2).
            # 用户需重启 nx_sensor 恢复 lidar time 同步.
            self.get_logger().warning(
                "/livox/lidar 恢复但 nx_sensor 已降级锁存 (wall time). "
                "重启 nx_sensor 恢复 lidar time 同步.",
                throttle_duration_sec=10.0)
            return
        try:
            lidar_ns = Time.from_msg(msg.header.stamp).nanoseconds
            wall_ns = self.get_clock().now().nanoseconds
            new_offset = lidar_ns - wall_ns
            alpha = 0.3  # EMA 平滑因子
            if not self._lidar_stamp_ok:
                self._lidar_clock_offset_ns = new_offset
                self._lidar_stamp_ok = True
            else:
                self._lidar_clock_offset_ns = int(
                    alpha * new_offset + (1 - alpha) * self._lidar_clock_offset_ns)
        except Exception:
            pass

    # ---- ROS2 定时器 ----
    def _publish_odom_imu(self):
        # H2: 启动后等 /livox/lidar 才发 TF/odom/imu, 避免 wall→lidar 跳变
        if not self._lidar_stamp_ok:
            waited = (self.get_clock().now().nanoseconds - self._startup_wall_ns) / 1e9
            if waited < self._lidar_wait_timeout:
                return  # 等 /livox/lidar, 不发 TF (避免 wall→lidar 跳变)
            else:
                if not self._lidar_timeout_logged:
                    self.get_logger().error(
                        f"/livox/lidar {self._lidar_wait_timeout}s 未到, 降级 wall time "
                        f"(costmap 可能 drop /scan). 检查 livox-mid360-driver")
                    self._lidar_timeout_logged = True
                self._lidar_degraded = True  # 降级锁存: lidar 恢复也不切 (H-NEW-2)
                # 继续往下发 (用 wall_now, 降级逻辑已有)

        with self._lock:
            rpy = list(self._imu['rpy'])
            gyro = list(self._imu['gyro'])
            accel = list(self._imu['accel'])
            quat = list(self._imu['quat'])
            imu_t = self._imu['t']
            imu_count = self._imu['count']

        if imu_count == 0:
            return

        wall_now = self.get_clock().now()
        if self._lidar_stamp_ok:
            # lidar time = wall_now + offset (全链统改 B': 和 /scan/FastLIO 同源)
            now = wall_now + Duration(nanoseconds=self._lidar_clock_offset_ns)
        else:
            now = wall_now  # /livox/lidar 还没来, 降级 wall time (启动初期)
        yaw = rpy[2]
        # 启动时归零 yaw (让起始朝向 = 0)
        if self._yaw_offset is None:
            self._yaw_offset = yaw
            self.get_logger().info(f"IMU yaw 归零: 原始 {yaw:.3f} → 0")
        yaw_zero = yaw - self._yaw_offset

        # wheel odom (成熟技术: 轮速积分): 4轮平均角速度 × 轮径 = 线速度, 沿 IMU yaw 积分得 xy。
        # yaw 用 IMU(准确无漂移), xy 用轮速(室内硬地无打滑, 够 slam 用)。原 xy 占位0 已废弃。
        now_s = time.time()
        dt = now_s - self._odom['last_t'] if self._odom['last_t'] > 0 else 0.0
        self._odom['last_t'] = now_s
        self._odom['yaw'] = yaw_zero
        wheel_dq = self._imu.get('wheel_dq') or [0.0, 0.0, 0.0, 0.0]
        v = sum(wheel_dq) / 4.0 * self._wheel_radius  # 线速度 m/s (4轮同向, 平均即前进速度)
        if 0 < dt < 1.0:  # 过滤首帧/大间隔(回调断流), 避免积分跳变
            self._odom['x'] += v * math.cos(yaw_zero) * dt
            self._odom['y'] += v * math.sin(yaw_zero) * dt
        self._odom['v'] = v

        # 发布 IMU
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x = quat[1]; imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]; imu_msg.orientation.w = quat[0]
        imu_msg.orientation_covariance[0] = -1  # 标记无方向协方差 (用 rpy 重建过, 这里给原四元数)
        imu_msg.angular_velocity.x = gyro[0]; imu_msg.angular_velocity.y = gyro[1]; imu_msg.angular_velocity.z = gyro[2]
        imu_msg.linear_acceleration.x = accel[0]; imu_msg.linear_acceleration.y = accel[1]; imu_msg.linear_acceleration.z = accel[2]
        self._imu_pub.publish(imu_msg)

        # 发布 Odometry (主要传 yaw + TF, xy 暂为0)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._odom['x']
        odom.pose.pose.position.y = self._odom['y']
        cy, sy = math.cos(yaw_zero * 0.5), math.sin(yaw_zero * 0.5)
        odom.pose.pose.orientation.z = sy; odom.pose.pose.orientation.w = cy
        odom.twist.twist.angular.z = gyro[2]
        odom.twist.twist.linear.x = self._odom['v']  # wheel odom 线速度 (供 nav2/cmd_vel 反馈)
        self._odom_pub.publish(odom)

        # TF: odom → base_link
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self._odom['x']
        tf.transform.translation.y = self._odom['y']
        tf.transform.rotation.z = sy; tf.transform.rotation.w = cy
        self._tf_broadcaster.sendTransform(tf)

    def _publish_scan(self):
        # H2: 启动后等 /livox/lidar 才发 scan, 避免 wall→lidar 跳变
        if not self._lidar_stamp_ok:
            waited = (self.get_clock().now().nanoseconds - self._startup_wall_ns) / 1e9
            if waited < self._lidar_wait_timeout:
                return  # 等 /livox/lidar, 不发 scan (避免 wall→lidar 跳变)
            else:
                if not self._lidar_timeout_logged:
                    self.get_logger().error(
                        f"/livox/lidar {self._lidar_wait_timeout}s 未到, 降级 wall time "
                        f"(costmap 可能 drop /scan). 检查 livox-mid360-driver")
                    self._lidar_timeout_logged = True
                self._lidar_degraded = True  # 降级锁存: lidar 恢复也不切 (H-NEW-2)
                # 继续往下发 (用 wall_now, 降级逻辑已有)

        with self._lock:
            ranges = list(self._ranges)
        if not ranges:
            return
        wall_now = self.get_clock().now()
        if self._lidar_stamp_ok:
            # lidar time = wall_now + offset (全链统改 B': 和 /scan/FastLIO 同源)
            now = wall_now + Duration(nanoseconds=self._lidar_clock_offset_ns)
        else:
            now = wall_now  # /livox/lidar 还没来, 降级 wall time (启动初期)
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / len(ranges)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 10.0
        scan.ranges = [float(r) for r in ranges]
        self._scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = NxSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
