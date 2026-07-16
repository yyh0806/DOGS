"""载荷NX 传感器节点 — 读狗数据 → 发 ROS2 话题。

这是 go2w_bridge 演进的第一步:把"PC直连狗SDK"改造成"载荷NX读狗SDK并发布ROS2话题",
让笔记本通过热点DDS订阅, 不再需要网线直连狗。

职责 (只读, 不控狗):
1. 用 unitree_sdk2py 订阅狗的 DDS 话题 (rt/lowstate, rt/utlidar/cloud)
2. 转成标准 ROS2 消息发布:
   - /odom      (nav_msgs/Odometry)  - 用 IMU rpy 做航向 + 死推算位移
   - /imu       (sensor_msgs/Imu)    - IMU 原始数据
   - /scan      (sensor_msgs/LaserScan) - 狗自带 utlidar 点云经官方外参变换后投影成唯一的 Nav2 障碍源
   - /tf        odom→base_link

不订阅 /cmd_vel, 不控狗 (运动控制是 nx_motion_node 的职责, 阶段C再写)。

运行 (载荷NX上):
  export LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$LD_LIBRARY_PATH
  source /opt/ros/humble/setup.bash
  ros2 run go2w_bridge nx_sensor_node

或在裸 python (不进 ROS2 包):
  python3 src/go2w_bridge/go2w_bridge/nx_sensor_node.py
"""

import json
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
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

try:
    from .motion_protocol import build_wheel_feedback_payload
except ImportError:  # Direct script deployment on the NX compatibility path.
    from motion_protocol import build_wheel_feedback_payload

# unitree SDK (装在 NX 的 ~/.local + ~/CycloneDDS)
try:
    from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_ import SportModeState_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_
    SDK_OK = True
except Exception as _e:
    SDK_OK = False
    _SDK_ERR = str(_e)


def _transform_lidar_to_base(
        x, y, z, tx, ty, tz, roll, pitch, yaw):
    """Apply ``base_link <- utlidar_lidar`` using fixed-axis Rz*Ry*Rx RPY."""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)

    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    return (
        tx + r00 * x + r01 * y + r02 * z,
        ty + r10 * x + r11 * y + r12 * z,
        tz + r20 * x + r21 * y + r22 * z,
    )


def _classify_base_scan_point(
        x, y, z, min_height, max_height,
        self_min_x, self_max_x, self_min_y, self_max_y,
        range_max, bin_count, scan_origin_x=0.0, scan_origin_y=0.0):
    """Classify in base geometry, but measure polar data from scan origin."""
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    scan_x = x - scan_origin_x
    scan_y = y - scan_origin_y
    planar_range = math.hypot(scan_x, scan_y)
    if planar_range > range_max:
        return None
    angle = math.atan2(scan_y, scan_x)
    bin_index = int((angle + math.pi) / (2.0 * math.pi) * bin_count) % bin_count
    is_self = (
        self_min_x <= x <= self_max_x
        and self_min_y <= y <= self_max_y
    )
    # Self occlusion is an XY geometry decision, independent of the external
    # obstacle height band.  Otherwise a low/high chassis return would become
    # range_max and let clearing rays pass through the robot.
    if is_self:
        return bin_index, planar_range, True
    if z < min_height or z > max_height:
        return None
    return bin_index, planar_range, is_self


def _build_scan_ranges(
        base_points, min_height, max_height,
        self_min_x, self_max_x, self_min_y, self_max_y,
        range_max, bin_count, scan_origin_x=0.0, scan_origin_y=0.0):
    """Reduce base-frame points, preserving a nearest self hit as an invalid ray.

    A self hit is encoded as NaN instead of ``range_max`` so ObstacleLayer does
    not clear through the robot.  Bins containing only ground/out-of-height
    points become positive infinity; ObstacleLayer's ``inf_is_valid`` path
    turns only those rays into finite range-max clearing observations.
    """
    nearest_ranges = [math.inf] * bin_count
    nearest_is_self = [False] * bin_count
    for point in base_points:
        observation = _classify_base_scan_point(
            point[0], point[1], point[2], min_height, max_height,
            self_min_x, self_max_x, self_min_y, self_max_y,
            range_max, bin_count, scan_origin_x, scan_origin_y,
        )
        if observation is None:
            continue
        bin_index, planar_range, is_self = observation
        if (
                planar_range < nearest_ranges[bin_index]
                or (planar_range == nearest_ranges[bin_index] and is_self)
        ):
            nearest_ranges[bin_index] = planar_range
            nearest_is_self[bin_index] = is_self

    ranges = []
    for planar_range, is_self in zip(nearest_ranges, nearest_is_self):
        if is_self:
            ranges.append(math.nan)
        elif math.isfinite(planar_range):
            ranges.append(planar_range)
        else:
            ranges.append(math.inf)
    return ranges


def _select_sport_odom_velocity(
        sport_velocity, sport_mode, sport_error_code, sport_age,
        telemetry_timeout, max_linear_speed):
    """Select a fail-closed body velocity from Unitree high-level state.

    Raw wheel motor ``dq`` describes wheel rotation, not chassis translation.
    A Go2-W can spin its wheels while balancing or slipping, so use the SDK's
    body-velocity estimate only in a wheel-capable mode with fresh, plausible
    telemetry.
    """
    if sport_mode not in (1, 3):
        return 0.0, 0.0, "mode_locked"
    try:
        error_code = int(sport_error_code)
    except (TypeError, ValueError, OverflowError):
        return 0.0, 0.0, "sport_invalid"
    if error_code != 0:
        return 0.0, 0.0, "sport_error"
    try:
        age = float(sport_age)
        timeout = float(telemetry_timeout)
        maximum = float(max_linear_speed)
    except (TypeError, ValueError, OverflowError):
        return 0.0, 0.0, "sport_invalid"
    if (
            not math.isfinite(age) or age < 0.0
            or not math.isfinite(timeout) or timeout <= 0.0
            or age > timeout
    ):
        return 0.0, 0.0, "sport_stale"
    if not math.isfinite(maximum) or maximum <= 0.0:
        return 0.0, 0.0, "sport_invalid"
    try:
        if sport_velocity is None or len(sport_velocity) < 2:
            raise ValueError("missing sport velocity")
        vx = float(sport_velocity[0])
        vy = float(sport_velocity[1])
    except (TypeError, ValueError, OverflowError):
        return 0.0, 0.0, "sport_invalid"
    if not all(math.isfinite(value) for value in (vx, vy)):
        return 0.0, 0.0, "sport_invalid"
    if math.hypot(vx, vy) > maximum:
        return 0.0, 0.0, "sport_implausible"
    return vx, vy, "sport_velocity"


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
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('scan_rate', 10.0)
        # Retained for telemetry/calibration compatibility.  Chassis odometry
        # must not integrate raw wheel dq because wheel spin is not displacement.
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('sport_odom_timeout', 0.25)
        self.declare_parameter('sport_odom_max_speed', 1.5)
        # Unitree 官方 Go2W URDF: base_link <- utlidar_lidar (base->radar joint)。
        self.declare_parameter('lidar_frame', 'utlidar_lidar')
        # 物理雷达有 164.9° pitch，不能把该倾斜 frame 直接声明成 LaserScan。
        # 发布一个位于同一平移原点、姿态水平的虚拟 scan frame。
        self.declare_parameter('scan_frame', 'utlidar_scan')
        self.declare_parameter('lidar_to_base_x', 0.28945)
        self.declare_parameter('lidar_to_base_y', 0.0)
        self.declare_parameter('lidar_to_base_z', -0.046825)
        self.declare_parameter('lidar_to_base_roll', 0.0)
        self.declare_parameter('lidar_to_base_pitch', 2.8782)
        self.declare_parameter('lidar_to_base_yaw', 0.0)
        self.declare_parameter('obstacle_min_height', 0.05)
        self.declare_parameter('obstacle_max_height', 1.5)
        # 与 nav2_params_3d.yaml 的 Go2W footprint 一致，仅屏蔽真实自体几何。
        self.declare_parameter('self_min_x', -0.25)
        self.declare_parameter('self_max_x', 0.30)
        self.declare_parameter('self_min_y', -0.20)
        self.declare_parameter('self_max_y', 0.20)
        self.declare_parameter('raw_scan_timeout', 0.3)

        if not SDK_OK:
            self.get_logger().error(f"unitree_sdk2py 不可用: {_SDK_ERR}")
            self.get_logger().error("确保已装 cyclonedds + unitree_sdk2py, 且 LD_LIBRARY_PATH 含 ~/CycloneDDS/lib")
            return

        self._publish_imu_enabled = bool(self.get_parameter('publish_imu').value)
        self._publish_odom_enabled = bool(self.get_parameter('publish_odom').value)
        self._publish_scan_enabled = bool(self.get_parameter('publish_scan').value)
        self._publish_odom_tf_enabled = bool(self.get_parameter('publish_odom_tf').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)

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
                     'quat': [1.0, 0.0, 0.0, 0.0], 'count': 0, 't': 0.0,
                     'motor_lost': [0, 0, 0, 0]}
        self._last_feedback_sample_id = 0
        self._ranges = []   # LiDAR 360 距离
        self._lidar_count = 0
        self._last_published_lidar_count = 0
        self._last_raw_scan_monotonic = None
        self._sport_mode = None
        self._sport_progress = None
        self._sport_gait_type = None
        self._sport_velocity = None
        self._sport_position = None
        self._sport_yaw_speed = None
        self._sport_error_code = None
        self._sport_received_monotonic = None

        # 死推算里程计状态
        # SDK body velocity integrates xy; IMU supplies absolute yaw.
        self._odom = {
            'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'last_t': 0.0,
            'vx': 0.0, 'vy': 0.0, 'velocity_source': 'startup',
        }
        self._yaw_offset = None  # 启动时把 yaw 归零 (让初始朝向=0)
        self._wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self._sport_odom_timeout = self.get_parameter(
            'sport_odom_timeout').get_parameter_value().double_value
        self._sport_odom_max_speed = self.get_parameter(
            'sport_odom_max_speed').get_parameter_value().double_value
        self._lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self._scan_frame = self.get_parameter('scan_frame').get_parameter_value().string_value
        self._lidar_to_base = tuple(
            self.get_parameter(name).get_parameter_value().double_value
            for name in (
                'lidar_to_base_x', 'lidar_to_base_y', 'lidar_to_base_z',
                'lidar_to_base_roll', 'lidar_to_base_pitch', 'lidar_to_base_yaw',
            )
        )
        self._obstacle_min_height = self.get_parameter(
            'obstacle_min_height').get_parameter_value().double_value
        self._obstacle_max_height = self.get_parameter(
            'obstacle_max_height').get_parameter_value().double_value
        self._self_box = tuple(
            self.get_parameter(name).get_parameter_value().double_value
            for name in ('self_min_x', 'self_max_x', 'self_min_y', 'self_max_y')
        )
        self._raw_scan_timeout = self.get_parameter(
            'raw_scan_timeout').get_parameter_value().double_value
        self._scan_origin = (self._lidar_to_base[0], self._lidar_to_base[1])

        # 订阅狗的 DDS 话题
        try:
            ChannelSubscriber('rt/lowstate', LowState_).Init(self._on_imu, 1)
            self.get_logger().info("已订阅 rt/lowstate (IMU)")
        except Exception as e:
            self.get_logger().error(f"IMU 订阅失败: {e}")
        try:
            ChannelSubscriber(
                'rt/lf/sportmodestate', SportModeState_).Init(
                    self._on_sport_mode, 1)
            self.get_logger().info("subscribed rt/lf/sportmodestate")
        except Exception as e:
            self.get_logger().error(f"sport mode subscription failed: {e}")
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
        self._imu_pub = self.create_publisher(Imu, '/imu', 10) if self._publish_imu_enabled else None
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10) if self._publish_odom_enabled else None
        self._drive_feedback_pub = self.create_publisher(String, '/wheel_feedback', 10)
        # /scan 是 Nav2 两个 costmap 的唯一障碍源；MID360 只供 FAST_LIO。
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10) if self._publish_scan_enabled else None
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        if self._publish_scan_enabled:
            self._publish_scan_frame_tf()

        # 定时器
        odom_rate = self.get_parameter('odom_rate').get_parameter_value().double_value
        scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        if odom_rate > 0 and (self._publish_imu_enabled or self._publish_odom_enabled):
            self._odom_timer = self.create_timer(1.0 / odom_rate, self._publish_odom_imu)
        if scan_rate > 0 and self._publish_scan_enabled:
            self._scan_timer = self.create_timer(1.0 / scan_rate, self._publish_scan)
        self._drive_feedback_timer = self.create_timer(0.05, self._publish_drive_feedback)

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
                    self._imu['motor_lost'] = [
                        int(getattr(ms[i], 'lost', 0)) for i in (12, 13, 14, 15)]
                self._imu['battery_soc'] = int(msg.bms_state.soc)
                self._imu['bms_status'] = int(msg.bms_state.status)
        except Exception:
            pass

    def _on_sport_mode(self, msg):
        try:
            mode = int(msg.mode)
            progress = float(msg.progress)
            gait_type = int(msg.gait_type)
            error_code = int(msg.error_code)
            velocity = tuple(float(value) for value in msg.velocity)
            position = tuple(float(value) for value in msg.position)
            yaw_speed = float(msg.yaw_speed)
            if (not 0 <= mode <= 255
                    or not math.isfinite(progress)
                    or not 0.0 <= progress <= 1.0
                    or not 0 <= gait_type <= 255
                    or not 0 <= error_code <= 0xFFFFFFFF
                    or len(velocity) != 3
                    or len(position) != 3
                    or not all(math.isfinite(value) for value in velocity)
                    or not all(math.isfinite(value) for value in position)
                    or not math.isfinite(yaw_speed)):
                return
            received_monotonic = time.monotonic()
            with self._lock:
                self._sport_mode = mode
                self._sport_progress = progress
                self._sport_gait_type = gait_type
                self._sport_velocity = velocity
                self._sport_position = position
                self._sport_yaw_speed = yaw_speed
                self._sport_error_code = error_code
                self._sport_received_monotonic = received_monotonic
        except (AttributeError, TypeError, ValueError, OverflowError):
            pass

    def _publish_drive_feedback(self):
        with self._lock:
            sample_id = self._imu['count']
            source_stamp = self._imu['t']
            wheel_dq = self._imu.get('wheel_dq')
            rpy = list(self._imu.get('rpy') or ())
            motor_lost = list(self._imu.get('motor_lost') or ())
            battery_soc = self._imu.get('battery_soc')
            bms_status = self._imu.get('bms_status')
            sport_mode = self._sport_mode
            sport_progress = self._sport_progress
            gait_type = self._sport_gait_type
            sport_velocity = self._sport_velocity
            sport_position = self._sport_position
            sport_yaw_speed = self._sport_yaw_speed
            sport_error_code = self._sport_error_code
            odom_velocity_source = self._odom['velocity_source']
        if sample_id <= self._last_feedback_sample_id:
            return
        if (wheel_dq is None or battery_soc is None or bms_status is None
                or sport_mode is None or sport_error_code is None
                or len(rpy) < 2 or len(motor_lost) != 4):
            return
        payload = build_wheel_feedback_payload(
            sample_id=sample_id,
            source_stamp=source_stamp,
            wheel_dq=wheel_dq,
            battery_soc=battery_soc,
            bms_status=bms_status,
            sport_mode=sport_mode,
            sport_error_code=sport_error_code,
            roll=rpy[0],
            pitch=rpy[1],
            motor_lost=motor_lost,
            extras={
                'sport_progress': sport_progress,
                'gait_type': gait_type,
                'sport_velocity': (
                    [round(value, 5) for value in sport_velocity]
                    if sport_velocity is not None else None),
                'sport_position': (
                    [round(value, 5) for value in sport_position]
                    if sport_position is not None else None),
                'sport_yaw_speed': (
                    round(float(sport_yaw_speed), 5)
                    if sport_yaw_speed is not None else None),
                'odom_velocity_source': odom_velocity_source,
            },
        )
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._drive_feedback_pub.publish(msg)
        self._last_feedback_sample_id = sample_id

    def _on_lidar(self, msg):
        try:
            # Unitree DDS cloud 必须是官方 ``utlidar_lidar`` frame。直接把倾斜雷达
            # 的 XY 当水平面会把 z≈0.5m 的地面回波误判成近障碍。
            frame_id = str(msg.header.frame_id)
            if frame_id != self._lidar_frame:
                self.get_logger().warning(
                    f"忽略 rt/utlidar/cloud: frame_id={frame_id!r}, "
                    f"期望 {self._lidar_frame!r}",
                    throttle_duration_sec=5.0)
                return

            step = int(msg.point_step)
            if step < 12:
                self.get_logger().warning(
                    f"忽略 rt/utlidar/cloud: point_step={step} 无法读取 x/y/z",
                    throttle_duration_sec=5.0)
                return
            data = bytes(msg.data)
            height = max(1, int(getattr(msg, 'height', 1)))
            point_count = min(int(msg.width) * height, len(data) // step)
            unpack_format = '>fff' if bool(getattr(msg, 'is_bigendian', False)) else '<fff'
            base_points = []
            parsed_points = 0
            for i in range(point_count):
                off = i * step
                x, y, z = struct.unpack_from(unpack_format, data, off)
                if not all(math.isfinite(value) for value in (x, y, z)):
                    continue
                parsed_points += 1
                base_points.append(_transform_lidar_to_base(
                    x, y, z, *self._lidar_to_base))

            if parsed_points == 0:
                self.get_logger().warning(
                    "rt/utlidar/cloud 没有可解析的有限 x/y/z 点",
                    throttle_duration_sec=5.0)
                return

            ranges = _build_scan_ranges(
                base_points,
                self._obstacle_min_height,
                self._obstacle_max_height,
                *self._self_box,
                10.0,
                360,
                *self._scan_origin,
            )
            received_monotonic = time.monotonic()
            with self._lock:
                self._ranges = ranges
                self._lidar_count += 1
                self._last_raw_scan_monotonic = received_monotonic
        except Exception as exc:
            self.get_logger().warning(
                f"解析 rt/utlidar/cloud 失败: {exc}",
                throttle_duration_sec=5.0)

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
    def _publish_scan_frame_tf(self):
        """Publish base_link→horizontal virtual scan origin (identity rotation)."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = self._scan_frame
        transform.transform.translation.x = self._lidar_to_base[0]
        transform.transform.translation.y = self._lidar_to_base[1]
        transform.transform.translation.z = self._lidar_to_base[2]
        transform.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(transform)

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
            sport_velocity = self._sport_velocity
            sport_mode = self._sport_mode
            sport_error_code = self._sport_error_code
            sport_received_monotonic = self._sport_received_monotonic

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

        # Use Unitree's reported body velocity, not raw motor dq.  Motor speed
        # remains useful for safety feedback but cannot distinguish translation
        # from balancing wheel spin or slip.
        now_monotonic = time.monotonic()
        sport_age = (
            math.inf if sport_received_monotonic is None
            else now_monotonic - sport_received_monotonic
        )
        vx_body, vy_body, velocity_source = _select_sport_odom_velocity(
            sport_velocity,
            sport_mode,
            sport_error_code,
            sport_age,
            self._sport_odom_timeout,
            self._sport_odom_max_speed,
        )
        dt = (
            now_monotonic - self._odom['last_t']
            if self._odom['last_t'] > 0 else 0.0
        )
        self._odom['last_t'] = now_monotonic
        self._odom['yaw'] = yaw_zero
        if 0 < dt < 1.0:  # 过滤首帧/大间隔(回调断流), 避免积分跳变
            cyaw = math.cos(yaw_zero)
            syaw = math.sin(yaw_zero)
            self._odom['x'] += (vx_body * cyaw - vy_body * syaw) * dt
            self._odom['y'] += (vx_body * syaw + vy_body * cyaw) * dt
        self._odom['vx'] = vx_body
        self._odom['vy'] = vy_body
        self._odom['velocity_source'] = velocity_source

        # 发布 IMU
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x = quat[1]; imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]; imu_msg.orientation.w = quat[0]
        imu_msg.orientation_covariance[0] = -1  # 标记无方向协方差 (用 rpy 重建过, 这里给原四元数)
        imu_msg.angular_velocity.x = gyro[0]; imu_msg.angular_velocity.y = gyro[1]; imu_msg.angular_velocity.z = gyro[2]
        imu_msg.linear_acceleration.x = accel[0]; imu_msg.linear_acceleration.y = accel[1]; imu_msg.linear_acceleration.z = accel[2]
        if self._imu_pub is not None:
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
        odom.twist.twist.linear.x = self._odom['vx']
        odom.twist.twist.linear.y = self._odom['vy']
        if self._odom_pub is not None:
            self._odom_pub.publish(odom)

        # TF: odom → base_link
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self._odom['x']
        tf.transform.translation.y = self._odom['y']
        tf.transform.rotation.z = sy; tf.transform.rotation.w = cy
        if self._publish_odom_tf_enabled:
            self._tf_broadcaster.sendTransform(tf)

    def _publish_scan(self):
        # 不得用新时间戳重复发布缓存点云，否则 DDS 断流时 costmap 会继续把旧射线
        # 当作实时清障数据。停止发布后 expected_update_rate 会使障碍层变为 non-current。
        with self._lock:
            ranges = list(self._ranges)
            lidar_count = self._lidar_count
            last_raw_scan_monotonic = self._last_raw_scan_monotonic
        if lidar_count == self._last_published_lidar_count:
            return
        if last_raw_scan_monotonic is None:
            self.get_logger().warning(
                "尚未收到有效 rt/utlidar/cloud，暂停 /scan",
                throttle_duration_sec=5.0)
            return
        raw_scan_age = time.monotonic() - last_raw_scan_monotonic
        if raw_scan_age > self._raw_scan_timeout:
            self.get_logger().warning(
                f"rt/utlidar/cloud 已断流 {raw_scan_age:.2f}s，暂停 /scan",
                throttle_duration_sec=5.0)
            return

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
        scan.header.frame_id = self._scan_frame
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / len(ranges)
        scan.angle_max = scan.angle_min + (len(ranges) - 1) * scan.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 10.0
        scan.ranges = [float(r) for r in ranges]
        self._scan_pub.publish(scan)
        # Mark only after a successful publication.  If a newer DDS callback
        # arrived meanwhile its larger generation remains pending for the next
        # timer tick.
        self._last_published_lidar_count = lidar_count


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
