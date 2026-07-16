"""map→odom TF fuser — 治 critic C1 TF 拓扑硬伤 (base_link 双 parent).

背景 (memory slam-nav2-bringup-gotchas 坑3, 2026-07-07 GAN-Flow critic 升级认知):
  原 nav2_3d.launch.py TF 桥发 body→base_link (static latched), nx_sensor 发
  odom→base_link (动态 50Hz 死推算), base_link 双 parent → tf2 缓存按时间戳选最新
  (odom 压过 static), 但 {map,camera_init,body} 与 {odom,base_link} 两棵树无
  map↔odom 连接边 → costmap 查 map→base_link 必报 two-trees (拓扑必然, 非偶发).

职责:
  订阅 TF camera_init→body (FastLIO/LIVO, 世界系含 LiDAR 修正) + odom→base_link
  (nx_sensor, 轮速+IMU 死推算漂移), 算
    map→odom = T(camera_init→body) × T(body→base_link) × inv(T(odom→base_link))
  发布 (20Hz). 等价 robot_localization EKF 的最小手写版 (无双源融合, 纯 TF 代数).

假设 (建图起始时成立, 运行中靠 FastLIO/LIVO 修正维持):
  - map == camera_init (FastLIO/LIVO 起始原点 = map 原点, identity)
  - body→base_link 可配静态外参 (默认 identity = body==base_link, 向后兼容老部署):
      MID360 模组装狗上的姿态。模组**整体倾斜** (如向下俯仰 20°) 时, body(IMU) 也跟着
      斜, 不再等于水平的 base_link(底盘)。在 bringup 设 body_to_base_{x,y,z,roll,pitch,yaw}
      参数 (平移=模组在底盘上的位置, 旋转=倾斜姿态), 公式自动含 T(body→base_link) 项。
      默认全零 = identity, 公式退化回原版 (T_cb @ inv(T_ob)), 老部署不受影响。

根治后 TF 树 (单链, base_link 单 parent):
  map ──(fuser, 20Hz)──▶ odom ──(nx_sensor, 50Hz)──▶ base_link
  camera_init → body (FastLIO/LIVO 内部, fuser 消费, costmap 不查)

运行:
  ros2 run go2w_bridge map_odom_fuser
  ros2 run go2w_bridge map_odom_fuser --ros-args -p body_to_base_pitch:=-0.349  # 20° 低头补偿
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from geometry_msgs.msg import TransformStamped


def _tf_to_mat(transform):
    """geometry_msgs/Transform → 4x4 numpy (右手系)."""
    t = transform
    tx, ty, tz = t.translation.x, t.translation.y, t.translation.z
    qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    # 四元数 (x,y,z,w) → 旋转矩阵
    R = np.array([
        [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw),     s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw),     1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw),     s * (qy * qz + qx * qw),     1 - s * (qx * qx + qy * qy)],
    ])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [tx, ty, tz]
    return M


def _rpy_to_mat(roll, pitch, yaw):
    """欧拉角 (XYZ intrinsic, REP-103) → 3x3 旋转矩阵."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rz(yaw) × Ry(pitch) × Rx(roll) (ROS REP-103 约定)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _build_static_tf(x, y, z, roll, pitch, yaw):
    """xyz + rpy → 4x4 齐次矩阵 T(body→base_link)."""
    M = np.eye(4)
    M[:3, :3] = _rpy_to_mat(roll, pitch, yaw)
    M[:3, 3] = [x, y, z]
    return M


def _mat_to_tf(M):
    """4x4 numpy → (tx,ty,tz, qx,qy,qz,qw). Shepperd 方法四元数."""
    tx, ty, tz = float(M[0, 3]), float(M[1, 3]), float(M[2, 3])
    R = M[:3, :3]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    return tx, ty, tz, qx / n, qy / n, qz / n, qw / n


class MapOdomFuser(Node):
    def __init__(self):
        super().__init__('map_odom_fuser')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('fastlio_world', 'camera_init')
        self.declare_parameter('fastlio_body', 'body')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_hz', 20.0)

        # body→base_link 静态外参 (MID360 模组在底盘上的安装姿态).
        # 默认全零 = identity = body==base_link (向后兼容, 老部署不改).
        # 模组整体倾斜 (如向下俯仰 20°) 时, 在 bringup 设这些参数:
        #   -p body_to_base_pitch:=-0.349  # -20° 弧度, 模组低头补偿
        #   -p body_to_base_z:=0.15        # 模组装在底盘上方 15cm
        self.declare_parameter('body_to_base_x', 0.0)
        self.declare_parameter('body_to_base_y', 0.0)
        self.declare_parameter('body_to_base_z', 0.0)
        self.declare_parameter('body_to_base_roll', 0.0)
        self.declare_parameter('body_to_base_pitch', 0.0)
        self.declare_parameter('body_to_base_yaw', 0.0)

        self._world = self.get_parameter('world_frame').value
        self._fl_world = self.get_parameter('fastlio_world').value
        self._fl_body = self.get_parameter('fastlio_body').value
        self._odom = self.get_parameter('odom_frame').value
        self._base = self.get_parameter('base_frame').value

        self._T_body_base = _build_static_tf(
            float(self.get_parameter('body_to_base_x').value),
            float(self.get_parameter('body_to_base_y').value),
            float(self.get_parameter('body_to_base_z').value),
            float(self.get_parameter('body_to_base_roll').value),
            float(self.get_parameter('body_to_base_pitch').value),
            float(self.get_parameter('body_to_base_yaw').value),
        )
        _tilt_nonidentity = not np.allclose(self._T_body_base, np.eye(4))

        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._broad = TransformBroadcaster(self)

        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / hz, self._fuse)
        tilt_hint = " × T(body→base_link)" if _tilt_nonidentity else " (body==base_link, identity)"
        self.get_logger().info(
            f"map_odom_fuser 启动: {self._world}→{self._odom} = "
            f"T({self._fl_world}→{self._fl_body}){tilt_hint} × inv(T({self._odom}→{self._base})) @ {hz}Hz")

    def _fuse(self):
        try:
            # camera_init→body: FastLIO/LIVO 世界系 (含 LiDAR scan-matching 修正)
            cb = self._buf.lookup_transform(self._fl_world, self._fl_body, Time())
            # odom→base_link: nx_sensor 死推算 (轮速+IMU yaw 积分, 光滑但漂移)
            ob = self._buf.lookup_transform(self._odom, self._base, Time())
        except TransformException as e:
            # 启动初期 FastLIO/LIVO/nx_sensor 未就绪, 静默等 (throttle 防日志爆炸)
            self.get_logger().warning(
                f"TF lookup 未就绪: {e}", throttle_duration_sec=5.0)
            return

        # 时间戳策略 (2026-07-08 全链统改 B'):
        # FastLIO TF stamp = lidar sensor time (旧~1.5s). nx_sensor odom→base_link 也改用 lidar time
        # (订阅 /livox/lidar 取 stamp offset). 全链 map→odom→base_link + /scan 都 lidar time 同源.
        # costmap @scan.stamp 命中 (同源), @now 靠 transform_tolerance 2.0 extrapolation (1.5s < 2.0s).
        # cb 过旧 >5s 才 skip (防 FastLIO/LIVO 真卡死, 不误杀 lidar 延迟).
        cb_age = self.get_clock().now() - Time.from_msg(cb.header.stamp)
        if cb_age.nanoseconds > 5e9:  # 5s
            self.get_logger().warning(
                f"camera_init→body 过旧 {cb_age.nanoseconds / 1e9:.2f}s, FastLIO/LIVO 真卡顿? skip",
                throttle_duration_sec=5.0)
            return
        # T_map_base = T(camera_init→body) × T(body→base_link)
        #   - body==base_link (老部署): T_body_base=I, 退化回 T_cb
        #   - 模组倾斜: T_body_base 含 20° 补偿, body(倾斜 IMU)→base_link(水平底盘)
        # T_map_odom = T_map_base × inv(T_odom_base)
        T_cb = _tf_to_mat(cb.transform)
        T_ob = _tf_to_mat(ob.transform)
        # R_level: map 倾斜补偿 (2026-07-09). camera_init(map) 跟随雷达倾斜 20°,
        # nav2 costmap 在 map frame 工作倾斜 → 障碍错位 planner no path.
        # R_level=R_x(+0.349) 抵消 camera_init 倾斜, 让 map frame 水平.
        # R_level @ _T_body_base rotation 相消 (R_x(+0.349) @ R_x(-0.349)=I), TF map→base_link pitch=0°.
        _R_level = _build_static_tf(0.0, 0.0, 0.0, 0.0, 0.349, 0.0)  # +20° pitch map 补偿
        T_map_odom = _R_level @ T_cb @ self._T_body_base @ np.linalg.inv(T_ob)

        tx, ty, tz, qx, qy, qz, qw = _mat_to_tf(T_map_odom)
        msg = TransformStamped()
        msg.header.stamp = cb.header.stamp  # lidar sensor time (全链统改: fuser+nx_sensor 都用 lidar time)
        msg.header.frame_id = self._world   # map
        msg.child_frame_id = self._odom     # odom
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = tz
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self._broad.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
