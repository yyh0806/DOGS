"""map→odom TF fuser — 治 critic C1 TF 拓扑硬伤 (base_link 双 parent).

背景 (memory slam-nav2-bringup-gotchas 坑3, 2026-07-07 GAN-Flow critic 升级认知):
  原 nav2_3d.launch.py TF 桥发 body→base_link (static latched), nx_sensor 发
  odom→base_link (动态 50Hz 死推算), base_link 双 parent → tf2 缓存按时间戳选最新
  (odom 压过 static), 但 {map,camera_init,body} 与 {odom,base_link} 两棵树无
  map↔odom 连接边 → costmap 查 map→base_link 必报 two-trees (拓扑必然, 非偶发).

职责:
  订阅 TF camera_init→body (FastLIO, 世界系含 LiDAR 修正) + odom→base_link
  (nx_sensor, 轮速+IMU 死推算漂移), 算
    map→odom = T(camera_init→body) × inv(T(odom→base_link))
  发布 (20Hz). 等价 robot_localization EKF 的最小手写版 (无双源融合, 纯 TF 代数).

假设 (建图起始时成立, 运行中靠 FastLIO 修正维持):
  - map == camera_init (FastLIO 起始原点 = map 原点, identity)
  - body == base_link (雷达装狗中心, 零偏移; 若装头部需改 nav2_3d.launch.py 加 offset)

根治后 TF 树 (单链, base_link 单 parent):
  map ──(fuser, 20Hz)──▶ odom ──(nx_sensor, 50Hz)──▶ base_link
  camera_init → body (FastLIO 内部, fuser 消费, costmap 不查)

运行:
  ros2 run go2w_bridge map_odom_fuser
  (或 nav2_3d.launch.py 内 Node 启动)
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

        self._world = self.get_parameter('world_frame').value
        self._fl_world = self.get_parameter('fastlio_world').value
        self._fl_body = self.get_parameter('fastlio_body').value
        self._odom = self.get_parameter('odom_frame').value
        self._base = self.get_parameter('base_frame').value

        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._broad = TransformBroadcaster(self)

        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / hz, self._fuse)
        self.get_logger().info(
            f"map_odom_fuser 启动: {self._world}→{self._odom} = "
            f"T({self._fl_world}→{self._fl_body}) × inv(T({self._odom}→{self._base})) @ {hz}Hz")

    def _fuse(self):
        try:
            # camera_init→body: FastLIO 世界系 (含 LiDAR scan-matching 修正)
            cb = self._buf.lookup_transform(self._fl_world, self._fl_body, Time())
            # odom→base_link: nx_sensor 死推算 (轮速+IMU yaw 积分, 光滑但漂移)
            ob = self._buf.lookup_transform(self._odom, self._base, Time())
        except TransformException as e:
            # 启动初期 FastLIO/nx_sensor 未就绪, 静默等 (throttle 防日志爆炸)
            self.get_logger().warning(
                f"TF lookup 未就绪: {e}", throttle_duration_sec=5.0)
            return

        # 时间戳策略 (实测调整, 推翻 critic M1 的 cb.header.stamp 建议):
        # FastLIO TF 时间戳是 lidar sensor time, 比 wall clock 旧 ~1.5s (传感器固有延迟).
        # 若用 cb.header.stamp 发 map→odom, costmap 查 @now extrapolation (超 transform_tolerance).
        # 改用 now() (wall clock) 让 costmap @now 命中; map→odom 是 SLAM 慢变化, lidar 延迟内容差
        # 可接受. cb 过旧 >5s 才 skip (只防 FastLIO 真卡死, 不误杀 lidar 延迟).
        cb_age = self.get_clock().now() - Time.from_msg(cb.header.stamp)
        if cb_age.nanoseconds > 5e9:  # 5s
            self.get_logger().warning(
                f"camera_init→body 过旧 {cb_age.nanoseconds / 1e9:.2f}s, FastLIO 真卡顿? skip",
                throttle_duration_sec=5.0)
            return
        # T_map_base = T_camera_init_body (假设 map==camera_init, body==base_link)
        # T_map_odom = T_map_base × inv(T_odom_base)
        T_cb = _tf_to_mat(cb.transform)
        T_ob = _tf_to_mat(ob.transform)
        T_map_odom = T_cb @ np.linalg.inv(T_ob)

        tx, ty, tz, qx, qy, qz, qw = _mat_to_tf(T_map_odom)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # wall clock (costmap @now 命中)
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
