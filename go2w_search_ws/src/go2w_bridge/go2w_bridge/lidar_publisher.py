"""LiDAR 数据发布: Go2W PointCloud2 → ROS2 LaserScan。

将 Go2W mid-360 LiDAR 的 3D 点云投影到 2D 平面，
转换为 sensor_msgs/LaserScan，供 SLAM Toolbox 和 Nav2 使用。
"""

import math
import struct
import threading

import numpy as np

from sensor_msgs.msg import LaserScan


class LidarPublisher:
    """Go2W LiDAR 到 ROS2 LaserScan 转换器。"""

    # 点云过滤参数（与 web/server.py LidarSlam 一致）
    HEIGHT_MIN = -0.1
    HEIGHT_MAX = 1.5
    MAX_RANGE = 8.0
    MIN_RANGE = 0.15

    # LaserScan 参数
    ANGLE_MIN = -math.pi
    ANGLE_MAX = math.pi
    ANGLE_INCREMENT = 0.0087  # ~0.5度，约720个采样点
    RANGE_MAX = 8.0

    def __init__(self):
        self._lock = threading.Lock()
        self._latest_scan = None
        self._pointcloud_queue = []
        self._prev_scan_2d = None
        self._frame_count = 0

        # ICP 里程计状态
        self._icp_good = False
        self._icp_window = []
        self._icp_window_size = 3
        self._icp_max_iterations = 15
        self._icp_max_frame_shift = 0.05
        self._icp_min_points = 30

    def push_pointcloud(self, info: dict):
        """接收一帧 DDS PointCloud2 数据（在 DDS 回调中调用）。

        Args:
            info: {'data': bytes, 'point_step': int, 'width': int}
        """
        self._pointcloud_queue.append(info)
        if len(self._pointcloud_queue) > 5:
            self._pointcloud_queue = self._pointcloud_queue[-3:]

    def process_frame(self, imu_yaw: float):
        """处理一帧点云，生成 LaserScan 并更新 ICP 里程计。

        在 ROS2 定时器中调用（非 DDS 回调线程）。

        Returns:
            (LaserScan, icp_dx, icp_dy) 或 (None, 0.0, 0.0)
        """
        if not self._pointcloud_queue:
            return None, 0.0, 0.0

        info = self._pointcloud_queue.pop(0)

        # 解析点云
        xyz = self._parse_pointcloud(info)
        if xyz is None or len(xyz) < 30:
            return None, 0.0, 0.0

        # 高度 + 距离过滤
        mask_h = (xyz[:, 2] >= self.HEIGHT_MIN) & (xyz[:, 2] <= self.HEIGHT_MAX)
        xyz_filtered = xyz[mask_h]
        if len(xyz_filtered) < 20:
            return None, 0.0, 0.0

        dist = np.sqrt(xyz_filtered[:, 0] ** 2 + xyz_filtered[:, 1] ** 2)
        mask_d = (dist >= self.MIN_RANGE) & (dist <= self.MAX_RANGE)
        xyz_filtered = xyz_filtered[mask_d]
        if len(xyz_filtered) < 20:
            return None, 0.0, 0.0

        # 2D 投影
        scan_2d = xyz_filtered[:, :2].astype(np.float32)
        if len(scan_2d) > 500:
            idx = np.linspace(0, len(scan_2d) - 1, 500, dtype=int)
            scan_2d = scan_2d[idx]

        # 生成 LaserScan
        laser_scan = self._to_laser_scan(xyz_filtered)

        # ICP 里程计
        icp_dx, icp_dy = 0.0, 0.0
        if self._prev_scan_2d is not None and len(self._prev_scan_2d) >= 20:
            icp_dx, icp_dy = self._run_icp(self._prev_scan_2d, scan_2d)

        self._prev_scan_2d = scan_2d.copy()
        self._frame_count += 1

        with self._lock:
            self._latest_scan = laser_scan

        return laser_scan, icp_dx, icp_dy

    def _to_laser_scan(self, xyz: np.ndarray) -> LaserScan:
        """将过滤后的 3D 点云转换为 LaserScan。"""
        num_bins = int((self.ANGLE_MAX - self.ANGLE_MIN) / self.ANGLE_INCREMENT) + 1
        ranges = [float('inf')] * num_bins

        for i in range(len(xyz)):
            x, y = xyz[i, 0], xyz[i, 1]
            r = math.sqrt(x * x + y * y)
            if r < self.MIN_RANGE or r > self.MAX_RANGE:
                continue
            angle = math.atan2(y, x)
            if angle < self.ANGLE_MIN or angle > self.ANGLE_MAX:
                continue
            bin_idx = int((angle - self.ANGLE_MIN) / self.ANGLE_INCREMENT)
            if 0 <= bin_idx < num_bins and r < ranges[bin_idx]:
                ranges[bin_idx] = r

        # inf → 0.0 (ROS2 LaserScan 惯例: 超出范围用 0 或 inf)
        ranges = [r if r != float('inf') else 0.0 for r in ranges]

        msg = LaserScan()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = self.ANGLE_MIN
        msg.angle_max = self.ANGLE_MAX
        msg.angle_increment = self.ANGLE_INCREMENT
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.MIN_RANGE
        msg.range_max = self.MAX_RANGE
        msg.ranges = ranges
        return msg

    # ---- ICP 里程计（从 web/server.py LidarSlam 移植）----

    def _run_icp(self, prev: np.ndarray, curr: np.ndarray):
        """2D ICP: 对齐 curr 到 prev，返回平移 (dx, dy)。"""
        best_dx, best_dy = 0.0, 0.0
        src = curr.copy()

        for _ in range(self._icp_max_iterations):
            matched_prev, matched_src = self._find_correspondences(prev, src)
            if len(matched_src) < self._icp_min_points:
                break

            dx = float(np.median(matched_prev[:, 0] - matched_src[:, 0]))
            dy = float(np.median(matched_prev[:, 1] - matched_src[:, 1]))

            if abs(dx) > 0.15 or abs(dy) > 0.15:
                break

            src[:, 0] += dx
            src[:, 1] += dy
            best_dx += dx
            best_dy += dy

            if abs(dx) < 0.001 and abs(dy) < 0.001:
                break

        total_shift = math.sqrt(best_dx ** 2 + best_dy ** 2)
        if total_shift > self._icp_max_frame_shift:
            self._icp_good = False
            return 0.0, 0.0

        self._icp_good = total_shift > 0.002

        # 滑动窗口平滑
        self._icp_window.append((best_dx, best_dy))
        if len(self._icp_window) > self._icp_window_size:
            self._icp_window.pop(0)

        if len(self._icp_window) >= 2:
            arr = np.array(self._icp_window)
            return float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))

        return best_dx, best_dy

    @staticmethod
    def _find_correspondences(ref: np.ndarray, src: np.ndarray, max_dist: float = 0.15):
        """找 src→ref 的最近点对应（向量化）。"""
        if len(src) > 200:
            idx = np.linspace(0, len(src) - 1, 200, dtype=int)
            src_sample = src[idx]
        else:
            src_sample = src

        if len(ref) > 300:
            idx = np.linspace(0, len(ref) - 1, 300, dtype=int)
            ref_sample = ref[idx]
        else:
            ref_sample = ref

        M = len(src_sample)
        matched_src = []
        matched_ref = []

        batch = 50
        for i0 in range(0, M, batch):
            i1 = min(i0 + batch, M)
            batch_src = src_sample[i0:i1]
            diff = batch_src[:, np.newaxis, :] - ref_sample[np.newaxis, :, :]
            dists = np.sqrt(np.sum(diff ** 2, axis=2))
            min_idx = np.argmin(dists, axis=1)
            min_dist = dists[np.arange(i1 - i0), min_idx]

            mask = min_dist < max_dist
            if mask.any():
                matched_src.append(batch_src[mask])
                matched_ref.append(ref_sample[min_idx[mask]])

        if not matched_src:
            return np.empty((0, 2)), np.empty((0, 2))

        return np.vstack(matched_src), np.vstack(matched_ref)

    @staticmethod
    def _parse_pointcloud(info: dict):
        """解析 PointCloud2 数据为 (N,3) numpy 数组。"""
        try:
            raw = info['data']
            point_step = info['point_step']
            width = info['width']
            if point_step < 12 or width == 0 or len(raw) < point_step:
                return None
            n = min(width, len(raw) // point_step)
            xyz = np.empty((n, 3), dtype=np.float32)
            for i in range(n):
                xyz[i] = struct.unpack_from('<fff', raw, i * point_step)
            return xyz
        except Exception:
            return None
