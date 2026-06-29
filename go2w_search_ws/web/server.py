#!/usr/bin/env python3
"""
❌ DEPRECATED — 不要在此文件上开发新功能。
本文件是 web/panel.py 的前身(老单体后端), 已被 panel.py 取代, 不在运行链路中。
保留仅供历史参考。新代码请用 web/panel.py。详见 docs/PROJECT_STRUCTURE.md。

Go2W 搜索系统 Web 后端 (纯 Python 版)
======================================
用 websockets + http.server 替代 FastAPI/uvicorn，
避免 CycloneDDS 的 ctypes 回调与 uvicorn C 扩展冲突导致 segfault。

集成：
  - LiDAR SLAM (ICP 定位 + 栅格建图)
  - YOLO 目标检测
  - Audio-Interaction 语音理解
  - Qwen2.5-VL 视觉定位
  - 目标跟踪状态机
"""

import asyncio
import base64
import faulthandler
import json
import math
import os
import sys
import time
import threading
import logging
import traceback
import struct
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from typing import List, Optional
from urllib.parse import urlparse, parse_qs
from collections import deque

import cv2
import numpy as np

# AI 模块（平台无关，可迁移到 Orin NX）
sys.path.insert(0, str(Path(__file__).parent.parent))
from ai.config import DEVICE, CUDA_AVAILABLE, memory_summary
from ai.detector import Detector
from ai.voice import VoiceEngine
from ai.vlm import VLMEngine
from ai.tracker import TargetTracker
from audio.capture import AudioCapture

faulthandler.enable()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("go2w_web")

# ============================================================================
# LiDAR SLAM — ICP 定位 + 累积栅格建图
# ============================================================================

class LidarSlam:
    """LiDAR SLAM：用 ICP 做帧间匹配算位移，累积 2D 栅格地图。

    定位原理：
      - IMU 提供准确的 yaw（500Hz，积分漂移小）
      - 每帧 LiDAR 点云做 2D ICP 匹配，算出 (dx, dy) 平移
      - yaw 用 IMU，平移用 ICP，融合为可靠的里程计

    建图原理：
      - 每帧点云根据定位结果投影到世界坐标
      - 写入固定大小的 2D 栅格（0.05m/格）
      - 栅格值 = 障碍物置信度，多次观测会叠加
    """

    # 点云过滤
    HEIGHT_MIN = -0.1       # 低于此高度视为地面
    HEIGHT_MAX = 1.5        # 高于此高度丢弃
    MAX_RANGE = 8.0         # 最大有效距离
    MIN_RANGE = 0.15        # 最小有效距离（机器人本体）

    # 栅格地图
    GRID_W = 400            # 栅格宽度（格数）
    GRID_H = 400            # 栅格高度
    CELL_SIZE = 0.05        # 每格 0.05m → 总覆盖 20m × 20m
    GRID_ORIGIN_X = -10.0   # 栅格左下角世界坐标 X
    GRID_ORIGIN_Y = -10.0   # 栅格左下角世界坐标 Y

    # ICP 参数
    ICP_MAX_CORRESPONDENCE = 0.15  # 最大对应点距离（米）
    ICP_MAX_ITERATIONS = 15
    ICP_MAX_FRAME_SHIFT = 0.05     # 单帧最大允许位移（米）
    ICP_MIN_POINTS = 30            # 最少匹配点数

    def __init__(self):
        self._lock = threading.Lock()
        # 栅格地图
        self._grid = np.zeros((self.GRID_H, self.GRID_W), dtype=np.int16)
        # 累积轨迹
        self._trail = [(0.0, 0.0)]
        # ICP 前一帧（2D 点，局部坐标）
        self._prev_scan = None
        # 当前帧的实时扫描点（局部坐标，给前端用）
        self._latest_scan_local = np.empty((0, 2), dtype=np.float32)
        # 上次 ICP 成功标志
        self._icp_good = False
        # 帧计数
        self._frame_count = 0
        # 滑动窗口平滑（存最近 N 帧 ICP 原始结果，取中位数）
        self._icp_window = []
        self._icp_window_size = 3

    def clear(self):
        with self._lock:
            self._grid.fill(0)
            self._trail = [(0.0, 0.0)]
            self._prev_scan = None
            self._latest_scan_local = np.empty((0, 2), dtype=np.float32)
            self._icp_good = False
            self._frame_count = 0

    def update(self, info: dict, imu_yaw: float):
        """处理一帧 LiDAR 数据：ICP 定位 + 栅格建图。

        Args:
            info: 深拷贝的 PointCloud2 数据 {'data', 'point_step', 'width'}
            imu_yaw: 当前 IMU yaw（弧度，已减去偏移）
        Returns:
            (dx, dy, dyaw)  ICP 算出的本帧位移，如果失败返回 (0,0,0)
        """
        try:
            # 1. 解析点云
            xyz = self._parse_pointcloud(info)
            if xyz is None or len(xyz) < 30:
                return 0.0, 0.0, 0.0

            # 2. 高度 + 距离过滤
            mask_h = (xyz[:, 2] >= self.HEIGHT_MIN) & (xyz[:, 2] <= self.HEIGHT_MAX)
            xyz = xyz[mask_h]
            if len(xyz) < 20:
                return 0.0, 0.0, 0.0

            dist = np.sqrt(xyz[:, 0] ** 2 + xyz[:, 1] ** 2)
            mask_d = (dist >= self.MIN_RANGE) & (dist <= self.MAX_RANGE)
            xyz = xyz[mask_d]
            if len(xyz) < 20:
                return 0.0, 0.0, 0.0

            # 3. 投影到 2D (x, y)，降采样到 ~500 点加速 ICP
            scan_2d = xyz[:, :2].astype(np.float32)
            if len(scan_2d) > 500:
                idx = np.linspace(0, len(scan_2d) - 1, 500, dtype=int)
                scan_2d = scan_2d[idx]

            icp_dx, icp_dy = 0.0, 0.0

            # 4. ICP 帧间匹配（有前一帧时才做）
            if self._prev_scan is not None and len(self._prev_scan) >= 20:
                icp_dx, icp_dy = self._run_icp(self._prev_scan, scan_2d)

            # 5. 更新状态
            self._frame_count += 1

            # 保存当前帧给下一次 ICP
            self._prev_scan = scan_2d.copy()

            # 保存实时扫描点（局部坐标，前端画射线用）
            with self._lock:
                self._latest_scan_local = scan_2d

            return icp_dx, icp_dy, 0.0  # yaw 变化由 IMU 负责

        except Exception as e:
            logger.debug(f"LiDAR SLAM 更新失败: {e}")
            return 0.0, 0.0, 0.0

    def update_grid(self, info: dict, world_x: float, world_y: float, world_yaw: float):
        """将一帧点云投影到世界坐标并写入栅格地图。
        在 update() 之后调用，此时定位已完成。
        """
        try:
            xyz = self._parse_pointcloud(info)
            if xyz is None or len(xyz) < 10:
                return

            mask_h = (xyz[:, 2] >= self.HEIGHT_MIN) & (xyz[:, 2] <= self.HEIGHT_MAX)
            xyz = xyz[mask_h]
            if len(xyz) == 0:
                return
            dist = np.sqrt(xyz[:, 0] ** 2 + xyz[:, 1] ** 2)
            mask_d = (dist >= self.MIN_RANGE) & (dist <= self.MAX_RANGE)
            xyz = xyz[mask_d]
            if len(xyz) == 0:
                return

            cos_y = math.cos(world_yaw)
            sin_y = math.sin(world_yaw)

            with self._lock:
                for i in range(len(xyz)):
                    px, py = xyz[i, 0], xyz[i, 1]
                    # 局部 → 世界
                    wx = cos_y * px - sin_y * py + world_x
                    wy = sin_y * px + cos_y * py + world_y
                    # 世界 → 栅格
                    gx = int((wx - self.GRID_ORIGIN_X) / self.CELL_SIZE)
                    gy = int((wy - self.GRID_ORIGIN_Y) / self.CELL_SIZE)
                    if 0 <= gx < self.GRID_W and 0 <= gy < self.GRID_H:
                        self._grid[gy, gx] = min(self._grid[gy, gx] + 2, 200)

                # 轨迹
                if len(self._trail) == 0:
                    self._trail.append((round(world_x, 3), round(world_y, 3)))
                else:
                    lx, ly = self._trail[-1]
                    if math.hypot(world_x - lx, world_y - ly) > 0.05:
                        self._trail.append((round(world_x, 3), round(world_y, 3)))

                # 衰减（每 100 帧衰减一次，避免旧障碍物永远存在）
                if self._frame_count % 100 == 0:
                    self._grid = np.clip(
                        self._grid.astype(np.int32) - 1, 0, 200
                    ).astype(np.int16)

        except Exception as e:
            logger.debug(f"栅格建图失败: {e}")

    def get_map_data(self, threshold=8, max_points=1200):
        """返回栅格地图中置信度 > threshold 的世界坐标点列表。
        用于前端渲染累积地图。返回 [(wx, wy), ...]
        """
        with self._lock:
            ys, xs = np.where(self._grid > threshold)
            if len(xs) == 0:
                return []
            # 栅格 → 世界坐标
            wxs = xs * self.CELL_SIZE + self.GRID_ORIGIN_X
            wys = ys * self.CELL_SIZE + self.GRID_ORIGIN_Y
            # 限制数量
            if len(wxs) > max_points:
                idx = np.random.choice(len(wxs), max_points, replace=False)
                wxs, wys = wxs[idx], wys[idx]
            return [(round(float(wx), 2), round(float(wy), 2))
                    for wx, wy in zip(wxs, wys)]

    def get_scan_local(self, max_points=500):
        """返回当前帧的局部 2D 扫描点，前端画射线用。"""
        with self._lock:
            pts = self._latest_scan_local
            if len(pts) == 0:
                return []
            if len(pts) > max_points:
                idx = np.linspace(0, len(pts) - 1, max_points, dtype=int)
                pts = pts[idx]
            return [(round(float(p[0]), 3), round(float(p[1]), 3)) for p in pts]

    @property
    def trail(self):
        with self._lock:
            return list(self._trail)

    @property
    def icp_good(self):
        return self._icp_good

    # ------------------------------------------------------------------
    # ICP 实现（2D，纯 numpy，无需额外依赖）
    # ------------------------------------------------------------------

    def _run_icp(self, prev: np.ndarray, curr: np.ndarray):
        """2D ICP：对齐 curr 到 prev，返回 (dx, dy) 平移量。

        用最近点匹配 + 中位数鲁棒估计。
        由于 yaw 由 IMU 提供，这里只估计平移 (dx, dy)。
        """
        best_dx, best_dy = 0.0, 0.0
        src = curr.copy()

        for iteration in range(self.ICP_MAX_ITERATIONS):
            matched_prev, matched_src = self._find_correspondences(prev, src)
            if len(matched_src) < self.ICP_MIN_POINTS:
                break

            # 用中位数代替均值，抗离群点
            dx = float(np.median(matched_prev[:, 0] - matched_src[:, 0]))
            dy = float(np.median(matched_prev[:, 1] - matched_src[:, 1]))

            # 单次迭代位移太大说明匹配异常，直接丢弃
            if abs(dx) > 0.15 or abs(dy) > 0.15:
                break

            # 应用变换
            src[:, 0] += dx
            src[:, 1] += dy
            best_dx += dx
            best_dy += dy

            # 收敛判断
            if abs(dx) < 0.001 and abs(dy) < 0.001:
                break

        # 总位移合理性检查
        total_shift = math.sqrt(best_dx ** 2 + best_dy ** 2)

        # 单帧位移超过阈值 → 不可信
        if total_shift > self.ICP_MAX_FRAME_SHIFT:
            self._icp_good = False
            return 0.0, 0.0

        self._icp_good = total_shift > 0.002  # 有实际运动
        return best_dx, best_dy

    @staticmethod
    def _find_correspondences(ref: np.ndarray, src: np.ndarray, max_dist=0.2):
        """找 src→ref 的最近点对应。用向量化距离计算。"""
        # 对 src 降采样（太多点匹配太慢）
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

        # 计算距离矩阵 (M, N)
        # M = len(src_sample), N = len(ref_sample)
        # 用分块计算避免内存爆炸
        M = len(src_sample)
        N = len(ref_sample)

        matched_src = []
        matched_ref = []

        # 按 50 个一批处理
        batch = 50
        for i0 in range(0, M, batch):
            i1 = min(i0 + batch, M)
            batch_src = src_sample[i0:i1]  # (B, 2)
            # (B, 1, 2) - (1, N, 2) → (B, N, 2) → (B, N)
            diff = batch_src[:, np.newaxis, :] - ref_sample[np.newaxis, :, :]
            dists = np.sqrt(np.sum(diff ** 2, axis=2))  # (B, N)
            min_idx = np.argmin(dists, axis=1)  # (B,)
            min_dist = dists[np.arange(i1 - i0), min_idx]  # (B,)

            mask = min_dist < max_dist
            if mask.any():
                matched_src.append(batch_src[mask])
                matched_ref.append(ref_sample[min_idx[mask]])

        if not matched_src:
            return np.empty((0, 2)), np.empty((0, 2))

        return np.vstack(matched_src), np.vstack(matched_ref)

    @staticmethod
    def _parse_pointcloud(info: dict):
        """解析已深拷贝的 PointCloud2 数据为 (N,3) numpy 数组。"""
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


# ============================================================================
# SLAM Bridge 数据接收器
# ============================================================================

class SlamBridgeState:
    """接收 Go2W 上 slam_toolbox bridge 发来的地图和位姿数据。"""

    def __init__(self):
        self._lock = threading.Lock()
        # 位姿（来自 slam_toolbox 的 map→odom TF）
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._pose_time = 0.0
        # 地图（来自 slam_toolbox 的 /map OccupancyGrid）
        self._map_occupied = []   # [(wx, wy), ...]
        self._map_free = []       # [(wx, wy), ...]
        self._map_info = {}       # {width, height, resolution, origin_x, origin_y}
        self._map_time = 0.0
        self._active = False

    def update_pose(self, x, y, yaw):
        with self._lock:
            self._pose_x = x
            self._pose_y = y
            self._pose_yaw = yaw
            self._pose_time = time.time()
            self._active = True

    def update_map(self, occupied, free, info):
        with self._lock:
            self._map_occupied = occupied
            self._map_free = free
            self._map_info = info
            self._map_time = time.time()

    @property
    def active(self):
        with self._lock:
            return self._active and (time.time() - self._pose_time < 5.0)

    @property
    def pose(self):
        with self._lock:
            return self._pose_x, self._pose_y, self._pose_yaw

    @property
    def map_data(self):
        """返回地图点列表（给前端渲染用），格式兼容 LidarSlam.get_map_data()。"""
        with self._lock:
            return list(self._map_occupied)

    @property
    def map_info(self):
        with self._lock:
            return dict(self._map_info)


slam_bridge = SlamBridgeState()


# ============================================================================
# SDK 封装
# ============================================================================

class RobotSDK:
    """Go2W SDK 封装，线程安全，集成 IMU + LiDAR SLAM。"""

    def __init__(self, interface: str = "enp65s0"):
        self._interface = interface
        self._sport = None
        self._video = None
        self._connected = False
        self._lock = threading.Lock()
        self._dds_inited = False
        # IMU
        self._imu_yaw = 0.0
        self._imu_yaw_offset = None
        self._imu_rpy = [0.0, 0.0, 0.0]
        # LiDAR SLAM（ICP 定位 + 栅格建图，作为 fallback）
        self._slam = LidarSlam()
        self._lidar_queue = []
        self._lidar_thread = None
        # 里程计（由 LiDAR ICP 驱动，fallback）
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def position(self):
        """优先使用 slam_toolbox bridge 数据，否则 fallback 到 IMU+ICP。"""
        if slam_bridge.active:
            return slam_bridge.pose
        with self._lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    @property
    def imu_yaw(self):
        with self._lock:
            return self._imu_yaw

    @property
    def slam(self):
        return self._slam

    def reset_odom(self):
        with self._lock:
            self._odom_x = 0.0
            self._odom_y = 0.0
            self._odom_yaw = 0.0
            self._imu_yaw_offset = None
            self._slam.clear()

    def connect(self) -> bool:
        try:
            from unitree_sdk2py.core.channel import ChannelFactory
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.go2.video.video_client import VideoClient

            logger.info(f"初始化 DDS (网卡: {self._interface})...")
            factory = ChannelFactory()
            factory.Init(0, self._interface)
            self._dds_inited = True

            self._sport = SportClient()
            self._sport.SetTimeout(10.0)
            self._sport.Init()

            self._video = VideoClient()
            self._video.SetTimeout(10.0)
            self._video.Init()

            self._subscribe_imu(factory)
            time.sleep(0.3)
            self._subscribe_lidar(factory)

            self._sport.BalanceStand()
            self.reset_odom()
            time.sleep(1.0)
            with self._lock:
                self._imu_yaw_offset = self._imu_yaw

            self._connected = True
            logger.info("Go2W 连接成功! (IMU + LiDAR 已订阅)")
            return True
        except Exception as e:
            logger.error(f"Go2W 连接失败: {e}")
            self._connected = False
            return False

    def _subscribe_imu(self, factory):
        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_

            def on_imu_msg(msg):
                with self._lock:
                    self._imu_rpy = list(msg.imu_state.rpy)
                    self._imu_yaw = float(msg.imu_state.rpy[2])
                    if self._imu_yaw_offset is not None:
                        self._odom_yaw = self._imu_yaw - self._imu_yaw_offset

            ch = factory.CreateRecvChannel('rt/lowstate', LowState_)
            ch.SetReader(handler=on_imu_msg)
            logger.info("IMU 订阅成功 (rt/lowstate)")
        except Exception as e:
            logger.warning(f"IMU 订阅失败: {e}")

    def _subscribe_lidar(self, factory):
        try:
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_

            def on_lidar_msg(msg):
                try:
                    data_copy = bytes(msg.data)
                    info = {
                        'data': data_copy,
                        'point_step': int(msg.point_step),
                        'width': int(msg.width),
                    }
                    self._lidar_queue.append(info)
                    if len(self._lidar_queue) > 5:
                        self._lidar_queue = self._lidar_queue[-3:]
                except Exception:
                    pass

            ch = factory.CreateRecvChannel('rt/utlidar/cloud', PointCloud2_)
            ch.SetReader(handler=on_lidar_msg)
            logger.info("LiDAR 订阅成功 (rt/utlidar/cloud)")

            self._lidar_thread = threading.Thread(target=self._lidar_worker, daemon=True)
            self._lidar_thread.start()
        except Exception as e:
            logger.warning(f"LiDAR 订阅失败: {e}")

    def _lidar_worker(self):
        """LiDAR 后台线程：SLAM bridge 活跃时跳过 ICP 以减少 DDS 冲突。"""
        while self._dds_inited:
            try:
                # SLAM bridge 活跃时，清空队列但不做 ICP，避免 DDS ctypes 冲突
                if slam_bridge.active:
                    if self._lidar_queue:
                        self._lidar_queue.clear()
                    time.sleep(0.1)
                    continue

                if not self._lidar_queue:
                    time.sleep(0.02)
                    continue
                info = self._lidar_queue.pop(0)

                with self._lock:
                    odom_yaw = self._odom_yaw

                icp_dx, icp_dy, _ = self._slam.update(info, odom_yaw)

                with self._lock:
                    self._odom_x += icp_dx
                    self._odom_y += icp_dy
                    cur_x, cur_y, cur_yaw = self._odom_x, self._odom_y, self._odom_yaw

                self._slam.update_grid(info, cur_x, cur_y, cur_yaw)

            except Exception as e:
                logger.debug(f"LiDAR worker error: {e}")
                time.sleep(0.05)

    def balance_stand(self) -> int:
        if not self._connected:
            return -1
        with self._lock:
            return self._sport.BalanceStand()

    def stop(self) -> int:
        if not self._connected:
            return -1
        with self._lock:
            return self._sport.Move(0, 0, 0)

    def move(self, vx: float, vy: float, vyaw: float) -> int:
        if not self._connected:
            return -1
        with self._lock:
            return self._sport.Move(vx, vy, vyaw)

    def get_frame(self) -> Optional[np.ndarray]:
        if not self._connected or self._video is None:
            return None
        try:
            code, data = self._video.GetImageSample()
            if code == 0 and data and len(data) > 0:
                raw = bytes(data)
                img_array = np.frombuffer(raw, dtype=np.uint8)
                return cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        except Exception as e:
            logger.debug(f"获取图像失败: {e}")
        return None


# ============================================================================
# Detector 已移至 ai/detector.py，此处保留向后兼容引用
# ============================================================================


# ============================================================================
# 搜索任务
# ============================================================================

def plan_lawnmower(width: float, height: float, spacing: float = 1.5) -> List[dict]:
    waypoints = []
    num_rows = max(1, int(math.ceil(height / spacing)))
    for row in range(num_rows + 1):
        y = min(row * spacing, height)
        if row % 2 == 0:
            waypoints.append({"x": 0.0, "y": y})
            waypoints.append({"x": width, "y": y})
        else:
            waypoints.append({"x": width, "y": y})
            waypoints.append({"x": 0.0, "y": y})
    return waypoints


class SearchMission:
    """搜索任务状态机。"""

    def __init__(self, robot: RobotSDK, detector: Detector):
        self._robot = robot
        self._detector = detector
        self._running = False
        self._thread = None
        self._progress = {}
        self._detections = []
        self._lock = threading.Lock()
        self._stop_flag = False
        self._target_classes = None
        self._wp_idx = 0

    @property
    def running(self) -> bool:
        return self._running

    @property
    def progress(self) -> dict:
        with self._lock:
            return dict(self._progress)

    @property
    def detections(self) -> list:
        with self._lock:
            return list(self._detections)

    def start(self, width, height, spacing, target_classes, speed):
        if self._running:
            return False
        self._stop_flag = False
        self._detections = []
        self._progress = {"status": "starting", "current": 0, "total": 0}
        self._thread = threading.Thread(target=self._run, args=(
            width, height, spacing, target_classes, speed
        ), daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop_flag = True
        self._robot.stop()

    def _update_progress(self, **kwargs):
        with self._lock:
            self._progress.update(kwargs)

    def _add_detection(self, det: dict, frame_b64: str = ""):
        with self._lock:
            det["timestamp"] = datetime.now().isoformat()
            det["frame"] = frame_b64
            self._detections.append(det)

    def _frame_to_b64(self, frame: np.ndarray, quality: int = 60) -> str:
        small = frame.copy()
        h, w = small.shape[:2]
        scale = 480 / max(h, 1)
        small = cv2.resize(small, (int(w * scale), int(h * scale)))
        _, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf).decode()

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _turn_to(self, target_yaw, tolerance=0.08, timeout=6.0):
        err = self._normalize_angle(target_yaw - self._robot.imu_yaw)
        if abs(err) < tolerance:
            return True
        start = time.time()
        while time.time() - start < timeout:
            if self._stop_flag:
                return False
            err = self._normalize_angle(target_yaw - self._robot.imu_yaw)
            if abs(err) < tolerance:
                break
            vyaw = max(0.15, min(0.6, abs(err) * 1.5)) * (1 if err > 0 else -1)
            self._robot.move(0.0, 0.0, vyaw)
            time.sleep(0.05)
        self._robot.stop()
        time.sleep(0.15)
        final_err = abs(self._normalize_angle(target_yaw - self._robot.imu_yaw))
        logger.info(f"  转向完成, 误差 {math.degrees(final_err):.1f}°")
        return final_err < tolerance * 2

    def _drive_distance(self, dist, speed, timeout_per_m=8.0):
        move_time = dist / speed
        deadline = time.time() + min(move_time, timeout_per_m * dist)
        logger.info(f"  前进 {dist:.2f}m, 速度{speed:.2f}m/s, 预计{move_time:.1f}s")
        self._robot.move(speed, 0.0, 0.0)
        try:
            while time.time() < deadline:
                if self._stop_flag:
                    break
                time.sleep(0.5)
                if self._detector.available and not self._stop_flag:
                    frame = self._robot.get_frame()
                    if frame is not None:
                        dets = self._detector.detect(frame, self._target_classes)
                        if dets:
                            annotated = self._detector.annotate(frame, dets)
                            frame_b64 = self._frame_to_b64(annotated)
                            rx, ry, _ = self._robot.position
                            for det in dets:
                                det["waypoint_idx"] = self._wp_idx
                                det["waypoint"] = {"x": round(rx, 2), "y": round(ry, 2)}
                                self._add_detection(det, frame_b64)
                            logger.info(f"  [行进中] 发现 {len(dets)} 个目标 @ ({rx:.1f}, {ry:.1f})")
        finally:
            self._robot.stop()
            time.sleep(0.15)

    def _run(self, width, height, spacing, target_classes, speed):
        self._running = True
        self._target_classes = target_classes
        try:
            self._update_progress(status="standing")
            self._robot.balance_stand()
            time.sleep(2.0)

            if self._stop_flag:
                self._update_progress(status="stopped")
                return

            waypoints = plan_lawnmower(width, height, spacing)
            total = len(waypoints)
            self._update_progress(status="searching", total=total, current=0,
                                  width=width, height=height, waypoints=waypoints)
            self._robot.reset_odom()

            # 记录搜索开始时的 IMU yaw 作为参考方向
            # atan2(dy,dx)=0 表示"正前方"，对应 IMU 的初始朝向
            ref_yaw = self._robot.imu_yaw
            logger.info(f"  搜索参考方向: IMU yaw={math.degrees(ref_yaw):.1f}°")

            prev_x, prev_y = 0.0, 0.0

            for i, wp in enumerate(waypoints):
                if self._stop_flag:
                    break
                self._wp_idx = i

                self._update_progress(current=i + 1, total=total,
                                      status="searching",
                                      waypoint={"x": wp["x"], "y": wp["y"]})

                dx = wp["x"] - prev_x
                dy = wp["y"] - prev_y
                dist = math.sqrt(dx * dx + dy * dy)

                logger.info(f"航点 {i+1}/{total}: ({wp['x']:.1f}, {wp['y']:.1f}) dist={dist:.2f}m")

                if dist < 0.1:
                    logger.info(f"  跳过（距离太近）")
                else:
                    # 计算目标方向：参考方向 + 世界坐标偏移
                    target_yaw = ref_yaw + math.atan2(dy, dx)
                    logger.info(f"  转向 {math.degrees(target_yaw - ref_yaw):.0f}° (相对) → IMU {math.degrees(target_yaw):.0f}°")
                    self._turn_to(target_yaw)

                    if self._stop_flag:
                        break
                    self._drive_distance(dist, speed)

                prev_x, prev_y = wp["x"], wp["y"]

                if self._stop_flag:
                    break

                frame = self._robot.get_frame()
                if frame is not None and self._detector.available:
                    dets = self._detector.detect(frame, target_classes)
                    if dets:
                        annotated = self._detector.annotate(frame, dets)
                        frame_b64 = self._frame_to_b64(annotated)
                        rx, ry, _ = self._robot.position
                        for det in dets:
                            det["waypoint_idx"] = i
                            det["waypoint"] = {"x": round(rx, 2), "y": round(ry, 2)}
                            self._add_detection(det, frame_b64)
                        logger.info(f"  [航点] 发现 {len(dets)} 个目标 @ ({rx:.1f}, {ry:.1f})")

                time.sleep(0.2)

            self._robot.stop()
            self._update_progress(status="completed", current=total, total=total)
            logger.info(f"搜索完成，发现 {len(self._detections)} 个目标")

        except Exception as e:
            logger.error(f"搜索任务异常: {e}")
            self._update_progress(status="error", error=str(e))
        finally:
            self._running = False


# ============================================================================
# Web 服务 — 纯 Python (websockets + http.server)
# ============================================================================

BASE_DIR = Path(__file__).parent
HTML_PATH = BASE_DIR / "static" / "index.html"

# ============================================================================
# 全局状态
# ============================================================================

# 基础模块
robot = RobotSDK()
detector = Detector.__new__(Detector)
detector._model = None
detector._confidence = 0.45

# AI 引擎（延迟加载，按需启动）
voice_engine = VoiceEngine()
vlm_engine = VLMEngine()
audio_capture = AudioCapture()

# 跟踪器（依赖 VLM + Robot）
tracker = None  # 初始化时创建

# 搜索任务
mission = SearchMission(robot, detector)

# WebSocket 连接
ws_clients = []

# 语音监听状态
voice_listening = False
voice_thread = None
voice_log = []  # 最近语音识别结果
voice_log_lock = threading.Lock()


async def _broadcast(msg: str):
    import websockets
    dead = []
    for ws in ws_clients[:]:
        try:
            await ws.send(msg)
        except Exception:
            dead.append(ws)
    for d in dead:
        if d in ws_clients:
            ws_clients.remove(d)


async def _video_stream():
    """后台任务：持续获取视频帧并推送给所有 WebSocket 客户端。"""
    yolo_counter = 0
    while True:
        try:
            if not robot.connected or not ws_clients:
                await asyncio.sleep(0.15)
                continue

            frame = robot.get_frame()
            if frame is None:
                await asyncio.sleep(0.15)
                continue

            # YOLO 每 3 帧推理一次
            dets = []
            yolo_counter += 1
            if yolo_counter >= 3 and detector.available:
                yolo_counter = 0
                dets = detector.detect(frame)

            if dets:
                frame = detector.annotate(frame, dets)

            # 编码
            h, w = frame.shape[:2]
            scale = 640 / max(w, 1)
            small = cv2.resize(frame, (int(w * scale), int(h * scale)))
            _, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
            img_b64 = base64.b64encode(buf).decode()

            msg = json.dumps({"type": "frame", "image": img_b64, "detections": dets})
            await _broadcast(msg)

            # SLAM 数据
            if yolo_counter == 0:
                rx, ry, ryaw = robot.position
                # 优先使用 slam_toolbox bridge 地图，fallback 到 ICP 栅格
                if slam_bridge.active:
                    map_data = slam_bridge.map_data
                else:
                    map_data = robot.slam.get_map_data()
                scan_local = robot.slam.get_scan_local()
                trail = robot.slam.trail
                det_positions = []
                for d in mission.detections:
                    wp = d.get("waypoint", {})
                    det_positions.append({"x": wp.get("x", 0), "y": wp.get("y", 0),
                                          "class": d.get("class", "?")})
                progress = mission.progress
                slam_msg = json.dumps({
                    "type": "slam",
                    "data": {
                        "x": round(rx, 2), "y": round(ry, 2), "yaw": round(ryaw, 2),
                        "trail": [[round(p[0], 2), round(p[1], 2)] for p in trail],
                        "map": map_data,
                        "scan": scan_local,
                        "detections": det_positions,
                        "waypoints": progress.get("waypoints", []),
                        "currentWP": progress.get("current", 0) - 1,
                        "slam_source": "slam_toolbox" if slam_bridge.active else "icp_fallback",
                    },
                })
                await _broadcast(slam_msg)

                if mission.running:
                    await _broadcast(json.dumps({"type": "status", "data": progress}))

            await asyncio.sleep(0.12)

        except Exception as e:
            logger.error(f"_video_stream 异常: {e}")
            await asyncio.sleep(1.0)


# ============================================================================
# 语音监听线程
# ============================================================================

def _voice_listener():
    """后台线程：持续监听麦克风，识别语音指令并执行。"""
    global voice_listening, voice_log, tracker
    logger.info("语音监听线程启动")
    while voice_listening:
        try:
            audio = audio_capture.get_utterance(timeout=5.0)
            if audio is None:
                continue

            logger.info(f"捕获语音片段: {len(audio)} 样本")

            # 语音 → 文本 + 指令
            result = voice_engine.process(audio)
            text = result.get("text", "")
            intent = result.get("intent", "unknown")
            target = result.get("target", "")

            if not text:
                continue

            logger.info(f"语音识别: \"{text}\" → intent={intent}, target={target}")

            # 记录日志
            with voice_log_lock:
                voice_log.append({
                    "text": text,
                    "intent": intent,
                    "target": target,
                    "timestamp": time.time(),
                })
                if len(voice_log) > 50:
                    voice_log = voice_log[-30:]

            # 执行指令
            _execute_voice_command(intent, target)

        except Exception as e:
            logger.error(f"语音处理异常: {e}")
            time.sleep(1.0)
    logger.info("语音监听线程退出")


def _execute_voice_command(intent: str, target: str):
    """根据语音指令执行对应动作。"""
    global tracker
    if intent == "follow":
        if not target:
            target = "我前面的人"
        logger.info(f">>> 执行跟踪: {target}")
        # 确保 VLM 已加载
        if not vlm_engine.loaded:
            vlm_engine.load()
        if tracker is None:
            tracker = TargetTracker(vlm_engine, robot, detector)
        tracker.start_follow(target)

    elif intent == "stop":
        logger.info(">>> 执行停止")
        robot.stop()
        if tracker:
            tracker.stop()
        if mission.running:
            mission.stop()

    elif intent == "search":
        logger.info(f">>> 执行搜索: {target}")
        if mission.running:
            mission.stop()
            time.sleep(0.5)
        target_classes = [target] if target else None
        mission.start(4.0, 4.0, 1.5, target_classes, 0.3)

    elif intent == "come":
        logger.info(">>> 执行回来（未实现）")
        # TODO: 导航回原点

    elif intent == "patrol":
        logger.info(">>> 执行巡逻")
        mission.start(6.0, 6.0, 2.0, None, 0.3)

    else:
        logger.info(f">>> 未知指令: {intent}")


# ============================================================================
# HTTP 请求处理
# ============================================================================

class Go2WHandler(BaseHTTPRequestHandler):
    """纯 Python HTTP 处理器，无 C 扩展依赖。"""

    def log_message(self, fmt, *args):
        pass  # 静默 HTTP 日志

    def _send_json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/" or path == "/index.html":
            html = HTML_PATH.read_text(encoding="utf-8")
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif path == "/api/status":
            tracker_state = tracker.state if tracker else "idle"
            self._send_json({
                "connected": robot.connected,
                "mission_running": mission.running,
                "progress": mission.progress,
                "detections_count": len(mission.detections),
                "voice_listening": voice_listening,
                "voice_model_loaded": voice_engine.loaded,
                "vlm_loaded": vlm_engine.loaded,
                "tracker_state": tracker_state,
                "tracker_target": tracker.target if tracker else "",
                "gpu": memory_summary(),
                "slam_bridge_active": slam_bridge.active,
                "slam_position": list(slam_bridge.pose) if slam_bridge.active else None,
            })

        elif path == "/api/detections":
            self._send_json({"detections": mission.detections})

        elif path == "/api/capture":
            frame = robot.get_frame()
            if frame is None:
                self._send_json({"success": False, "error": "无法获取图像"})
                return
            dets = detector.detect(frame) if detector.available else []
            annotated = detector.annotate(frame, dets) if dets else frame
            _, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
            img_b64 = base64.b64encode(buf).decode()
            self._send_json({"success": True, "image": img_b64,
                             "detections": dets, "count": len(dets)})

        elif path == "/api/voice/log":
            with voice_log_lock:
                self._send_json({"log": voice_log[-20:]})

        else:
            self.send_error(404)

    def do_POST(self):
        global voice_listening, voice_thread, tracker
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        def param(name, default="0"):
            vals = params.get(name, [str(default)])
            return vals[0]

        if path == "/api/slam/pose":
            # Go2W SLAM bridge 发来的位姿数据
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length).decode("utf-8"))
                slam_bridge.update_pose(
                    float(body.get("x", 0)),
                    float(body.get("y", 0)),
                    float(body.get("yaw", 0)),
                )
                self._send_json({"ok": True})
            except Exception as e:
                self._send_json({"ok": False, "error": str(e)}, 400)

        elif path == "/api/slam/map":
            # Go2W SLAM bridge 发来的地图数据
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length).decode("utf-8"))
                occupied = [tuple(p) for p in body.get("occupied", [])]
                free = [tuple(p) for p in body.get("free", [])]
                info = {
                    "width": body.get("width", 0),
                    "height": body.get("height", 0),
                    "resolution": body.get("resolution", 0.05),
                    "origin_x": body.get("origin_x", 0),
                    "origin_y": body.get("origin_y", 0),
                }
                slam_bridge.update_map(occupied, free, info)
                self._send_json({"ok": True})
            except Exception as e:
                self._send_json({"ok": False, "error": str(e)}, 400)

        elif path == "/api/connect":
            ok = robot.connect()
            self._send_json({"success": ok, "connected": robot.connected})

        elif path == "/api/stand":
            code = robot.balance_stand()
            self._send_json({"success": code == 0, "code": code})

        elif path == "/api/stop":
            code = robot.stop()
            if mission.running:
                mission.stop()
            if tracker:
                tracker.stop()
            self._send_json({"success": code == 0, "code": code})

        elif path == "/api/move":
            vx = float(param("vx", 0))
            vy = float(param("vy", 0))
            vyaw = float(param("vyaw", 0))
            code = robot.move(vx, vy, vyaw)
            self._send_json({"success": code == 0, "code": code})

        elif path == "/api/search":
            if not robot.connected:
                self._send_json({"success": False, "error": "Go2W 未连接"})
                return
            if mission.running:
                self._send_json({"success": False, "error": "搜索任务正在进行中"})
                return
            width = float(param("width", 4.0))
            height = float(param("height", 4.0))
            spacing = float(param("spacing", 1.5))
            target = param("target", "")
            speed = float(param("speed", 0.3))
            target_classes = [t.strip() for t in target.split(",") if t.strip()] or None
            ok = mission.start(width, height, spacing, target_classes, speed)
            self._send_json({"success": ok})

        elif path == "/api/search/stop":
            mission.stop()
            self._send_json({"success": True})

        # --- 语音相关 API ---
        elif path == "/api/voice/start":
            if voice_listening:
                self._send_json({"success": False, "error": "语音监听已在运行"})
                return
            # 加载语音模型
            if not voice_engine.loaded:
                ok = voice_engine.load()
                if not ok:
                    self._send_json({"success": False, "error": "语音模型加载失败"})
                    return
            # 启动麦克风
            audio_capture.start()
            voice_listening = True
            voice_thread = threading.Thread(target=_voice_listener, daemon=True)
            voice_thread.start()
            self._send_json({"success": True})

        elif path == "/api/voice/stop":
            voice_listening = False
            audio_capture.stop()
            self._send_json({"success": True})

        # --- 文本指令 API（用于前端测试，跳过语音识别） ---
        elif path == "/api/command":
            text = param("text", "")
            if not text:
                self._send_json({"success": False, "error": "缺少 text 参数"})
                return
            result = voice_engine.process_text(text)
            intent = result.get("intent", "unknown")
            target = result.get("target", "")
            _execute_voice_command(intent, target)
            self._send_json({"success": True, "intent": intent, "target": target,
                             "parsed": result})

        # --- VLM / 跟踪 API ---
        elif path == "/api/vlm/load":
            ok = vlm_engine.load()
            self._send_json({"success": ok, "loaded": vlm_engine.loaded})

        elif path == "/api/vlm/unload":
            vlm_engine.unload()
            self._send_json({"success": True})

        elif path == "/api/vlm/locate":
            target_desc = param("target", "")
            if not target_desc:
                self._send_json({"success": False, "error": "缺少 target 参数"})
                return
            if not vlm_engine.loaded:
                self._send_json({"success": False, "error": "VLM 未加载"})
                return
            frame = robot.get_frame()
            if frame is None:
                self._send_json({"success": False, "error": "无法获取图像"})
                return
            result = vlm_engine.locate(frame, target_desc)
            self._send_json({"success": True, "result": result})

        elif path == "/api/follow":
            target_desc = param("target", "")
            if not target_desc:
                self._send_json({"success": False, "error": "缺少 target 参数"})
                return
            if not robot.connected:
                self._send_json({"success": False, "error": "Go2W 未连接"})
                return
            # 确保 VLM 加载
            if not vlm_engine.loaded:
                vlm_engine.load()
            if tracker is None:
                tracker = TargetTracker(vlm_engine, robot, detector)
            ok = tracker.start_follow(target_desc)
            self._send_json({"success": ok})

        elif path == "/api/follow/stop":
            if tracker:
                tracker.stop()
            self._send_json({"success": True})

        else:
            self.send_error(404)


async def ws_handler(websocket):
    """处理 WebSocket 连接。"""
    ws_clients.append(websocket)
    logger.info(f"WebSocket 客户端连接, 共 {len(ws_clients)} 个")
    try:
        async for message in websocket:
            if message == "ping":
                await websocket.send("pong")
    except Exception:
        pass
    finally:
        if websocket in ws_clients:
            ws_clients.remove(websocket)
        logger.info(f"WebSocket 客户端断开, 剩余 {len(ws_clients)} 个")


async def main_async(host, port):
    """主异步入口：同时运行 HTTP 服务器和 WebSocket 服务器。"""
    import websockets

    # HTTP 服务器（在线程池中运行）
    httpd = HTTPServer((host, port), Go2WHandler)
    logger.info(f"HTTP 服务: http://{host}:{port}")

    loop = asyncio.get_event_loop()
    http_server = loop.run_in_executor(None, httpd.serve_forever)

    # WebSocket 服务器
    async def ws_main():
        async with websockets.serve(ws_handler, host, port + 1):
            logger.info(f"WebSocket 服务: ws://{host}:{port + 1}")
            await asyncio.Future()  # 永不结束

    # 视频流推送
    stream_task = asyncio.create_task(_video_stream())

    # 并行运行
    await asyncio.gather(ws_main(), stream_task, return_exceptions=True)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Go2W 搜索系统 Web 服务")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--interface", default="enp65s0")
    parser.add_argument("--model", default=None)
    parser.add_argument("--no-connect", action="store_true")
    args = parser.parse_args()

    # 查找 YOLO 模型
    model_path = args.model
    if model_path is None:
        for candidate in [
            Path.cwd() / "yolov8n.pt",
            BASE_DIR.parent / "yolov8n.pt",
            Path.home() / "yolov8n.pt",
        ]:
            if candidate.exists():
                model_path = str(candidate)
                break
        if model_path is None:
            model_path = "yolov8n.pt"

    global detector, mission, tracker
    detector = Detector(model_path)
    mission = SearchMission(robot, detector)
    robot._interface = args.interface

    logger.info(f"平台信息: {memory_summary()}")

    # DDS 在 Web 服务之前初始化
    if not args.no_connect:
        logger.info("正在连接 Go2W...")
        robot.connect()
        if not robot.connected:
            logger.warning("Go2W 连接失败，仅启动 Web 服务")

    logger.info(f"启动服务: http://{args.host}:{args.port}")
    asyncio.run(main_async(args.host, args.port))


if __name__ == "__main__":
    main()
