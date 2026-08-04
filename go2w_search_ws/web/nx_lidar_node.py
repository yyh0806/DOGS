#!/usr/bin/env python3
"""Livox MID360 雷达点云 2D 鸟瞰展示桥接 (组件, 注入 nx_web_server 同进程)。

订阅 /livox/lidar (livox CustomMsg), 把每帧点云投影成 XY 平面鸟瞰 png,
广播线程推 WS type=lidar, 前端 panel 显示 (类似 nx_gimbal_node 的视频流模式)。

线程: daemon ×1 = _spin_thread (rclpy.spin_once 订阅) + daemon ×1 = _bcast_thread。
配置 (环境变量): LIDAR_VIEW_RANGE(±米,10) / LIDAR_VIEW_SIZE(像素,400) / LIDAR_FPS(5)。
红线: 懒加载 rclpy/livox.msg (缺失禁用不崩); 异常只 debug 不抛; 三路(gimbal/lidar/sensor)并存。
"""
import base64
import logging
import os
import threading
import time

import numpy as np

logger = logging.getLogger("go2w.lidar")

_RANGE = float(os.environ.get("LIDAR_VIEW_RANGE", "10"))
_SIZE = int(os.environ.get("LIDAR_VIEW_SIZE", "400"))
_FPS = max(0.5, float(os.environ.get("LIDAR_FPS", "5")))
_MAX_WS_POINTS = max(1, int(os.environ.get("LIDAR_WS_POINTS", "600")))

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    _LIDAR_OK = True
except Exception as _e:
    _LIDAR_OK = False
    logger.warning(f"[lidar] rclpy/livox_ros_driver2 不可用 ({_e}), 雷达点云展示禁用")


def _decode_pointcloud_xy(msg):
    """Decode little-endian float32 x/y fields without per-point objects."""
    if getattr(msg, "is_bigendian", False):
        return np.empty((0, 2), dtype=np.float32)
    point_step = int(getattr(msg, "point_step", 0) or 0)
    data = getattr(msg, "data", b"")
    if point_step <= 0 or not data:
        return np.empty((0, 2), dtype=np.float32)
    fields = {field.name: field for field in getattr(msg, "fields", [])}
    if any(name not in fields for name in ("x", "y")):
        return np.empty((0, 2), dtype=np.float32)
    # sensor_msgs/PointField.FLOAT32 is wire value 7.  Keeping the constant
    # local lets workstation tests run without a ROS installation.
    if any(int(fields[name].datatype) != 7 for name in ("x", "y")):
        return np.empty((0, 2), dtype=np.float32)
    count = len(data) // point_step
    if count <= 0:
        return np.empty((0, 2), dtype=np.float32)
    raw = np.frombuffer(data, dtype=np.uint8, count=count * point_step).reshape(
        count, point_step
    )
    columns = [
        raw[:, fields[name].offset:fields[name].offset + 4]
        .copy().reshape(-1).view("<f4")
        for name in ("x", "y")
    ]
    return np.column_stack(columns)


class LidarBridge:
    """订阅 /livox/lidar → 投影 2D 鸟瞰 png → WS type=lidar。"""

    def __init__(self, ws_broadcast_fn):
        self._ws = ws_broadcast_fn
        self._lock = threading.Lock()
        self._latest_png = None
        self._latest_points = []
        self._running = False
        self._node = None
        self._subscription = None
        self._ctx = None
        self._executor = None
        self._next_render_t = 0.0

    def _cb(self, msg):
        """livox CustomMsg 回调: points[].x(前)/y(左) 投影鸟瞰图, 雷达在中心。

        M1 fix (2026-07-01): 向量化投影(一次 list→ndarray + numpy fancy indexing)替代
        逐点 Python 循环, 降低 ~24k 点帧的渲染耗时, 减少对同 spin 线程 /imu /scan /odom 的阻塞。
        契约不变: 仍由 _cb 节流渲染(test_lidar_latency_contract), _bcast_thread 推 WS。"""
        try:
            now = time.monotonic()
            if now < self._next_render_t:
                return
            self._next_render_t = now + (1.0 / _FPS)
            import cv2
            img = np.full((_SIZE, _SIZE, 3), 15, dtype=np.uint8)
            scale = _SIZE / (2.0 * _RANGE)
            cx = cy = _SIZE // 2
            for r in range(int(_RANGE), 0, -2):
                cv2.circle(img, (cx, cy), int(r * scale), (45, 45, 45), 1)
            cv2.line(img, (cx, 0), (cx, _SIZE), (45, 45, 45), 1)
            cv2.line(img, (0, cy), (_SIZE, cy), (45, 45, 45), 1)
            cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
            # M1: 一次构造 xy 数组, 后续全 numpy (替代逐点 float/abs/比较/赋值循环)
            n = 0
            xs = ys = np.empty(0, dtype=np.float32)
            xy = _decode_pointcloud_xy(msg)
            if xy.size:
                xs, ys = xy[:, 0], xy[:, 1]
                m = (np.abs(xs) <= _RANGE) & (np.abs(ys) <= _RANGE)
                xs, ys = xs[m], ys[m]
                n = int(xs.shape[0])
                if n:
                    pxs = (cx - ys * scale).astype(np.int32)
                    pys = (cy - xs * scale).astype(np.int32)
                    ok_px = (pxs >= 0) & (pxs < _SIZE) & (pys >= 0) & (pys < _SIZE)
                    img[pys[ok_px], pxs[ok_px]] = (0, 255, 0)
            # 抽稀点列表给前端 (保持 _MAX_WS_POINTS 上限)
            if n > 0:
                stride = max(1, n // _MAX_WS_POINTS)
                local_points = [[round(float(xs[i]), 2), round(float(ys[i]), 2)]
                                for i in range(0, n, stride)][:_MAX_WS_POINTS]
            else:
                local_points = []
            cv2.putText(img, f"pts:{n} range:{_RANGE}m", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
            cv2.putText(img, "MID360 bird-eye (up=forward)", (8, _SIZE - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
            ok, png = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if ok:
                with self._lock:
                    self._latest_png = base64.b64encode(png.tobytes()).decode()
                    self._latest_points = local_points
        except Exception as e:
            logger.warning(f"[lidar] 投影异常: {e}")

    def _spin_thread(self):
        # 独立 context 下 wait_for_ready_callbacks 不真正阻塞 (livox 持续 ready),
        # spin_once 循环会 busy-loop 占 70%+ CPU 拖垮 gimbal; rclpy.spin 更糟 (回调不触发).
        # 解法: spin_once + 显式 sleep 把每轮拉到 ~0.2s, CPU 砍半; livox 10Hz 仍够喂 _cb 的 5fps 节流.
        while self._running and self._node is not None:
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                logger.warning(f"[lidar] spin 异常: {e}")
                time.sleep(0.5)
            time.sleep(0.2)  # livox 10Hz 发, spin 取 ~5Hz 匹配 _FPS=5 渲染 (再快也被 _cb 节流掉, 白耗 CPU)

    def _bcast_thread(self):
        interval = 1.0 / _FPS
        warmup = time.time() + 8.0
        warned = False
        while self._running:
            with self._lock:
                p = self._latest_png
                pts = list(self._latest_points)
            if p:
                try:
                    self._ws({"type": "lidar", "data": p, "points": pts})
                except Exception as e:
                    logger.debug(f"[lidar] broadcast 异常: {e}")
            elif time.time() > warmup and not warned:
                logger.warning("[lidar] 启动 8s 仍无 /livox/lidar 数据, 检查 livox 驱动是否在跑")
                warned = True
            time.sleep(interval)

    def start(self, node):
        """自建独立 rclpy context + node 订阅 /livox/lidar + 自 spin。

        早期版本挂 NxWebNode 主 spin, 但 web 进程对 livox CustomMsg 反序列化静默失败
        (imu/scan/odom 标准 msg 正常, 独立 python 订阅也正常, 唯独挂 NxWebNode 不触发 _cb
        → _latest_png 终身空 → 前端无雷达)。独立 context 隔离 = 等价独立 python 订阅路径,
        实测能收; 同进程多 *context* 不冲突 (冲突的是同 context 多 spin)。

        GO2W_SIM: 仿真订 /mid360/points_nav (PointCloud2 标准 msg, 无 CustomMsg 反序列化
        问题), 用主 context 即可. 独立 context 在 WSL2 DDS 发现不稳 (同 wheel_feedback
        PUB_COUNT=0 根因). 真机不设 GO2W_SIM 走独立 context (避 livox CustomMsg 主 context
        反序列化失败).
        """
        if not _LIDAR_OK:
            logger.warning("[lidar] rclpy/livox msg 缺失, 不启动雷达点云展示")
            return
        import os
        if os.environ.get('GO2W_SIM'):
            try:
                self._node = node  # 主 NxWebNode (PointCloud2 标准 msg 无反序列化问题)
                self._subscription = self._node.create_subscription(
                    PointCloud2, "/mid360/points_nav", self._cb,
                    qos_profile_sensor_data,
                )
            except Exception as e:
                logger.error(f"[lidar] GO2W_SIM 订阅失败: {e}")
                return
            self._running = True
            threading.Thread(target=self._bcast_thread, daemon=True, name="lidar_bc").start()
            logger.info(f"[lidar] GO2W_SIM 主 context 订阅 /mid360/points_nav ({_FPS}fps, ±{_RANGE}m)")
            return
        try:
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            self._ctx = rclpy.Context()
            self._ctx.init()
            self._node = Node("nx_lidar_sub", context=self._ctx)
            self._subscription = self._node.create_subscription(
                PointCloud2, "/mid360/points_nav", self._cb,
                qos_profile_sensor_data,
            )
            self._executor = SingleThreadedExecutor(context=self._ctx)
            self._executor.add_node(self._node)
        except Exception as e:
            logger.error(f"[lidar] 订阅 /livox/lidar 失败: {e}")
            return
        self._running = True
        threading.Thread(target=self._bcast_thread, daemon=True, name="lidar_bc").start()
        threading.Thread(target=self._spin_thread, daemon=True, name="lidar_spin").start()
        logger.info(f"[lidar] 雷达点云展示启动 (独立 context 订阅 /livox/lidar, {_FPS}fps, ±{_RANGE}m)")

    def get_latest_points(self):
        """Return latest sampled Livox local points as [x_forward, y_left]."""
        with self._lock:
            return list(self._latest_points)

    def stop(self):
        self._running = False
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._ctx is not None:
            try:
                self._ctx.shutdown()
            except Exception:
                pass
