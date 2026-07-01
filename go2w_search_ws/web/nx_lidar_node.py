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
    from livox_ros_driver2.msg import CustomMsg
    _LIDAR_OK = True
except Exception as _e:
    _LIDAR_OK = False
    logger.warning(f"[lidar] rclpy/livox_ros_driver2 不可用 ({_e}), 雷达点云展示禁用")


class LidarBridge:
    """订阅 /livox/lidar → 投影 2D 鸟瞰 png → WS type=lidar。"""

    def __init__(self, ws_broadcast_fn):
        self._ws = ws_broadcast_fn
        self._lock = threading.Lock()
        self._latest_png = None
        self._latest_points = []
        self._running = False
        self._node = None
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
            pts = msg.points or []
            n = 0
            xs = ys = np.empty(0, dtype=np.float32)
            if pts:
                xy = np.array([[p.x, p.y] for p in pts], dtype=np.float32)
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
            logger.debug(f"[lidar] 投影异常: {e}")

    def _spin_thread(self):
        while self._running and self._node is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                logger.debug(f"[lidar] spin 异常: {e}")
                time.sleep(0.5)

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
        """接收已存在的 rclpy node (NxWebNode), 在其上订阅 /livox/lidar。
        不自建 node/spin — 避免同进程多线程 spin 同 rclpy context 冲突 (executor Traceback)。
        回调由 node 的主 spin 线程驱动, 本类只跑广播线程。
        """
        if not _LIDAR_OK:
            logger.warning("[lidar] rclpy/livox msg 缺失, 不启动雷达点云展示")
            return
        try:
            node.create_subscription(CustomMsg, "/livox/lidar", self._cb, 10)
        except Exception as e:
            logger.error(f"[lidar] 订阅 /livox/lidar 失败: {e}")
            return
        self._running = True
        threading.Thread(target=self._bcast_thread, daemon=True, name="lidar_bc").start()
        logger.info(f"[lidar] 雷达点云展示启动 (订阅 /livox/lidar on {node.get_name()}, {_FPS}fps, ±{_RANGE}m)")

    def stop(self):
        self._running = False
