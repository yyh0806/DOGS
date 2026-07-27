"""SimVideoBridge: 订 /sim_camera/image_raw (URDF libgazebo_ros_camera) → WS type=gimbal.

仿真 URDF 相机发 /sim_camera/image_raw (640x480 bgr8). 本桥订阅 → cv2 jpeg base64 →
ws_broadcast type=gimbal (vis, ir=None) 推前端视频区 (复用 GimbalRtspBridge 的 type=gimbal 通道,
前端 panel.html 已处理 vis jpeg 显示). 真机用 C13 RTSP (nx_gimbal_node GimbalRtspBridge).

importer: nx_web_server.main() GO2W_SIM 分支实例化 SimVideoBridge(ws_broadcast).start().
挂 NxWebNode subscription (主 spin 驱动回调), 仿 LidarBridge 架构.
"""
import base64
import threading
import time

import cv2
import numpy as np

try:
    from sensor_msgs.msg import Image as ImageMsg
except ImportError:
    ImageMsg = None


class SimVideoBridge:
    """URDF 相机 /sim_camera/image_raw → WS type=gimbal (vis jpeg)."""

    def __init__(self, node, ws_broadcast_fn):
        self._node = node
        self._ws = ws_broadcast_fn
        self._latest_jpg_b64 = None
        self._lock = threading.Lock()
        self._running = False
        if ImageMsg is None:
            node.get_logger().warning(
                "SimVideoBridge: sensor_msgs 不可导入, 仿真视频桥禁用")
            return
        self._sub = node.create_subscription(
            ImageMsg, '/sim_camera/image_raw', self._on_image, 10)
        self._running = True
        node.get_logger().info(
            "SimVideoBridge: 订 /sim_camera/image_raw → WS type=gimbal (10Hz)")

    def _on_image(self, msg):
        try:
            if msg.encoding not in ('bgr8', 'rgb8'):
                return
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            h, w = arr.shape[:2]
            if w > 480:
                scale = 480.0 / w
                arr = cv2.resize(arr, (480, int(h * scale)))
            ok, jpg = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if not ok:
                return
            b64 = base64.b64encode(jpg.tobytes()).decode()
            with self._lock:
                self._latest_jpg_b64 = b64
        except Exception:
            pass

    def _bcast_loop(self):
        count = 0
        warned_no_img = False
        while self._running:
            try:
                if self._node.executor.is_shutdown():
                    break
            except Exception:
                pass
            time.sleep(0.1)  # 10Hz
            with self._lock:
                b64 = self._latest_jpg_b64
            if b64:
                try:
                    self._ws({"type": "gimbal", "vis": b64,
                              "ir": None, "sim": True})
                    count += 1
                    if count % 50 == 1:
                        self._node.get_logger().info(
                            f"SimVideoBridge bcast #{count} vis_len={len(b64)}")
                except Exception as e:
                    self._node.get_logger().warning(
                        f"SimVideoBridge bcast err: {e}")
            elif not warned_no_img:
                self._node.get_logger().warning(
                    "SimVideoBridge: 5s 无 image (_on_image 未触发? 检查 /sim_camera/image_raw)")
                warned_no_img = True

    def start(self):
        if not self._running:
            return
        threading.Thread(target=self._bcast_loop, name="sim_video_bc",
                         daemon=True).start()


def is_enabled():
    """GO2W_SIM 时启用 (替代 C13 RTSP GimbalRtspBridge)."""
    import os
    return bool(os.environ.get('GO2W_SIM'))
