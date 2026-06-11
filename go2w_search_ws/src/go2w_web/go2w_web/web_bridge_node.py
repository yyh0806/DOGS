"""Go2W Web 桥接节点。

ROS2 节点 + HTTP/WebSocket 服务器。
从 ROS2 话题获取数据，通过 HTTP API 和 WebSocket 推送给前端。
不直连 Go2W SDK，所有数据来自 ROS2。

节点名: go2w_web_bridge
"""

import asyncio
import base64
import io
import json
import logging
import math
import os
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from std_msgs.msg import String

logger = logging.getLogger(__name__)

# WebSocket 在独立线程中运行
_ws_clients = set()
_ws_loop: Optional[asyncio.AbstractEventLoop] = None


def ws_broadcast(data: dict):
    """向所有 WebSocket 客户端广播数据。"""
    if _ws_loop and _ws_clients:
        msg = json.dumps(data, ensure_ascii=False)
        asyncio.run_coroutine_threadsafe(_async_broadcast(msg), _ws_loop)


async def _async_broadcast(msg: str):
    import websockets
    for ws in list(_ws_clients):
        try:
            await ws.send(msg)
        except Exception:
            _ws_clients.discard(ws)


class WebBridgeNode(Node):
    """Web 桥接节点: ROS2 话题 → HTTP/WebSocket。"""

    def __init__(self):
        super().__init__('go2w_web_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('ws_port', 8001)

        # 数据缓冲
        self._lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_map: Optional[dict] = None
        self._latest_scan: Optional[dict] = None
        self._task_queue: Optional[dict] = None
        self._robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._vlm_response = ""
        self._detections = []

        # ROS2 订阅
        img_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, img_qos)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.create_subscription(String, '/go2w/task_queue', self._task_queue_cb, 10)
        self.create_subscription(String, '/go2w/vlm_response', self._vlm_response_cb, 10)
        self.create_subscription(String, '/go2w/detections', self._detection_cb, 10)

        # cmd_vel 发布器
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 启动 HTTP 和 WebSocket 服务
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value

        self._start_http_server(host, port)
        self._start_ws_server(host, ws_port)

        self.get_logger().info(f"Web 桥接就绪: http://{host}:{port}")

    # ---- ROS2 回调 ----

    def _image_cb(self, msg: Image):
        """接收摄像头图像。"""
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
            with self._lock:
                self._latest_image = frame.copy()

            # 推送 WebSocket
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            b64 = base64.b64encode(jpeg.tobytes()).decode()
            ws_broadcast({"type": "frame", "data": b64})
        except Exception:
            pass

    def _map_cb(self, msg: OccupancyGrid):
        """接收 SLAM 地图。"""
        try:
            w, h = msg.info.width, msg.info.height
            res = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            data = np.array(msg.data, dtype=np.int8).reshape(h, w)

            # 压缩为 PNG
            vis = np.zeros_like(data, dtype=np.uint8)
            vis[data == -1] = 128   # 未知
            vis[data == 0] = 255    # 自由
            vis[data > 50] = 0      # 障碍

            _, png = cv2.imencode('.png', vis)
            b64 = base64.b64encode(png.tobytes()).decode()

            with self._lock:
                self._latest_map = {
                    "image": b64,
                    "width": w, "height": h,
                    "resolution": res,
                    "origin_x": origin_x,
                    "origin_y": origin_y,
                }

            ws_broadcast({
                "type": "map",
                "image": b64,
                "width": w, "height": h,
                "resolution": res,
                "origin_x": origin_x,
                "origin_y": origin_y,
            })
        except Exception:
            pass

    def _task_queue_cb(self, msg: String):
        """接收任务队列状态。"""
        with self._lock:
            self._task_queue = json.loads(msg.data)
        ws_broadcast({"type": "tasks", "data": json.loads(msg.data)})

    def _vlm_response_cb(self, msg: String):
        """接收 VLM 回复。"""
        with self._lock:
            self._vlm_response = msg.data
        try:
            data = json.loads(msg.data)
            ws_broadcast({"type": "vlm", "data": data})
        except Exception:
            pass

    def _detection_cb(self, msg: String):
        """接收检测结果。"""
        try:
            det = json.loads(msg.data)
            with self._lock:
                self._detections.append(det)
                if len(self._detections) > 50:
                    self._detections = self._detections[-25:]
        except Exception:
            pass

    # ---- HTTP 服务器 ----

    def _start_http_server(self, host, port):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == '/' or parsed.path == '/index.html':
                    self._serve_static('index.html', 'text/html')
                elif parsed.path == '/api/status':
                    self._json_response(node._get_status())
                elif parsed.path == '/api/detections':
                    with node._lock:
                        dets = list(node._detections)
                    self._json_response({"detections": dets})
                elif parsed.path == '/api/capture':
                    self._serve_capture()
                else:
                    self.send_error(404)

            def do_POST(self):
                parsed = urlparse(self.path)
                params = parse_qs(parsed.query)
                content_len = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_len).decode() if content_len else ''

                if parsed.path == '/api/move':
                    vx = float(params.get('vx', ['0'])[0])
                    vy = float(params.get('vy', ['0'])[0])
                    vyaw = float(params.get('vyaw', ['0'])[0])
                    cmd = Twist()
                    cmd.linear.x = vx
                    cmd.linear.y = vy
                    cmd.angular.z = vyaw
                    node._cmd_pub.publish(cmd)
                    self._json_response({"ok": True})

                elif parsed.path == '/api/stop':
                    cmd = Twist()
                    node._cmd_pub.publish(cmd)
                    self._json_response({"ok": True})

                elif parsed.path == '/api/command':
                    text = params.get('text', [''])[0] or body
                    # 发布到编排器的输入话题
                    msg = String()
                    msg.data = text
                    node._voice_text_pub.publish(msg)
                    self._json_response({"ok": True, "text": text})

                elif parsed.path == '/api/voice/start':
                    # TODO: 通过服务调用编排器启动语音
                    self._json_response({"ok": True})

                elif parsed.path == '/api/voice/stop':
                    self._json_response({"ok": True})

                else:
                    self.send_error(404)

            def _json_response(self, data):
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

            def _serve_static(self, filename, content_type):
                static_dir = os.path.join(os.path.dirname(__file__), 'static')
                filepath = os.path.join(static_dir, filename)
                if os.path.exists(filepath):
                    self.send_response(200)
                    self.send_header('Content-Type', content_type)
                    with open(filepath, 'rb') as f:
                        data = f.read()
                    self.send_header('Content-Length', len(data))
                    self.end_headers()
                    self.wfile.write(data)
                else:
                    self.send_error(404)

            def _serve_capture(self):
                with node._lock:
                    frame = node._latest_image
                if frame is not None:
                    _, jpeg = cv2.imencode('.jpg', frame)
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.end_headers()
                    self.wfile.write(jpeg.tobytes())
                else:
                    self.send_error(404)

            def log_message(self, format, *args):
                pass  # 静默 HTTP 日志

        # 发布文本指令的话题
        self._voice_text_pub = self.create_publisher(String, '/go2w/voice_text', 10)

        server = HTTPServer((host, port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

    # ---- WebSocket 服务器 ----

    def _start_ws_server(self, host, ws_port):
        def run_ws():
            global _ws_loop
            import websockets

            async def handler(websocket, path):
                _ws_clients.add(websocket)
                try:
                    await websocket.wait_closed()
                finally:
                    _ws_clients.discard(websocket)

            _ws_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(_ws_loop)
            _ws_loop.run_until_complete(
                websockets.serve(handler, host, ws_port)
            )
            _ws_loop.run_forever()

        thread = threading.Thread(target=run_ws, daemon=True)
        thread.start()

    def _get_status(self):
        with self._lock:
            return {
                "connected": True,
                "tasks": self._task_queue,
                "vlm_response": self._vlm_response,
                "detection_count": len(self._detections),
            }

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
