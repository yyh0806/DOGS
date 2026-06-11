#!/usr/bin/env python3
"""Go2W 独立 Web 控制面板。

无需 ROS2，直连 SDK。
实时: 视频流 + SLAM地图 + 任务队列 + 语音/文本指令。
"""

import asyncio
import base64
import json
import math
import os
import struct
import sys
import time
import threading
import logging
import traceback
import cv2
import numpy as np

from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from ai.config import DEVICE, CUDA_AVAILABLE, memory_summary
from ai.detector import Detector
from ai.vlm import VLMEngine
from audio.capture import AudioCapture

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("go2w.panel")

WS_CLIENTS = set()
WS_LOOP = None


def ws_broadcast(data):
    if WS_LOOP and WS_CLIENTS:
        msg = json.dumps(data, ensure_ascii=False)
        asyncio.run_coroutine_threadsafe(_async_broadcast(msg), WS_LOOP)


async def _async_broadcast(msg):
    for ws in list(WS_CLIENTS):
        try:
            await ws.send(msg)
        except Exception:
            WS_CLIENTS.discard(ws)


# ============================================================================
# Go2W SDK 连接
# ============================================================================
class RobotConnection:
    def __init__(self, interface="enp65s0"):
        self.interface = interface
        self.factory = None
        self.sport = None
        self.video = None
        self.connected = False
        self._imu_yaw = 0.0
        self._imu_lock = threading.Lock()
        self._lidar_queue = []
        self._imu_count = 0
        self._lidar_count = 0
        self._moving = False

    def connect(self):
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.go2.video.video_client import VideoClient
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_
        from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_

        self.factory = ChannelFactory()
        self.factory.Init(0, self.interface)

        self.sport = SportClient()
        self.sport.SetTimeout(10.0)
        self.sport.Init()

        self.video = VideoClient()
        self.video.SetTimeout(10.0)
        self.video.Init()

        def on_imu(msg):
            with self._imu_lock:
                self._imu_yaw = float(msg.imu_state.rpy[2])
                self._imu_count += 1

        def on_lidar(msg):
            self._lidar_queue.append({
                'data': bytes(msg.data),
                'point_step': int(msg.point_step),
                'width': int(msg.width),
            })
            if len(self._lidar_queue) > 10:
                self._lidar_queue.pop(0)
            self._lidar_count += 1

        ch1 = self.factory.CreateRecvChannel('rt/lowstate', LowState_)
        ch1.SetReader(handler=on_imu)
        ch2 = self.factory.CreateRecvChannel('rt/utlidar/cloud', PointCloud2_)
        ch2.SetReader(handler=on_lidar)

        self.connected = True

    def get_frame(self):
        if not self.video:
            return None
        try:
            code, data = self.video.GetImageSample()
            if code == 0 and data:
                return cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            pass
        return None

    def stand(self):
        if self.sport:
            try:
                # 必须经过 StandDown→Sit 才能进入可移动模式
                self.sport.StandDown()
                time.sleep(1)
                self.sport.Sit()
                time.sleep(1)
                self.sport.StandUp()
                time.sleep(2)
                self.sport.BalanceStand()
                self._moving = False
                logger.info("Stand: StandDown→Sit→StandUp→BalanceStand")
            except Exception as e:
                logger.error(f"Stand failed: {e}")

    def sit(self):
        if self.sport:
            try:
                self.sport.Move(0, 0, 0)
                self.sport.StandDown()
                self._moving = False
                logger.info("Sit: StandDown")
            except Exception as e:
                logger.error(f"Sit failed: {e}")

    def move(self, vx, vy, vyaw):
        if self.sport:
            try:
                if not self._moving:
                    self.sport.BalanceStand()
                    self._moving = True
                self.sport.Move(vx, vy, vyaw)
            except Exception as e:
                logger.error(f"Move failed: {e}")

    def stop_move(self):
        if self.sport:
            try:
                self.sport.Move(0, 0, 0)
                self._moving = False
                logger.info("StopMove: Move(0,0,0)")
            except Exception as e:
                logger.error(f"StopMove failed: {e}")

    def e_stop(self):
        if self.sport:
            try:
                self.sport.Damp()
                self._moving = False
                logger.info("E-Stop: Damp()")
            except Exception as e:
                logger.error(f"E-Stop failed: {e}")

    @property
    def imu_yaw(self):
        with self._imu_lock:
            return self._imu_yaw

    @property
    def stats(self):
        return {"imu_count": self._imu_count, "lidar_count": self._lidar_count}


# ============================================================================
# 任务队列
# ============================================================================
import uuid

class Task:
    def __init__(self, task_type, params=None, priority=5):
        self.id = uuid.uuid4().hex[:8]
        self.type = task_type
        self.params = params or {}
        self.priority = priority
        self.status = "pending"
        self.result = None
        self.created_at = time.time()

    def to_dict(self):
        return {"id": self.id, "type": self.type, "params": self.params,
                "priority": self.priority, "status": self.status,
                "result": self.result, "created_at": self.created_at}


class TaskManager:
    def __init__(self, robot, vlm_engine=None):
        self.robot = robot
        self.vlm = vlm_engine
        self._lock = threading.Lock()
        self._tasks = []
        self._active = None
        self._running = False
        self._thread = None
        self._detections = []

    def add(self, task):
        with self._lock:
            self._tasks.append(task)
        ws_broadcast({"type": "tasks", "data": self.get_state()})
        logger.info(f"任务加入: {task.type} (优先级 {task.priority})")

    def add_list(self, tasks):
        with self._lock:
            for i, t in enumerate(tasks):
                if t.priority == 5:
                    t.priority = max(1, 8 - i)
                self._tasks.append(t)
        ws_broadcast({"type": "tasks", "data": self.get_state()})

    def cancel_all(self):
        with self._lock:
            if self._active:
                self._active.status = "cancelled"
            self._active = None
            self._tasks.clear()
        self.robot.stop_move()
        ws_broadcast({"type": "tasks", "data": self.get_state()})

    def get_state(self):
        with self._lock:
            return {
                "active": self._active.to_dict() if self._active else None,
                "pending": [t.to_dict() for t in self._tasks if t.status == "pending"],
                "completed_count": sum(1 for t in self._tasks if t.status == "completed"),
            }

    def process_command(self, text):
        """VLM 解析指令或降级解析，加入任务队列。"""
        thread = threading.Thread(target=self._process_command_bg, args=(text,), daemon=True)
        thread.start()

    def _process_command_bg(self, text):
        try:
            if self.vlm and self.vlm.loaded:
                from go2w_orchestrator.vlm_integration import VLMIntegration
                integ = VLMIntegration()
                integ._engine = self.vlm
                result = integ.process_command(text)
            else:
                result = self._fallback_parse(text)

            response = result.get("response", "")
            tasks = result.get("tasks", [])

            ws_broadcast({"type": "vlm", "data": {"text": text, "response": response, "tasks": tasks}})

            if tasks:
                task_items = []
                for t in tasks:
                    task_items.append(Task(t.get("type", "navigate"), t.get("params", {}), t.get("priority", 5)))
                self.add_list(task_items)
                logger.info(f"VLM 拆解: {len(task_items)} 个子任务")
            else:
                # 单条指令直接处理
                self._handle_simple(text)

        except Exception as e:
            logger.error(f"指令处理失败: {e}")
            traceback.print_exc()
            ws_broadcast({"type": "vlm", "data": {"text": text, "response": f"处理失败: {e}", "tasks": []}})

    def _handle_simple(self, text):
        text_lower = text.lower()
        if "停" in text:
            self.cancel_all()
        elif "前进" in text or "向前" in text:
            self.add(Task("move", {"vx": 0.5, "duration": 2.0}, 6))
        elif "后退" in text or "向后" in text:
            self.add(Task("move", {"vx": -0.3, "duration": 2.0}, 6))
        elif "左转" in text:
            self.add(Task("move", {"vyaw": 0.5, "duration": 2.0}, 6))
        elif "右转" in text:
            self.add(Task("move", {"vyaw": -0.5, "duration": 2.0}, 6))

    @staticmethod
    def _fallback_parse(text):
        result = {"understanding": text, "tasks": [], "response": ""}
        if "跟着" in text or "跟随" in text:
            target = ""
            for kw in ["跟着", "跟随"]:
                if kw in text:
                    target = text[text.index(kw)+len(kw):].strip().rstrip("。，！？")
            result["tasks"] = [{"type": "follow", "priority": 8, "params": {"target": target}}]
            result["response"] = f"好的，跟踪{target}"
        elif "搜索" in text or "找" in text:
            result["tasks"] = [{"type": "search_area", "priority": 5, "params": {"pattern": "lawnmower", "width": 10, "height": 10}}]
            result["response"] = "好的，开始搜索"
        elif "停" in text:
            result["tasks"] = [{"type": "stop", "priority": 10, "params": {}}]
            result["response"] = "已停止"
        elif "回来" in text or "返回" in text:
            result["tasks"] = [{"type": "return_home", "priority": 7, "params": {}}]
            result["response"] = "正在返回"
        elif "前进" in text or "向前" in text:
            result["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": 0.5, "duration": 2.0}}]
            result["response"] = "前进"
        elif "后退" in text or "向后" in text:
            result["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": -0.3, "duration": 2.0}}]
            result["response"] = "后退"
        elif "左转" in text:
            result["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": 0.5, "duration": 2.0}}]
            result["response"] = "左转"
        elif "右转" in text:
            result["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": -0.5, "duration": 2.0}}]
            result["response"] = "右转"
        else:
            result["response"] = f"收到: {text}（暂不支持此指令）"
        return result

    def start_worker(self):
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop_worker(self):
        self._running = False

    def _worker(self):
        while self._running:
            task = None
            with self._lock:
                if self._active is None:
                    for t in self._tasks:
                        if t.status == "pending":
                            t.status = "active"
                            self._active = t
                            task = t
                            break

            if task is None:
                time.sleep(0.1)
                continue

            ws_broadcast({"type": "tasks", "data": self.get_state()})
            logger.info(f"执行任务: {task.type}")

            try:
                if task.type == "move":
                    p = task.params
                    self.robot.move(p.get("vx", 0), p.get("vy", 0), p.get("vyaw", 0))
                    time.sleep(p.get("duration", 1.0))
                    self.robot.stop_move()
                    task.status = "completed"
                    task.result = "done"

                elif task.type == "stop":
                    self.robot.stop_move()
                    self.cancel_all()
                    task.status = "completed"

                elif task.type == "search_area":
                    # 简单的前方扫描模拟
                    p = task.params
                    task.result = f"搜索 {p.get('width', 10)}x{p.get('height', 10)}m"
                    task.status = "completed"

                elif task.type == "follow":
                    task.result = "跟踪模式需要视觉支持"
                    task.status = "completed"

                elif task.type == "return_home":
                    self.robot.stop_move()
                    task.status = "completed"
                    task.result = "已停止（无导航目标）"

                else:
                    task.status = "completed"
                    task.result = f"未实现: {task.type}"

            except Exception as e:
                task.status = "failed"
                task.result = str(e)
                logger.error(f"任务失败: {e}")

            with self._lock:
                self._tasks = [t for t in self._tasks if t.id != task.id]
                self._active = None

            ws_broadcast({"type": "tasks", "data": self.get_state()})


# ============================================================================
# HTTP + WebSocket 服务器
# ============================================================================
robot: RobotConnection = None
task_mgr: TaskManager = None
detector: Detector = None


def create_server(host, port, ws_port, static_dir):
    global robot, task_mgr, detector

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ('/', '/index.html'):
                self._serve_file(os.path.join(static_dir, 'panel.html'), 'text/html')
            elif parsed.path == '/api/status':
                data = {
                    "connected": robot.connected if robot else False,
                    "imu_yaw": robot.imu_yaw if robot else 0,
                    "stats": robot.stats if robot else {},
                    "tasks": task_mgr.get_state() if task_mgr else {},
                }
                self._json(data)
            elif parsed.path == '/api/capture':
                frame = robot.get_frame() if robot else None
                if frame is not None:
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.end_headers()
                    self.wfile.write(jpeg.tobytes())
                else:
                    self.send_error(404)
            else:
                self.send_error(404)

        def do_POST(self):
            parsed = urlparse(self.path)
            params = parse_qs(parsed.query)
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode() if length else ''

            if parsed.path == '/api/connect':
                if not robot.connected:
                    robot.connect()
                    robot.stand()
                    time.sleep(1)
                    self._json({"ok": True, "msg": "已连接并站立"})
                else:
                    self._json({"ok": True, "msg": "已连接"})

            elif parsed.path == '/api/stand':
                logger.info("API: stand requested")
                robot.stand()
                self._json({"ok": True})

            elif parsed.path == '/api/sit':
                logger.info("API: sit requested")
                robot.sit()
                self._json({"ok": True})

            elif parsed.path == '/api/stop':
                logger.info("API: stop_move requested")
                robot.stop_move()
                self._json({"ok": True})

            elif parsed.path == '/api/e_stop':
                logger.info("API: e_stop requested")
                robot.e_stop()
                task_mgr.cancel_all()
                self._json({"ok": True})

            elif parsed.path == '/api/move':
                vx = float(params.get('vx', ['0'])[0])
                vy = float(params.get('vy', ['0'])[0])
                vyaw = float(params.get('vyaw', ['0'])[0])
                logger.info(f"API: move vx={vx} vy={vy} vyaw={vyaw}")
                robot.move(vx, vy, vyaw)
                self._json({"ok": True})

            elif parsed.path == '/api/command':
                text = params.get('text', [''])[0] or body or json.loads(body).get('text', '') if body else ''
                if not text and body:
                    try:
                        text = json.loads(body).get('text', '')
                    except Exception:
                        text = body
                if text:
                    task_mgr.process_command(text)
                    self._json({"ok": True, "text": text})
                else:
                    self._json({"ok": False, "msg": "空指令"})

            elif parsed.path == '/api/detect':
                frame = robot.get_frame() if robot else None
                if frame is not None and detector:
                    dets = detector.detect(frame)
                    self._json({"ok": True, "detections": dets})
                else:
                    self._json({"ok": False, "detections": []})

            else:
                self.send_error(404)

        def _json(self, data):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

        def _serve_file(self, path, ctype):
            if os.path.exists(path):
                with open(path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', len(data))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass

    return HTTPServer((host, port), Handler)


def run_ws(host, port):
    global WS_LOOP
    import websockets

    async def handler(websocket, path):
        WS_CLIENTS.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            WS_CLIENTS.discard(websocket)

    WS_LOOP = asyncio.new_event_loop()
    asyncio.set_event_loop(WS_LOOP)
    WS_LOOP.run_until_complete(websockets.serve(handler, host, port))
    WS_LOOP.run_forever()


# ============================================================================
# 后台广播线程
# ============================================================================
def broadcast_loop():
    logger.info("广播线程启动")
    frame_count = 0
    while True:
        try:
            if robot and robot.connected:
                # 视频
                frame = robot.get_frame()
                if frame is not None:
                    # 检测
                    dets = []
                    if detector:
                        dets = detector.detect(frame)
                        if dets:
                            frame = detector.annotate(frame, dets)

                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    b64 = base64.b64encode(jpeg.tobytes()).decode()
                    ws_broadcast({"type": "frame", "data": b64, "detections": len(dets)})
                    frame_count += 1
                    if frame_count == 1:
                        logger.info(f"首帧广播成功, {frame.shape}, clients={len(WS_CLIENTS)}")
                else:
                    if frame_count == 0:
                        logger.warning("get_frame() 返回 None，视频不可用")

                # 状态
                ws_broadcast({
                    "type": "status",
                    "imu_yaw": round(robot.imu_yaw, 3),
                    "stats": robot.stats,
                    "tasks": task_mgr.get_state() if task_mgr else {},
                })

            time.sleep(0.15)  # ~6-7 FPS
        except Exception as e:
            logger.warning(f"广播错误: {e}")
            time.sleep(0.5)


# ============================================================================
# Main
# ============================================================================
def main():
    global robot, task_mgr, detector

    host = os.environ.get("GO2W_HOST", "0.0.0.0")
    port = int(os.environ.get("GO2W_PORT", "8000"))
    ws_port = int(os.environ.get("GO2W_WS_PORT", "8001"))
    interface = os.environ.get("GO2W_INTERFACE", "enp65s0")

    robot = RobotConnection(interface)
    task_mgr = TaskManager(robot)
    detector = Detector()

    # 启动时自动连接 Go2W
    try:
        logger.info("连接 Go2W...")
        robot.connect()
        logger.info("Go2W 已连接! 发送站立指令...")
        robot.sport.StandDown()
        time.sleep(1)
        robot.sport.Sit()
        time.sleep(1)
        robot.sport.StandUp()
        time.sleep(2)
        robot.sport.BalanceStand()
        logger.info("已站立")
    except Exception as e:
        logger.warning(f"Go2W 连接失败: {e}，可稍后通过页面连接")

    static_dir = os.path.join(os.path.dirname(__file__), 'static')

    logger.info(f"启动 Web 面板: http://{host}:{port}")
    logger.info(f"WebSocket: ws://{host}:{ws_port}")
    logger.info(f"网卡: {interface}")

    # 启动 WebSocket
    threading.Thread(target=run_ws, args=(host, ws_port), daemon=True).start()

    # 启动任务 worker
    task_mgr.start_worker()

    # 启动广播
    threading.Thread(target=broadcast_loop, daemon=True).start()

    # 启动 HTTP
    server = create_server(host, port, ws_port, static_dir)
    logger.info("就绪! 浏览器打开 http://localhost:8000")
    server.serve_forever()


if __name__ == '__main__':
    main()
