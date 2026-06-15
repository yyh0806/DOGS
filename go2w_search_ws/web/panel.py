#!/usr/bin/env python3
"""Go2W 独立 Web 控制面板。

状态机:
  DISCONNECTED → STANDING → STOPPED (BalanceStand 静止)
  STOPPED → MOVING (Move @ 20Hz)
  MOVING → STOPPED (StopMove + BalanceStand)
  STOPPED → SITTING → SEATED (Sit)
  SEATED → STANDING → STOPPED (RiseSit + BalanceStand)
  任意 → EMERGENCY (Damp)

规则:
  1. 所有 SDK 调用只在控制线程内执行
  2. STOPPED = StopMove + BalanceStand, 每 0.5s 发 Move(0,0,0) 防超时
  3. MOVING = BalanceStand + 持续 Move @ 20Hz
  4. 订阅 rt/sportmodestate 获取机器人真实状态反馈
  5. 看门狗: MOVING 状态 0.3s 无指令自动停止
"""

import asyncio, base64, json, os, sys, time, threading, logging, traceback, cv2, numpy as np
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
# 状态机: STOPPED / MOVING / SITTING / SEATED / STANDING / EMERGENCY
# ============================================================================
class RobotConnection:
    STOPPED, MOVING, SITTING, SEATED, STANDING, EMERGENCY = range(6)

    STATE_NAMES = {
        0: "STOPPED", 1: "MOVING", 2: "SITTING",
        3: "SEATED", 4: "STANDING", 5: "EMERGENCY"
    }

    # SportModeState.mode 值 (从机器人反馈)
    SPORT_MODE_NORMAL = 0       # 普通/空闲
    SPORT_MODE_GAIT = 1         # 步态/行走
    SPORT_MODE_SIT = 2          # 坐下
    SPORT_MODE_STANDING = 3     # 站立中
    SPORT_MODE_DAMP = 4         # 阻尼/趴下

    def __init__(self, interface="enp65s0"):
        self.interface = interface
        self.sport = None
        self.video = None
        self.connected = False
        # IMU / LiDAR 统计
        self._imu_yaw = 0.0
        self._imu_lock = threading.Lock()
        self._imu_count = 0
        self._lidar_count = 0
        # 机器人反馈状态
        self._robot_mode = 0     # SportModeState.mode
        self._robot_progress = 0.0
        self._robot_velocity = [0.0, 0.0, 0.0]
        self._feedback_lock = threading.Lock()
        # 状态机变量
        self._lock = threading.RLock()
        self._state = self.STOPPED
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._last_cmd = 0.0             # 最后一次收到 move() 的时间戳
        self._cmd = None                 # 待处理命令: 'stand' | 'sit' | 'estop'
        self._balance_done = False       # BalanceStand 是否已执行
        self._stand_done = threading.Event()
        self._ctrl_ready = threading.Event()  # 控制线程就绪

    # ========================================================================
    # 连接 + 启动控制线程
    # ========================================================================
    def connect(self):
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.go2.video.video_client import VideoClient
        from unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_ import SportModeState_
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
            self._lidar_count += 1

        def on_sport_state(msg):
            with self._feedback_lock:
                self._robot_mode = msg.mode
                self._robot_progress = msg.progress
                self._robot_velocity = [msg.velocity[0], msg.velocity[1], msg.velocity[2]]

        ch1 = self.factory.CreateRecvChannel('rt/lowstate', LowState_)
        ch1.SetReader(handler=on_imu)
        ch2 = self.factory.CreateRecvChannel('rt/utlidar/cloud', PointCloud2_)
        ch2.SetReader(handler=on_lidar)
        ch3 = self.factory.CreateRecvChannel('rt/sportmodestate', SportModeState_)
        ch3.SetReader(handler=on_sport_state)

        threading.Thread(target=self._ctrl_loop, daemon=True).start()
        self._ctrl_ready.wait(5)  # 等待控制线程就绪
        self.connected = True
        logger.info("DDS 连接成功, 控制线程启动, 已订阅 rt/sportmodestate")

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

    # ========================================================================
    # 控制线程 — 唯一允许调用 SDK 的地方
    # ========================================================================
    def _ctrl_loop(self):
        logger.info("CTRL 启动")
        self._ctrl_ready.set()

        last_zero_move = 0  # STOPPED 状态发送 Move(0,0,0) 的时间

        while True:
            # --- 0. 读取机器人反馈 ---
            robot_mode = 0
            with self._feedback_lock:
                robot_mode = self._robot_mode

            # --- 1. 处理特殊命令 ---
            cmd = None
            with self._lock:
                cmd = self._cmd
                self._cmd = None

            if cmd == 'stand':
                self._do_stand()
                last_zero_move = 0
                continue
            if cmd == 'sit':
                self._do_sit()
                last_zero_move = 0
                continue
            if cmd == 'estop':
                self._do_estop()
                last_zero_move = 0
                continue

            # --- 2. 读取当前状态 ---
            state = self.STOPPED
            vx = vy = vyaw = 0.0
            with self._lock:
                state = self._state
                vx, vy, vyaw = self._vx, self._vy, self._vyaw

            # --- 3. 状态机主循环 ---
            if state == self.STOPPED:
                # STOPPED: 保持 BalanceStand + 定期 Move(0,0,0) 防超时
                now = time.time()
                if now - last_zero_move > 0.5:
                    try:
                        self.sport.Move(0, 0, 0)
                    except Exception as e:
                        logger.error(f"STOPPED Move(0,0,0) 失败: {e}")
                    last_zero_move = now

            elif state == self.MOVING:
                # MOVING: 发送速度指令
                try:
                    self.sport.Move(vx, vy, vyaw)
                except Exception as e:
                    logger.error(f"Move 失败: {e}")

            # SEATED / SITTING / STANDING / EMERGENCY: 不发送任何命令

            time.sleep(0.05)  # 20Hz

    # ---- 特殊命令实现 (在控制线程内执行) ----
    def _do_stand(self):
        """站立序列: StandUp → BalanceStand → STOPPED"""
        try:
            with self._lock:
                self._state = self.STANDING
            logger.info("STANDING: StandUp → BalanceStand")
            code = self.sport.StandUp()
            logger.info(f"STANDING: StandUp → code={code}")
            time.sleep(2)
            code = self.sport.BalanceStand()
            logger.info(f"STANDING: BalanceStand → code={code}")
            time.sleep(0.5)
            self.sport.Move(0, 0, 0)
            with self._lock:
                self._state = self.STOPPED
                self._vx = self._vy = self._vyaw = 0.0
                self._last_cmd = 0.0
            logger.info("STANDING: → STOPPED")
            self._stand_done.set()
        except Exception as e:
            logger.error(f"站立失败: {e}")
            traceback.print_exc()
            with self._lock:
                self._state = self.STOPPED

    def _do_sit(self):
        """坐下: Move(0,0,0) → StopMove → Damp → SEATED"""
        try:
            with self._lock:
                self._state = self.SITTING
            logger.info("SITTING: Move(0,0,0) → StopMove → Damp")
            self.sport.Move(0, 0, 0)
            time.sleep(0.05)
            code = self.sport.StopMove()
            logger.info(f"SITTING: StopMove → code={code}")
            time.sleep(0.3)
            code = self.sport.Damp()
            logger.info(f"SITTING: Damp → code={code}")
            with self._lock:
                self._state = self.SEATED
                self._last_cmd = 0.0
            logger.info("SITTING: → SEATED")
        except Exception as e:
            logger.error(f"坐下失败: {e}")
            traceback.print_exc()
            with self._lock:
                self._state = self.STOPPED

    def _do_estop(self):
        """急停: Damp → EMERGENCY"""
        try:
            with self._lock:
                self._state = self.EMERGENCY
            code = self.sport.Damp()
            logger.info(f"EMERGENCY: Damp → code={code}")
            with self._lock:
                self._last_cmd = 0.0
        except Exception as e:
            logger.error(f"急停失败: {e}")
            traceback.print_exc()

    # ========================================================================
    # 公开 API — 非阻塞，只设标志/状态
    # ========================================================================
    def stand(self):
        with self._lock:
            self._cmd = 'stand'
        logger.info("API: stand 入队")

    def sit(self):
        with self._lock:
            self._cmd = 'sit'
        logger.info("API: sit 入队")

    def e_stop(self):
        with self._lock:
            self._cmd = 'estop'
        logger.info("API: estop 入队")

    def move(self, vx, vy, vyaw):
        """设置运动速度: STOPPED → MOVING, 或更新 MOVING 速度"""
        with self._lock:
            if self._state not in (self.STOPPED, self.MOVING):
                logger.info(f"MOVE: 忽略 (state={self.STATE_NAMES.get(self._state, '?')})")
                return
            self._state = self.MOVING
            self._vx = vx
            self._vy = vy
            self._vyaw = vyaw
            self._last_cmd = time.time()

    def stop_move(self):
        """停止运动: MOVING → STOPPED (StopMove + BalanceStand)"""
        with self._lock:
            if self._state not in (self.MOVING,):
                return
            self._state = self.STOPPED
            self._vx = self._vy = self._vyaw = 0.0
            self._last_cmd = 0.0
        # 控制线程下次循环会进入 STOPPED 分支: 发送 Move(0,0,0) 保持静止
        logger.info("API: stop → STOPPED")

    def start_watchdog(self):
        """看门狗: MOVING 状态 0.3s 无新 move() 自动停止"""
        def wd():
            while True:
                state, last = self.STOPPED, 0.0
                with self._lock:
                    state = self._state
                    last = self._last_cmd
                if state == self.MOVING and last > 0 and time.time() - last > 0.3:
                    logger.info("看门狗: 0.3s 无指令, 自动停止")
                    self.stop_move()
                time.sleep(0.1)
        threading.Thread(target=wd, daemon=True).start()
        logger.info("看门狗启动")

    @property
    def imu_yaw(self):
        with self._imu_lock:
            return self._imu_yaw

    @property
    def robot_mode(self):
        with self._feedback_lock:
            return self._robot_mode

    @property
    def robot_velocity(self):
        with self._feedback_lock:
            return list(self._robot_velocity)

    @property
    def stats(self):
        with self._feedback_lock:
            return {
                "imu_count": self._imu_count,
                "lidar_count": self._lidar_count,
                "robot_mode": self._robot_mode,
                "robot_progress": self._robot_progress,
                "robot_velocity": list(self._robot_velocity),
            }


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

    def add(self, task):
        with self._lock:
            self._tasks.append(task)
        ws_broadcast({"type": "tasks", "data": self.get_state()})

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
        threading.Thread(target=self._process_command_bg, args=(text,), daemon=True).start()

    def _process_command_bg(self, text):
        try:
            result = self._vlm_parse_command(text) if (self.vlm and self.vlm.loaded) else self._fallback_parse(text)
            response = result.get("response", "")
            tasks = result.get("tasks", [])
            ws_broadcast({"type": "vlm", "data": {"text": text, "response": response, "tasks": tasks}})
            if tasks:
                self.add_list([Task(t.get("type", "navigate"), t.get("params", {}), t.get("priority", 5)) for t in tasks])
            else:
                self._handle_simple(text)
        except Exception as e:
            logger.error(f"指令处理失败: {e}")

    def _handle_simple(self, text):
        if "停" in text: self.cancel_all()
        elif "前进" in text or "向前" in text: self.add(Task("move", {"vx": 0.5, "duration": 2.0}, 6))
        elif "后退" in text or "向后" in text: self.add(Task("move", {"vx": -0.5, "duration": 2.0}, 6))
        elif "左转" in text: self.add(Task("move", {"vyaw": 0.5, "duration": 2.0}, 6))
        elif "右转" in text: self.add(Task("move", {"vyaw": -0.5, "duration": 2.0}, 6))

    def _vlm_parse_command(self, text):
        sys_prompt = """你是一个机器狗助手。将用户指令分解为任务序列。
可用任务：navigate/move/follow/search_area/observe/wait/stop/return_home
输出 JSON: {"understanding":"...","tasks":[{"type":"...","priority":1-10,"params":{...}}],"response":"..."}"""
        response = self.vlm.chat([
            {"role": "system", "content": sys_prompt},
            {"role": "user", "content": text}
        ], max_new_tokens=300)
        import re as _re, json as _json
        try:
            m = _re.search(r'\{[^}]*"tasks"[^}]*\}', response, _re.DOTALL)
            if m: return _json.loads(m.group())
        except Exception: pass
        return self._fallback_parse(text)

    @staticmethod
    def _fallback_parse(text):
        r = {"understanding": text, "tasks": [], "response": ""}
        if "跟着" in text or "跟随" in text:
            for kw in ["跟着", "跟随"]:
                if kw in text: target = text[text.index(kw)+len(kw):].strip().rstrip("。，！？")
            r["tasks"] = [{"type": "follow", "priority": 8, "params": {"target": target}}]
            r["response"] = f"跟踪{target}"
        elif "搜索" in text or "找" in text:
            r["tasks"] = [{"type": "search_area", "priority": 5, "params": {"pattern": "lawnmower", "width": 10, "height": 10}}]
            r["response"] = "开始搜索"
        elif "停" in text:
            r["tasks"] = [{"type": "stop", "priority": 10, "params": {}}]; r["response"] = "已停止"
        elif "回来" in text or "返回" in text:
            r["tasks"] = [{"type": "return_home", "priority": 7, "params": {}}]; r["response"] = "返回"
        elif "前进" in text or "向前" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": 0.5, "duration": 2.0}}]; r["response"] = "前进"
        elif "后退" in text or "向后" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": -0.5, "duration": 2.0}}]; r["response"] = "后退"
        elif "左转" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": 0.5, "duration": 2.0}}]; r["response"] = "左转"
        elif "右转" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": -0.5, "duration": 2.0}}]; r["response"] = "右转"
        else:
            r["response"] = f"收到: {text}"
        return r

    def start_worker(self):
        self._running = True
        threading.Thread(target=self._worker, daemon=True).start()

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
            try:
                if task.type == "move":
                    p = task.params
                    self.robot.move(p.get("vx", 0), p.get("vy", 0), p.get("vyaw", 0))
                    time.sleep(p.get("duration", 1.0))
                    self.robot.stop_move()
                    task.status = "completed"
                elif task.type == "stop":
                    self.robot.stop_move(); self.cancel_all(); task.status = "completed"
                else:
                    task.status = "completed"
            except Exception as e:
                task.status = "failed"; task.result = str(e)
            with self._lock:
                self._tasks = [t for t in self._tasks if t.id != task.id]
                self._active = None
            ws_broadcast({"type": "tasks", "data": self.get_state()})


# ============================================================================
# HTTP + WebSocket
# ============================================================================
robot = task_mgr = detector = None

def create_server(host, port, static_dir):
    global robot, task_mgr, detector

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            p = urlparse(self.path)
            if p.path in ('/', '/index.html'):
                self._serve(os.path.join(static_dir, 'panel.html'), 'text/html')
            elif p.path == '/api/status':
                self._json({
                    "connected": robot.connected if robot else False,
                    "imu_yaw": robot.imu_yaw if robot else 0,
                    "stats": robot.stats if robot else {},
                    "tasks": task_mgr.get_state() if task_mgr else {},
                })
            else:
                self.send_error(404)

        def do_POST(self):
            p = urlparse(self.path)
            q = parse_qs(p.query)
            L = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(L).decode() if L else ''

            if p.path == '/api/connect':
                if not robot.connected:
                    robot.connect()
                    robot.stand()
                self._json({"ok": True, "msg": "已连接"})

            elif p.path == '/api/stand':
                robot.stand()
                self._json({"ok": True})

            elif p.path == '/api/sit':
                robot.sit()
                self._json({"ok": True})

            elif p.path == '/api/stop':
                robot.stop_move()
                self._json({"ok": True})

            elif p.path == '/api/e_stop':
                robot.e_stop()
                task_mgr.cancel_all()
                self._json({"ok": True})

            elif p.path == '/api/move':
                vx = float(q.get('vx', ['0'])[0])
                vy = float(q.get('vy', ['0'])[0])
                vyaw = float(q.get('vyaw', ['0'])[0])
                robot.move(vx, vy, vyaw)
                self._json({"ok": True})

            elif p.path == '/api/command':
                text = q.get('text', [''])[0] or body
                if body:
                    try: text = json.loads(body).get('text', '')
                    except Exception: text = body
                if text:
                    task_mgr.process_command(text)
                self._json({"ok": True, "text": text})

            else:
                self.send_error(404)

        def _json(self, d):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(d, ensure_ascii=False).encode())

        def _serve(self, path, ct):
            if os.path.exists(path):
                with open(path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', ct)
                self.send_header('Content-Length', len(data))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)

        def log_message(self, *a):
            pass

    return HTTPServer((host, port), H)


def run_ws(host, port):
    global WS_LOOP
    import websockets

    async def h(ws, path):
        WS_CLIENTS.add(ws)
        try: await ws.wait_closed()
        finally: WS_CLIENTS.discard(ws)

    WS_LOOP = asyncio.new_event_loop()
    asyncio.set_event_loop(WS_LOOP)
    WS_LOOP.run_until_complete(websockets.serve(h, host, port))
    WS_LOOP.run_forever()


def broadcast_loop():
    logger.info("广播启动")
    while True:
        try:
            if robot and robot.connected:
                frame = robot.get_frame()
                dets = []
                if frame is not None:
                    if detector:
                        dets = detector.detect(frame)
                        if dets: frame = detector.annotate(frame, dets)
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    ws_broadcast({"type": "frame", "data": base64.b64encode(jpeg.tobytes()).decode(),
                                  "detections": len(dets)})
                ws_broadcast({"type": "status", "imu_yaw": round(robot.imu_yaw, 3),
                              "stats": robot.stats,
                              "tasks": task_mgr.get_state() if task_mgr else {}})
            time.sleep(0.15)
        except Exception as e:
            logger.warning(f"广播: {e}")
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
    logger.info("加载 VLM 模型...")
    vlm = VLMEngine()
    if not vlm.load():
        logger.warning("VLM 加载失败，将使用关键词匹配")
        vlm = None
    else:
        logger.info("VLM 就绪")

    task_mgr = TaskManager(robot, vlm_engine=vlm)
    detector = Detector()

    # 连接 + 自动站立
    try:
        logger.info("连接 Go2W ...")
        robot.connect()
        logger.info("已连接，执行站立序列 ...")
        robot._stand_done.clear()
        with robot._lock:
            robot._cmd = 'stand'
        robot._stand_done.wait(10)
        robot.start_watchdog()
        logger.info("就绪 — STOPPED 状态，机器人静止")
    except Exception as e:
        logger.warning(f"连接失败: {e}")

    static_dir = os.path.join(os.path.dirname(__file__), 'static')
    logger.info(f"Web: http://{host}:{port}  WS: ws://{host}:{ws_port}")

    threading.Thread(target=run_ws, args=(host, ws_port), daemon=True).start()
    task_mgr.start_worker()
    threading.Thread(target=broadcast_loop, daemon=True).start()

    server = create_server(host, port, static_dir)
    server.serve_forever()


if __name__ == '__main__':
    main()