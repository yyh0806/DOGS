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
from ai.tracker import TargetTracker
from audio.capture import AudioCapture

# ---- 搜索路径规划 (内联，无 ROS2 依赖) ----
import math as _math

def plan_lawnmower(width, height, spacing=2.5, origin_x=0.0, origin_y=0.0):
    if spacing <= 0: spacing = 2.5
    waypoints = []
    num_rows = max(1, int(_math.ceil(height / spacing)))
    actual_spacing = height / num_rows
    for row in range(num_rows + 1):
        y = origin_y + min(row * actual_spacing, height)
        x_start = origin_x if row % 2 == 0 else origin_x + width
        if row > 0 and waypoints:
            if abs(waypoints[-1]["x"] - x_start) > 0.01:
                waypoints.append({"x": x_start, "y": y, "yaw": 0.0, "is_scan": False})
        if row % 2 == 0:
            waypoints.append({"x": origin_x, "y": y, "yaw": 0.0, "is_scan": True})
            waypoints.append({"x": origin_x + width, "y": y, "yaw": 0.0, "is_scan": True})
        else:
            waypoints.append({"x": origin_x + width, "y": y, "yaw": _math.pi, "is_scan": True})
            waypoints.append({"x": origin_x, "y": y, "yaw": _math.pi, "is_scan": True})
    return waypoints

def plan_spiral(width, height, spacing=2.5, origin_x=0.0, origin_y=0.0):
    if spacing <= 0: spacing = 2.5
    cx = origin_x + width / 2.0; cy = origin_y + height / 2.0
    max_radius = _math.sqrt(width**2 + height**2) / 2.0
    num_turns = max(3, int(_math.ceil(max_radius / spacing)))
    points_per_turn = 12
    total_points = num_turns * points_per_turn
    waypoints = []
    for i in range(total_points + 1):
        angle = i * 2.0 * _math.pi / points_per_turn
        radius = (i / total_points) * max_radius if total_points > 0 else 0.0
        x = max(origin_x, min(cx + radius * _math.cos(angle), origin_x + width))
        y = max(origin_y, min(cy + radius * _math.sin(angle), origin_y + height))
        waypoints.append({"x": x, "y": y, "yaw": angle, "is_scan": (i % points_per_turn == 0)})
    return waypoints

def _wp_to_moves(waypoints, speed=0.3, ang_speed=0.5):
    """航点列表 → move 任务参数 (死推算: 距离/速度=duration)"""
    tasks = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        dx = b["x"] - a["x"]; dy = b["y"] - a["y"]
        dist = _math.sqrt(dx*dx + dy*dy)
        if dist < 0.05: continue
        target_yaw = _math.atan2(dy, dx)
        if abs(target_yaw) > 0.1:
            dur = round(abs(target_yaw) / ang_speed, 1)
            tasks.append({"type": "move", "priority": 5,
                "params": {"vyaw": ang_speed if target_yaw > 0 else -ang_speed, "duration": dur}})
        dur = round(dist / speed, 1)
        tasks.append({"type": "move", "priority": 5,
            "params": {"vx": speed, "duration": dur}})
    return tasks


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
    STATE_NAMES = {0: "STOPPED", 1: "MOVING", 2: "SITTING", 3: "SEATED", 4: "STANDING", 5: "EMERGENCY"}
    SPORT_MODE_NORMAL = 0; SPORT_MODE_GAIT = 1; SPORT_MODE_SIT = 2
    SPORT_MODE_STANDING = 3; SPORT_MODE_DAMP = 4

    def __init__(self, interface="enp65s0"):
        self.interface = interface; self.sport = None; self.video = None; self.connected = False
        self._imu_yaw = 0.0; self._imu_lock = threading.Lock(); self._imu_count = 0
        self._lidar_count = 0
        self._robot_mode = 0; self._robot_progress = 0.0
        self._robot_velocity = [0.0, 0.0, 0.0]; self._feedback_lock = threading.Lock()
        self._lock = threading.RLock(); self._state = self.STOPPED
        self._vx = 0.0; self._vy = 0.0; self._vyaw = 0.0
        self._last_cmd = 0.0; self._cmd = None
        self._balance_done = False; self._stand_done = threading.Event()
        self._ctrl_ready = threading.Event()

    def connect(self):
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.go2.video.video_client import VideoClient
        from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
        self.factory = ChannelFactory(); self.factory.Init(0, self.interface)
        self.sport = SportClient(enableLease=True); self.sport.SetTimeout(10.0); self.sport.Init()
        self.video = VideoClient(); self.video.SetTimeout(10.0); self.video.Init()
        def on_imu(msg):
            with self._imu_lock: self._imu_yaw = float(msg.imu_state.rpy[2]); self._imu_count += 1
        ch1 = self.factory.CreateRecvChannel('rt/lowstate', LowState_); ch1.SetReader(handler=on_imu)
        # 注意: SportModeState 和 LiDAR 在某些 CycloneDDS 版本会导致 segfault,
        # 暂时禁用, 用 IMU + 状态机内部状态代替
        # ch2 = self.factory.CreateRecvChannel('rt/utlidar/cloud', PointCloud2_); ch2.SetReader(handler=on_lidar)
        # ch3 = self.factory.CreateRecvChannel('rt/sportmodestate', SportModeState_); ch3.SetReader(handler=on_sport_state)
        threading.Thread(target=self._ctrl_loop, daemon=True).start()
        self._ctrl_ready.wait(5); self.connected = True
        logger.info("DDS 连接成功, 控制线程启动")

    def get_frame(self):
        if not self.video: return None
        try:
            code, data = self.video.GetImageSample()
            if code == 0 and data:
                return cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception: pass
        return None

    def _ctrl_loop(self):
        logger.info("CTRL 启动"); self._ctrl_ready.set()
        last_zero_move = 0
        while True:
            with self._feedback_lock: robot_mode = self._robot_mode
            cmd = None
            with self._lock: cmd = self._cmd; self._cmd = None
            if cmd == 'stand': self._do_stand(); last_zero_move = 0; continue
            if cmd == 'sit': self._do_sit(); last_zero_move = 0; continue
            if cmd == 'estop': self._do_estop(); last_zero_move = 0; continue
            state = self.STOPPED; vx = vy = vyaw = 0.0
            with self._lock: state = self._state; vx, vy, vyaw = self._vx, self._vy, self._vyaw
            if state == self.STOPPED:
                now = time.time()
                if now - last_zero_move > 0.5:
                    try: self.sport.Move(0, 0, 0)
                    except Exception as e: logger.error(f"STOPPED Move(0,0,0) 失败: {e}")
                    last_zero_move = now
            elif state == self.MOVING:
                try: self.sport.Move(vx, vy, vyaw)
                except Exception as e: logger.error(f"Move 失败: {e}")
            time.sleep(0.05)

    def _do_stand(self):
        try:
            with self._lock: self._state = self.STANDING
            logger.info("STANDING: StandUp → BalanceStand")
            self.sport.StandUp(); time.sleep(2)
            self.sport.BalanceStand(); time.sleep(0.5)
            self.sport.Move(0, 0, 0)
            with self._lock: self._state = self.STOPPED; self._vx = self._vy = self._vyaw = 0.0; self._last_cmd = 0.0
            logger.info("STANDING: → STOPPED"); self._stand_done.set()
        except Exception as e:
            logger.error(f"站立失败: {e}"); traceback.print_exc()
            with self._lock: self._state = self.STOPPED

    def _do_sit(self):
        try:
            with self._lock: self._state = self.SITTING
            logger.info("SITTING: Move(0,0,0) → StopMove → Damp")
            self.sport.Move(0, 0, 0); time.sleep(0.05)
            self.sport.StopMove(); time.sleep(0.3)
            self.sport.Damp()
            with self._lock: self._state = self.SEATED; self._last_cmd = 0.0
            logger.info("SITTING: → SEATED")
        except Exception as e:
            logger.error(f"坐下失败: {e}"); traceback.print_exc()
            with self._lock: self._state = self.STOPPED

    def _do_estop(self):
        try:
            with self._lock: self._state = self.EMERGENCY
            self.sport.Damp(); logger.info("EMERGENCY: Damp")
            with self._lock: self._last_cmd = 0.0
        except Exception as e: logger.error(f"急停失败: {e}")

    def stand(self):
        with self._lock: self._cmd = 'stand'
        logger.info("API: stand 入队")

    def sit(self):
        with self._lock: self._cmd = 'sit'
        logger.info("API: sit 入队")

    def e_stop(self):
        with self._lock: self._cmd = 'estop'
        logger.info("API: estop 入队")

    def move(self, vx, vy, vyaw):
        with self._lock:
            if self._state not in (self.STOPPED, self.MOVING): return
            self._state = self.MOVING; self._vx = vx; self._vy = vy; self._vyaw = vyaw
            self._last_cmd = time.time()

    def stop_move(self):
        with self._lock:
            if self._state not in (self.MOVING,): return
            self._state = self.STOPPED; self._vx = self._vy = self._vyaw = 0.0; self._last_cmd = 0.0
        logger.info("API: stop → STOPPED")

    def start_watchdog(self):
        def wd():
            while True:
                state, last = self.STOPPED, 0.0
                with self._lock: state = self._state; last = self._last_cmd
                if state == self.MOVING and last > 0 and time.time() - last > 0.3:
                    logger.info("看门狗: 0.3s 无指令, 自动停止"); self.stop_move()
                time.sleep(0.1)
        threading.Thread(target=wd, daemon=True).start(); logger.info("看门狗启动")

    @property
    def imu_yaw(self):
        with self._imu_lock: return self._imu_yaw

    @property
    def robot_mode(self):
        with self._feedback_lock: return self._robot_mode

    @property
    def robot_velocity(self):
        with self._feedback_lock: return list(self._robot_velocity)

    @property
    def stats(self):
        with self._feedback_lock:
            return {"imu_count": self._imu_count, "lidar_count": self._lidar_count,
                    "robot_mode": self._robot_mode, "robot_progress": self._robot_progress,
                    "robot_velocity": list(self._robot_velocity)}


# ============================================================================
# 任务队列
# ============================================================================
import uuid

class Task:
    def __init__(self, task_type, params=None, priority=5):
        self.id = uuid.uuid4().hex[:8]; self.type = task_type
        self.params = params or {}; self.priority = priority
        self.status = "pending"; self.result = None; self.created_at = time.time()
    def to_dict(self):
        return {"id": self.id, "type": self.type, "params": self.params,
                "priority": self.priority, "status": self.status,
                "result": self.result, "created_at": self.created_at}


class TaskManager:
    def __init__(self, robot, vlm_engine=None, detector=None):
        self.robot = robot; self.vlm = vlm_engine; self.detector = detector
        self._lock = threading.Lock(); self._tasks = []; self._active = None
        self._running = False; self._follow_active = False; self._search_targets = []
        self._tracker = TargetTracker(vlm_engine, robot, detector) if (vlm_engine and vlm_engine.loaded) else None

    def add(self, task):
        with self._lock: self._tasks.append(task)
        ws_broadcast({"type": "tasks", "data": self.get_state()})

    def add_list(self, tasks):
        with self._lock:
            for i, t in enumerate(tasks):
                if t.priority == 5: t.priority = max(1, 8 - i)
                self._tasks.append(t)
        ws_broadcast({"type": "tasks", "data": self.get_state()})

    def cancel_all(self):
        with self._lock:
            if self._active: self._active.status = "cancelled"
            self._active = None; self._tasks.clear()
        self.robot.stop_move()
        if self._tracker and self._follow_active:
            self._tracker.stop(); self._follow_active = False
        ws_broadcast({"type": "tasks", "data": self.get_state()})

    def get_state(self):
        with self._lock:
            return {"active": self._active.to_dict() if self._active else None,
                    "pending": [t.to_dict() for t in self._tasks if t.status == "pending"],
                    "completed_count": sum(1 for t in self._tasks if t.status == "completed")}

    def process_command(self, text):
        threading.Thread(target=self._process_command_bg, args=(text,), daemon=True).start()

    def _process_command_bg(self, text):
        try:
            result = self._vlm_parse_command(text) if (self.vlm and self.vlm.loaded) else self._fallback_parse(text)
            response = result.get("response", ""); tasks = result.get("tasks", [])
            logger.info(f"指令解析: '{text}' → response='{response}' tasks={len(tasks)}")
            ws_broadcast({"type": "vlm", "data": {"text": text, "response": response, "tasks": tasks}})
            if tasks:
                self.add_list([Task(t.get("type", "move"), t.get("params", {}), t.get("priority", 5)) for t in tasks])
        except Exception as e:
            logger.error(f"指令处理失败: {e}"); traceback.print_exc()

    def _vlm_parse_command(self, text):
        sys_prompt = """你是机器狗指令解析器。把用户中文指令转成JSON任务列表。

任务类型和参数:
- move: {"vx":前进速度m/s, "vy":侧移, "vyaw":旋转(正=左转), "duration":秒}
- follow: {"target":"目标"}
- search_area: {"pattern":"lawnmower", "width":米, "height":米}
- stop: {}
- return_home: {}

示例:
输入"前进两米然后左转"
输出: {"tasks":[{"type":"move","priority":8,"params":{"vx":0.5,"duration":4.0}},{"type":"move","priority":7,"params":{"vyaw":0.5,"duration":3.0}}]}

输入"搜索这个房间"
输出: {"tasks":[{"type":"search_area","priority":5,"params":{"pattern":"lawnmower","width":8,"height":8}}]}

输入"跟着前面的人"
输出: {"tasks":[{"type":"follow","priority":8,"params":{"target":"前面的人"}}]}

只输出JSON, 不要解释, 不要markdown代码块。"""
        response = self.vlm.chat([
            {"role": "system", "content": sys_prompt},
            {"role": "user", "content": text}
        ], max_new_tokens=512)
        import re as _re, json as _json
        logger.info(f"VLM 原始响应: {response[:200]}")
        try:
            clean = _re.sub(r'```(?:json)?\s*', '', response)
            clean = _re.sub(r'```\s*$', '', clean)
            clean = _re.sub(r'//[^\n]*', '', clean)
            m = _re.search(r'\{', clean)
            if m:
                start = m.start(); depth = 0; end = start
                for i, ch in enumerate(clean[start:], start):
                    if ch == '{': depth += 1
                    elif ch == '}':
                        depth -= 1
                        if depth == 0: end = i + 1; break
                data = _json.loads(clean[start:end])
                if "tasks" in data: return data
        except Exception as e:
            logger.warning(f"VLM JSON 解析失败: {e}")
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
                            t.status = "active"; self._active = t; task = t; break
            if task is None: time.sleep(0.1); continue
            ws_broadcast({"type": "tasks", "data": self.get_state()})
            try:
                p = task.params
                if task.type in ("move", "navigate"):
                    duration = p.get("duration", 1.0); vx = p.get("vx", 0); vy = p.get("vy", 0); vyaw = p.get("vyaw", 0)
                    end_time = time.time() + duration
                    while time.time() < end_time:
                        self.robot.move(vx, vy, vyaw); time.sleep(0.1)
                    self.robot.stop_move(); task.status = "completed"
                elif task.type == "stop":
                    self.robot.stop_move(); self.cancel_all(); task.status = "completed"
                elif task.type == "follow":
                    self._execute_follow(task)
                elif task.type == "search_area":
                    self._execute_search(task)
                elif task.type == "return_home":
                    logger.info("return_home: 无定位，原地停住"); self.robot.stop_move(); task.status = "completed"
                else:
                    logger.warning(f"未知任务类型: {task.type}, 跳过"); task.status = "completed"
            except Exception as e:
                task.status = "failed"; task.result = str(e)
            with self._lock:
                self._tasks = [t for t in self._tasks if t.id != task.id]; self._active = None
            ws_broadcast({"type": "tasks", "data": self.get_state()})

    # ------------------------------------------------------------------
    # follow — 视觉跟踪
    # ------------------------------------------------------------------
    def _execute_follow(self, task):
        target = task.params.get("target", "")
        if not target: task.status = "failed"; task.result = "未指定跟踪目标"; return
        if not self._tracker: task.status = "failed"; task.result = "VLM未加载, 无法跟踪"; return
        logger.info(f"开始跟踪: {target}")
        self._follow_active = True
        ws_broadcast({"type": "follow", "data": {"status": "started", "target": target}})
        self._tracker.start_follow(target)
        while self._follow_active and self._tracker.state != TargetTracker.IDLE:
            time.sleep(0.5)
        self._follow_active = False; task.status = "completed"; task.result = "跟踪结束"
        ws_broadcast({"type": "follow", "data": {"status": "stopped"}})

    # ------------------------------------------------------------------
    # search_area — 覆盖路径搜索
    # ------------------------------------------------------------------
    def _execute_search(self, task):
        p = task.params
        width = p.get("width", 10); height = p.get("height", 10)
        spacing = p.get("spacing", 2.5); pattern = p.get("pattern", "lawnmower"); speed = p.get("speed", 0.3)
        logger.info(f"搜索: {pattern} {width}x{height}m spacing={spacing}")
        wp = plan_lawnmower(width, height, spacing) if pattern != "spiral" else plan_spiral(width, height, spacing)
        move_tasks = _wp_to_moves(wp, speed=speed)
        logger.info(f"生成 {len(move_tasks)} 个移动任务")
        self._search_targets.clear()

        for i, mt in enumerate(move_tasks):
            with self._lock:
                if self._active is None or self._active.id != task.id:
                    logger.info("搜索被取消"); self.robot.stop_move(); task.status = "cancelled"; return
            mp = mt["params"]
            duration = mp.get("duration", 1.0); vx = mp.get("vx", 0); vy = mp.get("vy", 0); vyaw = mp.get("vyaw", 0)
            logger.info(f"搜索 {i+1}/{len(move_tasks)}: move(vx={vx},vyaw={vyaw},dur={duration})")
            end_time = time.time() + duration
            while time.time() < end_time:
                self.robot.move(vx, vy, vyaw)
                if vx != 0 and self.detector:
                    frame = self.robot.get_frame()
                    if frame is not None:
                        dets = self.detector.detect(frame)
                        for d in (dets or []):
                            label = d.get("class_name", d.get("label", "?"))
                            conf = d.get("confidence", d.get("score", 0))
                            if conf > 0.5:
                                found = f"{label}({conf:.0%})"
                                if found not in self._search_targets:
                                    self._search_targets.append(found)
                                    logger.info(f"搜索发现: {found}")
                                    ws_broadcast({"type": "search", "data": {"found": self._search_targets}})
                time.sleep(0.1)
            self.robot.stop_move(); time.sleep(0.2)

        task.status = "completed"
        task.result = {"waypoints": len(wp), "moves": len(move_tasks), "found": self._search_targets}
        logger.info(f"搜索完成, 发现: {self._search_targets}")
        ws_broadcast({"type": "search", "data": {"status": "done", "found": self._search_targets}})


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
                self._json({"connected": robot.connected if robot else False,
                            "imu_yaw": robot.imu_yaw if robot else 0,
                            "stats": robot.stats if robot else {},
                            "tasks": task_mgr.get_state() if task_mgr else {}})
            else: self.send_error(404)

        def do_POST(self):
            p = urlparse(self.path); q = parse_qs(p.query)
            L = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(L).decode() if L else ''
            if p.path == '/api/connect':
                if not robot.connected: robot.connect(); robot.stand()
                self._json({"ok": True, "msg": "已连接"})
            elif p.path == '/api/stand': robot.stand(); self._json({"ok": True})
            elif p.path == '/api/sit': robot.sit(); self._json({"ok": True})
            elif p.path == '/api/stop': robot.stop_move(); self._json({"ok": True})
            elif p.path == '/api/e_stop': robot.e_stop(); task_mgr.cancel_all(); self._json({"ok": True})
            elif p.path == '/api/move':
                robot.move(float(q.get('vx', ['0'])[0]), float(q.get('vy', ['0'])[0]), float(q.get('vyaw', ['0'])[0]))
                self._json({"ok": True})
            elif p.path == '/api/command':
                text = q.get('text', [''])[0] or body
                if body:
                    try: text = json.loads(body).get('text', '')
                    except Exception: text = body
                if text: task_mgr.process_command(text)
                self._json({"ok": True, "text": text})
            else: self.send_error(404)

        def _json(self, d):
            self.send_response(200); self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*'); self.end_headers()
            self.wfile.write(json.dumps(d, ensure_ascii=False).encode())

        def _serve(self, path, ct):
            if os.path.exists(path):
                with open(path, 'rb') as f: data = f.read()
                self.send_response(200); self.send_header('Content-Type', ct)
                self.send_header('Content-Length', len(data))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers(); self.wfile.write(data)
            else: self.send_error(404)

        def log_message(self, *a): pass

    return HTTPServer((host, port), H)


def run_ws(host, port):
    global WS_LOOP
    import websockets
    async def h(ws, path):
        WS_CLIENTS.add(ws)
        try: await ws.wait_closed()
        finally: WS_CLIENTS.discard(ws)
    WS_LOOP = asyncio.new_event_loop(); asyncio.set_event_loop(WS_LOOP)
    WS_LOOP.run_until_complete(websockets.serve(h, host, port)); WS_LOOP.run_forever()


def broadcast_loop():
    logger.info("广播启动")
    while True:
        try:
            if robot and robot.connected:
                frame = robot.get_frame(); dets = []
                if frame is not None:
                    if detector:
                        dets = detector.detect(frame)
                        if dets: frame = detector.annotate(frame, dets)
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    ws_broadcast({"type": "frame", "data": base64.b64encode(jpeg.tobytes()).decode(),
                                  "detections": len(dets)})
                ws_broadcast({"type": "status", "imu_yaw": round(robot.imu_yaw, 3),
                              "stats": robot.stats, "tasks": task_mgr.get_state() if task_mgr else {}})
            time.sleep(0.15)
        except Exception as e: logger.warning(f"广播: {e}"); time.sleep(0.5)


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
    if not vlm.load(): logger.warning("VLM 加载失败，将使用关键词匹配"); vlm = None
    else: logger.info("VLM 就绪")

    detector = Detector()
    task_mgr = TaskManager(robot, vlm_engine=vlm, detector=detector)

    try:
        logger.info("连接 Go2W ..."); robot.connect()
        logger.info("已连接，执行站立序列 ...")
        robot._stand_done.clear()
        with robot._lock: robot._cmd = 'stand'
        robot._stand_done.wait(10); robot.start_watchdog()
        logger.info("就绪 — STOPPED 状态，机器人静止")
    except Exception as e: logger.warning(f"连接失败: {e}")

    static_dir = os.path.join(os.path.dirname(__file__), 'static')
    logger.info(f"Web: http://{host}:{port}  WS: ws://{host}:{ws_port}")
    threading.Thread(target=run_ws, args=(host, ws_port), daemon=True).start()
    task_mgr.start_worker()
    threading.Thread(target=broadcast_loop, daemon=True).start()
    server = create_server(host, port, static_dir); server.serve_forever()


if __name__ == '__main__':
    main()