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
        self._last_logged_vx = self._last_logged_vy = self._last_logged_vyaw = None
        self._start_time = time.time()  # 启动时间, 用于move保护
        self._lock = threading.RLock(); self._state = self.STOPPED
        self._vx = 0.0; self._vy = 0.0; self._vyaw = 0.0
        self._last_cmd = 0.0; self._cmd = None
        self._balance_done = False; self._stand_done = threading.Event()
        self._ctrl_ready = threading.Event()

    def connect(self):
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.go2.video.video_client import VideoClient
        self.factory = ChannelFactory()
        # 先尝试指定网卡，失败则自动检测
        if not self.factory.Init(0, self.interface):
            logger.warning(f"网卡 {self.interface} 初始化失败, 尝试自动检测")
            self.factory.Init(0, None)
        self.sport = SportClient(enableLease=True); self.sport.SetTimeout(10.0); self.sport.Init()
        self.video = VideoClient(); self.video.SetTimeout(10.0); self.video.Init()
        # IMU 子进程延迟5秒启动, 确保lease先获取
        threading.Timer(5.0, self._start_imu_subprocess).start()
        threading.Thread(target=self._ctrl_loop, daemon=True).start()
        self._ctrl_ready.wait(5); self.connected = True
        logger.info("DDS 连接成功, 控制线程启动")

    def _start_imu_subprocess(self):
        """独立子进程读取IMU yaw, 避免同进程 reader+writer segfault"""
        import subprocess
        self._imu_file = "/tmp/go2w_imu.json"
        # 用系统python3跑子进程 (只需要cyclonedds, 不需要torch)
        script = (
            "import sys,time,json,multiprocessing\n"
            "from unitree_sdk2py.core.channel import ChannelFactory\n"
            "from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_\n"
            "f=ChannelFactory();f.Init(0,None)\n"
            "st={'yaw':0.0,'count':0}\n"
            "def cb(msg):\n"
            "    st['yaw']=float(msg.imu_state.rpy[2]);st['count']+=1\n"
            "    if st['count']%20==0:\n"
            "        json.dump(st,open('/tmp/go2w_imu.json','w'))\n"
            "f.CreateRecvChannel('rt/lowstate',LowState_).SetReader(handler=cb)\n"
            "while True:time.sleep(1)\n"
        )
        self._imu_proc = subprocess.Popen(
            [sys.executable, "-c", script],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        logger.info(f"IMU 子进程 PID={self._imu_proc.pid}")
        def reader():
            import os
            while True:
                try:
                    if os.path.exists(self._imu_file):
                        with open(self._imu_file) as fp:
                            d = json.load(fp)
                            with self._imu_lock:
                                self._imu_yaw = d.get("yaw", 0.0)
                                self._imu_count = d.get("count", 0)
                except Exception: pass
                time.sleep(0.2)
        threading.Thread(target=reader, daemon=True).start()

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
                try:
                    if vx != self._last_logged_vx or vy != self._last_logged_vy or vyaw != self._last_logged_vyaw:
                        logger.info(f"CTRL Move: sport.Move({vx}, {vy}, {vyaw})")
                        self._last_logged_vx = vx; self._last_logged_vy = vy; self._last_logged_vyaw = vyaw
                    self.sport.Move(vx, vy, vyaw)
                except Exception as e: logger.error(f"Move 失败: {e}")
            time.sleep(0.05)

    def _do_stand(self):
        try:
            with self._lock:
                self._state = self.STANDING
                self._vx = self._vy = self._vyaw = 0.0  # 清零速度防冲
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
        """body frame → Go2W SDK。
        Go2W SDK Move(x,y,z): x=前后(正=前进), y=左右, z=旋转(正=左转)
        """
        with self._lock:
            if self._state not in (self.STOPPED, self.MOVING): return
            self._state = self.MOVING
            self._vx = vx; self._vy = vy; self._vyaw = -vyaw  # Go2W: z正=右转,需反转
            self._last_cmd = time.time()

    def stop_move(self):
        with self._lock:
            if self._state not in (self.MOVING,): return
            self._state = self.STOPPED; self._vx = self._vy = self._vyaw = 0.0; self._last_cmd = 0.0
        logger.info("API: stop → STOPPED")

    # alias for TargetTracker compatibility
    def stop(self): self.stop_move()

    def start_watchdog(self):
        def wd():
            while True:
                state, last = self.STOPPED, 0.0
                with self._lock: state = self._state; last = self._last_cmd
                if state == self.MOVING and last > 0 and time.time() - last > 1.0:
                    logger.info("看门狗: 1.0s 无指令, 自动停止"); self.stop_move()
                time.sleep(0.2)
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
# RosRobotBridge — ROS2 模式下替代 RobotConnection 的控狗实现
# ----------------------------------------------------------------------------
# 不直连狗 SDK, 而是通过容器内常驻的 cmd_publisher.py 发 ROS2 话题
# (/cmd_vel /cmd_pose), 由 NX 上的 nx_motion_node 控狗。
# 公共 API 与 RobotConnection 保持一致 (move/stop_move/stand/sit/e_stop +
# imu_yaw/connected/stats/_lock/_vx 等), 让 TaskManager / broadcast_loop 无感切换。
# ============================================================================
class RosRobotBridge:
    def __init__(self):
        self.connected = False
        self._proc = None           # cmd_publisher.py 子进程
        self._lock = threading.RLock()
        self._vx = 0.0; self._vy = 0.0; self._vyaw = 0.0
        # IMU/yaw 仍从 dog_state.json 读 (ros_to_json 写), 这里复用
        self._imu_yaw = 0.0; self._imu_count = 0
        # NX nx_motion_node 通过 /dog_state 上报的真实狗状态
        self._robot_state = "UNKNOWN"   # STOPPED/MOVING/STANDING/...
        self._state_lock = threading.Lock()

    def connect(self):
        """拉起容器内 cmd_publisher.py, 通过它的 stdin 发指令 / stdout 读状态。"""
        import subprocess
        try:
            # 必须先 source ROS2 环境, 否则容器内 python3 找不到 rclpy
            self._proc = subprocess.Popen(
                ['docker', 'exec', '-i', 'go2w_humble', 'bash', '-c',
                 'source /opt/ros/humble/setup.bash && exec python3 -u /workspace/web/cmd_publisher.py'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, bufsize=1, text=True)
        except Exception as e:
            logger.error(f"启动 cmd_publisher 失败 (容器没起?): {e}")
            raise
        # 后台线程读 stdout (NX /dog_state 上报)
        threading.Thread(target=self._read_state, daemon=True).start()
        self.connected = True
        logger.info("RosRobotBridge 就绪 (经容器 cmd_publisher → NX nx_motion_node)")

    def _read_state(self):
        while self._proc and self._proc.stdout:
            try:
                line = self._proc.stdout.readline()
                if not line:
                    break
                d = json.loads(line)
                if d.get("type") == "dog_state":
                    with self._state_lock:
                        self._robot_state = d.get("state", "UNKNOWN")
            except Exception:
                pass
        logger.info("cmd_publisher stdout 已关闭")

    def _send(self, obj):
        if not self._proc or not self._proc.stdin:
            return
        try:
            self._proc.stdin.write(json.dumps(obj) + "\n")
            self._proc.stdin.flush()
        except Exception as e:
            logger.warning(f"发指令失败 (cmd_publisher 挂了?): {e}")

    def move(self, vx, vy, vyaw):
        with self._lock:
            self._vx = vx; self._vy = vy; self._vyaw = vyaw
        self._send({"type": "vel", "vx": vx, "vy": vy, "vyaw": vyaw})

    def stop_move(self):
        with self._lock:
            self._vx = self._vy = self._vyaw = 0.0
        self._send({"type": "stop"})

    def stand(self): self._send({"type": "pose", "cmd": "stand"})
    def sit(self): self._send({"type": "pose", "cmd": "sit"})
    def e_stop(self): self._send({"type": "pose", "cmd": "estop"})

    @property
    def robot_state(self):
        with self._state_lock:
            return self._robot_state

    @property
    def imu_yaw(self):
        # yaw 仍从 dog_state.json 读 (broadcast_loop 的 _read_ros2_state 已处理)
        # 这里返回 0, 真值由 broadcast_loop 直接推给前端
        return self._imu_yaw

    @property
    def stats(self):
        return {"imu_count": self._imu_count, "robot_mode": 0,
                "robot_velocity": [self._vx, self._vy, self._vyaw]}


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
            ws_broadcast({"type": "follow", "data": {"status": "stopped"}})
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

输入"后退"
输出: {"tasks":[{"type":"move","priority":6,"params":{"vx":-0.5,"duration":2.0}}]}

只输出JSON, 不要解释, 不要markdown代码块。注意: 后退要用vx负数, 不是vyaw!"""
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
                    duration = min(p.get("duration", 1.0), 8.0)  # 最多8秒防失控
                    vx = p.get("vx", 0); vy = p.get("vy", 0); vyaw = p.get("vyaw", 0)
                    end_time = time.time() + duration
                    while time.time() < end_time:
                        # 随时可被 cancel_all / stop 任务中断
                        if task.status == "cancelled": break
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
        # 不阻塞worker! 跟踪在后台线程运行, 通过 cancel_all/stop 停止
        task.status = "completed"
        task.result = "跟踪已启动 (后台运行)"

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
            elif p.path == '/map.js':
                self._serve(os.path.join(static_dir, 'map.js'), 'application/javascript')
            elif p.path == '/api/foxglove':
                # Foxglove bridge URL (需另起 foxglove_bridge 节点, 默认 8765)
                host_ip = os.environ.get("GO2W_PUBLIC_IP", "localhost")
                self._json({"url": f"http://{host_ip}:8080", "ws": f"ws://{host_ip}:8765"})
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
            elif p.path == '/api/search':
                # 表单/地图选区搜索: width,height,spacing,origin_x,origin_y
                w = float(q.get('width', ['8'])[0])
                h = float(q.get('height', ['8'])[0])
                sp = float(q.get('spacing', ['2'])[0])
                ox = float(q.get('origin_x', ['0'])[0])
                oy = float(q.get('origin_y', ['0'])[0])
                pattern = q.get('pattern', ['lawnmower'])[0]
                task_mgr.add_list([{
                    "type": "search_area",
                    "priority": 5,
                    "params": {"pattern": pattern, "width": w, "height": h,
                               "spacing": sp, "origin_x": ox, "origin_y": oy},
                }])
                self._json({"ok": True, "msg": f"搜索 {w}x{h}m 间距{sp}m 已入队"})
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


# ---- 死推算位姿 (阶段A: 没有真SLAM时, 用速度积分近似狗的位置) ----
# 真 SLAM (FAST_LIO) 通了后, 这套会被 slam_subscriber 提供的真里程计覆盖 (阶段B)
_dead_reckon = {"x": 0.0, "y": 0.0, "yaw": 0.0, "trail": [], "last_t": 0.0, "last_dets": []}


def _update_dead_reckon():
    """用当前速度 × dt 积分, 近似狗位移。yaw 用 IMU 真值(准), xy 是近似(无真里程计时)。"""
    if not (robot and robot.connected):
        return
    now = time.time()
    if _dead_reckon["last_t"] == 0.0:
        _dead_reckon["last_t"] = now
        return
    dt = now - _dead_reckon["last_t"]
    if dt > 1.0: dt = 0.15  # 异常间隔, 用默认
    _dead_reckon["last_t"] = now
    yaw = robot.imu_yaw  # 朝向用 IMU 真值
    with robot._lock:
        vx, vy, vyaw = robot._vx, robot._vy, robot._vyaw
    # body frame → world frame 速度 (用 yaw 旋转)
    import math as _m
    cos_y, sin_y = _m.cos(yaw), _m.sin(yaw)
    _dead_reckon["x"] += (vx * cos_y - vy * sin_y) * dt
    _dead_reckon["y"] += (vx * sin_y + vy * cos_y) * dt
    _dead_reckon["yaw"] = yaw
    # 轨迹采样: 移动超过 0.1m 才记一个点
    if not _dead_reckon["trail"]:
        _dead_reckon["trail"].append([round(_dead_reckon["x"], 2), round(_dead_reckon["y"], 2)])
    else:
        lx, ly = _dead_reckon["trail"][-1]
        if _m.hypot(_dead_reckon["x"] - lx, _dead_reckon["y"] - ly) > 0.1:
            _dead_reckon["trail"].append([round(_dead_reckon["x"], 2), round(_dead_reckon["y"], 2)])
            if len(_dead_reckon["trail"]) > 2000:
                _dead_reckon["trail"] = _dead_reckon["trail"][-2000:]


# ---- ROS2 模式: 从容器写的 dog_state.json 读真狗数据 ----
ROS2_STATE_FILE = os.path.join(os.path.dirname(__file__), "dog_state.json")
_ros2_trail = []


def _read_ros2_state():
    """读 ros_to_json.py 写的真狗状态。返回 None 表示文件不存在/过期。"""
    global _ros2_trail
    try:
        with open(ROS2_STATE_FILE) as f:
            d = json.load(f)
        # 数据新鲜度检查 (3秒内的才算有效)
        if time.time() - d.get("last_t", 0) > 3.0:
            return None
        # 轨迹累积 (文件里的trail会被ros_to_json截断, 这里本地保留)
        if d.get("trail"):
            _ros2_trail = d["trail"]
        return d
    except Exception:
        return None


def broadcast_loop():
    logger.info("广播启动")
    slam_counter = 0
    # Mock 模式: 没连狗时, 推假数据让前端布局可见 (转圈走的轨迹)
    mock_t = 0.0
    while True:
        try:
            # ROS2 模式: 从载荷NX经容器桥接来的真狗数据 (优先)
            if os.environ.get("GO2W_USE_ROS2"):
                st = _read_ros2_state()
                if st:
                    # 把扫描点转世界坐标 (机体系 → 用yaw旋转) 供地图显示
                    import math as _rm
                    yaw = st.get("yaw", 0.0)
                    cos_y, sin_y = _rm.cos(yaw), _rm.sin(yaw)
                    scan_pts = []
                    for i, r in enumerate(st.get("ranges", [])):
                        if 0.1 < r < 9.9:
                            ang = -_rm.pi + i * 2 * _rm.pi / max(len(st["ranges"]), 1)
                            lx, ly = r * _rm.cos(ang), r * _rm.sin(ang)
                            scan_pts.append([round(cos_y * lx - sin_y * ly + st["x"], 2),
                                             round(sin_y * lx + cos_y * ly + st["y"], 2)])
                    ws_broadcast({"type": "slam", "data": {
                        "x": round(st.get("x", 0.0), 2), "y": round(st.get("y", 0.0), 2),
                        "yaw": round(st.get("yaw", 0.0), 2),
                        "trail": _ros2_trail,
                        "map": [], "scan": scan_pts[:200],
                        "detections": [], "waypoints": [], "currentWP": -1,
                        "slam_source": "ros2_nx",
                    }})
                    ws_broadcast({"type": "status", "imu_yaw": round(st.get("yaw", 0.0), 3),
                                  "stats": {"imu_count": st.get("imu_count", 0),
                                            "robot_mode": 0, "connected": True},
                                  "dog_state": getattr(robot, "robot_state", "UNKNOWN"),
                                  "tasks": task_mgr.get_state() if task_mgr else {}})
                else:
                    ws_broadcast({"type": "status", "imu_yaw": 0.0,
                                  "dog_state": getattr(robot, "robot_state", "UNKNOWN"),
                                  "tasks": task_mgr.get_state() if task_mgr else {}})
                time.sleep(0.15)
                continue
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

                # SLAM 推送 (每帧都推, 前端用 requestAnimationFrame 自己控制渲染)
                _update_dead_reckon()
                slam_counter += 1
                if slam_counter % 2 == 0:  # 约 3Hz, 够画地图
                    ws_broadcast({"type": "slam", "data": {
                        "x": round(_dead_reckon["x"], 2),
                        "y": round(_dead_reckon["y"], 2),
                        "yaw": round(_dead_reckon["yaw"], 2),
                        "trail": _dead_reckon["trail"],
                        "map": [],          # 阶段A无栅格地图 (阶段B接nav2)
                        "scan": [],          # 阶段A无扫描点 (阶段B接MID360)
                        "detections": _dead_reckon["last_dets"],
                        "waypoints": [],
                        "currentWP": -1,
                        "slam_source": "dead_reckon",  # 标明这是死推算, 非真SLAM
                    }})
            elif os.environ.get("GO2W_NO_ROBOT"):
                # Mock 模式: 没连狗时推假数据, 让前端布局可见 (狗沿螺旋转圈走)
                import math as _m2
                mock_t += 0.15
                mx = round(_m2.cos(mock_t * 0.3) * mock_t * 0.15, 2)
                my = round(_m2.sin(mock_t * 0.3) * mock_t * 0.15, 2)
                myaw = round(_m2.sin(mock_t * 0.5), 2)
                ws_broadcast({"type": "slam", "data": {
                    "x": mx, "y": my, "yaw": myaw,
                    "trail": [[round(_m2.cos(mock_t * 0.3 - i * 0.1) * (mock_t - i * 0.15) * 0.15, 2),
                               round(_m2.sin(mock_t * 0.3 - i * 0.1) * (mock_t - i * 0.15) * 0.15, 2)]
                              for i in range(0, int(mock_t / 0.15), 3) if i < 40],
                    "map": [], "scan": [],
                    "detections": [{"x": 2.5, "y": 1.0, "class": "person"}],
                    "waypoints": [{"x": 3.0, "y": 0.0}, {"x": 3.0, "y": 3.0}, {"x": 0.0, "y": 3.0}],
                    "currentWP": 1,
                    "slam_source": "mock",
                }})
                ws_broadcast({"type": "status", "imu_yaw": myaw,
                              "stats": {"imu_count": int(mock_t * 10), "robot_mode": 0},
                              "tasks": task_mgr.get_state() if task_mgr else {}})
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

    use_ros2 = bool(os.environ.get("GO2W_USE_ROS2"))
    if use_ros2:
        # ROS2 模式: 控狗经 NX 的 nx_motion_node (发 /cmd_vel /cmd_pose)
        # 自动站立/看门狗/lease 都在 NX 节点上, PC 这边只转发指令
        robot = RosRobotBridge()
    else:
        robot = RobotConnection(interface)
    logger.info("加载 VLM 模型...")
    vlm = VLMEngine()
    if not vlm.load(): logger.warning("VLM 加载失败，将使用关键词匹配"); vlm = None
    else: logger.info("VLM 就绪")

    detector = Detector()
    task_mgr = TaskManager(robot, vlm_engine=vlm, detector=detector)

    try:
        if use_ros2:
            logger.info("ROS2 模式: 连接 cmd_publisher (转发指令到 NX) ...")
            robot.connect()
            logger.info("就绪 — 指令经 /cmd_vel → NX nx_motion_node 控狗 (自动站立由NX负责)")
        elif os.environ.get("GO2W_NO_ROBOT"):
            logger.info("GO2W_NO_ROBOT=1, 跳过连狗 (前端验证模式)")
        else:
            logger.info("连接 Go2W ..."); robot.connect()
            logger.info("已连接，执行站立序列 ...")
            robot._stand_done.clear()
            with robot._lock: robot._cmd = 'stand'
            robot._stand_done.wait(10); robot.start_watchdog()
            logger.info("就绪 — STOPPED 状态，机器人静止")
    except Exception as e: logger.warning(f"连接失败: {e} (继续起 Web 服务, 前端可访问)")

    static_dir = os.path.join(os.path.dirname(__file__), 'static')
    logger.info(f"Web: http://{host}:{port}  WS: ws://{host}:{ws_port}")
    threading.Thread(target=run_ws, args=(host, ws_port), daemon=True).start()
    task_mgr.start_worker()
    threading.Thread(target=broadcast_loop, daemon=True).start()
    server = create_server(host, port, static_dir); server.serve_forever()


if __name__ == '__main__':
    main()