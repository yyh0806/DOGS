#!/usr/bin/env python3
"""Go2W NX Web 服务 (阶段A: web 通信层上移 NX)。

== 职责 ==
载荷 NX 上跑的 web 服务进程，内嵌 rclpy 节点：
  - HTTP:8000  + WebSocket:8001  (与 web/panel.py 端口/契约完全一致, 前端无感)
  - 本机发布 /cmd_vel (Twist) /cmd_pose (String)   → 被 nx_motion_node 消费控狗
  - 本机订阅 /dog_state /imu /scan /odom          → broadcast_loop 推 WS 给前端

== 退役说明 (可追溯) ==
阶段A 用本文件替代了下列 PC 端组件, 但源文件**保留不删** (PC fallback, 可回滚):
  - web/panel.py            : NX 路径不再启动 (HTTP/WS 契约已原样照搬到此)
  - web/cmd_publisher.py    : 不再经容器 stdin 桥接, rclpy publisher 直接发
  - web/ros_to_json.py      : 不再写 dog_state.json 文件, broadcast_loop 直接读订阅缓存
  - go2w_humble Docker 容器: PC 不再起容器, 浏览器直连 NX:8000

== 红线 (与 spec §6.3 一致) ==
  - 不直连 unitree_sdk2py (狗 SDK), 控狗全交 nx_motion_node; 本进程 destroy_node 不调 Damp
  - 坐标不反转: /cmd_vel.angular.z 直接透传前端 vyaw (反转在 nx_motion_node:120 做)
  - 不 import ai.detector / ai.vlm / audio (NX web 不跑 AI, 决策 1)
  - rclpy.spin 独立线程 (决策 2), 主线程跑 HTTPServer.serve_forever

运行 (载荷 NX):
  source /opt/ros/humble/setup.bash
  python3 web/nx_web_server.py
"""

import asyncio
import json
import math
import os
import sys
import threading
import time
import traceback
import uuid
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

_WEB_DIR = Path(__file__).resolve().parent
_WS_DIR = _WEB_DIR.parent
if str(_WS_DIR) not in sys.path:
    sys.path.insert(0, str(_WS_DIR))

# ---- ROS2 (NX 本机, Humble) ----
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from nx_slam_map import ObstacleGridAccumulator

import logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("go2w.nx_web")

# web/ 目录 (static 资源与本文件同目录, 和 panel.py 共用)
_WEB_DIR = os.path.dirname(os.path.abspath(__file__))

# ============================================================================
# 阶段B: AI 适配层 (懒加载, import 时不触发 torch, 仅在 NxAiEngine 实例化/方法调用时才 import)
# spec 决策 1 方案 (b): 同进程组件注入, 不破坏阶段A 红线。
# AI_OK=False 时退化阶段A 行为 (vlm/detector 传 None, C1.x 契约不变)。
# ============================================================================
AI_OK = False
_AI_ERR = ""
NxAiEngine = None
NxAiVlmProxy = None
NxAiDetectorProxy = None
try:
    from nx_ai_node import NxAiEngine, NxAiVlmProxy, NxAiDetectorProxy, set_ws_broadcast
    AI_OK = True
    logger.info("阶段B AI 适配层可用 (nx_ai_node)")
except Exception as _e:
    _AI_ERR = str(_e)
    logger.warning(f"阶段B AI 适配层不可用, 退化阶段A 行为 (无视频/检测/VLM): {_e}")


# ============================================================================
# 阶段E: 房间级搜索编排 (懒加载, 与 NxAiEngine 同款注入; spec-stage-e §7.2)
# ----------------------------------------------------------------------------
# RoomSearchOrchestrator 作为"组件"注入 TaskManager (决策 3):
#   - TaskManager._worker 遇 task.type=="search_room" 调 self.room_orchestrator.run(task)
#   - 不破坏阶段A/B 契约: HTTP 现有 12 端点逐字不动, 仅新增 3 个; WS 现有 type 不动, 仅新增 2 个
#   - Nav2 action client 用 ReentrantCallbackGroup (决策 4), 在 worker 线程 spin_until_complete
# ROOM_ORCH_OK=False 时退化 (search_room 任务会标 failed "房间编排未启用"), 不影响阶段A/B。
# ============================================================================
ROOM_ORCH_OK = False
_ROOM_ORCH_ERR = ""
RoomSearchOrchestrator = None
try:
    from nx_room_orchestrator import RoomSearchOrchestrator
    ROOM_ORCH_OK = True
    logger.info("阶段E 房间级搜索编排可用 (nx_room_orchestrator)")
except Exception as _e:
    _ROOM_ORCH_ERR = str(_e)
    logger.warning(f"阶段E 房间级搜索编排不可用 (search_room 任务将标 failed): {_e}")


# ============================================================================
# Product command parser (deterministic offline path; falls back to VLM/fallback)
# ============================================================================
PRODUCT_COMMAND_OK = False
_PRODUCT_COMMAND_ERR = ""
parse_product_command = None
resolve_current_room = None
try:
    from nx_product_command import parse_product_command, resolve_current_room
    PRODUCT_COMMAND_OK = True
    logger.info("Product room person command parser available (nx_product_command)")
except Exception as _e:
    _PRODUCT_COMMAND_ERR = str(_e)
    logger.warning(f"Product command parser unavailable, using existing parse path: {_e}")


# ============================================================================
# C13 云台 RTSP 双流桥接 (独立组件, 懒加载 cv2; 不动 AI/VideoClient 路径)
# GimbalRtspBridge 拉 C13 可见光+红外 RTSP → WS type=gimbal。
# GIMBAL_OK=False (cv2 缺失/C13_ENABLE=0) 时跳过, 主服务不受影响。
# ============================================================================
GIMBAL_OK = False
GimbalRtspBridge = None
try:
    from nx_gimbal_node import GimbalRtspBridge, is_enabled as _gimbal_enabled
    GIMBAL_OK = _gimbal_enabled()
    logger.info(f"C13 双流桥接: {'可用' if GIMBAL_OK else '未启用 (C13_ENABLE=0 或 cv2 缺失)'}")
except Exception as _e:
    logger.warning(f"C13 双流桥接不可用 (前端无云台画面, 主服务不受影响): {_e}")


# ============================================================================
# Livox MID360 雷达点云 2D 鸟瞰展示 (独立组件, 订阅 /livox/lidar → type=lidar)
# 需 go2w-web.service source ~/ws_livox/install/setup.bash (livox_ros_driver2.msg)。
# ============================================================================
LIDAR_OK = False
LidarBridge = None
try:
    from nx_lidar_node import LidarBridge, _LIDAR_OK as _lidar_ok
    LIDAR_OK = bool(_lidar_ok)
    logger.info(f"雷达点云展示: {'可用' if LIDAR_OK else '未启用 (livox.msg 缺失, 需 source ws_livox)'}")
except Exception as _e:
    logger.warning(f"雷达点云展示不可用 (前端无雷达画面, 主服务不受影响): {_e}")


# ============================================================================
# WS 广播 (照抄 panel.py:92-105)
# ============================================================================
WS_CLIENTS = set()
WS_LOOP = None
_WS_PENDING = 0
_WS_PENDING_LOCK = threading.Lock()
_WS_MAX_PENDING = max(1, int(os.environ.get("GO2W_WS_MAX_PENDING", "3")))
_WS_SEND_TIMEOUT = max(0.02, float(os.environ.get("GO2W_WS_SEND_TIMEOUT", "1.0")))  # H4 fix: 0.2→1.0 (本地/局域网, 避免瞬时抖动误踢健康连接)
# C1 fix: NxRobotBridge.move 执行层 guard — 前进时前方 /scan 障碍 < 此阈值→强制 vx=0 (防自主跟踪撞墙)
_FRONT_CLEARANCE_M = float(os.environ.get("GO2W_FRONT_CLEARANCE", "0.5"))


def ws_broadcast(data, force=False):
    global _WS_PENDING
    if WS_LOOP and WS_CLIENTS:
        if not force:
            with _WS_PENDING_LOCK:
                if _WS_PENDING >= _WS_MAX_PENDING:
                    return
                _WS_PENDING += 1
        msg = json.dumps(data, ensure_ascii=False)
        fut = asyncio.run_coroutine_threadsafe(_async_broadcast(msg), WS_LOOP)

        def _done(_):
            global _WS_PENDING
            if not force:
                with _WS_PENDING_LOCK:
                    _WS_PENDING = max(0, _WS_PENDING - 1)

        fut.add_done_callback(_done)


async def _send_ws(ws, msg):
    try:
        await asyncio.wait_for(ws.send(msg), timeout=_WS_SEND_TIMEOUT)
        return None
    except Exception:
        return ws


async def _async_broadcast(msg):
    stale = await asyncio.gather(*[_send_ws(ws, msg) for ws in list(WS_CLIENTS)])
    for ws in stale:
        if ws is not None:
            WS_CLIENTS.discard(ws)


# 阶段B: 把 ws_broadcast 注入 nx_ai_node (避免循环 import; nx_ai_node 顶层不能
# import 本模块, 因本模块顶层 import rclpy, 无 rclpy 环境反复 import 会挂起 worker 线程)。
# 此处 ws_broadcast 已定义, 可安全传引用。AI_OK=False 时 set_ws_broadcast 未导入, 跳过。
if AI_OK:
    try:
        set_ws_broadcast(ws_broadcast)
    except Exception as _e:
        logger.warning(f"ws_broadcast 注入 nx_ai_node 失败 (AI 仍可用, 仅 vlm 状态广播跳过): {_e}")


# ============================================================================
# 搜索路径规划 (内联复制自 panel.py:35-86, 无 ROS2 依赖)
# ============================================================================

def plan_lawnmower(width, height, spacing=2.5, origin_x=0.0, origin_y=0.0):
    if spacing <= 0:
        spacing = 2.5
    waypoints = []
    num_rows = max(1, int(math.ceil(height / spacing)))
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
            waypoints.append({"x": origin_x + width, "y": y, "yaw": math.pi, "is_scan": True})
            waypoints.append({"x": origin_x, "y": y, "yaw": math.pi, "is_scan": True})
    return waypoints


def plan_spiral(width, height, spacing=2.5, origin_x=0.0, origin_y=0.0):
    if spacing <= 0:
        spacing = 2.5
    cx = origin_x + width / 2.0
    cy = origin_y + height / 2.0
    max_radius = math.sqrt(width ** 2 + height ** 2) / 2.0
    num_turns = max(3, int(math.ceil(max_radius / spacing)))
    points_per_turn = 12
    total_points = num_turns * points_per_turn
    waypoints = []
    for i in range(total_points + 1):
        angle = i * 2.0 * math.pi / points_per_turn
        radius = (i / total_points) * max_radius if total_points > 0 else 0.0
        x = max(origin_x, min(cx + radius * math.cos(angle), origin_x + width))
        y = max(origin_y, min(cy + radius * math.sin(angle), origin_y + height))
        waypoints.append({"x": x, "y": y, "yaw": angle, "is_scan": (i % points_per_turn == 0)})
    return waypoints


def _wp_to_moves(waypoints, speed=0.3, ang_speed=0.5):
    """航点列表 → move 任务参数 (距离/速度=duration)。"""
    tasks = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        dx = b["x"] - a["x"]
        dy = b["y"] - a["y"]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 0.05:
            continue
        target_yaw = math.atan2(dy, dx)
        if abs(target_yaw) > 0.1:
            dur = round(abs(target_yaw) / ang_speed, 1)
            tasks.append({"type": "move", "priority": 5,
                          "params": {"vyaw": ang_speed if target_yaw > 0 else -ang_speed, "duration": dur}})
        dur = round(dist / speed, 1)
        tasks.append({"type": "move", "priority": 5,
                      "params": {"vx": speed, "duration": dur}})
    return tasks


# ============================================================================
# NxWebNode — 内嵌 rclpy 节点 (决策 2: 独立 spin 线程)
# ----------------------------------------------------------------------------
# 线程模型 (spec 决策 2):
#   主线程: HTTPServer.serve_forever (阻塞)
#   线程1 (daemon): rclpy.spin(node)            ← ROS2 回调在此执行
#   线程2 (daemon): run_ws 的 asyncio loop      ← WS server
#   线程3 (daemon): broadcast_loop              ← 读订阅缓存 → ws_broadcast
# publisher 在 __init__ 创建一次, handler 只 publish (H2.3)。
# 订阅缓存用 threading.Lock 保护 (H2.2, 参考 nx_sensor_node.py:82 _lock 模式)。
# ============================================================================
class NxWebNode(Node):
    def __init__(self):
        super().__init__('nx_web_node')

        # ---- 参数 (与 panel.py main 的环境变量对齐) ----
        self.declare_parameter('host', os.environ.get('GO2W_HOST', '0.0.0.0'))
        self.declare_parameter('port', int(os.environ.get('GO2W_PORT', '8000')))
        self.declare_parameter('ws_port', int(os.environ.get('GO2W_WS_PORT', '8001')))
        self.declare_parameter('state_timeout', 3.0)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value
        self.state_timeout = self.get_parameter('state_timeout').get_parameter_value().double_value

        # ---- 发布器 (本机 → nx_motion_node 订阅) ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pose_pub = self.create_publisher(String, '/cmd_pose', 10)

        # ---- 订阅器 (本机 ← nx_motion_node / nx_sensor_node 发布) ----
        # QoS 说明: nx_sensor_node.py:104-107 的 /imu /scan /odom 发布端用的是
        # 默认 RELIABLE QoS (depth=10), 不是 qos_profile_sensor_data。ROS2 QoS 兼容性:
        # REL 发布 + BE 订阅 = 不兼容 (订阅收不到任何数据)。为能收到 nx_sensor_node 的数据,
        # /imu /scan /odom 这里都用 depth=10 的默认 RELIABLE, 与发布端匹配。
        # /dog_state 由 nx_motion_node 发布 (RELIABLE depth=10)。
        # (spec H2.4 字面写 sensor_data, 但与真实发布端冲突 → 以"能收到数据"为准, 见本注释)
        self.create_subscription(String, '/dog_state', self._on_dog_state, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        # ---- 订阅缓存 (Lock 保护, H2.2) ----
        self._lock = threading.Lock()
        self._dog_state = "UNKNOWN"      # STOPPED/MOVING/STANDING/... (来自 /dog_state)
        self._dog_vx = self._dog_vy = self._dog_vyaw = 0.0
        self._imu_yaw = 0.0
        self._imu_count = 0
        self._scan_count = 0
        self._scan_ranges = []           # 原始 ranges (机体系)
        self._scan_angle_min = 0.0
        self._scan_angle_increment = 0.0
        self._scan_range_min = 0.0
        self._scan_range_max = 0.0
        self._scan_timestamp = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_count = 0
        self._odom_t = 0.0
        self._last_state_t = 0.0         # 最近一次 /dog_state 时间 (判 connected)

        self.get_logger().info(
            f"NxWebNode 就绪: 发 /cmd_vel /cmd_pose, 订阅 /dog_state /imu /scan /odom")

    # ---- ROS2 回调 (在 spin 线程内执行) ----
    def _on_dog_state(self, msg: String):
        """nx_motion_node 发布的 JSON: {state, vx, vy, vyaw}。"""
        try:
            d = json.loads(msg.data)
            with self._lock:
                self._dog_state = d.get('state', 'UNKNOWN')
                self._dog_vx = float(d.get('vx', 0.0))
                self._dog_vy = float(d.get('vy', 0.0))
                self._dog_vyaw = float(d.get('vyaw', 0.0))
                self._last_state_t = time.time()
        except Exception as e:
            self.get_logger().warning(f"/dog_state 解析失败: {e}")

    def _on_imu(self, msg: Imu):
        """四元数 → yaw。公式照抄 ros_to_json.py:52-55。
        ROS Imu.orientation 字段顺序是 (x,y,z,w); ros_to_json 用 q.w/q.z/q.x/q.y 访问,
        这里按字段名访问, 公式等价: yaw = atan2(2(wz+xy), 1-2(y²+z²))。
        """
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self._imu_yaw = math.atan2(siny_cosp, cosy_cosp)
            self._imu_count += 1

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._scan_ranges = [round(float(r), 3) for r in msg.ranges]
            self._scan_angle_min = float(msg.angle_min)
            self._scan_angle_increment = float(msg.angle_increment)
            self._scan_range_min = float(msg.range_min)
            self._scan_range_max = float(msg.range_max)
            self._scan_timestamp = time.time()
            self._scan_count += 1

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._odom_x = float(msg.pose.pose.position.x)
            self._odom_y = float(msg.pose.pose.position.y)
            self._odom_t = time.time()
            self._odom_count += 1

    # ---- 发布 (HTTP handler 线程调用; rclpy publisher.publish 线程安全) ----
    def publish_cmd_vel(self, vx, vy, vyaw):
        """发布 /cmd_vel。坐标系约定 (spec §4 决策):
        前端 vyaw 正=左转, /cmd_vel.angular.z 正=左转 (ROS REP-103), **直接透传不反转**。
        真正的反转在 nx_motion_node:120 做 (Go2W SDK z 正=右转需反转)。
        若此处也反转 = 双重反转 bug = Critical (eval-rubric C2.4)。
        """
        try:
            tw = Twist()
            tw.linear.x = float(vx)
            tw.linear.y = float(vy)
            tw.angular.z = float(vyaw)   # 透传, 不取负
            self.cmd_vel_pub.publish(tw)
        except Exception as e:
            logger.warning(f"publish /cmd_vel 失败: {e}")

    def publish_cmd_pose(self, cmd):
        """发布 /cmd_pose: data ∈ {'stand','sit','estop'} (nx_motion_node:126 接收)。"""
        try:
            s = String()
            s.data = str(cmd)
            self.cmd_pose_pub.publish(s)
        except Exception as e:
            logger.warning(f"publish /cmd_pose 失败: {e}")

    # ---- 给 /api/status & NxRobotBridge 取缓存 ----
    def get_status_snapshot(self):
        with self._lock:
            now = time.time()
            connected = (now - self._last_state_t) < self.state_timeout
            return {
                "connected": connected,
                "imu_yaw": round(self._imu_yaw, 3),
                "dog_state": self._dog_state,
                "stats": {
                    "imu_count": self._imu_count,
                    "scan_count": self._scan_count,
                    "odom_count": self._odom_count,
                    "robot_mode": 0,
                    "robot_velocity": [self._dog_vx, self._dog_vy, self._dog_vyaw],
                },
            }

    def get_scan_snapshot(self):
        with self._lock:
            timestamp = float(getattr(self, "_scan_timestamp", 0.0) or 0.0)
            age_sec = (time.time() - timestamp) if timestamp > 0.0 else None
            return {
                "angle_min": self._scan_angle_min,
                "angle_increment": self._scan_angle_increment,
                "range_min": self._scan_range_min,
                "range_max": self._scan_range_max,
                "ranges": list(self._scan_ranges),
                "count": self._scan_count,
                "timestamp": timestamp,
                "age_sec": age_sec,
            }


# ============================================================================
# NxRobotBridge — 替代 panel.py:RosRobotBridge 的 NX 版机器人抽象
# ----------------------------------------------------------------------------
# 公共 API 与 RosRobotBridge 字段级一致 (move/stop_move/stand/sit/e_stop +
# connected/imu_yaw/robot_state/stats/_lock/_vx/_vy/_vyaw), 让 panel.py 的
# TaskManager 无感复用 (TaskManager 依赖 robot.move/stop_move/stand/sit/e_stop
# + robot._lock/_vx/_vy/_vyaw + robot.robot_state/imu_yaw/stats/connected)。
# ============================================================================
class NxRobotBridge:
    def __init__(self, node: NxWebNode):
        self._node = node
        self._lock = threading.RLock()
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        # 阶段B: NxAiEngine 注入 (main 里 robot._ai_engine = ai_engine),
        # 让 get_frame() 能委托取最新缓存帧 (供 TaskManager._execute_search 检测)。
        self._ai_engine = None
        self._gimbal_bridge = None

    @property
    def connected(self):
        """connected = /dog_state 在 state_timeout(3s) 内有数据。
        判 nx_motion_node 是否活着 (它挂了 connected=false, 前端顶栏"狗"点变灰)。
        """
        with self._node._lock:
            return (time.time() - self._node._last_state_t) < self._node.state_timeout

    @property
    def imu_yaw(self):
        with self._node._lock:
            return self._node._imu_yaw

    @property
    def robot_state(self):
        """兼容 panel.py:833 getattr(robot, 'robot_state', 'UNKNOWN')。"""
        with self._node._lock:
            return self._node._dog_state

    @property
    def stats(self):
        with self._node._lock:
            return {
                "imu_count": self._node._imu_count,
                "scan_count": self._node._scan_count,
                "odom_count": self._node._odom_count,
                "robot_mode": 0,
                "robot_velocity": [self._node._dog_vx, self._node._dog_vy, self._node._dog_vyaw],
            }

    # ---- 动作: 转发到 NxWebNode 的 rclpy publisher ----
    def front_clearance(self, half_fov_deg=30.0):
        """机体前方 ±half_fov_deg 最近障碍距离(m)。无 /scan 返回大值(视为畅通, 不卡跟踪)。
        LaserScan: angle_min=-pi, 索引 i→angle=-pi+i*2pi/n, 前方 angle=0→i=n/2 (nx_sensor /scan)。"""
        with self._node._lock:
            ranges = list(self._node._scan_ranges)
        if not ranges:
            return 999.0
        n = len(ranges)
        center = n / 2.0
        span = int(round(math.radians(half_fov_deg) / (2.0 * math.pi / n)))
        lo = max(0, int(center - span)); hi = min(n, int(center + span + 1))
        valid = [r for r in ranges[lo:hi] if 0.05 < r < 10.0]
        return min(valid) if valid else 999.0

    def move(self, vx, vy, vyaw, manual=False):
        # C1 fix (2026-07-01, critic 收敛): guard 仅对自主路径(manual=False)生效 —
        # tracker/TaskManager 调 move 不传 manual → 受 connected+前方障碍保护;
        # 操作员 /api/move 传 manual=True → 透传(可顶近障碍/过门框, 全权控制)。
        # 不破阶段B tracker 红线(tracker 调 move 无参自动走 guard), 且不误伤手动控制。
        vx = float(vx); vy = float(vy); vyaw = float(vyaw)
        if not manual:
            if not self.connected:
                return
            if vx > 0.0:
                try:
                    front = self.front_clearance(30.0)
                except Exception:
                    front = 999.0
                if front < _FRONT_CLEARANCE_M:
                    logger.info(f"[move] 自主前进前方障碍 {front:.2f}m<{_FRONT_CLEARANCE_M}m, 暂停(仅转向/侧移)")
                    vx = 0.0
        with self._lock:
            self._vx = vx
            self._vy = vy
            self._vyaw = vyaw
        self._node.publish_cmd_vel(vx, vy, vyaw)

    def stop_move(self):
        with self._lock:
            self._vx = self._vy = self._vyaw = 0.0
        self._node.publish_cmd_vel(0.0, 0.0, 0.0)

    def stand(self):
        self._node.publish_cmd_pose('stand')

    def sit(self):
        self._node.publish_cmd_pose('sit')

    def e_stop(self):
        self._node.publish_cmd_pose('estop')

    def stop(self):
        """Compatibility for ai.tracker.TargetTracker.stop()."""
        self.stop_move()

    # ---- 阶段B 新增: get_frame 委托给 NxAiEngine (spec §7.2.3) ----
    def get_frame(self):
        """阶段B: 返回最新缓存帧 (带 YOLO 框, 720p BGR), 供 TaskManager._execute_search 检测。
        帧来自 NxAiEngine._video_yolo_loop 的缓存 (Lock 保护), 不调 GetImageSample (不阻塞)。
        无 AI 引擎/无缓存帧 → None (TaskManager 跳过检测, 与 panel.py:581 一致)。
        """
        if self._gimbal_bridge is not None:
            try:
                f = self._gimbal_bridge.get_vis_frame()
                if f is not None:
                    return f
            except Exception:
                pass
        if self._ai_engine is None:
            return None
        try:
            with self._ai_engine._lock:
                f = self._ai_engine._latest_frame
                return f.copy() if f is not None else None
        except Exception:
            return None


# ============================================================================
# Task / TaskManager (复制自 panel.py:411-646, detector/vlm 传 None)
# ============================================================================
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
    def __init__(self, robot, vlm_engine=None, detector=None, room_orchestrator=None):
        self.robot = robot
        self.vlm = vlm_engine
        self.detector = detector
        self._lock = threading.Lock()
        self._tasks = []
        self._active = None
        self._running = False
        self._follow_active = False
        self._search_targets = []
        # 阶段A 不跑 AI: vlm/detector 传 None, tracker 不创建
        self._tracker = None
        if self.vlm is not None:
            try:
                from ai.tracker import TargetTracker
                self._tracker = TargetTracker(self.vlm, self.robot, detector=self.detector)
                logger.info("TargetTracker 已启用 (VLM/LocateAnything follow backend)")
            except Exception as e:
                logger.warning(f"TargetTracker 初始化失败, follow 将不可用: {e}")
        # 阶段E: 房间级搜索编排注入 (spec §7.2.2); None 时 search_room 任务标 failed
        self.room_orchestrator = room_orchestrator

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
        if self._tracker and self._follow_active:
            self._tracker.stop()
            self._follow_active = False
            ws_broadcast({"type": "follow", "data": {"status": "stopped"}})
        # 阶段E: 取消进行中的房间编排 (Nav2 cancel_current, spec §11 cancel 响应)
        if self.room_orchestrator is not None:
            try:
                self.room_orchestrator.cancel()
            except Exception as _e:
                logger.warning(f"room_orchestrator.cancel 异常: {_e}")
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
            result = self._parse_product_command(text)
            if result is None:
                result = self._vlm_parse_command(text) if (self.vlm and getattr(self.vlm, 'loaded', False)) \
                    else self._fallback_parse(text)
            response = result.get("response", "")
            tasks = result.get("tasks", [])
            logger.info(f"指令解析: '{text}' → response='{response}' tasks={len(tasks)}")
            ws_broadcast({"type": "vlm", "data": {"text": text, "response": response, "tasks": tasks}})
            if tasks:
                self.add_list([Task(t.get("type", "move"), t.get("params", {}), t.get("priority", 5)) for t in tasks])
        except Exception as e:
            logger.error(f"指令处理失败: {e}")
            traceback.print_exc()

    def _parse_product_command(self, text):
        if parse_product_command is None:
            return None
        try:
            result = parse_product_command(text)
        except Exception as e:
            logger.warning(f"Product command parser failed, using existing parse path: {e}")
            return None
        if result is not None:
            self._resolve_product_current_room(result)
        return result

    def _resolve_product_current_room(self, result):
        if resolve_current_room is None:
            return
        tasks = result.get("tasks", []) if isinstance(result, dict) else []
        needs_current_room = any(
            isinstance(t, dict)
            and isinstance(t.get("params"), dict)
            and t["params"].get("room") == "__current__"
            for t in tasks
        )
        if not needs_current_room:
            return
        pose = self._latest_robot_map_pose()
        rooms = self._room_details_for_resolution()
        if pose is None or not rooms:
            return
        try:
            room_name = resolve_current_room(pose[0], pose[1], rooms)
        except Exception as e:
            logger.warning(f"Current-room resolution failed; keeping __current__: {e}")
            return
        if not room_name:
            return
        for task in tasks:
            params = task.get("params") if isinstance(task, dict) else None
            if isinstance(params, dict) and params.get("room") == "__current__":
                params["room"] = room_name

    def _latest_robot_map_pose(self):
        try:
            node_obj = getattr(self.robot, "_node", None)
            if node_obj is None:
                return None
            lock = getattr(node_obj, "_lock", threading.Lock())
            with lock:
                if int(getattr(node_obj, "_odom_count", 0)) <= 0:
                    return None
                odom_t = float(getattr(node_obj, "_odom_t", 0.0) or 0.0)
                x = float(getattr(node_obj, "_odom_x"))
                y = float(getattr(node_obj, "_odom_y"))
            if not all(math.isfinite(value) for value in (x, y, odom_t)):
                return None
            if odom_t <= 0.0:
                return None
            try:
                max_age_sec = float(os.environ.get("GO2W_ODOM_MAX_AGE_SEC", "2.0"))
            except (TypeError, ValueError):
                max_age_sec = 2.0
            if not math.isfinite(max_age_sec) or max_age_sec <= 0.0:
                max_age_sec = 2.0
            age_sec = time.time() - odom_t
            if not math.isfinite(age_sec) or age_sec < 0.0 or age_sec > max_age_sec:
                return None
            return x, y
        except Exception:
            return None

    def _room_details_for_resolution(self):
        orch_obj = self.room_orchestrator or getattr(TaskManager, "_global_room_orchestrator", None)
        if orch_obj is None:
            return []
        try:
            rooms = orch_obj.list_rooms_detail()
        except Exception as e:
            logger.warning(f"Room detail read failed; keeping __current__: {e}")
            return []
        return rooms if isinstance(rooms, list) else []

    def _vlm_parse_command(self, text):
        # 阶段B: 真接 VLM (vlm 是 NxAiVlmProxy 时, chat 同步阻塞后台线程等队列结果)。
        # panel.py:472-518 等价: sys_prompt + chat + JSON 解析 + 失败 fallback。
        # 阶段A 时 self.vlm=None, 走不到这里 (上层 _process_command_bg 先判 vlm.loaded)。
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
        try:
            response = self.vlm.chat([
                {"role": "system", "content": sys_prompt},
                {"role": "user", "content": text}
            ], max_new_tokens=512)
        except Exception as e:
            logger.warning(f"VLM chat 异常: {e}")
            return self._fallback_parse(text)
        import re as _re, json as _json
        logger.info(f"VLM 原始响应: {str(response)[:200]}")
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
                if "tasks" in data:
                    data.setdefault("response", "已解析")
                    return data
        except Exception as e:
            logger.warning(f"VLM JSON 解析失败: {e}")
        return self._fallback_parse(text)

    @staticmethod
    def _extract_room_name(text):
        """阶段E (spec §7.2.4): 从中文指令提取房间名。
        扫描注入的 room_orchestrator 的房间地图 (name + aliases), 用 RoomMap.find 匹配。
        匹配不到 → None (调用方退化阶段A 矩形搜索)。
        room_orchestrator 未注入/房间地图未加载 → None (不破坏现有行为)。
        """
        try:
            orch_obj = getattr(TaskManager, "_global_room_orchestrator", None)
            if orch_obj is None:
                return None
            # 用 RoomMap.find 走完整 4 级匹配 (name>aliases 完全相等>子串)
            import nx_room_orchestrator as _orch
            # 优先用已加载的 _room_map (避免每次解析都 IO)
            rm = getattr(orch_obj, "_room_map", None)
            if rm is None:
                rooms_detail = orch_obj.list_rooms_detail()
                if not rooms_detail:
                    return None
                # 临时构造 RoomMap (path 不重要)
                import os as _os
                yaml_path = getattr(orch_obj, "_rooms_yaml",
                                    _os.path.join(_WEB_DIR, "..", "config", "rooms.yaml"))
                rm = _orch.RoomMap.load(yaml_path)
            room = rm.find(text)
            return room.name if room is not None else None
        except Exception:
            return None

    @staticmethod
    def _fallback_parse(text):
        r = {"understanding": text, "tasks": [], "response": ""}
        if "跟着" in text or "跟随" in text or "跟上" in text:
            target = ""
            for kw in ["跟着", "跟随", "跟上"]:
                if kw in text:
                    target = text[text.index(kw) + len(kw):].strip().rstrip("。，！？")
            r["tasks"] = [{"type": "follow", "priority": 8, "params": {"target": target}}]
            r["response"] = f"跟踪{target}"
        elif "搜索" in text or "找" in text:
            # 阶段E (spec §7.2.4): 尝试从指令提取房间名 (扫 rooms.yaml 的 name/aliases)
            room = TaskManager._extract_room_name(text)
            if room:
                r["tasks"] = [{"type": "search_room", "priority": 7,
                               "params": {"room": room}}]
                r["response"] = f"搜索{room}"
            else:
                # 无房间名 → 退化阶段A 矩形覆盖搜索 (现有行为不破坏)
                r["tasks"] = [{"type": "search_area", "priority": 5,
                               "params": {"pattern": "lawnmower", "width": 10, "height": 10}}]
                r["response"] = "开始搜索"
        elif "停" in text:
            r["tasks"] = [{"type": "stop", "priority": 10, "params": {}}]
            r["response"] = "已停止"
        elif "回来" in text or "返回" in text:
            r["tasks"] = [{"type": "return_home", "priority": 7, "params": {}}]
            r["response"] = "返回"
        elif "前进" in text or "向前" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": 0.5, "duration": 2.0}}]
            r["response"] = "前进"
        elif "后退" in text or "向后" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": -0.5, "duration": 2.0}}]
            r["response"] = "后退"
        elif "左转" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": 0.5, "duration": 2.0}}]
            r["response"] = "左转"
        elif "右转" in text:
            r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": -0.5, "duration": 2.0}}]
            r["response"] = "右转"
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
                p = task.params
                if task.type in ("move", "navigate"):
                    duration = min(p.get("duration", 1.0), 8.0)
                    vx = p.get("vx", 0)
                    vy = p.get("vy", 0)
                    vyaw = p.get("vyaw", 0)
                    end_time = time.time() + duration
                    while time.time() < end_time:
                        if task.status == "cancelled":
                            break
                        self.robot.move(vx, vy, vyaw)
                        time.sleep(0.1)
                    self.robot.stop_move()
                    task.status = "completed"
                elif task.type == "stop":
                    self.robot.stop_move()
                    self.cancel_all()
                    task.status = "completed"
                elif task.type == "follow":
                    self._execute_follow(task)
                elif task.type == "search_area":
                    self._execute_search(task)
                elif task.type == "search_room":
                    # 阶段E: 房间级搜索编排 (spec §7.2.2)
                    # RoomSearchOrchestrator.run 同步跑状态机, 内部设 task.status/result
                    if self.room_orchestrator is None:
                        logger.warning("search_room 任务但 room_orchestrator 未启用")
                        task.status = "failed"
                        task.result = "房间编排未启用"
                    else:
                        try:
                            self.room_orchestrator.run(task)
                        except Exception as _e:
                            logger.error(f"room_orchestrator.run 异常: {_e}")
                            traceback.print_exc()
                            task.status = "failed"
                            task.result = f"orchestrator 异常: {_e}"
                elif task.type == "return_home":
                    logger.info("return_home: 无定位，原地停住")
                    self.robot.stop_move()
                    task.status = "completed"
                else:
                    logger.warning(f"未知任务类型: {task.type}, 跳过")
                    task.status = "completed"
            except Exception as e:
                task.status = "failed"
                task.result = str(e)
            with self._lock:
                self._tasks = [t for t in self._tasks if t.id != task.id]
                self._active = None
            ws_broadcast({"type": "tasks", "data": self.get_state()})

    def _execute_follow(self, task):
        target = task.params.get("target", "")
        if not target:
            task.status = "failed"
            task.result = "未指定跟踪目标"
            return
        if not self._tracker:
            # 阶段A 无 VLM/detector, follow 无视觉跟踪能力
            task.status = "failed"
            task.result = "VLM未加载, 无法跟踪"
            return
        logger.info(f"开始跟踪: {target}")
        self._follow_active = True
        ws_broadcast({"type": "follow", "data": {"status": "started", "target": target}})
        self._tracker.start_follow(target)
        task.status = "completed"
        task.result = "跟踪已启动 (后台运行)"

    def _execute_search(self, task):
        p = task.params
        width = p.get("width", 10)
        height = p.get("height", 10)
        spacing = p.get("spacing", 2.5)
        pattern = p.get("pattern", "lawnmower")
        speed = p.get("speed", 0.3)
        logger.info(f"搜索: {pattern} {width}x{height}m spacing={spacing}")
        wp = plan_lawnmower(width, height, spacing) if pattern != "spiral" \
            else plan_spiral(width, height, spacing)
        move_tasks = _wp_to_moves(wp, speed=speed)
        logger.info(f"生成 {len(move_tasks)} 个移动任务")
        self._search_targets.clear()

        for i, mt in enumerate(move_tasks):
            with self._lock:
                if self._active is None or self._active.id != task.id:
                    logger.info("搜索被取消")
                    self.robot.stop_move()
                    task.status = "cancelled"
                    return
            mp = mt["params"]
            duration = mp.get("duration", 1.0)
            vx = mp.get("vx", 0)
            vy = mp.get("vy", 0)
            vyaw = mp.get("vyaw", 0)
            logger.info(f"搜索 {i + 1}/{len(move_tasks)}: move(vx={vx},vyaw={vyaw},dur={duration})")
            end_time = time.time() + duration
            while time.time() < end_time:
                self.robot.move(vx, vy, vyaw)
                # 阶段A 无 detector (传 None), 不做视觉检测 — 与 panel.py 一致的判断条件
                if vx != 0 and self.detector:
                    frame = getattr(self.robot, "get_frame", lambda: None)()
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
                                    ws_broadcast({"type": "search", "data": {"found": list(self._search_targets)}})
                time.sleep(0.1)
            self.robot.stop_move()
            time.sleep(0.2)

        task.status = "completed"
        task.result = {"waypoints": len(wp), "moves": len(move_tasks),
                       "found": list(self._search_targets)}
        logger.info(f"搜索完成, 发现: {self._search_targets}")
        ws_broadcast({"type": "search", "data": {"status": "done", "found": list(self._search_targets)}})


# ============================================================================
# HTTP server (照抄 panel.py:654-728, 12 个端点逐字对齐; 阶段E 新增 3 个)
# ============================================================================
robot = None            # NxRobotBridge
task_mgr = None         # TaskManager
node = None             # NxWebNode
room_orchestrator = None  # 阶段E: RoomSearchOrchestrator (main 注入)


def create_server(host, port, static_dir):
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
                snap = node.get_status_snapshot() if node else {}
                self._json({
                    "connected": robot.connected if robot else False,
                    "imu_yaw": robot.imu_yaw if robot else 0,
                    "stats": robot.stats if robot else {},
                    "tasks": task_mgr.get_state() if task_mgr else {},
                })
            elif p.path == '/api/rooms':
                # 阶段E (spec §7.2.3): 列出 rooms.yaml 所有房间 (主名 + 详情)
                if room_orchestrator is not None:
                    rooms_detail = room_orchestrator.list_rooms_detail()
                    self._json({"ok": True,
                                "rooms": [r["name"] for r in rooms_detail],
                                "details": rooms_detail})
                else:
                    self._json({"ok": False, "rooms": [], "msg": "房间编排未启用"})
            elif p.path == '/api/reload_rooms':
                # 阶段E (spec §7.2.3): 热加载 rooms.yaml (改 YAML 后无需重启 nx_web)
                ok_reload = False
                err = ""
                if room_orchestrator is not None:
                    try:
                        ok_reload = room_orchestrator.reload_rooms()
                    except Exception as e:
                        err = str(e)
                self._json({"ok": ok_reload, "err": err})
            else:
                self.send_error(404)

        def do_POST(self):
            p = urlparse(self.path)
            q = parse_qs(p.query)
            L = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(L).decode() if L else ''
            if p.path == '/api/connect':
                # NX 模式: 发 /cmd_pose="stand" 让 nx_motion_node 站起 (异步, 不阻塞等待完成)
                if robot and not robot.connected:
                    robot.stand()
                elif robot:
                    robot.stand()
                self._json({"ok": True, "msg": "已连接"})
            elif p.path == '/api/stand':
                robot.stand(); self._json({"ok": True})
            elif p.path == '/api/sit':
                robot.sit(); self._json({"ok": True})
            elif p.path == '/api/stop':
                robot.stop_move(); self._json({"ok": True})
            elif p.path == '/api/e_stop':
                robot.e_stop(); task_mgr.cancel_all(); self._json({"ok": True})
            elif p.path == '/api/move':
                # query string: vx,vy,vyaw (panel.html:242/248/317 约定)
                vx = float(q.get('vx', ['0'])[0])
                vy = float(q.get('vy', ['0'])[0])
                vyaw = float(q.get('vyaw', ['0'])[0])
                robot.move(vx, vy, vyaw, manual=True)  # /api/move = 操作员手动, bypass 自主 guard (C1 critic 收敛)
                self._json({"ok": True})
            elif p.path == '/api/command':
                text = q.get('text', [''])[0] or body
                if body:
                    try:
                        text = json.loads(body).get('text', '')
                    except Exception:
                        text = body
                if text:
                    task_mgr.process_command(text)
                self._json({"ok": True, "text": text})
            elif p.path == '/api/locate':
                target = q.get('target', [''])[0] or q.get('text', [''])[0] or body
                if body:
                    try:
                        jb = json.loads(body)
                        if isinstance(jb, dict):
                            target = jb.get('target') or jb.get('text') or target
                    except Exception:
                        pass
                target = (target or "").strip()
                if not target:
                    self._json({"ok": False, "msg": "缺少 target/text 参数"})
                    return
                if robot is None or getattr(robot, "_ai_engine", None) is None:
                    self._json({"ok": False, "msg": "AI 引擎未启用"})
                    return
                frame = robot.get_frame()
                if frame is None:
                    self._json({"ok": False, "msg": "当前没有可用视频帧"})
                    return
                result = robot._ai_engine.locate_target(frame, target)
                payload = dict(result)
                payload["target"] = target
                payload["status"] = result.get("description") or ("found" if result.get("found") else "no detections")
                ws_broadcast({"type": "locate", "data": payload}, force=True)
                self._json({"ok": bool(result.get("found")), "target": target, "result": result})
            elif p.path == '/api/search':
                # 表单/地图选区搜索: width,height,spacing,origin_x,origin_y,pattern
                w = float(q.get('width', ['8'])[0])
                h = float(q.get('height', ['8'])[0])
                sp = float(q.get('spacing', ['2'])[0])
                ox = float(q.get('origin_x', ['0'])[0])
                oy = float(q.get('origin_y', ['0'])[0])
                pattern = q.get('pattern', ['lawnmower'])[0]
                task_mgr.add_list([Task("search_area",
                                         {"pattern": pattern, "width": w, "height": h,
                                          "spacing": sp, "origin_x": ox, "origin_y": oy}, 5)])
                self._json({"ok": True, "msg": f"搜索 {w}x{h}m 间距{sp}m 已入队"})
            elif p.path == '/api/search_room':
                # 阶段E (spec §7.2.3): 房间级搜索任务入队
                # query: room (必填), target_classes (可选, 逗号分隔)
                # body JSON 也支持: {"room":"客厅","target_classes":["person"]}
                room = q.get('room', [''])[0]
                if body:
                    try:
                        jb = json.loads(body)
                        if isinstance(jb, dict) and jb.get('room'):
                            room = jb.get('room', room)
                    except Exception:
                        pass
                if not room:
                    self._json({"ok": False, "msg": "缺少 room 参数"})
                    return
                tc_str = q.get('target_classes', [''])[0]
                target_classes = [s.strip() for s in tc_str.split(',') if s.strip()] if tc_str else []
                if not target_classes and body:
                    try:
                        jb = json.loads(body)
                        if isinstance(jb, dict) and jb.get('target_classes'):
                            target_classes = list(jb.get('target_classes'))
                    except Exception:
                        pass
                # search_strategy 透传 (frontier_explore / next_best_view);
                # 不传则 task_params 与原代码一致, 走 RoomSearchOrchestrator 默认路径
                search_strategy = q.get('search_strategy', [''])[0]
                if not search_strategy and body:
                    try:
                        jb = json.loads(body)
                        if isinstance(jb, dict):
                            search_strategy = jb.get('search_strategy', '') or ''
                    except Exception:
                        pass
                task_params = {"room": room, "target_classes": target_classes}
                if search_strategy:
                    task_params["search_strategy"] = search_strategy
                task_mgr.add_list([Task("search_room", task_params, 5)])
                self._json({"ok": True, "msg": f"搜索房间 '{room}' 已入队"
                            + (f" (strategy={search_strategy})" if search_strategy else "")})
            else:
                self.send_error(404)

        def _json(self, d):
            # CORS: 跨端口(8000 HTTP ↔ 8001 WS) 与跨机访问都需要 (panel.py:714)
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

    return ThreadingHTTPServer((host, port), H)  # H5 fix: 每请求独立线程, /api/locate 长 subprocess 不再阻塞 /api/stop 急停


# ============================================================================
# WS server (照抄 panel.py:731-739, 端口固定 8001)
# ============================================================================
def run_ws(host, port):
    global WS_LOOP
    import websockets
    async def h(ws, path):
        WS_CLIENTS.add(ws)
        try:
            await ws.wait_closed()
        finally:
            WS_CLIENTS.discard(ws)
    WS_LOOP = asyncio.new_event_loop()
    asyncio.set_event_loop(WS_LOOP)
    WS_LOOP.run_until_complete(websockets.serve(h, host, port))
    WS_LOOP.run_forever()


# ============================================================================
# broadcast_loop (改造自 panel.py:800-891)
# ----------------------------------------------------------------------------
# 与 panel.py 的差异:
#   - 数据源从 _read_ros2_state() 文件 → 改读 NxWebNode 的订阅缓存 (退役 dog_state.json)
#   - xy 用 /odom 真值 (不再 _update_dead_reckon 速度积分)
#   - trail 本地累积, 每 0.1m 一个点, 上限 2000 (panel.py:772-775 同款)
#   - slam_source = "ros2_nx" (H1.1, 前端地图右上角显示 "SLAM: ros2_nx")
#   - scan 转世界坐标公式照抄 panel.py:815-821
#   - 不发 frame (阶段A 不直连狗 VideoClient, 前端 type==='frame' 自然显示"等待视频")
# ============================================================================
_trail = []
_obstacle_grid = ObstacleGridAccumulator(
    resolution=float(os.environ.get("GO2W_MAP_RESOLUTION", "0.1")),
    max_points=int(os.environ.get("GO2W_MAP_MAX_POINTS", "50000")),  # M3 fix: 5000→50000 (0.1m 栅格 20m×20m=40000 cell, LRU 太小会让墙点被淘汰)
)


def broadcast_loop(robot_bridge: NxRobotBridge, nx_node: NxWebNode, task_manager: TaskManager, ai_engine=None):
    global _trail
    logger.info("广播启动")
    slam_counter = 0
    while True:
        try:
            # ---- 取订阅缓存快照 (一次锁定) ----
            with nx_node._lock:
                yaw = nx_node._imu_yaw
                imu_count = nx_node._imu_count
                x = nx_node._odom_x
                y = nx_node._odom_y
                ranges = list(nx_node._scan_ranges)
                dog_state = nx_node._dog_state
                dog_vx = nx_node._dog_vx
                dog_vy = nx_node._dog_vy
                dog_vyaw = nx_node._dog_vyaw
                connected = (time.time() - nx_node._last_state_t) < nx_node.state_timeout

            # ---- 阶段B: 视频/YOLO 帧 (来自 ai_engine._video_yolo_loop 缓存, 不阻塞) ----
            # type=frame 格式严格对齐 panel.py:847-849 / panel.html:384-389:
            #   detections = 整数计数 (C1.4), 不是数组! 像素 bbox 已画在 jpeg 里。
            if ai_engine is not None:
                det_count = ai_engine.get_frame_detection_count()
                if det_count is not None:
                    ws_broadcast({"type": "frame", "detections": int(det_count)})

            # ---- trail 累积 (每 0.1m 一个点, 上限 2000) ----
            if not _trail:
                _trail.append([round(x, 2), round(y, 2)])
            else:
                lx, ly = _trail[-1]
                if math.hypot(x - lx, y - ly) > 0.1:
                    _trail.append([round(x, 2), round(y, 2)])
                    if len(_trail) > 2000:
                        _trail = _trail[-2000:]

            # ---- scan 机体系 → 世界坐标 (yaw 旋转 + 平移), 截断 200 点 ----
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            scan_pts = []
            if ranges:
                n = len(ranges)
                for i, r in enumerate(ranges):
                    if 0.1 < r < 9.9:
                        ang = -math.pi + i * 2 * math.pi / n
                        lx = r * math.cos(ang)
                        ly = r * math.sin(ang)
                        scan_pts.append([round(cos_y * lx - sin_y * ly + x, 2),
                                         round(sin_y * lx + cos_y * ly + y, 2)])
                        if len(scan_pts) >= 200:
                            break
            map_pts = _obstacle_grid.update(scan_pts) if scan_pts else _obstacle_grid.points()

            # ---- SLAM 推送 (字段名严格匹配 map.js update()) ----
            # 阶段B: slam.data.detections 是数组 [{x,y,class}] (C1.5, map.js:52),
            # 与 type=frame 的整数 detections 相反! 由 ai_engine 世界坐标转换填值。
            slam_counter += 1
            if slam_counter % 2 == 0:   # 约 3Hz, 够画地图 (与 panel.py:856 一致)
                det_world = (ai_engine.get_detections_world(x, y, yaw)
                             if ai_engine is not None else [])
                ws_broadcast({"type": "slam", "data": {
                    "x": round(x, 2),
                    "y": round(y, 2),
                    "yaw": round(yaw, 2),
                    "trail": _trail,
                    "map": map_pts,
                    "scan": scan_pts,
                    "detections": det_world,
                    "waypoints": [],
                    "currentWP": -1,
                    "slam_source": "ros2_nx",
                }})

            # ---- status 推送 (字段名匹配 panel.html:396-400) ----
            det_list = ai_engine.get_detection_list() if ai_engine is not None and hasattr(ai_engine, "get_detection_list") else []
            ws_broadcast({"type": "status",
                          "imu_yaw": round(yaw, 3),
                          "stats": {"imu_count": imu_count,
                                     "robot_mode": 0,
                                     "robot_velocity": [dog_vx, dog_vy, dog_vyaw],
                                     "connected": connected},
                          "dog_state": dog_state,
                          "tasks": task_manager.get_state() if task_manager else {},
                          "det_list": det_list})

            time.sleep(0.15)
        except Exception as e:
            logger.warning(f"广播: {e}")
            time.sleep(0.5)


# ============================================================================
# Main
# ============================================================================
def main():
    global robot, task_mgr, node, room_orchestrator

    rclpy.init()
    node = NxWebNode()

    # 决策 2: rclpy.spin 独立 daemon 线程 (不能与 HTTPServer.serve_forever 同线程)
    # 参考 cmd_publisher.py:88-90
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()

    # 机器人抽象 + 任务管理器 (阶段A detector/vlm 传 None; 阶段B 注入 AI 代理)
    robot = NxRobotBridge(node)

    # 阶段B (spec §7.2.5): 创建 NxAiEngine + 启动 + 注入 TaskManager / NxRobotBridge
    ai_engine = None
    vlm_proxy = None
    detector_proxy = None
    if AI_OK and NxAiEngine is not None:
        try:
            ai_engine = NxAiEngine()
            ai_engine.start()  # 启动 3 daemon 线程 (video/vlm/mem), 懒加载不拖慢启动
            robot._ai_engine = ai_engine  # 让 get_frame() 能委托 (spec §7.2.3)
            vlm_proxy = NxAiVlmProxy(ai_engine)
            detector_proxy = NxAiDetectorProxy(ai_engine)
            logger.info("阶段B: NxAiEngine 已注入 TaskManager + NxRobotBridge")
        except Exception as e:
            logger.error(f"阶段B NxAiEngine 启动失败, 退化阶段A: {e}")
            ai_engine = None
            vlm_proxy = None
            detector_proxy = None

    task_mgr = TaskManager(robot, vlm_engine=vlm_proxy, detector=detector_proxy)

    # C13 云台双流桥接 (独立 daemon 线程拉 vis+ir RTSP → type=gimbal 推前端)
    gimbal_bridge = None
    if GIMBAL_OK and GimbalRtspBridge is not None:
        try:
            gimbal_bridge = GimbalRtspBridge(ws_broadcast)
            gimbal_bridge.start()
            robot._gimbal_bridge = gimbal_bridge
            logger.info("C13 云台双流桥接已启动")
        except Exception as e:
            logger.error(f"C13 双流桥接启动失败 (前端无云台画面, 主服务不受影响): {e}")
            gimbal_bridge = None

    # Livox MID360 雷达点云展示 (订阅 /livox/lidar → type=lidar 推前端鸟瞰 png)
    lidar_bridge = None
    if LIDAR_OK and LidarBridge is not None:
        try:
            lidar_bridge = LidarBridge(ws_broadcast)
            lidar_bridge.start(node)
            logger.info("雷达点云展示桥接已启动")
        except Exception as e:
            logger.error(f"雷达点云展示启动失败 (主服务不受影响): {e}")
            lidar_bridge = None

    # 阶段E (spec §7.2.5): 创建 RoomSearchOrchestrator + 注入 TaskManager
    room_orchestrator = None
    if ROOM_ORCH_OK and RoomSearchOrchestrator is not None:
        try:
            room_orchestrator = RoomSearchOrchestrator(
                node=node,
                ai_engine=ai_engine,           # 阶段B NxAiEngine (读 get_detections_world 快照)
                ws_broadcast_fn=ws_broadcast,  # 推 type=search_room / mission_report / search
            )
            task_mgr.room_orchestrator = room_orchestrator
            # _fallback_parse._extract_room_name 通过类属性拿到 room_orchestrator
            TaskManager._global_room_orchestrator = room_orchestrator
            logger.info("阶段E: RoomSearchOrchestrator 已注入 TaskManager")
        except Exception as e:
            logger.error(f"阶段E RoomSearchOrchestrator 启动失败 (search_room 任务将标 failed): {e}")
            room_orchestrator = None

    static_dir = os.path.join(_WEB_DIR, 'static')
    host = node.host
    port = node.port
    ws_port = node.ws_port
    logger.info(f"Web: http://{host}:{port}  WS: ws://{host}:{ws_port}  "
                f"AI={'on' if ai_engine else 'off'}  Room={'on' if room_orchestrator else 'off'}")

    # 启动 WS server 线程 + 任务 worker + 广播线程
    threading.Thread(target=run_ws, args=(host, ws_port), daemon=True).start()
    task_mgr.start_worker()
    threading.Thread(target=broadcast_loop, args=(robot, node, task_mgr, ai_engine), daemon=True).start()

    server = create_server(host, port, static_dir)
    try:
        # 主线程阻塞 serve_forever (M2.3, 与 panel.py:940 一致)
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("收到 SIGINT, 退出中...")
    finally:
        # 退出顺序 (M2.1, LOW-2 修正): HTTPServer.shutdown 先停 (拒绝新请求/释放 serve_forever)
        # → ai_engine.stop 停 3 daemon 线程 + unload VLM → WS loop.stop → rclpy.shutdown → destroy_node
        # web 进程不控狗, destroy_node 不调 Damp (M2.2, 与 nx_motion_node 不同)
        try:
            server.shutdown()
        except Exception:
            pass
        try:
            if ai_engine is not None:
                ai_engine.stop()  # 阶段B: 停 3 daemon 线程 + unload VLM
        except Exception:
            pass
        try:
            if gimbal_bridge is not None:
                gimbal_bridge.stop()
        except Exception:
            pass
        try:
            if lidar_bridge is not None:
                lidar_bridge.stop()
        except Exception:
            pass
        # 阶段E: 取消进行中的 Nav2 goal (spec §11 进程退出清理)
        try:
            if room_orchestrator is not None:
                room_orchestrator.cancel()
        except Exception:
            pass
        if WS_LOOP:
            try:
                WS_LOOP.call_soon_threadsafe(WS_LOOP.stop)
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        logger.info("nx_web 已退出")


if __name__ == '__main__':
    main()
