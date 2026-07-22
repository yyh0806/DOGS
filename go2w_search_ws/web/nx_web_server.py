#!/usr/bin/env python3
"""Go2W NX Web 服务 (阶段A: web 通信层上移 NX)。

== 职责 ==
载荷 NX 上跑的 web 服务进程，内嵌 rclpy 节点：
  - HTTP:8000  + WebSocket:8001  (与 web/panel.py 端口/契约完全一致, 前端无感)
  - 本机发布 /cmd_vel (Twist) /cmd_pose (String)   → 被 nx_motion_node 消费控狗
  - 本机订阅 /dog_state /imu /scan /localization_pose → broadcast_loop 推 WS 给前端

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
from urllib.parse import parse_qs, unquote, urlparse

_WEB_DIR = Path(__file__).resolve().parent
_WS_DIR = _WEB_DIR.parent
if str(_WS_DIR) not in sys.path:
    sys.path.insert(0, str(_WS_DIR))

_BRIDGE_ROOT = _WS_DIR / "src" / "go2w_bridge"
if _BRIDGE_ROOT.is_dir() and str(_BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_ROOT))

try:
    from go2w_bridge.build_info import release_id
except ImportError:  # Direct-file compatibility deployment on the NX.
    from build_info import release_id


RELEASE_ID = release_id()

# ---- ROS2 (NX 本机, Humble) ----
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
try:
    from sensor_msgs.msg import PointCloud2
except ImportError:  # ROS-free contract-test stubs may omit this optional type.
    class PointCloud2:  # pragma: no cover - production ROS always supplies it
        pass
from std_msgs.msg import String

from nx_slam_map import ObstacleGridAccumulator
from nx_point_nav import PointNavigationController
from nx_navigation_gateway import (
    MissionNavigationPort,
    NavigationGateway,
    OwnerNavigationPort,
    RosComputePathPort,
)
from nx_navigation_arbiter import NavigationArbiter
from nx_camera_calibration import resolve_camera_calibration
from nx_person_localizer import decode_pointcloud_xyz
from nx_motion_intent import build_motion_intent
from nx_ws_latest import (
    LatestValueOutbox,
    ReliableQueueFull,
    classify_message,
    serialize_message,
)
from nx_mission_schema import (
    MissionValidationError,
    SearchMissionRequest,
    canonicalize_move_tasks,
    canonicalize_search_tasks,
)
from nx_move_executor import (
    compute_angular_target_yaw,
    compute_linear_target,
    directional_clearance_from_scan,
    run_angular_turn,
    run_linear_translation,
    sanitize_clearance_margin,
)
from nx_observation_sync import ObservationSynchronizer
from nx_control_auth import (
    authorize_request,
    cors_origin_allowed,
    parse_allowed_origins,
)

import logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("go2w.nx_web")

try:
    POINT_GOAL_MAX_DISTANCE = float(
        os.environ.get("GO2W_POINT_GOAL_MAX_DISTANCE", "20.5")
    )
except (TypeError, ValueError):
    POINT_GOAL_MAX_DISTANCE = 20.5
if not math.isfinite(POINT_GOAL_MAX_DISTANCE) or POINT_GOAL_MAX_DISTANCE <= 0.0:
    POINT_GOAL_MAX_DISTANCE = 20.5


def _point_goal_within_local_radius(x, y, localization, max_distance=None):
    """Reject map-frame goals too far from the current trusted pose."""
    limit = POINT_GOAL_MAX_DISTANCE if max_distance is None else float(max_distance)
    try:
        values = (
            float(x), float(y), float(localization["x"]),
            float(localization["y"]), float(limit),
        )
    except (KeyError, TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in values) or values[4] <= 0.0:
        return False
    return math.hypot(values[0] - values[2], values[1] - values[3]) <= values[4]

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
#   - Nav2 action callbacks 仅由主 executor 驱动；worker 只等 Condition
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
# Product command parser (deterministic offline path; VLM remains fail-closed)
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
_WS_RELIABLE_CAPACITY = max(
    8, int(os.environ.get("GO2W_WS_RELIABLE_CAPACITY", "256")))
_WS_SEND_TIMEOUT = max(0.02, float(os.environ.get("GO2W_WS_SEND_TIMEOUT", "1.0")))  # H4 fix: 0.2→1.0 (本地/局域网, 避免瞬时抖动误踢健康连接)
_WS_OUTBOXES = {}
_WS_SENDER_TASKS = {}
_WS_CLOSING = set()
_WS_REGISTRY_LOCK = threading.Lock()
_WS_INGRESS = LatestValueOutbox(reliable_capacity=_WS_RELIABLE_CAPACITY)
_WS_INGRESS_LOCK = threading.Lock()
_WS_INGRESS_SCHEDULED = False
_WS_INGRESS_OVERFLOWED = False
_WS_STREAM_REPLACED = 0
_WS_METRICS_LOCK = threading.Lock()
# C1 fix: NxRobotBridge.move 执行层 guard — 前进时前方 /scan 障碍 < 此阈值→强制 vx=0 (防自主跟踪撞墙)
_FRONT_CLEARANCE_M = float(os.environ.get("GO2W_FRONT_CLEARANCE", "0.5"))
_REVERSE_CLEARANCE_M = sanitize_clearance_margin(
    os.environ.get("GO2W_REVERSE_CLEARANCE", "0.55"))


def ws_broadcast(data, force=False):
    """Queue a broadcast without putting network work on a ROS callback.

    ``force`` remains accepted for older producers (notably the C13 bridge),
    but it no longer bypasses backpressure or creates unbounded coroutines.
    """
    global _WS_INGRESS_SCHEDULED, _WS_INGRESS_OVERFLOWED
    del force
    loop = WS_LOOP
    with _WS_REGISTRY_LOCK:
        has_clients = bool(WS_CLIENTS)
    if loop is None or not has_clients:
        return
    should_schedule = False
    with _WS_INGRESS_LOCK:
        try:
            replaced = _WS_INGRESS.enqueue(data, notify=False)
            if replaced:
                _add_ws_stream_replaced()
        except ReliableQueueFull:
            # An event cannot be silently dropped.  Mark the batch failed and
            # close clients from the WS loop with a retryable overload code.
            _WS_INGRESS_OVERFLOWED = True
        if not _WS_INGRESS_SCHEDULED:
            _WS_INGRESS_SCHEDULED = True
            should_schedule = True
    if should_schedule:
        try:
            loop.call_soon_threadsafe(_drain_ws_ingress)
        except RuntimeError:
            with _WS_INGRESS_LOCK:
                _WS_INGRESS_SCHEDULED = False


def _add_ws_stream_replaced(amount=1):
    global _WS_STREAM_REPLACED
    with _WS_METRICS_LOCK:
        _WS_STREAM_REPLACED += int(amount)


def _drain_ws_ingress():
    """Run on WS_LOOP; distribute one bounded/coalesced producer batch."""
    global _WS_INGRESS_SCHEDULED, _WS_INGRESS_OVERFLOWED
    # Drain exactly one atomic snapshot and yield.  Producers arriving after
    # scheduled is reset below arrange another call_soon_threadsafe callback;
    # this prevents a hot producer from monopolizing the WS event loop.
    with _WS_INGRESS_LOCK:
        messages = _WS_INGRESS.drain_nowait()
        overflowed = _WS_INGRESS_OVERFLOWED
        _WS_INGRESS_OVERFLOWED = False
        _WS_INGRESS_SCHEDULED = False
    if overflowed:
        logger.error("WebSocket reliable ingress overflow; closing clients")
        for ws in _ws_client_snapshot():
            _schedule_ws_disconnect(ws, "reliable ingress overflow")
    for message in messages:
        try:
            serialized = serialize_message(message)
        except (TypeError, ValueError, OverflowError) as exc:
            logger.warning("WebSocket JSON serialization failed: %s", exc)
            if classify_message(message) == "reliable":
                # A reliable event cannot be silently skipped.  Close every
                # affected connection so clients must rehydrate after retry.
                for ws in _ws_client_snapshot():
                    _schedule_ws_disconnect(
                        ws, "reliable serialization failure")
            continue
        with _WS_REGISTRY_LOCK:
            clients = [
                (ws, _WS_OUTBOXES.get(ws)) for ws in WS_CLIENTS
            ]
        for ws, outbox in clients:
            if outbox is None:
                continue
            try:
                if outbox.enqueue_serialized(serialized):
                    _add_ws_stream_replaced()
            except ReliableQueueFull:
                logger.warning("WebSocket reliable client queue overflow")
                _schedule_ws_disconnect(ws, "reliable queue overflow")


def _schedule_ws_disconnect(ws, reason):
    if ws in _WS_CLOSING:
        return
    _WS_CLOSING.add(ws)
    asyncio.create_task(_disconnect_ws(ws, reason))


def _ws_client_snapshot():
    with _WS_REGISTRY_LOCK:
        return list(WS_CLIENTS)


async def _disconnect_ws(ws, reason):
    try:
        await asyncio.wait_for(
            ws.close(code=1013, reason=reason), timeout=_WS_SEND_TIMEOUT)
    except Exception:
        pass
    finally:
        await _unregister_ws(ws)
        _WS_CLOSING.discard(ws)


async def _ws_client_sender(ws, outbox):
    close_reason = "sender stopped"
    try:
        await outbox.send_forever(ws, timeout=_WS_SEND_TIMEOUT)
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        logger.debug("WebSocket sender stopped: %s", exc)
        close_reason = "send timeout"
    finally:
        try:
            await asyncio.wait_for(
                ws.close(code=1013, reason=close_reason),
                timeout=_WS_SEND_TIMEOUT,
            )
        except Exception:
            pass
        # Do not rely on close() waking the handler promptly: sender failure
        # owns its registry cleanup too.  _unregister_ws is idempotent.
        await _unregister_ws(ws)


async def _register_ws(ws):
    outbox = LatestValueOutbox(reliable_capacity=_WS_RELIABLE_CAPACITY)
    sender = asyncio.create_task(_ws_client_sender(ws, outbox))
    with _WS_REGISTRY_LOCK:
        WS_CLIENTS.add(ws)
        _WS_OUTBOXES[ws] = outbox
        _WS_SENDER_TASKS[ws] = sender


async def _unregister_ws(ws):
    with _WS_REGISTRY_LOCK:
        WS_CLIENTS.discard(ws)
        outbox = _WS_OUTBOXES.pop(ws, None)
        sender = _WS_SENDER_TASKS.pop(ws, None)
    if outbox is not None:
        outbox.close()
    if sender is not None and sender is not asyncio.current_task():
        sender.cancel()
        await asyncio.gather(sender, return_exceptions=True)


def ws_telemetry():
    """Return WS pressure metrics for status snapshots.

    ``ws_stream_replaced`` is cumulative across producer ingress and every
    client outbox. ``ws_reliable_depth`` is an aggregate gauge and therefore
    includes duplicated per-client reliable backlog plus producer ingress.
    """
    with _WS_REGISTRY_LOCK:
        outboxes = list(_WS_OUTBOXES.values())
        connected_clients = len(WS_CLIENTS)
    with _WS_METRICS_LOCK:
        replaced = _WS_STREAM_REPLACED
    return {
        "ws_stream_replaced": replaced,
        "ws_reliable_depth": (
            _WS_INGRESS.reliable_depth
            + sum(outbox.reliable_depth for outbox in outboxes)
        ),
        "ws_connected_clients": connected_clients,
    }


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

# ---- nav2 自主导航链路服务状态 (前端顶部红/黄/绿灯) ----
# 用户需求 (2026-07-17): "梳理执行 nav2 所需的所有服务, 前端上方红黄绿灯".
# 11 个服务 = 雷达→LIO→TF→建图→地图→扫描→Nav2→运动→底盘→代价图→Web 全链.
NAV2_SERVICE_LIST = [
    "livox-mid360-driver",
    "fastlio",
    "map-odom-fuser",
    "slam-online",
    "map-padding",
    "mid360-nav-bridge",
    "nav2-3d",
    "go2w-motion",
    "go2w-sport-gateway",
    "costmap-bridge",
    "go2w-web",
]
_NAV2_SERVICE_LABEL = {
    "livox-mid360-driver": "雷达",
    "fastlio": "LIO",
    "map-odom-fuser": "TF",
    "slam-online": "建图",
    "map-padding": "地图",
    "mid360-nav-bridge": "扫描",
    "nav2-3d": "Nav2",
    "go2w-motion": "运动",
    "go2w-sport-gateway": "底盘",
    "costmap-bridge": "代价图",
    "go2w-web": "Web",
}


def _systemctl_services_active(names):
    """一次 systemctl is-active 批量查询. 返回 {name: state_str}.

    systemctl is-active 接受多 unit, stdout 每行一个状态 (active/inactive/failed/
    activating/...), exit code 仅当全 active 才 0 (故用 stdout 解析, 不看 rc).
    局部 import subprocess: 顶部未导入 (与 /api/clear_all 端点一致).
    """
    import subprocess  # 局部: 顶部未 import (见 /api/clear_all 同模式)
    try:
        r = subprocess.run(
            ["systemctl", "is-active", *names],
            capture_output=True, text=True, timeout=4)
        lines = (r.stdout or "").strip().splitlines()
        return {n: (lines[i].strip() if i < len(lines) else "unknown")
                for i, n in enumerate(names)}
    except Exception:
        return {n: "err" for n in names}


class NxWebNode(Node):
    @staticmethod
    def _validate_localization_safety_parameters(timeout, max_tilt_deg):
        """Reject values that could disable stale/tilt safety gates."""
        try:
            timeout = float(timeout)
            max_tilt_deg = float(max_tilt_deg)
        except (TypeError, ValueError) as exc:
            raise ValueError("localization safety parameters must be finite numbers") from exc
        if not math.isfinite(timeout) or timeout <= 0.0 or timeout > 10.0:
            raise ValueError("localization_timeout must be in (0, 10] seconds")
        if (
            not math.isfinite(max_tilt_deg)
            or max_tilt_deg <= 0.0
            or max_tilt_deg > 45.0
        ):
            raise ValueError("localization_max_tilt must be in (0, 45] degrees")
        return timeout, max_tilt_deg

    def __init__(self):
        super().__init__('nx_web_node')

        # ---- 参数 (与 panel.py main 的环境变量对齐) ----
        self.declare_parameter('host', os.environ.get('GO2W_HOST', '0.0.0.0'))
        self.declare_parameter('port', int(os.environ.get('GO2W_PORT', '8000')))
        self.declare_parameter('ws_port', int(os.environ.get('GO2W_WS_PORT', '8001')))
        self.declare_parameter('state_timeout', 3.0)
        self.declare_parameter(
            'localization_timeout',
            float(os.environ.get('GO2W_LOCALIZATION_TIMEOUT', '1.0')),
        )
        self.declare_parameter(
            'localization_max_tilt',
            float(os.environ.get('GO2W_LOCALIZATION_MAX_TILT_DEG', '10.0')),
        )

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value
        self.state_timeout = self.get_parameter('state_timeout').get_parameter_value().double_value
        localization_timeout = self.get_parameter(
            'localization_timeout').get_parameter_value().double_value
        localization_max_tilt_deg = self.get_parameter(
            'localization_max_tilt').get_parameter_value().double_value
        self.localization_timeout, localization_max_tilt_deg = (
            self._validate_localization_safety_parameters(
                localization_timeout, localization_max_tilt_deg
            )
        )
        self.localization_max_tilt = math.radians(localization_max_tilt_deg)
        self.observation_sync = ObservationSynchronizer(max_samples=240)

        # ---- 发布器 (本机 → nx_motion_node 订阅) ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.cmd_pose_pub = self.create_publisher(String, '/cmd_pose', 10)
        self.motion_session_pub = self.create_publisher(String, '/motion_session', 10)

        # ---- 订阅器 (本机 ← nx_motion_node / nx_sensor_node 发布) ----
        # QoS 说明: nx_sensor_node.py 的 /imu /scan 发布端与 map_odom_fuser 的
        # /localization_pose 发布端都使用默认 RELIABLE QoS (depth=10)。
        # 默认 RELIABLE QoS (depth=10), 不是 qos_profile_sensor_data。ROS2 QoS 兼容性:
        # REL 发布 + BE 订阅 = 不兼容 (订阅收不到任何数据)。为能收到 nx_sensor_node 的数据,
        # /imu /scan /localization_pose 这里都用 depth=10 的默认 RELIABLE, 与发布端匹配。
        # /dog_state 由 nx_motion_node 发布 (RELIABLE depth=10)。
        # (spec H2.4 字面写 sensor_data, 但与真实发布端冲突 → 以"能收到数据"为准, 见本注释)
        # Keep strong references.  rclpy entities can otherwise be garbage
        # collected while their DDS endpoints remain visible, leaving Panel
        # status permanently stale after a service restart.
        self._dog_state_sub = self.create_subscription(
            String, '/dog_state', self._on_dog_state, 10)
        self._imu_sub = self.create_subscription(
            Imu, '/livox/imu', self._on_imu, 10)
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan_mid360', self._on_scan, 10)
        self._pointcloud_sub = self.create_subscription(
            PointCloud2, '/mid360/points_nav', self._on_pointcloud,
            qos_profile_sensor_data)
        self._localization_sub = self.create_subscription(
            Odometry, '/localization_pose', self._on_localization_pose, 10)
        # Diagnostic-only pose from the dog-mounted MID360 LIO.  Navigation
        # continues to use /localization_pose in map; exposing /odom lets the
        # operator distinguish LIO motion from SLAM map correction.
        self._lio_odometry_sub = self.create_subscription(
            Odometry, '/odom', self._on_lio_odometry, 10)

        # ---- 订阅缓存 (Lock 保护, H2.2) ----
        self._lock = threading.RLock()
        self._dog_state = "UNKNOWN"      # STOPPED/MOVING/STANDING/... (来自 /dog_state)
        self._dog_vx = self._dog_vy = self._dog_vyaw = 0.0
        self._motion_sdk_ready = False
        self._motion_nav_scan_fresh = False
        self._motion_nav_guard_reason = None
        self._motion_battery_soc = None
        self._motion_drive_fault = None
        self._motion_sport_mode = None
        self._motion_wheel_dq = None
        self._motion_wheel_activation_phase = "unknown"
        self._motion_drive_session = "startup"
        self._motion_drive_session_owner = None
        self._motion_drive_session_phase = "startup"
        self._motion_drive_session_reason = "waiting_for_feedback"
        self._motion_release_id = None
        self._motion_schema_version = 0
        self._motion_physical_mode = "unknown"
        self._motion_actual_motion = "unknown"
        self._motion_velocity_authorized = False
        self._motion_service = None
        self.motion_min_battery_soc = 20.0
        self._imu_yaw = 0.0
        self._imu_count = 0
        self._scan_count = 0
        self._scan_ranges = []           # 原始 ranges (机体系)
        self._scan_angle_min = 0.0
        self._scan_angle_increment = 0.0
        self._scan_range_min = 0.0
        self._scan_range_max = 0.0
        self._scan_timestamp = 0.0
        self._scan_received_monotonic = None
        self._pointcloud_msg = None
        self._pointcloud_timestamp = 0.0
        self._pointcloud_count = 0
        self._map_x = 0.0
        self._map_y = 0.0
        self._map_z = 0.0
        self._map_yaw = 0.0
        self._localization_count = 0
        self._svc_cache = (0.0, None)  # (ts, {svc: state}) systemctl 批量结果缓存 2s
        self._localization_received_monotonic = None
        self._localization_frame_id = ""
        self._localization_child_frame_id = ""
        self._localization_stamp = {"sec": 0, "nanosec": 0}
        self._localization_valid = False
        self._localization_reason = "not_received"
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_z = 0.0
        self._odom_yaw = 0.0
        self._odom_count = 0
        self._odom_received_monotonic = None
        self._odom_frame_id = ""
        self._odom_child_frame_id = ""
        self._odom_stamp = {"sec": 0, "nanosec": 0}
        self._odom_valid = False
        self._odom_reason = "not_received"
        self._last_state_t = 0.0         # 最近一次 /dog_state 时间 (判 connected)

        self.get_logger().info(
            "NxWebNode 就绪: 发 /cmd_vel /cmd_pose, "
            "订阅 /dog_state /livox/imu /scan_mid360 "
            "/mid360/points_nav /localization_pose /odom(diagnostic)")

    # ---- ROS2 回调 (在 spin 线程内执行) ----
    def _on_dog_state(self, msg: String):
        """Cache motion state and its explicit SDK/scan readiness gates."""
        try:
            d = json.loads(msg.data)
            with self._lock:
                self._motion_schema_version = int(d.get('schema_version', 0))
                canonical_session = str(d.get('session', d.get(
                    'drive_session', 'boot_hold')))
                self._motion_physical_mode = str(
                    d.get('physical_mode', 'unknown'))
                self._motion_actual_motion = str(
                    d.get('actual_motion', 'unknown'))
                self._motion_velocity_authorized = (
                    d.get('velocity_authorized') is True)
                motion_service = d.get('motion_service')
                self._motion_service = (
                    str(motion_service) if motion_service else None)
                raw = d.get('raw', {})
                if not isinstance(raw, dict):
                    raw = {}
                self._dog_state = d.get('state', 'UNKNOWN')
                self._dog_vx = float(d.get('vx', 0.0))
                self._dog_vy = float(d.get('vy', 0.0))
                self._dog_vyaw = float(d.get('vyaw', 0.0))
                self._motion_sdk_ready = (
                    self._motion_service == 'ai-w'
                    if self._motion_schema_version >= 4
                    else d.get('sdk_ready') is True)
                self._motion_nav_scan_fresh = d.get('nav_scan_fresh') is True
                nav_guard_reason = d.get('nav_guard_reason')
                self._motion_nav_guard_reason = (
                    str(nav_guard_reason) if nav_guard_reason else None)
                battery_soc = d.get('battery_soc')
                try:
                    battery_soc = float(battery_soc)
                except (TypeError, ValueError, OverflowError):
                    battery_soc = None
                self._motion_battery_soc = (
                    battery_soc if battery_soc is not None
                    and math.isfinite(battery_soc) and 0.0 <= battery_soc <= 100.0
                    else None)
                self._motion_drive_fault = d.get('drive_fault')
                sport_mode = raw.get('sport_mode', d.get('sport_mode'))
                try:
                    sport_mode = int(sport_mode)
                except (TypeError, ValueError, OverflowError):
                    sport_mode = None
                self._motion_sport_mode = (
                    sport_mode if sport_mode is not None
                    and 0 <= sport_mode <= 255 else None)
                wheel_dq = d.get('wheel_dq')
                try:
                    wheel_dq = [float(value) for value in wheel_dq]
                except (TypeError, ValueError, OverflowError):
                    wheel_dq = None
                self._motion_wheel_dq = (
                    wheel_dq if wheel_dq is not None and len(wheel_dq) == 4
                    and all(math.isfinite(value) for value in wheel_dq)
                    else None)
                self._motion_wheel_activation_phase = str(
                    d.get('wheel_activation_phase', 'unknown'))
                self._motion_drive_session = canonical_session
                owner = d.get('owner', d.get('drive_session_owner'))
                self._motion_drive_session_owner = (
                    str(owner) if owner in ('manual', 'nav', 'startup', 'safety')
                    else None)
                self._motion_drive_session_phase = str(
                    d.get('drive_session_phase', self._motion_drive_session))
                self._motion_drive_session_reason = str(
                    d.get('drive_session_reason', 'unknown'))
                motion_release_id = d.get('release_id')
                self._motion_release_id = (
                    str(motion_release_id).strip()[:64]
                    if motion_release_id else None)
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
        stamp = self._message_stamp_seconds(msg)
        received_monotonic = time.monotonic()
        scan_snapshot = {
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "ranges": [float(r) for r in msg.ranges],
            "timestamp": stamp,
        }
        with self._lock:
            self._scan_ranges = [round(r, 3) for r in scan_snapshot["ranges"]]
            self._scan_angle_min = scan_snapshot["angle_min"]
            self._scan_angle_increment = scan_snapshot["angle_increment"]
            self._scan_range_min = scan_snapshot["range_min"]
            self._scan_range_max = scan_snapshot["range_max"]
            self._scan_timestamp = stamp or time.time()
            self._scan_received_monotonic = received_monotonic
            self._scan_count += 1
        synchronizer = getattr(self, "observation_sync", None)
        if stamp is not None and synchronizer is not None:
            synchronizer.add_scan(stamp=stamp, scan=scan_snapshot)

    def _on_pointcloud(self, msg: PointCloud2):
        # Retain the ROS message and decode only when a person observation
        # needs height evidence.  This keeps the 10 Hz callback inexpensive.
        stamp = self._message_stamp_seconds(msg)
        with self._lock:
            self._pointcloud_msg = msg
            self._pointcloud_timestamp = stamp or time.time()
            self._pointcloud_count += 1
        synchronizer = getattr(self, "observation_sync", None)
        if stamp is not None and synchronizer is not None:
            synchronizer.add_cloud(stamp=stamp, cloud=msg)

    @staticmethod
    def _message_stamp_seconds(msg):
        try:
            stamp = msg.header.stamp
            value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        return value if math.isfinite(value) and value > 0.0 else None

    def _mark_localization_invalid(self, msg, reason):
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        with self._lock:
            self._localization_count += 1
            self._localization_received_monotonic = time.monotonic()
            self._localization_frame_id = str(getattr(header, "frame_id", ""))
            self._localization_child_frame_id = str(getattr(msg, "child_frame_id", ""))
            self._localization_stamp = {
                "sec": int(getattr(stamp, "sec", 0)),
                "nanosec": int(getattr(stamp, "nanosec", 0)),
            }
            self._localization_valid = False
            self._localization_reason = str(reason)

    def _on_localization_pose(self, msg: Odometry):
        """Cache only a finite, horizontal map -> base_link localization pose."""
        header = getattr(msg, "header", None)
        source_stamp = self._message_stamp_seconds(msg)
        frame_id = str(header.frame_id) if header is not None else ""
        child_frame_id = str(getattr(msg, "child_frame_id", ""))
        if frame_id != "map":
            self._mark_localization_invalid(msg, "invalid_frame")
            return
        if child_frame_id != "base_link":
            self._mark_localization_invalid(msg, "invalid_child_frame")
            return

        try:
            pose = msg.pose.pose
            p = pose.position
            q = pose.orientation
            values = tuple(float(value) for value in (
                p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            ))
        except (AttributeError, TypeError, ValueError):
            self._mark_localization_invalid(msg, "nonfinite_pose")
            return
        if not all(math.isfinite(value) for value in values):
            self._mark_localization_invalid(msg, "nonfinite_pose")
            return

        x, y, z, qx, qy, qz, qw = values
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or norm <= 1e-12:
            self._mark_localization_invalid(msg, "invalid_quaternion")
            return
        qx, qy, qz, qw = (component / norm for component in (qx, qy, qz, qw))

        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch_sine = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.asin(pitch_sine)
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        if abs(roll) > self.localization_max_tilt or abs(pitch) > self.localization_max_tilt:
            self._mark_localization_invalid(msg, "excessive_tilt")
            return

        stamp = getattr(header, "stamp", None)
        with self._lock:
            self._map_x = x
            self._map_y = y
            self._map_z = z
            self._map_yaw = yaw
            self._localization_count += 1
            self._localization_received_monotonic = time.monotonic()
            self._localization_frame_id = frame_id
            self._localization_child_frame_id = child_frame_id
            self._localization_stamp = {
                "sec": int(getattr(stamp, "sec", 0)),
                "nanosec": int(getattr(stamp, "nanosec", 0)),
            }
            self._localization_valid = True
            self._localization_reason = "ok"
        synchronizer = getattr(self, "observation_sync", None)
        if source_stamp is not None and synchronizer is not None:
            synchronizer.add_pose(
                stamp=source_stamp, x=x, y=y, yaw=yaw)

    def _on_lio_odometry(self, msg: Odometry):
        """Cache the planar MID360 LIO pose for operator diagnostics only."""
        header = getattr(msg, "header", None)
        frame_id = str(getattr(header, "frame_id", ""))
        child_frame_id = str(getattr(msg, "child_frame_id", ""))
        stamp = getattr(header, "stamp", None)
        reason = None
        if frame_id != "odom":
            reason = "invalid_frame"
        elif child_frame_id != "base_link":
            reason = "invalid_child_frame"
        try:
            pose = msg.pose.pose
            values = tuple(float(value) for value in (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ))
        except (AttributeError, TypeError, ValueError, OverflowError):
            values = ()
            reason = reason or "nonfinite_pose"
        if values and not all(math.isfinite(value) for value in values):
            reason = reason or "nonfinite_pose"

        yaw = 0.0
        if reason is None:
            x, y, z, qx, qy, qz, qw = values
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if not math.isfinite(norm) or norm <= 1e-12:
                reason = "invalid_quaternion"
            else:
                qx, qy, qz, qw = (
                    component / norm for component in (qx, qy, qz, qw)
                )
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz),
                )

        with self._lock:
            self._odom_count += 1
            self._odom_received_monotonic = time.monotonic()
            self._odom_frame_id = frame_id
            self._odom_child_frame_id = child_frame_id
            self._odom_stamp = {
                "sec": int(getattr(stamp, "sec", 0)),
                "nanosec": int(getattr(stamp, "nanosec", 0)),
            }
            self._odom_valid = reason is None
            self._odom_reason = reason or "ok"
            if reason is None:
                self._odom_x = x
                self._odom_y = y
                self._odom_z = z
                self._odom_yaw = yaw

    def get_odometry_snapshot(self):
        """Return raw LIO odometry without using it as the navigation pose."""
        with self._lock:
            received = self._odom_received_monotonic
            snapshot = {
                "frame_id": self._odom_frame_id,
                "child_frame_id": self._odom_child_frame_id,
                "stamp": dict(self._odom_stamp),
                "x": self._odom_x,
                "y": self._odom_y,
                "z": self._odom_z,
                "yaw": self._odom_yaw,
                "count": self._odom_count,
                "healthy": bool(self._odom_valid),
                "reason": str(self._odom_reason),
            }
        age_sec = None if received is None else time.monotonic() - received
        if snapshot["healthy"] and (
            age_sec is None
            or not math.isfinite(age_sec)
            or age_sec < 0.0
            or age_sec > self.localization_timeout
        ):
            snapshot["healthy"] = False
            snapshot["reason"] = "stale"
        snapshot["age_sec"] = age_sec
        snapshot["timeout_sec"] = self.localization_timeout
        return snapshot

    def get_localization_health(self):
        """Return a thread-safe map-localization snapshot and reception age."""
        with self._lock:
            received = self._localization_received_monotonic
            valid = bool(self._localization_valid)
            reason = str(self._localization_reason)
            snapshot = {
                "frame_id": self._localization_frame_id,
                "child_frame_id": self._localization_child_frame_id,
                "stamp": dict(self._localization_stamp),
                "x": self._map_x,
                "y": self._map_y,
                "z": float(getattr(self, "_map_z", 0.0)),
                "yaw": self._map_yaw,
                "count": self._localization_count,
            }
        age_sec = None if received is None else time.monotonic() - received
        if valid and (age_sec is None or not math.isfinite(age_sec) or age_sec < 0.0):
            valid = False
            reason = "invalid_reception_age"
        elif valid and age_sec > self.localization_timeout:
            valid = False
            reason = "stale"
        snapshot.update({
            "healthy": valid,
            "reason": reason,
            "age_sec": age_sec,
            "timeout_sec": self.localization_timeout,
            "max_tilt_deg": math.degrees(self.localization_max_tilt),
        })
        return snapshot

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

    def publish_nav_cmd_vel(self, vx, vy, vyaw):
        """Publish obstacle-gated autonomous velocity to the Nav2 channel."""
        try:
            tw = Twist()
            tw.linear.x = float(vx)
            tw.linear.y = float(vy)
            tw.angular.z = float(vyaw)
            self.cmd_vel_nav_pub.publish(tw)
        except Exception as e:
            logger.warning(f"publish /cmd_vel_nav 失败: {e}")

    def publish_cmd_pose(self, cmd):
        """发布 /cmd_pose: data ∈ {'stand','sit','estop'} (nx_motion_node:126 接收)。"""
        try:
            s = String()
            s.data = str(cmd)
            self.cmd_pose_pub.publish(s)
        except Exception as e:
            logger.warning(f"publish /cmd_pose 失败: {e}")

    def publish_motion_session(self, intent, source="nx_web"):
        try:
            msg = String()
            msg.data = build_motion_intent(intent, source=source)
            self.motion_session_pub.publish(msg)
        except Exception as e:
            logger.warning(f"publish /motion_session 失败: {e}")

    # ---- 给 /api/status & NxRobotBridge 取缓存 ----
    def get_status_snapshot(self):
        localization = self.get_localization_health()
        navigation = self.get_navigation_readiness()
        with self._lock:
            now = time.time()
            connected = (now - self._last_state_t) < self.state_timeout
            return {
                "release_id": RELEASE_ID,
                "motion_release_id": self._motion_release_id,
                "release_consistent": (
                    self._motion_release_id == RELEASE_ID
                    if self._motion_release_id is not None else False),
                "connected": connected,
                "imu_yaw": round(self._imu_yaw, 3),
                "dog_state": self._dog_state,
                "stats": {
                    "imu_count": self._imu_count,
                    "scan_count": self._scan_count,
                    "odom_count": self._localization_count,
                    "localization_count": self._localization_count,
                    "robot_mode": 0,
                    "robot_velocity": [self._dog_vx, self._dog_vy, self._dog_vyaw],
                },
                "localization": localization,
                "odometry": self.get_odometry_snapshot(),
                "navigation": navigation,
                "services": self._collect_nav2_services(),
            }

    def _collect_nav2_services(self):
        """nav2 链路服务红/黄/绿灯 (前端顶部状态条).

        红 = systemd inactive/failed/err; 绿 = active + 关联数据流活;
        黄 = active 但数据流停滞 (service 活但 topic 无数据, 如 fastlio
        stamp 非单调那种隐性挂). systemctl 批量结果缓存 2s —— broadcast_loop
        每 0.15s 调一次, 每次 fork 子进程会拖死 NX. 数据流指标复用现有
        count/health (imu/scan/localization/navigation.ready/connected), 不加
        ROS 订阅 (web wait_set 已临界, 见 costmap_bridge.py 头注释).
        """
        now = time.time()
        cached_ts, cached_states = self._svc_cache
        if cached_states is not None and (now - cached_ts) < 2.0:
            states = cached_states
        else:
            states = _systemctl_services_active(NAV2_SERVICE_LIST)
            self._svc_cache = (now, states)
        loc_healthy = bool(self.get_localization_health().get("healthy"))
        # nav2-3d 灯只判 nav2 栈本身 (active + sdk + scan + 定位 + 无 fault),
        # 不判 drive_session 激活态: 狗未激活(parked)也能导航, 激活属 motion 层非 nav2 层.
        _nav = self.get_navigation_readiness()
        nav2_stack_ok = (bool(_nav.get("sdk_ready")) and bool(_nav.get("nav_scan_fresh"))
                         and loc_healthy and not _nav.get("drive_fault"))
        connected = (now - self._last_state_t) < self.state_timeout
        flow_alive = {
            "livox-mid360-driver": self._imu_count > 0,
            "fastlio": loc_healthy,
            "map-odom-fuser": loc_healthy,
            "slam-online": loc_healthy,
            "map-padding": loc_healthy,
            "mid360-nav-bridge": self._scan_count > 0,
            "nav2-3d": nav2_stack_ok,
            "go2w-motion": connected,
            "go2w-sport-gateway": True,   # 无直接 topic 指标, active 即绿
            "costmap-bridge": True,
            "go2w-web": True,
        }
        out = {}
        for n in NAV2_SERVICE_LIST:
            st = states.get(n, "unknown")
            color = "red" if st != "active" else ("green" if flow_alive.get(n) else "yellow")
            out[n] = {
                "label": _NAV2_SERVICE_LABEL.get(n, n),
                "state": st,
                "color": color,
            }
        return out

    def get_navigation_readiness(self):
        """Separate parked-goal admission from active Nav2 continuation."""
        localization = self.get_localization_health()
        localization_healthy = bool(localization.get("healthy"))
        localization_reason = str(localization.get("reason", "unknown"))
        localization_age_sec = localization.get("age_sec")
        with self._lock:
            state = str(self._dog_state)
            sdk_ready = bool(self._motion_sdk_ready)
            scan_fresh = bool(self._motion_nav_scan_fresh)
            nav_guard_reason = getattr(
                self, '_motion_nav_guard_reason', None)
            battery_soc = getattr(self, '_motion_battery_soc', None)
            drive_fault = getattr(self, '_motion_drive_fault', None)
            sport_mode = getattr(self, '_motion_sport_mode', None)
            wheel_dq = getattr(self, '_motion_wheel_dq', None)
            wheel_activation_phase = getattr(
                self, '_motion_wheel_activation_phase', 'unknown')
            drive_session = str(getattr(
                self, '_motion_drive_session', 'startup'))
            drive_session_owner = getattr(
                self, '_motion_drive_session_owner', None)
            drive_session_phase = str(getattr(
                self, '_motion_drive_session_phase', drive_session))
            drive_session_reason = str(getattr(
                self, '_motion_drive_session_reason', 'unknown'))
            physical_mode = str(getattr(
                self, '_motion_physical_mode', 'unknown'))
            actual_motion = str(getattr(
                self, '_motion_actual_motion', 'unknown'))
            velocity_authorized = bool(getattr(
                self, '_motion_velocity_authorized', False))
            status_schema = int(getattr(
                self, '_motion_schema_version', 0))
            motion_release_id = getattr(self, '_motion_release_id', None)
            minimum_soc = float(getattr(self, 'motion_min_battery_soc', 20.0))
            age_sec = time.time() - self._last_state_t
            fresh = math.isfinite(age_sec) and 0.0 <= age_sec < self.state_timeout
        drivable_state = state in {"STOPPED", "MOVING"}
        wheels_stopped = (
            wheel_dq is not None and len(wheel_dq) == 4
            and all(math.isfinite(float(value)) for value in wheel_dq)
            and sum(abs(float(value)) for value in wheel_dq) / 4.0 < 0.15)
        if status_schema < 4:
            physical_mode = (
                "joint_lock" if sport_mode == 6
                else "wheel_balance" if sport_mode in {1, 3}
                else "unknown")
            actual_motion = "stopped" if wheels_stopped else "moving"
            velocity_authorized = drive_session == "active"
            if drive_session == "active":
                drive_session = (
                    "nav_active" if drive_session_owner == "nav"
                    else "manual_active")
        release_consistent = (
            status_schema < 4 or motion_release_id == RELEASE_ID)
        parked = (
            drive_session == "parked"
            and physical_mode in {"joint_lock", "wheel_balance", "wheel_locomotion"}
            and actual_motion == "stopped"
            and wheels_stopped)
        drive_ready = (
            drive_session in {"manual_active", "nav_active"}
            and drive_session_owner in {"manual", "nav"}
            and physical_mode in {"wheel_balance", "wheel_locomotion"}
            and velocity_authorized
            and drivable_state)
        battery_ok = (
            battery_soc is not None and math.isfinite(float(battery_soc))
            and float(battery_soc) >= minimum_soc)
        drive_ok = drive_fault is None
        drive_fault_reset_available = (
            state in {"STOPPED", "EMERGENCY"}
            and drive_fault == "wheel_no_response"
            and sdk_ready and sport_mode == 6
            and wheel_dq is not None and len(wheel_dq) == 4
            and all(math.isfinite(float(value)) for value in wheel_dq)
            and sum(abs(float(value)) for value in wheel_dq) / 4.0 < 0.15
            and battery_ok)
        base_ready = (
            fresh and sdk_ready and scan_fresh and battery_ok
            and release_consistent and nav_guard_reason is None
            and drive_ok and localization_healthy)
        activatable = base_ready and (parked or drive_ready)
        ready = (
            base_ready and drive_ready
            and drive_session == "nav_active"
            and drive_session_owner == "nav")
        if not fresh:
            reason = "dog_state_stale"
        elif not sdk_ready:
            reason = "sdk_not_ready"
        elif not release_consistent:
            reason = "release_mismatch"
        elif not scan_fresh:
            reason = "nav_scan_stale"
        elif nav_guard_reason is not None:
            reason = str(nav_guard_reason)
        elif not drive_ok:
            reason = str(drive_fault)
        elif not battery_ok:
            reason = "battery_low"
        elif not localization_healthy:
            reason = f"localization_{localization_reason}"
        elif ready:
            reason = "ok"
        elif parked:
            reason = "drive_session_parked"
        elif drive_session == "activating":
            reason = "drive_session_activating"
        elif drive_session in {"stopping", "parking"}:
            reason = "drive_session_parking"
        elif not drivable_state and drive_session in {"manual_active", "nav_active"}:
            reason = "motion_state_not_drivable"
        else:
            reason = "drive_session_not_activatable"
        return {
            "ready": ready,
            "activatable": activatable,
            "drive_ready": drive_ready,
            "reason": reason,
            "state": state,
            "sdk_ready": sdk_ready,
            "nav_scan_fresh": scan_fresh,
            "nav_guard_reason": nav_guard_reason,
            "battery_soc": battery_soc,
            "minimum_battery_soc": minimum_soc,
            "drive_fault": drive_fault,
            "drive_fault_reset_available": drive_fault_reset_available,
            "sport_mode": sport_mode,
            "wheel_dq": list(wheel_dq) if wheel_dq is not None else None,
            "wheel_activation_phase": wheel_activation_phase,
            "drive_session": drive_session,
            "drive_session_owner": drive_session_owner,
            "drive_session_phase": drive_session_phase,
            "drive_session_reason": drive_session_reason,
            "physical_mode": physical_mode,
            "actual_motion": actual_motion,
            "velocity_authorized": velocity_authorized,
            "release_consistent": release_consistent,
            "motion_release_id": motion_release_id,
            "dog_state_age_sec": age_sec,
            "localization_healthy": localization_healthy,
            "localization_reason": localization_reason,
            "localization_age_sec": localization_age_sec,
        }

    def get_scan_snapshot(self):
        with self._lock:
            timestamp = float(getattr(self, "_scan_timestamp", 0.0) or 0.0)
            received = getattr(self, "_scan_received_monotonic", None)
            age_sec = (time.monotonic() - received
                       if received is not None else None)
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

    def get_pointcloud_snapshot(self):
        with self._lock:
            message = self._pointcloud_msg
            timestamp = float(self._pointcloud_timestamp or 0.0)
            count = int(self._pointcloud_count)
        if message is None or timestamp <= 0.0:
            return {
                "frame_id": "",
                "points": (),
                "count": count,
                "timestamp": timestamp,
                "age_sec": None,
            }
        points = decode_pointcloud_xyz(message, max_points=50000)
        return {
            "frame_id": str(getattr(message.header, "frame_id", "")),
            "points": points,
            "count": count,
            "timestamp": timestamp,
            "age_sec": time.time() - timestamp,
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
    def navigation_ready(self):
        """True only when motion SDK, scan watchdog, and state are all ready."""
        return bool(self._node.get_navigation_readiness().get("ready"))

    def start_drive_session(self, owner):
        owner = str(owner).strip().lower()
        if owner not in ("manual", "nav"):
            return {"ok": False, "reason": "invalid_drive_session_owner"}
        self._node.publish_motion_session(
            f"start_{owner}", source="navigation_arbiter")
        return {"ok": True, "phase": "activating", "owner": owner}

    def wait_drive_ready(self, owner, timeout=5.0):
        owner = str(owner).strip().lower()
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            readiness = self._node.get_navigation_readiness()
            if (readiness.get("drive_ready") is True
                    and readiness.get("drive_session_owner") == owner):
                return {"ok": True, "phase": "active", "owner": owner}
            if readiness.get("drive_fault"):
                return {"ok": False, "reason": readiness["drive_fault"]}
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return {
                    "ok": False,
                    "reason": "drive_session_activation_timeout",
                    "navigation": readiness,
                }
            time.sleep(min(0.05, remaining))

    def park_drive_session(self, reason="park"):
        reason = str(reason or "park")
        self.stop_move()
        self._node.publish_motion_session("park", source="navigation_arbiter")
        return {"ok": True, "phase": "parking", "reason": reason}

    def wait_drive_parked(self, timeout=5.0):
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            readiness = self._node.get_navigation_readiness()
            if readiness.get("drive_session") == "parked":
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.05, remaining))

    def get_drive_session_state(self):
        """Return cached physical feedback used for zero-speed handoff."""
        return dict(self._node.get_navigation_readiness())

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
                "odom_count": self._node._localization_count,
                "localization_count": self._node._localization_count,
                "robot_mode": 0,
                "robot_velocity": [self._node._dog_vx, self._node._dog_vy, self._node._dog_vyaw],
            }

    # ---- 动作: 转发到 NxWebNode 的 rclpy publisher ----
    def directional_clearance(self, center_deg, half_fov_deg=30.0,
                              max_age_sec=0.5):
        """Closest fresh valid scan return around a body-frame direction."""
        try:
            max_age_sec = float(max_age_sec)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(max_age_sec) or max_age_sec <= 0.0:
            return None
        try:
            snapshot = self._node.get_scan_snapshot()
            age_sec = snapshot.get("age_sec")
            age_sec = float(age_sec) if age_sec is not None else None
        except Exception:
            return None
        if (age_sec is None or not math.isfinite(age_sec)
                or age_sec < 0.0 or age_sec > max_age_sec):
            return None
        return directional_clearance_from_scan(
            snapshot.get("angle_min"), snapshot.get("angle_increment"),
            snapshot.get("range_min"), snapshot.get("range_max"),
            snapshot.get("ranges"), center_deg=center_deg,
            half_fov_deg=half_fov_deg,
        )

    def front_clearance(self, half_fov_deg=30.0):
        """Legacy permissive front wrapper used by existing forward callers."""
        clearance = self.directional_clearance(0.0, half_fov_deg)
        return 999.0 if clearance is None else clearance

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
            elif vx < 0.0:
                try:
                    rear = self.directional_clearance(180.0, 30.0)
                except Exception:
                    rear = None
                if rear is None or rear <= _REVERSE_CLEARANCE_M:
                    detail = "missing/stale" if rear is None else f"{rear:.2f}m"
                    logger.info(f"[move] autonomous reverse rear clearance {detail}; stopping")
                    vx = 0.0
        with self._lock:
            self._vx = vx
            self._vy = vy
            self._vyaw = vyaw
        if manual:
            self._node.publish_cmd_vel(vx, vy, vyaw)
        else:
            self._node.publish_nav_cmd_vel(vx, vy, vyaw)

    def stop_move(self):
        with self._lock:
            self._vx = self._vy = self._vyaw = 0.0
        self._node.publish_cmd_vel(0.0, 0.0, 0.0)
        self._node.publish_nav_cmd_vel(0.0, 0.0, 0.0)

    def stand(self):
        self._node.publish_cmd_pose('stand')

    def confirm_stand(self):
        """Record visual StandUp confirmation while velocity stays locked."""
        self._node.publish_cmd_pose('confirm_stand')

    def balance(self):
        """Explicitly request the feedback-gated wheel-balance transition."""
        self._node.publish_cmd_pose('balance')

    def adopt_stand(self):
        """Adopt an already-upright pose after a safe motion-service restart."""
        self._node.publish_cmd_pose('adopt_stand')

    def confirm_balance(self):
        """Confirm a wheel mode that already existed before service startup."""
        self._node.publish_cmd_pose('confirm_balance')

    def sit(self):
        self._node.publish_cmd_pose('sit')

    def e_stop(self):
        self._node.publish_cmd_pose('estop')

    def reset_drive_fault(self):
        """Request the motion node's guarded, zero-motion fault reset."""
        self._node.publish_cmd_pose('reset_drive_fault')

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
        self._navigation_arbiter = None
        # move_relative (spec §1.3): linear→point_nav, angular→cmd_vel+odom
        self._point_nav = None

    def set_navigation_arbiter(self, arbiter):
        self._navigation_arbiter = arbiter

    def set_point_nav(self, port):
        """Inject the PointNavigationController owner port for linear moves."""
        self._point_nav = port

    def _notify_navigation_drained(self):
        if self._navigation_arbiter is None:
            return
        with self._lock:
            drained = self._active is None and not any(
                task.status == "pending" for task in self._tasks)
        if drained:
            self._navigation_arbiter.on_tasks_drained()

    def add(self, task, reason=None):
        if self._navigation_arbiter is not None:
            return self._navigation_arbiter.start_tasks(
                [task], reason=reason or task.type)
        return self._add_list_unchecked([task])

    def add_list(self, tasks, reason=None):
        task_list = list(tasks)
        if self._navigation_arbiter is not None:
            inferred = reason or (task_list[0].type if task_list else "task_command")
            return self._navigation_arbiter.start_tasks(task_list, reason=inferred)
        return self._add_list_unchecked(task_list)

    def _add_list_unchecked(self, tasks):
        with self._lock:
            for i, t in enumerate(tasks):
                if t.priority == 5:
                    t.priority = max(1, 8 - i)
                self._tasks.append(t)
        ws_broadcast({"type": "tasks", "data": self.get_state()})
        return {"ok": True, "count": len(tasks)}

    def cancel_all(self):
        with self._lock:
            if self._active:
                self._active.status = "cancelled"
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

    def wait_drained(self, timeout):
        """Bounded wait for worker exit and room action terminal ownership."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            with self._lock:
                local_drained = self._active is None and not any(
                    task.status == "pending" for task in self._tasks)
            room_drained = True
            room = self.room_orchestrator
            if room is not None:
                waiter = getattr(room, "wait_drained", None)
                if callable(waiter):
                    try:
                        room_drained = bool(waiter(0.0))
                    except Exception:
                        room_drained = False
            if local_drained and room_drained:
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.02, remaining))

    def get_state(self):
        with self._lock:
            return {"active": self._active.to_dict() if self._active else None,
                    "pending": [t.to_dict() for t in self._tasks if t.status == "pending"],
                    "completed_count": sum(1 for t in self._tasks if t.status == "completed")}

    def process_command(self, text):
        threading.Thread(target=self._process_command_bg, args=(text,), daemon=True).start()

    def submit_command(self, text):
        """Synchronously admit deterministic product commands.

        The voice client must not report success before the navigation arbiter
        has accepted a room-search task.  Commands that require the VLM keep
        the existing asynchronous path because model inference can block.
        """
        result = self._parse_product_command(text)
        if result is None:
            self.process_command(text)
            return {
                "ok": True,
                "accepted": None,
                "queued": True,
                "parser": "async",
                "text": text,
            }
        return self._admit_command_result(text, result, parser="product")

    def _process_command_bg(self, text):
        try:
            result = self._parse_product_command(text)
            if result is None:
                result = self._vlm_parse_command(text) if (self.vlm and getattr(self.vlm, 'loaded', False)) \
                    else {
                        "response": "无法解析为受支持的搜索任务",
                        "tasks": [],
                        "parse_error": "deterministic_parser_no_match",
                    }
            self._admit_command_result(text, result, parser="async")
        except Exception as e:
            logger.error(f"指令处理失败: {e}")
            traceback.print_exc()

    def _admit_command_result(self, text, result, *, parser):
        response = result.get("response", "")
        tasks = result.get("tasks", [])
        invalid_reason = None
        if tasks:
            try:
                first_type = (tasks[0].get("type")
                              if isinstance(tasks[0], dict) else None)
                if first_type == "move_relative":
                    tasks = canonicalize_move_tasks(tasks)
                else:
                    tasks = canonicalize_search_tasks(tasks)
            except MissionValidationError as exc:
                logger.warning("拒绝非规范任务: %s", exc)
                response = "任务格式无效"
                tasks = []
                invalid_reason = "invalid_task"
        logger.info(f"指令解析: '{text}' → response='{response}' tasks={len(tasks)}")
        ws_broadcast({
            "type": "vlm",
            "data": {"text": text, "response": response, "tasks": tasks},
        })
        payload = {
            "ok": False,
            "accepted": False,
            "parser": parser,
            "text": text,
            "response": response,
            "tasks": tasks,
        }
        if not tasks:
            payload["reason"] = invalid_reason or "no_tasks"
            return payload

        admission = self.add_list(
            [Task(t["type"], t["params"], t["priority"]) for t in tasks],
            reason="task_command",
        )
        if not isinstance(admission, dict):
            admission = {"ok": bool(admission)}
        accepted = bool(admission.get("ok"))
        payload.update({
            "ok": accepted,
            "accepted": accepted,
            "admission": admission,
        })
        if not accepted:
            payload["reason"] = admission.get("reason", "admission_failed")
            logger.warning("指令任务因导航仲裁失败未启动: %s", admission)
            ws_broadcast({"type": "tasks", "data": {
                "admission_failed": admission,
            }})
        return payload

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
            and t["params"].get("search_strategy") != "frontier_explore"
            for t in tasks
        )
        if not needs_current_room:
            return
        pose = self._latest_robot_map_pose()
        rooms = self._room_details_for_resolution()
        if pose is None or not rooms:
            # 无房间图 (RoomMap 未加载 / 无 pose): next_best_view 必失败 (_ensure_room_map 推 FAILED),
            # 自动降级到 frontier_explore (无预建图探索, 已有测试覆盖) 让狗仍能搜人。
            # frontier_explore 在 run() 入口分流, 不进 SELECT_ROOM, 忽略 room=__current__。
            for task in tasks:
                params = task.get("params") if isinstance(task, dict) else None
                if isinstance(params, dict) and params.get("room") == "__current__":
                    params["search_strategy"] = "frontier_explore"
                    logger.info("无房间图: __current__ 房间搜索自动降级 frontier_explore")
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
            health_getter = getattr(node_obj, "get_localization_health", None)
            if callable(health_getter):
                health = health_getter()
                if not health.get("healthy"):
                    return None
                return float(health["x"]), float(health["y"])

            # Compatibility for injected non-ROS adapters. Production
            # NxWebNode always supplies get_localization_health() above.
            lock = getattr(node_obj, "_lock", threading.Lock())
            with lock:
                count = int(getattr(node_obj, "_odom_count", 0) or 0)
                received = float(getattr(node_obj, "_odom_t", 0.0) or 0.0)
                x = float(getattr(node_obj, "_odom_x", 0.0))
                y = float(getattr(node_obj, "_odom_y", 0.0))
            if count <= 0 or not all(math.isfinite(v) for v in (x, y, received)):
                return None
            try:
                max_age = float(os.environ.get("GO2W_ODOM_MAX_AGE_SEC", "2.0"))
            except (TypeError, ValueError):
                max_age = 2.0
            if not math.isfinite(max_age) or max_age <= 0.0:
                max_age = 2.0
            age = time.time() - received
            if received <= 0.0 or not math.isfinite(age) or age < 0.0 or age > max_age:
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
        # sys_prompt + chat + JSON 解析；失败时返回空任务。
        # 阶段A 时 self.vlm=None, 走不到这里 (上层 _process_command_bg 先判 vlm.loaded)。
        sys_prompt = """你是机器狗搜索任务解析器。只把用户的搜索指令转换成一个 JSON 任务。

唯一允许的任务类型和参数:
- search_room: {"room":"房间名或__current__(当前房间)", "target_classes":["person"], "require_photos":true, "mark_on_map":true}

target_classes 是需要搜索和地图标注的英文视觉类别数组，例如 person、table、chair；不要替用户改写类别。

示例:
输入"搜索这个房间标注所有人"
输出: {"tasks":[{"type":"search_room","priority":8,"params":{"room":"__current__","target_classes":["person"],"require_photos":true,"mark_on_map":true}}]}

输入"去客厅找桌子并标在地图上"
输出: {"tasks":[{"type":"search_room","priority":8,"params":{"room":"客厅","target_classes":["table"],"require_photos":true,"mark_on_map":true}}]}

无法解析为搜索任务时输出 {"tasks":[]}。只输出 JSON，不要解释，不要 markdown 代码块。"""
        try:
            response = self.vlm.chat([
                {"role": "system", "content": sys_prompt},
                {"role": "user", "content": text}
            ], max_new_tokens=512)
        except Exception as e:
            logger.warning(f"VLM chat 异常: {e}")
            return {
                "response": "搜索任务解析失败",
                "tasks": [],
                "parse_error": "vlm_unavailable",
            }
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
                    return {
                        "response": str(data.get("response") or "已解析搜索任务"),
                        "tasks": canonicalize_search_tasks(data.get("tasks")),
                    }
        except Exception as e:
            logger.warning(f"VLM JSON 解析失败: {e}")
        return {
            "response": "搜索任务解析失败",
            "tasks": [],
            "parse_error": "invalid_vlm_mission",
        }

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
            if task.status == "cancelled":
                with self._lock:
                    self._tasks = [t for t in self._tasks if t.id != task.id]
                    if self._active is task:
                        self._active = None
                ws_broadcast({"type": "tasks", "data": self.get_state()})
                self._notify_navigation_drained()
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
                    if task.status != "cancelled":
                        task.status = "completed"
                elif task.type == "stop":
                    self.robot.stop_move()
                    self.cancel_all()
                    task.status = "completed"
                elif task.type == "follow":
                    self._execute_follow(task)
                elif task.type == "search_area":
                    self._execute_search(task)
                elif task.type == "move_relative":
                    self._execute_move_relative(task)
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
                if self._active is task:
                    self._active = None
            ws_broadcast({"type": "tasks", "data": self.get_state()})
            self._notify_navigation_drained()

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

    def _execute_move_relative(self, task):
        """move_relative 执行 (spec §1.3): linear→nav2, angular→cmd_vel+odom 闭环."""
        p = task.params
        mode = p.get("mode")
        direction = p.get("direction")

        def broadcast(phase, **extra):
            ws_broadcast({"type": "move_result",
                          "data": {"phase": phase, "direction": direction, **extra}})

        node_obj = getattr(self.robot, "_node", None)
        health_getter = getattr(node_obj, "get_localization_health", None)
        health = health_getter() if callable(health_getter) else {}
        if not health.get("healthy"):
            task.status = "failed"
            task.result = "localization_unhealthy"
            broadcast("aborted", reason="localization_unhealthy")
            return

        if mode == "linear":
            if direction == "forward":
                if self._point_nav is None:
                    task.status = "failed"
                    task.result = "point_nav_unavailable"
                    broadcast("aborted", reason="point_nav_unavailable")
                    return
                x = float(health["x"]); y = float(health["y"]); yaw = float(health["yaw"])
                tx, ty, tyaw = compute_linear_target(
                    x, y, yaw, direction, float(p["distance_m"]))
                self._point_nav.submit(tx, ty, tyaw)
                phase = self._await_point_nav_terminal(task)
                if phase == "succeeded":
                    task.status = "completed"
                    task.result = {"distance_m": p["distance_m"], "direction": direction}
                else:
                    task.status = "failed"
                    task.result = phase
                broadcast(phase, distance_m=p["distance_m"])
            elif direction == "backward":
                start_xy = (float(health["x"]), float(health["y"]))

                def read_xy():
                    try:
                        fresh_health_getter = getattr(
                            node_obj, "get_localization_health", None)
                        fresh = (fresh_health_getter()
                                 if callable(fresh_health_getter) else {})
                        if not fresh.get("healthy"):
                            return None
                        return (float(fresh["x"]), float(fresh["y"]))
                    except (KeyError, TypeError, ValueError, OverflowError):
                        return None

                def read_rear_clearance(_direction):
                    return self.robot.directional_clearance(
                        180.0, 30.0, max_age_sec=0.5)

                def send_reverse_cmd(vx, vy, vyaw_v):
                    self.robot.move(vx, vy, vyaw_v, manual=False)

                try:
                    phase = run_linear_translation(
                        read_xy, read_rear_clearance, send_reverse_cmd,
                        time.sleep, time.monotonic, start_xy, direction,
                        float(p["distance_m"]),
                        start_yaw=float(health["yaw"]),
                        is_cancelled=lambda: task.status == "cancelled",
                        clearance_margin_m=_REVERSE_CLEARANCE_M,
                    )
                finally:
                    self.robot.stop_move()

                if phase == "succeeded":
                    task.status = "completed"
                    task.result = {"distance_m": p["distance_m"],
                                   "direction": direction}
                    broadcast("succeeded", distance_m=p["distance_m"])
                elif phase == "cancelled":
                    task.status = "cancelled"
                    task.result = "cancelled"
                    broadcast("cancelled", reason="cancelled",
                              distance_m=p["distance_m"])
                elif phase == "timed_out":
                    task.status = "failed"
                    task.result = "timed_out"
                    broadcast("timed_out", reason="timed_out",
                              distance_m=p["distance_m"])
                else:
                    task.status = "failed"
                    task.result = phase
                    broadcast("aborted", reason=phase,
                              distance_m=p["distance_m"])
        else:  # angular
            yaw0 = float(health["yaw"])
            target_yaw = compute_angular_target_yaw(
                yaw0, direction, float(p["angle_deg"]))

            def read_yaw():
                h = health_getter() if callable(health_getter) else {}
                return float(h.get("yaw", yaw0))

            def send_cmd(vx, vy, vyaw_v):
                self.robot.move(vx, vy, vyaw_v, manual=True)

            phase = run_angular_turn(
                read_yaw, send_cmd, time.sleep, time.monotonic,
                target_yaw, direction, vyaw=0.5,
                tolerance_rad=math.radians(3.0),
            )
            self.robot.stop_move()
            if phase == "succeeded":
                task.status = "completed"
                task.result = {"angle_deg": p["angle_deg"], "direction": direction}
            else:
                task.status = "failed"
                task.result = phase
            broadcast(phase, angle_deg=p["angle_deg"])

    def _await_point_nav_terminal(self, task, timeout=60.0):
        """轮询 point_nav 终态; cancel-aware. 返回 phase str."""
        if self._point_nav is None:
            return "aborted"
        terminal = {"succeeded", "aborted", "timed_out", "canceled",
                    "error", "rejected"}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if task.status == "cancelled":
                try:
                    self._point_nav.cancel("task_cancelled")
                except Exception:
                    pass
                return "cancelled"
            status = self._point_nav.get_state().get("status")
            if status in terminal:
                return status
            time.sleep(0.1)
        try:
            self._point_nav.cancel("timeout")
        except Exception:
            pass
        return "timed_out"

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
                if task.status == "cancelled":
                    self.robot.stop_move()
                    return
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
point_nav = None        # OwnerNavigationPort
navigation_gateway = None  # sole NavigateToPose owner
room_orchestrator = None  # 阶段E: RoomSearchOrchestrator (main 注入)
navigation_arbiter = None  # NavigationArbiter: process-wide autonomous owner


def _perception_health(robot_bridge):
    ai = getattr(robot_bridge, "_ai_engine", None) if robot_bridge else None
    get_health = getattr(ai, "get_person_detection_health", None)
    if not callable(get_health):
        health = {
            "healthy": False,
            "reason": "ai_engine_unavailable",
            "running": False,
            "detector_initialized": False,
            "detector_ready": False,
            "detector_open_vocabulary": False,
            "detector_model": "",
            "frame_available": False,
            "source": "",
            "timestamp": 0.0,
            "age_sec": None,
            "frame_width": 0,
            "frame_height": 0,
            "detection_count": 0,
            "person_count": 0,
        }
        health["map_annotation"] = resolve_camera_calibration("")
        return health
    try:
        health = dict(get_health())
        health["map_annotation"] = resolve_camera_calibration(
            health.get("source"))
        return health
    except Exception as exc:
        logger.warning(f"perception health snapshot failed: {exc}")
        health = {
            "healthy": False,
            "reason": "perception_health_error",
            "running": True,
            "detector_initialized": False,
            "detector_ready": False,
            "detector_open_vocabulary": False,
            "detector_model": "",
            "frame_available": False,
            "source": "",
            "timestamp": 0.0,
            "age_sec": None,
            "frame_width": 0,
            "frame_height": 0,
            "detection_count": 0,
            "person_count": 0,
        }
        health["map_annotation"] = resolve_camera_calibration("")
        return health


def _handle_point_nav_state(state):
    ws_broadcast({"type": "nav_goal", "data": state}, force=True)
    if navigation_arbiter is not None:
        navigation_arbiter.on_point_state(state)


def _point_navigation_health(nx_node):
    """Cancel/block PointNav if either localization or motion safety drops."""
    return bool(_point_navigation_health_sample(nx_node).get("healthy"))


def _point_navigation_health_sample(nx_node):
    """Classify hard motion failures separately from transient localization loss."""
    try:
        localization = nx_node.get_localization_health()
        motion = nx_node.get_navigation_readiness()
    except Exception:
        return {
            "healthy": False,
            "immediate": True,
            "reason": "health_check_error",
            "motion_reason": None,
            "localization_reason": None,
        }

    localization_ok = bool(localization.get("healthy"))
    motion_ok = bool(motion.get("ready"))
    common = {
        "motion_reason": motion.get("reason"),
        "localization_reason": localization.get("reason"),
    }
    if not motion_ok:
        # A parked drive session is a recoverable ownership transition, not a
        # hard safety fault.  Give the mission gateway its health-grace window
        # so it can reactivate the session without canceling the active goal.
        recoverable_parked = (
            motion.get("reason") == "drive_session_parked"
            and bool(motion.get("activatable"))
        )
        return {
            "healthy": False,
            "immediate": not recoverable_parked,
            "reason": "motion_unhealthy",
            **common,
        }
    if not localization_ok:
        return {
            "healthy": False,
            "immediate": False,
            "reason": "localization_unhealthy",
            **common,
        }
    return {
        "healthy": True,
        "immediate": False,
        "reason": None,
        **common,
    }


def create_server(host, port, static_dir, mission_root=None):
    mission_root = os.path.realpath(
        mission_root
        or os.environ.get("GO2W_MISSION_ROOT")
        or os.path.join(static_dir, "missions")
    )
    control_token = os.environ.get("GO2W_CONTROL_TOKEN", "")
    allowed_origins = parse_allowed_origins(os.environ.get(
        "GO2W_PANEL_ORIGINS",
        "http://127.0.0.1:8000,http://localhost:8000,http://192.168.43.41:8000",
    ))

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            p = urlparse(self.path)
            q = parse_qs(p.query)
            if p.path in ('/', '/index.html'):
                self._serve(os.path.join(static_dir, 'panel.html'), 'text/html')
            elif p.path == '/map.js':
                self._serve(os.path.join(static_dir, 'map.js'), 'application/javascript')
            elif p.path.startswith('/missions/'):
                self._serve_mission_artifact(p.path)
            elif p.path == '/api/detection_snapshot':
                snapshot_id = q.get('id', [''])[0]
                kind = q.get('kind', ['crop'])[0]
                ai = getattr(robot, "_ai_engine", None) if robot is not None else None
                data = ai.get_detection_snapshot_jpeg(snapshot_id, kind) if ai is not None else None
                if not data:
                    self.send_error(404)
                    return
                self._send_jpeg(data)
            elif p.path == '/api/video_frame':
                source = q.get('source', [''])[0]
                ai = getattr(robot, "_ai_engine", None) if robot is not None else None
                data = ai.get_video_frame_jpeg(source) if ai is not None and hasattr(ai, "get_video_frame_jpeg") else None
                if not data:
                    self.send_error(404)
                    return
                self._send_jpeg(data)
            elif p.path == '/api/foxglove':
                # Foxglove bridge URL (需另起 foxglove_bridge 节点, 默认 8765)
                host_ip = os.environ.get("GO2W_PUBLIC_IP", "localhost")
                self._json({"url": f"http://{host_ip}:8080", "ws": f"ws://{host_ip}:8765"})
            elif p.path == '/api/status':
                snap = node.get_status_snapshot() if node else {}
                self._json({
                    "release_id": RELEASE_ID,
                    "motion_release_id": snap.get("motion_release_id"),
                    "release_consistent": snap.get("release_consistent", False),
                    "connected": robot.connected if robot else False,
                    "imu_yaw": robot.imu_yaw if robot else 0,
                    "dog_state": snap.get("dog_state", "UNKNOWN"),
                    "stats": robot.stats if robot else {},
                    "tasks": task_mgr.get_state() if task_mgr else {},
                    "localization": snap.get("localization", {}),
                    "odometry": snap.get("odometry", {}),
                    "navigation": snap.get("navigation", {}),
                    "services": snap.get("services", {}),
                    "point_nav": point_nav.get_state() if point_nav else {},
                    "room_nav": (
                        room_orchestrator.get_navigation_state()
                        if room_orchestrator else {}
                    ),
                    "perception": _perception_health(robot),
                })
            elif p.path == '/api/version':
                snap = node.get_status_snapshot() if node else {}
                self._json({
                    "release_id": RELEASE_ID,
                    "motion_release_id": snap.get("motion_release_id"),
                    "release_consistent": snap.get("release_consistent", False),
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

        def do_OPTIONS(self):
            origin = self.headers.get("Origin")
            if not cors_origin_allowed(origin, allowed_origins):
                self.send_error(403)
                return
            self.send_response(204)
            self._send_cors_headers()
            self.send_header(
                "Access-Control-Allow-Headers", "Authorization, Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.end_headers()

        def do_POST(self):
            p = urlparse(self.path)
            q = parse_qs(p.query)
            decision = authorize_request(
                method="POST",
                path=p.path,
                headers=self.headers,
                configured_token=control_token,
            )
            if not decision.allowed:
                self._json({"ok": False, "reason": decision.reason},
                           status=decision.status_code)
                return
            L = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(L).decode() if L else ''
            audited_paths = {
                "/api/connect", "/api/stand", "/api/confirm_stand",
                "/api/balance", "/api/confirm_balance", "/api/sit",
                "/api/activate",
                "/api/e_stop", "/api/reset_drive_fault", "/api/stop",
                "/api/manual_stop", "/api/navigate", "/api/search", "/api/clear_all",
            }
            audit_request = p.path in audited_paths
            if p.path == "/api/move" and navigation_arbiter is not None:
                try:
                    audit_request = (
                        navigation_arbiter.get_motion_owner() != "manual"
                    )
                except Exception:
                    audit_request = True
            if audit_request:
                logger.warning(
                    "control request path=%s client=%s user_agent=%r referer=%r",
                    p.path,
                    self.client_address[0] if self.client_address else "unknown",
                    self.headers.get("User-Agent"),
                    self.headers.get("Referer"),
                )
            if p.path == '/api/connect':
                result = navigation_arbiter.run_operator_action(
                    "connect_stand", robot.stand) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/stand':
                result = navigation_arbiter.run_operator_action(
                    "stand", robot.stand) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/confirm_stand':
                result = navigation_arbiter.run_operator_action(
                    "confirm_stand", robot.confirm_stand) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/balance':
                result = navigation_arbiter.run_operator_action(
                    "balance", robot.balance) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/adopt_stand':
                result = navigation_arbiter.run_operator_action(
                    "adopt_stand", robot.adopt_stand) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/confirm_balance':
                result = navigation_arbiter.run_operator_action(
                    "confirm_balance", robot.confirm_balance) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/sit':
                result = navigation_arbiter.run_operator_action(
                    "sit", robot.sit) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/activate':
                # 一键激活: parked→active 可导航态. start_drive_session 发 start_nav
                # intent (与点导航时 arbiter 自动激活同路径, 已验证安全), wait_drive_ready
                # 阻塞等 drive_ready (≤8s). 进入轮式平衡轮子可能小幅移动, 注意场地.
                if robot is None:
                    self._json({"ok": False, "reason": "robot_unavailable"}, status=409)
                else:
                    _r = robot.start_drive_session("nav")
                    if _r.get("ok"):
                        _r.update(robot.wait_drive_ready("nav", timeout=8.0))
                    self._json(_r, status=200 if _r.get("ok") else 409)
            elif p.path == '/api/manual_stop':
                result = navigation_arbiter.release_manual(
                    "manual_release") if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/stop':
                result = navigation_arbiter.stop_all(
                    "operator_stop") if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=(
                    200 if result.get("ok") else
                    503 if result.get("reason") == "arbiter_unavailable" else 409))
            elif p.path == '/api/clear_all':
                # 清前端轨迹源头: broadcast_loop 每帧推 _trail 给前端 (slam.trail),
                # 不清后端则前端 clearTrail() 清了下一帧又被灌回 = "清不掉"。
                import subprocess  # 局部 import: 顶部未导入 subprocess, /api/locate 走别的机制
                _trail.clear()
                # 清 nav2 costmap: subprocess 调 nav2 clear service, 隔离 web rclpy 节点
                # wait_set (costmap_bridge.py 注释: web 订阅数已临界, 加 service client 有
                # "IndexError: wait set index too big" 溢出风险)。串行 ~3s 可接受 (清操作低频)。
                # 类型 nav2_msgs/srv/ClearEntireCostmap (ros2 service type 实测确认)。
                # 语义: 清当前 obstacle_layer 标记 (含云台历史 ghost/动态滞留), static_layer
                # (/map_frontier) 下次 update 会重载; 结合云台盲区修复后不再产生新假点。
                cleared = {}
                for _srv in ('/global_costmap/clear_entirely_global_costmap',
                             '/local_costmap/clear_entirely_local_costmap'):
                    _key = _srv.rsplit('/', 1)[-1]
                    try:
                        _r = subprocess.run(
                            ['ros2', 'service', 'call', _srv,
                             'nav2_msgs/srv/ClearEntireCostmap'],
                            capture_output=True, text=True, timeout=6)
                        cleared[_key] = (_r.returncode == 0)
                    except subprocess.TimeoutExpired:
                        cleared[_key] = 'timeout'
                    except Exception as _e:
                        cleared[_key] = f'err:{type(_e).__name__}'
                self._json({"ok": True, "trail_cleared": True, "costmap": cleared})
            elif p.path == '/api/e_stop':
                result = navigation_arbiter.emergency_stop() if navigation_arbiter else {
                    "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=(
                    200 if result.get("ok") else 503))
            elif p.path == '/api/reset_drive_fault':
                result = navigation_arbiter.run_operator_action(
                    "reset_drive_fault", robot.reset_drive_fault
                ) if navigation_arbiter else {
                    "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/move':
                # query string: vx,vy,vyaw (panel.html:242/248/317 约定)
                try:
                    vx = float(q.get('vx', ['0'])[0])
                    vy = float(q.get('vy', ['0'])[0])
                    vyaw = float(q.get('vyaw', ['0'])[0])
                except (TypeError, ValueError):
                    self._json({"ok": False, "msg": "vx/vy/vyaw 必须是数字"}, status=400)
                    return
                if not all(math.isfinite(v) for v in (vx, vy, vyaw)):
                    self._json({"ok": False, "msg": "vx/vy/vyaw 必须是有限数字"}, status=400)
                    return
                if abs(vx) > 0.4 or abs(vy) > 0.3 or abs(vyaw) > 0.5:
                    self._json({
                        "ok": False,
                        "msg": "手动速度超过安全上限 (vx=0.4, vy=0.3, vyaw=0.5)",
                    }, status=400)
                    return
                if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
                    result = navigation_arbiter.release_manual(
                        "manual_zero") if navigation_arbiter else {
                            "ok": False, "reason": "arbiter_unavailable"}
                else:
                    result = navigation_arbiter.run_manual_action(
                        "manual_move",
                        lambda: robot.move(vx, vy, vyaw, manual=True),
                    ) if navigation_arbiter else {
                        "ok": False, "reason": "arbiter_unavailable"}
                self._json(result, status=200 if result.get("ok") else 409)
            elif p.path == '/api/command':
                text = q.get('text', [''])[0] or body
                if body:
                    try:
                        payload = json.loads(body)
                        text = payload.get('text', '') if isinstance(payload, dict) else ''
                    except Exception:
                        text = body
                text = text.strip() if isinstance(text, str) else ''
                if not text:
                    self._json({
                        "ok": False,
                        "accepted": False,
                        "reason": "empty_command",
                    }, status=400)
                    return
                if task_mgr is None:
                    self._json({
                        "ok": False,
                        "accepted": False,
                        "reason": "task_manager_unavailable",
                    }, status=503)
                    return
                result = task_mgr.submit_command(text)
                self._json(result, status=(202 if result.get("ok") else 409))
            elif p.path == '/api/navigate':
                try:
                    payload = json.loads(body)
                except (TypeError, ValueError, json.JSONDecodeError):
                    self._json({"ok": False, "msg": "请求体必须是 JSON 对象"}, status=400)
                    return
                if not isinstance(payload, dict):
                    self._json({"ok": False, "msg": "请求体必须是 JSON 对象"}, status=400)
                    return
                if payload.get("frame_id", "map") != "map":
                    self._json({"ok": False, "msg": "frame_id 必须是 map"}, status=400)
                    return
                if "x" not in payload or "y" not in payload:
                    self._json({"ok": False, "msg": "缺少 x/y"}, status=400)
                    return
                try:
                    values = (payload["x"], payload["y"], payload.get("yaw", 0.0))
                    if any(isinstance(value, bool) for value in values):
                        raise ValueError("boolean is not a coordinate")
                    x, y, yaw = (float(value) for value in values)
                except (TypeError, ValueError):
                    self._json({"ok": False, "msg": "x/y/yaw 必须是数字"}, status=400)
                    return
                if not all(math.isfinite(value) for value in (x, y, yaw)):
                    self._json({"ok": False, "msg": "x/y/yaw 必须是有限数字"}, status=400)
                    return
                if point_nav is None or node is None or navigation_arbiter is None:
                    self._json({"ok": False, "msg": "点导航仲裁器未就绪"}, status=503)
                    return
                localization = node.get_localization_health()
                if not localization.get("healthy"):
                    self._json({
                        "ok": False,
                        "msg": "地图定位不可用",
                        "localization": localization,
                    }, status=409)
                    return
                if not _point_goal_within_local_radius(x, y, localization):
                    self._json({
                        "ok": False,
                        "msg": f"目标必须在当前位置 {POINT_GOAL_MAX_DISTANCE:.0f}m 内",
                    }, status=400)
                    return
                navigation = node.get_navigation_readiness()
                if robot is None or not navigation.get("activatable"):
                    self._json({
                        "ok": False,
                        "msg": "机器人运动链未就绪",
                        "navigation": navigation,
                    }, status=503)
                    return
                result = navigation_arbiter.start_point_goal(x, y, yaw)
                if result.get("ok"):
                    self._json(result, status=202)
                else:
                    self._json(result, status=(
                        400 if result.get("reason") == "invalid_goal" else 409))
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
                admission = task_mgr.add_list([Task("search_area",
                    {"pattern": pattern, "width": w, "height": h,
                     "spacing": sp, "origin_x": ox, "origin_y": oy}, 5)],
                    reason="search_area")
                payload = dict(admission or {})
                if payload.get("ok"):
                    payload["msg"] = f"搜索 {w}x{h}m 间距{sp}m 已入队"
                self._json(payload, status=200 if payload.get("ok") else 409)
            elif p.path == '/api/search_room':
                body_payload = {}
                if body:
                    try:
                        parsed = json.loads(body)
                        if isinstance(parsed, dict):
                            body_payload = parsed
                    except (TypeError, ValueError, json.JSONDecodeError):
                        self._json({"ok": False, "reason": "invalid_json"}, status=400)
                        return
                if not body_payload:
                    body_payload = {
                        "room": q.get('room', [''])[0],
                        "target_classes": [
                            value.strip() for value in
                            q.get('target_classes', [''])[0].split(',')
                            if value.strip()
                        ],
                    }
                    strategy = q.get('search_strategy', [''])[0]
                    if strategy:
                        body_payload["search_strategy"] = strategy
                try:
                    mission = SearchMissionRequest.from_api_payload(body_payload)
                except MissionValidationError as exc:
                    self._json({
                        "ok": False,
                        "reason": "invalid_mission_request",
                        "message": str(exc),
                    }, status=400)
                    return
                task_params = mission.to_task_params()
                for key in ("max_frontiers", "max_frontier_plan_probes"):
                    if key in body_payload:
                        task_params[key] = body_payload[key]
                if task_mgr is None:
                    self._json({
                        "ok": False, "reason": "task_manager_unavailable",
                    }, status=503)
                    return
                admission = task_mgr.add_list(
                    [Task("search_room", task_params, 5)], reason="search_room")
                payload = dict(admission or {})
                payload["mission_request"] = mission.to_dict()
                if payload.get("ok"):
                    payload["msg"] = f"搜索任务 {mission.request_id} 已入队"
                self._json(payload, status=200 if payload.get("ok") else 409)
            else:
                self.send_error(404)

        def _send_jpeg(self, data):
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(data))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self._send_cors_headers()
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                logger.debug("client disconnected while sending jpeg")

        def _json(self, d, status=200):
            # CORS: 跨端口(8000 HTTP ↔ 8001 WS) 与跨机访问都需要 (panel.py:714)
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self._send_cors_headers()
            self.end_headers()
            self.wfile.write(json.dumps(d, ensure_ascii=False).encode())

        def _send_cors_headers(self):
            origin = self.headers.get("Origin")
            if origin and cors_origin_allowed(origin, allowed_origins):
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")

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

        def _serve_mission_artifact(self, url_path):
            """Serve only persisted mission evidence below mission_root."""
            relative = unquote(str(url_path or ''))
            prefix = '/missions/'
            relative = relative[len(prefix):] if relative.startswith(prefix) else ''
            candidate = os.path.realpath(os.path.join(mission_root, relative))
            try:
                contained = os.path.commonpath(
                    [mission_root, candidate]) == mission_root
            except ValueError:
                contained = False
            if not contained or not os.path.isfile(candidate):
                self.send_error(404)
                return
            extension = os.path.splitext(candidate)[1].lower()
            content_type = {
                '.jpg': 'image/jpeg',
                '.jpeg': 'image/jpeg',
                '.json': 'application/json',
            }.get(extension)
            if content_type is None:
                self.send_error(404)
                return
            self._serve(candidate, content_type)

        def log_message(self, *a):
            pass

    return ThreadingHTTPServer((host, port), H)  # H5 fix: 每请求独立线程, /api/locate 长 subprocess 不再阻塞 /api/stop 急停


# ============================================================================
# WS server (照抄 panel.py:731-739, 端口固定 8001)
# ============================================================================
async def _start_ws_server(websockets, host, port):
    async def h(ws, path=None):
        # ``path`` is optional for compatibility with both the legacy
        # websockets two-argument handler and newer one-argument releases.
        del path
        await _register_ws(ws)
        try:
            await ws.wait_closed()
        finally:
            await _unregister_ws(ws)
    # Recent websockets releases require serve() to be constructed while an
    # event loop is running.  This coroutine is entered by run_until_complete.
    return await websockets.serve(h, host, port)


async def _shutdown_ws_server(server):
    server.close()
    clients = _ws_client_snapshot()
    if clients:
        await asyncio.gather(*[
            _disconnect_ws(ws, "server shutdown") for ws in clients
        ], return_exceptions=True)
    try:
        await asyncio.wait_for(
            server.wait_closed(), timeout=max(1.0, 2.0 * _WS_SEND_TIMEOUT))
    except asyncio.TimeoutError:
        logger.warning("WebSocket server close timed out; cancelling handlers")
    with _WS_REGISTRY_LOCK:
        senders = list(_WS_SENDER_TASKS.values())
    for sender in senders:
        sender.cancel()
    if senders:
        await asyncio.gather(*senders, return_exceptions=True)


def run_ws(host, port):
    global WS_LOOP
    import websockets
    loop = asyncio.new_event_loop()
    WS_LOOP = loop
    asyncio.set_event_loop(loop)
    server = None
    try:
        server = loop.run_until_complete(
            _start_ws_server(websockets, host, port))
        loop.run_forever()
    finally:
        try:
            if server is not None:
                loop.run_until_complete(_shutdown_ws_server(server))
        finally:
            remaining = list(asyncio.all_tasks(loop))
            for task in remaining:
                task.cancel()
            if remaining:
                loop.run_until_complete(asyncio.gather(
                    *remaining, return_exceptions=True))
            WS_LOOP = None
            asyncio.set_event_loop(None)
            loop.close()


# ============================================================================
# broadcast_loop (改造自 panel.py:800-891)
# ----------------------------------------------------------------------------
# 与 panel.py 的差异:
#   - 数据源从 _read_ros2_state() 文件 → 改读 NxWebNode 的订阅缓存 (退役 dog_state.json)
#   - xy/yaw 用 /localization_pose 的 map -> base_link 位姿
#   - trail 本地累积, 每 0.1m 一个点, 上限 2000 (panel.py:772-775 同款)
#   - slam_source = "ros2_nx" (H1.1, 前端地图右上角显示 "SLAM: ros2_nx")
#   - scan 转世界坐标公式照抄 panel.py:815-821
#   - 不发 frame (阶段A 不直连狗 VideoClient, 前端 type==='frame' 自然显示"等待视频")
# ============================================================================
_trail = []
_obstacle_grid = ObstacleGridAccumulator(
    resolution=float(os.environ.get("GO2W_MAP_RESOLUTION", "0.1")),
    max_points=int(os.environ.get("GO2W_MAP_MAX_POINTS", "2000")),
)


def broadcast_loop(robot_bridge: NxRobotBridge, nx_node: NxWebNode, task_manager: TaskManager, ai_engine=None, lidar_bridge=None):
    global _trail
    logger.info("广播启动")
    slam_counter = 0
    ipc_mtimes = {}

    def _broadcast_json_if_changed(path, event_type, *, force=True):
        try:
            modified = os.path.getmtime(path)
            if modified <= float(ipc_mtimes.get(path, 0.0)):
                return False
            with open(path, encoding="utf-8") as source:
                payload = json.load(source)
            ipc_mtimes[path] = modified
            ws_broadcast(
                {"type": event_type, "data": payload}, force=force)
            return True
        except Exception:
            return False

    while True:
        try:
            # ---- 取订阅缓存快照 (一次锁定) ----
            with nx_node._lock:
                localization = nx_node.get_localization_health()
                navigation = nx_node.get_navigation_readiness()
                localization_healthy = bool(localization.get("healthy"))
                yaw = nx_node._map_yaw
                raw_imu_heading = nx_node._imu_yaw
                imu_count = nx_node._imu_count
                x = nx_node._map_x
                y = nx_node._map_y
                ranges = list(nx_node._scan_ranges)
                dog_state = nx_node._dog_state
                dog_vx = nx_node._dog_vx
                dog_vy = nx_node._dog_vy
                dog_vyaw = nx_node._dog_vyaw
                connected = (time.time() - nx_node._last_state_t) < nx_node.state_timeout

            # ---- 阶段B: 视频/YOLO 帧 (来自 ai_engine._video_yolo_loop 缓存, 不阻塞) ----
            # type=frame 格式严格对齐 panel.py:847-849 / panel.html:384-389:
            #   detections = 整数计数 (C1.4), 不是数组! 像素 bbox 已画在 jpeg 里。
            if ai_engine is not None and hasattr(ai_engine, "submit_external_frame"):
                try:
                    gimbal = getattr(robot_bridge, "_gimbal_bridge", None)
                    if gimbal is not None:
                        c13_vis_frame = gimbal.get_vis_frame()
                        if c13_vis_frame is not None:
                            ai_engine.submit_external_frame(c13_vis_frame, source="c13_vis")
                        # 红外不进 YOLO (256x205 测小物体不准 + 减一路推理提 fps);
                        # 红外视频流不受影响 (gimbal ir 推 WS 在 nx_gimbal_node, 独立于 ai)
                        # if hasattr(gimbal, "get_ir_frame"):
                        #     c13_ir_frame = gimbal.get_ir_frame()
                        #     if c13_ir_frame is not None:
                        #         ai_engine.submit_external_frame(c13_ir_frame, source="c13_ir")
                except Exception as e:
                    logger.debug(f"C13 frame submit failed: {e}")
            if ai_engine is not None and (hasattr(ai_engine, "get_detection_overlays") or hasattr(ai_engine, "get_detection_overlay")):
                try:
                    payload = (ai_engine.get_detection_overlays()
                               if hasattr(ai_engine, "get_detection_overlays")
                               else ai_engine.get_detection_overlay())
                    ws_broadcast({"type": "detections", "data": payload})
                except Exception as e:
                    logger.debug(f"detection overlay broadcast failed: {e}")
            if ai_engine is not None:
                det_count = ai_engine.get_frame_detection_count()
                if det_count is not None:
                    ws_broadcast({"type": "frame", "detections": int(det_count)})

            # ---- trail 累积 (每 0.1m 一个点, 上限 2000) ----
            if localization_healthy:
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
            if localization_healthy and ranges:
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
            lidar_points = []
            if lidar_bridge is not None and hasattr(lidar_bridge, "get_latest_points"):
                try:
                    lidar_points = lidar_bridge.get_latest_points()
                except Exception:
                    lidar_points = []

            # ---- SLAM 推送 (字段名严格匹配 map.js update()) ----
            # 阶段B: slam.data.detections 是数组 [{x,y,class}] (C1.5, map.js:52),
            # 与 type=frame 的整数 detections 相反! 由 ai_engine 世界坐标转换填值。
            slam_counter += 1
            if slam_counter % 2 == 0:
                det_world = (ai_engine.get_detections_world(x, y, yaw, ranges=ranges, lidar_points=lidar_points)
                    if ai_engine is not None and localization_healthy else [])
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
                }}, force=True)

            # ---- costmap 推送 (独立 costmap_bridge 写 /tmp/costmap_lite.json, web 读转发) ----
            # 不在 web 直接订阅 costmap: web rclpy 订阅数临界 wait_set 上限, 加订阅触发
            # "IndexError: wait set index too big" → spin 崩 (见 costmap_bridge.py 头注释).
            # bridge 独立进程降采写 JSON, web 读文件转发 WS (不加 rclpy 订阅), 前端 map.js 渲染.
            if slam_counter % 10 == 0:
                # Only forward new IPC snapshots. Re-sending identical maps
                # every loop used to congest the same WebSocket as room state.
                _broadcast_json_if_changed(
                    '/tmp/costmap_lite.json', 'costmap', force=True)
                # 2026-07-18: global costmap (规划用, 前端切换看 ghost 对比) + plan (规划路线 polyline)
                _broadcast_json_if_changed(
                    '/tmp/costmap_global.json', 'costmap_global', force=True)
                _broadcast_json_if_changed(
                    '/tmp/map_frontier_walls.json', 'occupancy_map', force=True)
                _broadcast_json_if_changed(
                    '/tmp/plan_lite.json', 'plan', force=True)

            # ---- status 推送 (字段名匹配 panel.html:396-400) ----
            det_list = ai_engine.get_detection_list() if ai_engine is not None and hasattr(ai_engine, "get_detection_list") else []
            ws_broadcast({"type": "status",
                          "imu_yaw": round(raw_imu_heading, 3),
                          "stats": {"imu_count": imu_count,
                                     "robot_mode": 0,
                                     "robot_velocity": [dog_vx, dog_vy, dog_vyaw],
                                     "connected": connected,
                                     **ws_telemetry()},
                          "dog_state": dog_state,
                          "tasks": task_manager.get_state() if task_manager else {},
                           "localization": localization,
                           "odometry": nx_node.get_odometry_snapshot(),
                           "navigation": navigation,
                           "services": nx_node._collect_nav2_services(),
                           "point_nav": point_nav.get_state() if point_nav else {},
                           "room_nav": room_orchestrator.get_navigation_state() if room_orchestrator else {},
                           "perception": _perception_health(robot_bridge),
                           "det_list": det_list})

            time.sleep(0.15)
        except Exception as e:
            logger.warning(f"广播: {e}")
            time.sleep(0.5)


def _spin_loop_yielding(node):
    """GIL-yielding spin: spin_once + sleep 替代 rclpy.spin, 避免订阅频繁时
    wait_for_ready busy 持有 GIL 饿死 HTTP/arbiter 线程 (nav2 goal 超时根因)。

    rclpy.spin 内部循环 spin_once(timeout_sec=0), 订阅 (/imu 200Hz /dog_state 10Hz
    /scan 10Hz /points_nav 10Hz /localization 30Hz) 频繁时 wait_for_ready 立即
    ready → 不阻塞 → busy 持有 GIL → HTTP handler 调 arbiter.start_point_goal
    抢不到 GIL → nav2 goal 发不出 → 超时。

    每 spin_once 后 time.sleep(0.001) 强制释放 GIL (Python sleep 释放 GIL),
    arbiter 在 HTTP 线程能拿 GIL 执行 nav2 action client 调用。1000Hz spin
    足够覆盖 /imu 200Hz + 其他订阅, callback 不丢。

    若 sleep(1ms) 仍不解 (callback execute 重持 GIL), 再做进程拆分。
    """
    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0)
        except Exception as exc:
            logger.warning("spin_once 异常, 退出 spin 线程: %s", exc)
            break
        time.sleep(0.001)  # 强制 yield GIL, 解 busy 饿死


# ============================================================================
# Main
# ============================================================================
def main():
    global robot, task_mgr, node, point_nav, navigation_gateway
    global room_orchestrator, navigation_arbiter

    rclpy.init()
    node = NxWebNode()
    static_dir = os.path.join(_WEB_DIR, 'static')
    mission_root = os.environ.get(
        "GO2W_MISSION_ROOT", os.path.join(static_dir, "missions"))

    # 创建控制器和 watchdog timer 后才启动 executor，避免请求在控制器未注入时进入。
    raw_point_nav = PointNavigationController(
        node,
        state_callback=_handle_point_nav_state,
        health_check=lambda: _point_navigation_health_sample(node),
    )
    try:
        path_port = RosComputePathPort(node)
    except Exception as exc:
        logger.warning("ComputePathToPose adapter unavailable: %s", exc)
        path_port = None
    navigation_gateway = NavigationGateway(
        action_port=raw_point_nav,
        path_port=path_port,
    )
    point_nav = OwnerNavigationPort(navigation_gateway, "point")
    mission_navigation = MissionNavigationPort(
        navigation_gateway, owner="mission")
    node.create_timer(0.1, navigation_gateway.tick)

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
    task_mgr.set_point_nav(point_nav)

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
                navigation_port=mission_navigation,
                observation_sync=node.observation_sync,
                mission_root=mission_root,
            )
            task_mgr.room_orchestrator = room_orchestrator
            # Current-room resolution compatibility for injected orchestrators.
            TaskManager._global_room_orchestrator = room_orchestrator
            logger.info("阶段E: RoomSearchOrchestrator 已注入 TaskManager")
        except Exception as e:
            logger.error(f"阶段E RoomSearchOrchestrator 启动失败 (search_room 任务将标 failed): {e}")
            room_orchestrator = None

    # All HTTP threads and background command parsing share this one admission
    # lock.  It never spins ROS; each ownership hand-off is terminal-state gated.
    navigation_arbiter = NavigationArbiter(point_nav, task_mgr, robot)
    task_mgr.set_navigation_arbiter(navigation_arbiter)
    mission_navigation.set_recovery_callback(
        navigation_arbiter.recover_task_motion)

    # Humble's rclpy wait-set cannot be mutated while an executor is waiting.
    # PointNav and RoomSearch both create ActionClients, so start spin only
    # after every ROS entity has been constructed.
    # 2026-07-19 spin 改 spin_once+sleep 循环 (替代 rclpy.spin):
    # rclpy.spin 内部 spin_once(timeout_sec=0) busy 持有 GIL → arbiter 饿死 →
    # nav2 goal 超时。spin_once+sleep(1ms) 强制 yield GIL, 解饿死。见上方
    # _spin_loop_yielding 注释。若仍不解再做进程拆分。
    spin_th = threading.Thread(target=_spin_loop_yielding, args=(node,), daemon=True)
    spin_th.start()

    host = node.host
    port = node.port
    ws_port = node.ws_port
    logger.info(f"Web: http://{host}:{port}  WS: ws://{host}:{ws_port}  "
                f"AI={'on' if ai_engine else 'off'}  Room={'on' if room_orchestrator else 'off'}")

    # 启动 WS server 线程 + 任务 worker + 广播线程
    threading.Thread(target=run_ws, args=(host, ws_port), daemon=True).start()
    task_mgr.start_worker()
    threading.Thread(target=broadcast_loop, args=(robot, node, task_mgr, ai_engine, lidar_bridge), daemon=True).start()

    server = create_server(host, port, static_dir, mission_root=mission_root)
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
        # ROS executor 仍在运行时封住新请求，并同时 drain PointNav、
        # TaskManager worker 与 Room Nav2 action ownership。
        try:
            shutdown_state = navigation_arbiter.shutdown()
            if not shutdown_state.get("ok"):
                logger.critical(
                    "Autonomous owners did not drain during shutdown: %s",
                    shutdown_state,
                )
        except Exception as exc:
            logger.critical("导航仲裁退出清理失败，执行急停: %s", exc)
            try:
                if robot is not None:
                    robot.e_stop()
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
