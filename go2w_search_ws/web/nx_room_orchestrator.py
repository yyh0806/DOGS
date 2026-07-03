#!/usr/bin/env python3
"""Go2W 阶段E — 房间级搜索编排器 (spec-stage-e §7.1)。

== 职责 ==
作为"组件"注入 nx_web_server.py 的 TaskManager (同进程, spec 决策 3 方案 b):
  1. RoomMap: 加载静态 YAML 房间地图 (config/rooms.yaml), 校验 schema + 房间匹配
  2. Nav2ActionClient: Nav2 NavigateToPose action client 封装 (worker 线程 spin_until_complete)
  3. RoomSearchOrchestrator: 状态机驱动 (SELECT_ROOM→NAVIGATE→ARRIVED→SEARCH→DETECT→REPORT)
     每阶段切换 ws_broadcast type=search_room; 完成推 type=mission_report

== 线程模型 (spec 决策 4) ==
  - 主线程: HTTPServer.serve_forever
  - 线程1 (daemon): rclpy.spin(NxWebNode)         ← 主 spin (驱动订阅回调)
  - 线程X (TaskManager worker): 执行 search_room 时
       └─ RoomSearchOrchestrator.run(task)
            └─ Nav2ActionClient.send_goal_and_wait()
                 ├─ send_goal_async → goal_future
                 ├─ rclpy.spin_until_complete(node, goal_future, timeout=5s)   ← worker 临时驱动
                 └─ rclpy.spin_until_complete(node, result_future, timeout=120s)
  关键: ActionClient 用 ReentrantCallbackGroup (否则与主 spin 的默认组死锁)
  关键: 所有共享状态 (_current_handle/_cancelled) 用 threading.Lock 保护

== 红线 (spec §0 + §12 反模式) ==
  - 懒加载: __init__ 不加载 YAML, 不创建 Nav2Client (启动快, Nav2 未就绪不报错)
  - 不自己发 /cmd_vel 做航点导航 (决策 1, 走 Nav2 action 标准接口)
  - 不用 NavigateThroughPoses 一次发整条航点链 (决策 1, 丢失中间停留检测语义)
  - 不复活 src/go2w_orchestrator 包 (决策 3, planner 已在 nx_web_server 内联)
  - 不用 MultiThreadedExecutor (阶段A 决策 2 继续生效)
  - 不改 panel.html / map.js (前端零改动, 新 ws type 走 console.log)
  - 不直接 import ai.detector (走 ai_engine 注入, 阶段A 退化时 ai_engine=None)
  - 不调 ai_engine._detector.detect (重复推理), 只读 ai_engine.get_detections_world 快照
"""

import json
import logging
import math
import os
import threading
import time
import uuid
from typing import Optional, List, Dict

logger = logging.getLogger("go2w.room_orch")

try:
    from nx_active_search import ActiveSearchPlanner
except Exception:
    ActiveSearchPlanner = None

try:
    from nx_person_localizer import DetectionFrame, LaserScanSnapshot, localize_person_detection
except Exception:
    DetectionFrame = None
    LaserScanSnapshot = None
    localize_person_detection = None

try:
    from nx_person_mission import PersonMissionStore
except Exception:
    PersonMissionStore = None

try:
    from nx_product_command import resolve_current_room
except Exception:
    resolve_current_room = None

# web/ 目录 (与 nx_web_server.py 同目录, 复用其内联的 plan_lawnmower/plan_spiral)
_WEB_DIR = os.path.dirname(os.path.abspath(__file__))

# rclpy / Nav2 action 接口 (懒导入, Windows 开发机无 rclpy 时也能 import 本模块做静态检查)
_rclpy = None
_NavigateToPose = None
_ReentrantCallbackGroup = None
_ActionClient = None


def _import_ros():
    """懒导入 rclpy + nav2 action 类型。返回 True/False。
    NX 部署: True; Windows 开发机: False (静态检查/纯逻辑测试用 mock 替代)。
    """
    global _rclpy, _NavigateToPose, _ReentrantCallbackGroup, _ActionClient
    if _rclpy is not None:
        return True
    try:
        import rclpy as _r
        from rclpy.action import ActionClient as _AC
        from rclpy.callback_groups import ReentrantCallbackGroup as _RCG
        from nav2_msgs.action import NavigateToPose as _NTP
        _rclpy = _r
        _ActionClient = _AC
        _ReentrantCallbackGroup = _RCG
        _NavigateToPose = _NTP
        return True
    except Exception as e:
        logger.debug(f"rclpy/nav2 不可导入 (NX 部署外正常): {e}")
        return False


# ============================================================================
# Room / RoomMap — 房间地图 (YAML 加载, spec §6)
# ============================================================================
class Room:
    """单个房间定义 (从 rooms.yaml 一条 room 反序列化)。"""

    def __init__(self, name: str, aliases: List[str], nav_pose: dict,
                 search_area: dict, target_classes: Optional[List[str]] = None):
        self.name = name
        self.aliases = list(aliases or [])
        self.nav_pose = {
            "x": float(nav_pose["x"]),
            "y": float(nav_pose["y"]),
            "yaw": float(nav_pose["yaw"]),
        }
        self.search_area = {
            "width": float(search_area["width"]),
            "height": float(search_area["height"]),
            "origin_x": float(search_area["origin_x"]),
            "origin_y": float(search_area["origin_y"]),
            "spacing": float(search_area.get("spacing", 2.5)),
            "pattern": str(search_area.get("pattern", "lawnmower")),
        }
        self.target_classes = list(target_classes or [])

    def to_dict(self) -> dict:
        """序列化 (供 /api/rooms 或 mission_report.area 用)。"""
        return {
            "name": self.name,
            "aliases": list(self.aliases),
            "nav_pose": dict(self.nav_pose),
            "search_area": dict(self.search_area),
            "target_classes": list(self.target_classes),
        }

    @classmethod
    def from_yaml_dict(cls, d: dict) -> "Room":
        """从 YAML 解析的 dict 构造, 校验必填字段, 失败抛 ValueError (spec §6.2)。
        校验规则 (§6.2 rule 2):
          - name 非空字符串
          - nav_pose.{x,y,yaw} 必填且数值
          - search_area.{width,height,origin_x,origin_y} 必填且数值
        """
        if not isinstance(d, dict):
            raise ValueError("room 条目必须是 dict")
        name = d.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("room.name 缺失或非空字符串")

        nav_pose = d.get("nav_pose")
        if not isinstance(nav_pose, dict):
            raise ValueError(f"room[{name}].nav_pose 缺失或非 dict")
        for k in ("x", "y", "yaw"):
            if k not in nav_pose:
                raise ValueError(f"room[{name}].nav_pose.{k} 缺失")
            try:
                float(nav_pose[k])
            except (TypeError, ValueError):
                raise ValueError(f"room[{name}].nav_pose.{k} 非数值: {nav_pose[k]!r}")

        search_area = d.get("search_area")
        if not isinstance(search_area, dict):
            raise ValueError(f"room[{name}].search_area 缺失或非 dict")
        for k in ("width", "height", "origin_x", "origin_y"):
            if k not in search_area:
                raise ValueError(f"room[{name}].search_area.{k} 缺失")
            try:
                float(search_area[k])
            except (TypeError, ValueError):
                raise ValueError(f"room[{name}].search_area.{k} 非数值: {search_area[k]!r}")
        # §6.2 rule 4: 数值约束 width>0, height>0, spacing>0
        if float(search_area["width"]) <= 0:
            raise ValueError(f"room[{name}].search_area.width 必须 > 0 (得 {search_area['width']})")
        if float(search_area["height"]) <= 0:
            raise ValueError(f"room[{name}].search_area.height 必须 > 0 (得 {search_area['height']})")
        if "spacing" in search_area:
            try:
                if float(search_area["spacing"]) <= 0:
                    raise ValueError(f"room[{name}].search_area.spacing 必须 > 0")
            except (TypeError, ValueError) as e:
                raise ValueError(f"room[{name}].search_area.spacing 非法: {e}")

        # §6.2 rule 3: pattern 必须 lawnmower 或 spiral
        pattern = str(search_area.get("pattern", "lawnmower"))
        if pattern not in ("lawnmower", "spiral"):
            raise ValueError(f"room[{name}].search_area.pattern 必须 lawnmower|spiral (得 {pattern!r})")

        aliases = d.get("aliases", [])
        if not isinstance(aliases, list):
            raise ValueError(f"room[{name}].aliases 必须是 list")
        target_classes = d.get("target_classes", [])
        if not isinstance(target_classes, list):
            raise ValueError(f"room[{name}].target_classes 必须是 list")

        return cls(name, aliases, nav_pose, search_area, target_classes)


class RoomMap:
    """房间地图集合 (rooms.yaml 的内存表示)。"""

    def __init__(self, frame_id: str, rooms: List[Room],
                 default_spacing: float = 2.5, default_pattern: str = "lawnmower",
                 version: str = "1.0"):
        self.frame_id = frame_id
        self.rooms = list(rooms)
        self.default_spacing = float(default_spacing)
        self.default_pattern = str(default_pattern)
        self.version = str(version)

    @classmethod
    def load(cls, path: str) -> "RoomMap":
        """从 YAML 文件加载 + 校验 (spec §6.2 全部 6 条规则)。
        失败:
          - 文件不存在 → FileNotFoundError
          - YAML 格式错 → yaml.YAMLError
          - 字段缺失/数值非法/name 重复 → ValueError
        """
        import yaml
        if not os.path.exists(path):
            raise FileNotFoundError(f"房间地图文件不存在: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise ValueError(f"YAML 顶层必须是 dict (得 {type(data).__name__})")

        # §6.2 rule 1: 顶层必填 frame_id / version / rooms
        for k in ("frame_id", "version", "rooms"):
            if k not in data:
                raise ValueError(f"顶层缺少必填字段: {k}")
        frame_id = str(data["frame_id"])
        version = str(data["version"])
        rooms_raw = data["rooms"]
        if not isinstance(rooms_raw, list):
            raise ValueError(f"rooms 必须是 list (得 {type(rooms_raw).__name__})")

        default_spacing = float(data.get("default_search_spacing", 2.5))
        default_pattern = str(data.get("default_search_pattern", "lawnmower"))

        # 逐条构造 Room (会校验字段); 同时用顶层 default 填充 room 级缺省
        rooms: List[Room] = []
        seen_names = set()
        for rd in rooms_raw:
            # 把顶层 default 灌进 room.search_area 缺省项 (room 级未指定时用顶层)
            if isinstance(rd, dict):
                sa = rd.setdefault("search_area", {})
                sa.setdefault("spacing", default_spacing)
                sa.setdefault("pattern", default_pattern)
            room = Room.from_yaml_dict(rd)
            # §6.2 rule 5: name 唯一
            if room.name in seen_names:
                raise ValueError(f"room.name 重复: {room.name}")
            seen_names.add(room.name)
            rooms.append(room)

        return cls(frame_id, rooms, default_spacing, default_pattern, version)

    def find(self, query: str) -> Optional[Room]:
        """房间匹配 (spec §6.3): name>aliases 完全相等 > name/aliases 子串包含。
        多匹配返回第一个 (name 唯一性保证主名不冲突)。
        """
        if not query:
            return None
        q = query.strip()
        # 优先级 1: name 完全相等
        for r in self.rooms:
            if r.name == q:
                return r
        # 优先级 2: aliases 完全相等
        for r in self.rooms:
            if q in r.aliases:
                return r
        # 优先级 3: name 子串包含 (如 "搜索客厅" 含 "客厅")
        for r in self.rooms:
            if r.name in q:
                return r
        # 优先级 4: aliases 子串包含
        for r in self.rooms:
            for a in r.aliases:
                if a and a in q:
                    return r
        return None

    def list_rooms(self) -> List[str]:
        """返回所有房间主名 (供 /api/rooms 列出可搜房间)。"""
        return [r.name for r in self.rooms]

    def list_rooms_detail(self) -> List[dict]:
        """返回所有房间完整 dict (供 /api/rooms 详情)。"""
        return [r.to_dict() for r in self.rooms]


# ============================================================================
# Nav2ActionClient — Nav2 NavigateToPose action client 封装 (spec 决策 1/4)
# ============================================================================
# NavigateToPose action 标准状态码 (rclpy rclpy.action GoalStatus)
_STATUS_SUCCEEDED = 4
_STATUS_ABORTED = 6
_STATUS_CANCELED = 7


class Nav2ActionClient:
    """Nav2 NavigateToPose action client 的同步等待封装。

    线程模型 (spec 决策 4):
      - 构造时挂在 nx_web_server 的 NxWebNode 上 (与主 rclpy.spin 共享 node)
      - send_goal_and_wait() 在 TaskManager worker 线程内调,
        用 rclpy.spin_until_complete(node, future, timeout) 同步等
      - 主 spin 线程 (线程1) 不干涉 (spin_until_complete 临时驱动 future 回调)
    """

    def __init__(self, node, action_name: str = '/navigate_to_pose',
                 goal_accept_timeout: float = 5.0,
                 nav_complete_timeout: float = 120.0):
        if not _import_ros():
            raise RuntimeError("rclpy/nav2_msgs 不可用, 无法创建 Nav2ActionClient")
        self._node = node
        self._client = _ActionClient(
            node, _NavigateToPose, action_name,
            callback_group=_ReentrantCallbackGroup()  # spec 决策 4 约束 2: 必须 Reentrant
        )
        self._goal_accept_timeout = float(goal_accept_timeout)
        self._nav_complete_timeout = float(nav_complete_timeout)
        self._lock = threading.Lock()
        self._current_handle = None      # 当前 goal_handle (cancel 用)
        self._feedback_callback = None   # 外部注入的 feedback 回调 (推进度用)
        self._cancelled = False

    def wait_for_server(self, timeout: float = 2.0) -> bool:
        """探测 Nav2 action server 是否在线 (阶段D/mock 判据)。"""
        try:
            return self._client.wait_for_server(timeout_sec=float(timeout))
        except Exception as e:
            logger.warning(f"Nav2 wait_for_server 异常: {e}")
            return False

    def set_feedback_callback(self, cb):
        """注入 feedback 回调: cb(distance_remaining, estimated_time_remaining, *extra)。"""
        self._feedback_callback = cb

    def _yaw_to_quaternion(self, yaw: float):
        """yaw (弧度) → (qx, qy, qz, qw) 四元数 (REP-103)。
        spec §7.1: qz=sin(yaw/2), qw=cos(yaw/2), qx=qy=0
        (对齐休眠包 orchestrator_node.py:176-180)
        """
        half = float(yaw) / 2.0
        qz = math.sin(half)
        qw = math.cos(half)
        return 0.0, 0.0, qz, qw

    def _on_feedback(self, feedback_msg):
        """Nav2 feedback 回调 (主 spin 或 worker spin 驱动, 线程安全)。
        feedback_msg.feedback 含 distance_remaining / estimated_time_remaining。
        """
        try:
            fb = getattr(feedback_msg, "feedback", None)
            if fb is None or self._feedback_callback is None:
                return
            dist = float(getattr(fb, "distance_remaining", 0.0) or 0.0)
            try:
                eta = float(getattr(fb, "estimated_time_remaining", 0.0).sec
                            + getattr(fb, "estimated_time_remaining", 0.0).nanosec * 1e-9)
            except Exception:
                eta = 0.0
            self._feedback_callback(dist, eta)
        except Exception as e:
            logger.debug(f"Nav2 feedback 处理异常: {e}")

    def send_goal_and_wait(self, x: float, y: float, yaw: float,
                           frame_id: str = 'map') -> Dict:
        """同步发 Nav2 goal + 等接受 + 等到达 (spec 决策 4)。

        返回:
          {"ok": True, "status": 4}                              成功 (STATUS_SUCCEEDED)
          {"ok": False, "reason": "no_server"}                   server 不在线
          {"ok": False, "reason": "rejected"}                    goal 被拒绝
          {"ok": False, "reason": "timeout"}                     导航超时
          {"ok": False, "reason": "aborted", "status": N}        Nav2 主动 abort
          {"ok": False, "reason": "cancelled"}                   被 cancel_goal 取消
        """
        # 1. wait_for_server → 不在则 no_server
        if not self.wait_for_server(self._goal_accept_timeout):
            return {"ok": False, "reason": "no_server"}

        # 2. 构造 NavigateToPose.Goal (PoseStamped, yaw→四元数)
        goal = _NavigateToPose.Goal()
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.header.frame_id = frame_id
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        with self._lock:
            self._cancelled = False
            self._current_handle = None

        # 3. send_goal_async → goal_future
        try:
            goal_future = self._client.send_goal_async(
                goal, feedback_callback=self._on_feedback)
        except Exception as e:
            logger.warning(f"send_goal_async 异常: {e}")
            return {"ok": False, "reason": "no_server"}

        # 4. spin_until_complete(node, goal_future, goal_accept_timeout)
        #    → 超时 no_server / rejected / 拿到 handle
        try:
            _rclpy.spin_until_complete(self._node, goal_future,
                                       timeout_sec=self._goal_accept_timeout)
        except Exception as e:
            logger.warning(f"spin_until_complete(goal) 异常: {e}")
            return {"ok": False, "reason": "no_server"}

        if not goal_future.done():
            return {"ok": False, "reason": "no_server"}  # 接受超时
        goal_handle = goal_future.result()
        if goal_handle is None:
            return {"ok": False, "reason": "no_server"}
        if not getattr(goal_handle, "accepted", False):
            return {"ok": False, "reason": "rejected"}

        # 持有 handle (cancel 用)
        with self._lock:
            self._current_handle = goal_handle

        # 5. handle.get_result_async → result_future
        try:
            result_future = goal_handle.get_result_async()
        except Exception as e:
            logger.warning(f"get_result_async 异常: {e}")
            return {"ok": False, "reason": "aborted", "status": -1}

        # 6. spin_until_complete(node, result_future, nav_complete_timeout)
        #    → 超时 timeout / status==4 ok / 其他 aborted / cancelled
        try:
            _rclpy.spin_until_complete(self._node, result_future,
                                       timeout_sec=self._nav_complete_timeout)
        except Exception as e:
            logger.warning(f"spin_until_complete(result) 异常: {e}")
            # 尝试 cancel 后返回 timeout
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return {"ok": False, "reason": "timeout"}

        # 检查是否被外部 cancel
        with self._lock:
            if self._cancelled:
                return {"ok": False, "reason": "cancelled"}
            self._current_handle = None

        if not result_future.done():
            # 导航超时 → 尝试 cancel
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return {"ok": False, "reason": "timeout"}

        result = result_future.result()
        status = int(getattr(result, "status", -1))
        if status == _STATUS_SUCCEEDED:
            return {"ok": True, "status": status}
        if status == _STATUS_ABORTED:
            return {"ok": False, "reason": "aborted", "status": status}
        if status == _STATUS_CANCELED:
            return {"ok": False, "reason": "cancelled"}
        # 其他非成功状态统一算 aborted
        return {"ok": False, "reason": "aborted", "status": status}

    def cancel_current(self) -> bool:
        """取消当前进行中的 Nav2 goal (cancel_all / e_stop 时调)。
        spec 决策 4 约束 4: 标 _cancelled=True + 调 handle.cancel_goal_async()
        """
        with self._lock:
            self._cancelled = True
            handle = self._current_handle
        if handle is None:
            return False
        try:
            handle.cancel_goal_async()
            return True
        except Exception as e:
            logger.warning(f"cancel_goal_async 异常: {e}")
            return False


# ============================================================================
# MissionReport — 任务最终报告 (go2w_interfaces/MissionReport.msg 的 dict 等价)
# ============================================================================
def build_mission_report(mission_id: str, room: Room, waypoints_total: int,
                         waypoints_visited: int, detections: List[dict],
                         start_time: float, end_time: float,
                         result_path: str = "") -> dict:
    """生成 MissionReport dict (spec §5 REPORT 阶段)。
    返回结构对齐 go2w_interfaces/MissionReport.msg:
      {mission_id, room, status, start_time, end_time, duration_sec,
       waypoints_total, waypoints_visited, targets_found, detections, area, result_path}
    """
    return {
        "mission_id": str(mission_id),
        "room": room.name if room is not None else "",
        "status": "completed",
        "start_time": float(start_time),
        "end_time": float(end_time),
        "duration_sec": round(float(end_time) - float(start_time), 2),
        "waypoints_total": int(waypoints_total),
        "waypoints_visited": int(waypoints_visited),
        "targets_found": int(len(detections)),
        "detections": list(detections),
        "area": (room.search_area if room is not None else {}),
        "result_path": str(result_path),
    }


# ============================================================================
# RoomSearchOrchestrator — 房间级搜索编排器 (spec §5 状态机)
# ============================================================================
# 默认 rooms.yaml 路径 (仓库根 config/rooms.yaml)
_DEFAULT_ROOMS_YAML = os.path.normpath(
    os.path.join(_WEB_DIR, "..", "config", "rooms.yaml"))


class RoomSearchOrchestrator:
    """房间级搜索编排器 (spec 决策 3, 集成进 TaskManager)。

    生命周期: nx_web_server.main() 创建一个实例, 注入 TaskManager
    (task_mgr.room_orchestrator = RoomSearchOrchestrator(node, ai_engine, ws_broadcast))
    TaskManager._execute_search_room 调用 self.room_orchestrator.run(task)。
    """

    def __init__(self, node=None, ai_engine=None,
                 ws_broadcast_fn=None,
                 rooms_yaml_path: Optional[str] = None):
        self._node = node                  # NxWebNode (供 Nav2ActionClient 挂载)
        self._ai = ai_engine               # NxAiEngine (阶段B), 读 get_detections_world
        self._ws = ws_broadcast_fn         # ws_broadcast 注入 (与 nx_ai_node 同款)
        self._lock = threading.Lock()
        # 房间地图 (懒加载, 首次 run 时 load)
        self._rooms_yaml = rooms_yaml_path or os.environ.get(
            'GO2W_ROOMS_YAML', _DEFAULT_ROOMS_YAML)
        self._room_map: Optional[RoomMap] = None
        self._room_map_err: Optional[str] = None  # 加载失败原因 (供 /api/rooms 显示)
        # Nav2 client (懒创建, 首次 run 时)
        self._nav: Optional[Nav2ActionClient] = None
        # 当前 mission 状态 (cancel 检查 + 推进度用)
        self._current_mission_id: Optional[str] = None
        self._current_room_name: Optional[str] = None
        self._current_total_wp: int = 0
        self._current_wp_idx: int = 0
        self._current_targets_found: int = 0
        self._person_markers: List[dict] = []
        self._static_root = os.path.join(_WEB_DIR, "static")
        self._cancelled = False

    # ---- 房间地图加载 ----
    def _ensure_room_map(self) -> Optional[RoomMap]:
        """懒加载房间地图。失败记 _room_map_err, ws 推 phase:FAILED, reason, 返回 None。
        失败原因映射:
          FileNotFoundError / yaml.YAMLError → no_room_map / invalid_yaml
        """
        if self._room_map is not None:
            return self._room_map
        try:
            self._room_map = RoomMap.load(self._rooms_yaml)
            self._room_map_err = None
            logger.info(f"房间地图加载: {len(self._room_map.rooms)} 个房间 ({self._rooms_yaml})")
            return self._room_map
        except FileNotFoundError as e:
            self._room_map_err = f"no_room_map: {e}"
            logger.error(f"房间地图文件不存在: {e}")
            self._fail("no_room_map", room=None, msg=str(e))
        except Exception as e:
            # YAML 格式错 / ValueError (字段/数值/name 重复)
            import yaml
            if isinstance(e, yaml.YAMLError):
                self._room_map_err = f"invalid_yaml: {e}"
                self._fail("invalid_yaml", room=None, msg=str(e))
            else:
                self._room_map_err = f"invalid_yaml: {e}"
                self._fail("invalid_yaml", room=None, msg=str(e))
            logger.error(f"房间地图加载失败: {e}")
        return None

    def reload_rooms(self) -> bool:
        """重新加载 YAML (供 HTTP /api/reload_rooms 调, 改 YAML 后热加载)。
        spec §11: reload 只更新 self._room_map, 不影响进行中的 mission
                  (mission 已拿到 room 对象引用)。下次 search_room 用新地图。
        """
        # 清缓存让下次 _ensure_room_map 重 load
        prev = self._room_map
        self._room_map = None
        rm = self._ensure_room_map_quiet()
        if rm is None:
            # 重 load 失败 → 恢复旧地图 (避免 reload 把可用地图清成 None)
            self._room_map = prev
            return False
        return True

    def _ensure_room_map_quiet(self) -> Optional[RoomMap]:
        """与 _ensure_room_map 同, 但失败不 ws 推送 (reload 用)。"""
        try:
            self._room_map = RoomMap.load(self._rooms_yaml)
            self._room_map_err = None
            logger.info(f"房间地图重加载: {len(self._room_map.rooms)} 个房间")
            return self._room_map
        except Exception as e:
            self._room_map_err = str(e)
            logger.error(f"房间地图重加载失败: {e}")
            return None

    def list_rooms(self) -> List[str]:
        """供 /api/rooms 调: 返回所有房间主名。地图未加载时不强制 load (返回空)。"""
        if self._room_map is None:
            # 尝试静默加载一次 (不推 ws)
            self._ensure_room_map_quiet()
        if self._room_map is None:
            return []
        return self._room_map.list_rooms()

    def list_rooms_detail(self) -> List[dict]:
        """供 /api/rooms 调: 返回所有房间完整 dict。"""
        if self._room_map is None:
            self._ensure_room_map_quiet()
        if self._room_map is None:
            return []
        return self._room_map.list_rooms_detail()

    # ---- Nav2 client ----
    def _ensure_nav(self) -> Optional[Nav2ActionClient]:
        """懒创建 Nav2ActionClient (首次 run 时)。失败返回 None。"""
        if self._nav is not None:
            return self._nav
        if self._node is None:
            logger.error("Nav2 创建失败: rclpy node 未注入")
            return None
        if not _import_ros():
            logger.error("Nav2 创建失败: rclpy/nav2_msgs 不可用")
            return None
        try:
            self._nav = Nav2ActionClient(self._node)
            # 注入 feedback 回调: 把 distance_remaining 写进当前 mission 推进度
            self._nav.set_feedback_callback(self._on_nav_feedback)
            logger.info("Nav2ActionClient 已创建 (/navigate_to_pose)")
            return self._nav
        except Exception as e:
            logger.error(f"Nav2ActionClient 创建失败: {e}")
            self._nav = None
            return None

    def _on_nav_feedback(self, distance_remaining: float, eta_sec: float):
        """Nav2 feedback 回调: 推 type=search_room 进度 (NAVIGATING 时)。"""
        try:
            self._phase("NAVIGATING", progress=None, distance_remaining=float(distance_remaining),
                        eta_sec=float(eta_sec))
        except Exception as e:
            logger.debug(f"nav feedback 推送异常: {e}")

    # ---- 状态机驱动 (spec §5) ----
    def run(self, task) -> None:
        """房间级搜索主入口 (TaskManager worker 线程调)。

        task.params = {"room": "客厅", "target_classes": [...], ...}
        状态机 (spec §5): SELECT_ROOM → NAVIGATE → ARRIGATING → ARRIVED → SEARCH → DETECT → REPORT
        每阶段切换 ws_broadcast type=search_room, data.phase 更新。
        检测发现 ws_broadcast type=search (增量 found 列表, 复用阶段B 格式)。
        完成 ws_broadcast type=mission_report, data=MissionReport dict。
        任何阶段失败: task.status=failed, ws 推 phase:FAILED + reason。
        """
        params = task.params or {}
        room_query = params.get("room", "")
        # 任务级 target_classes 覆盖 (room.target_classes 优先, 任务级次之)
        task_target_classes = params.get("target_classes", []) or []

        mission_id = uuid.uuid4().hex[:8]
        start_time = time.time()
        with self._lock:
            self._cancelled = False
            self._current_mission_id = mission_id
            self._current_room_name = room_query
            self._current_total_wp = 0
            self._current_wp_idx = 0
            self._current_targets_found = 0

        detections_log: List[dict] = []   # 累积所有检测 (含 robot 位姿 + 时间戳)
        found_list: List[str] = []         # 去重标签 (复用阶段B type=search 格式)

        # ---- 1. SELECT_ROOM ----
        self._phase("SELECT_ROOM", progress=0.0, room=room_query)
        with self._lock:
            if self._cancelled:
                self._fail("cancelled", room=room_query); return
        room_map = self._ensure_room_map()
        if room_map is None:
            # _ensure_room_map 已推 FAILED (no_room_map/invalid_yaml)
            task.status = "failed"
            task.result = {"reason": self._room_map_err or "no_room_map"}
            return
        if room_query == "__current__":
            resolved = None
            if resolve_current_room is not None:
                robot_x, robot_y, _ = self._get_robot_pose()
                try:
                    resolved = resolve_current_room(
                        robot_x, robot_y, room_map.list_rooms_detail())
                except Exception as e:
                    logger.warning(f"resolve_current_room failed: {e}")
                    resolved = None
            if resolved is None:
                self._fail("no_room", room=room_query,
                           msg="unable to resolve current room from robot pose")
                task.status = "failed"
                task.result = {"reason": "no_room", "query": room_query}
                return
            room_query = resolved
            with self._lock:
                self._current_room_name = room_query
        room = room_map.find(room_query)
        if room is None:
            self._fail("no_room", room=room_query)
            task.status = "failed"
            task.result = {"reason": "no_room", "query": room_query}
            return
        with self._lock:
            self._current_room_name = room.name
        self._phase("SELECT_ROOM", progress=0.1, room=room.name,
                    info=f"匹配到房间 '{room.name}'")

        # 决定 target_classes: room 优先, 任务级次之, 空则全记
        target_classes = room.target_classes if room.target_classes else task_target_classes
        if params.get("search_strategy") == "next_best_view":
            self._run_product_person_search(
                task, mission_id, start_time, room_map, room,
                target_classes, params)
            return

        # ---- 2. NAVIGATE (发房间入口 goal) ----
        self._phase("NAVIGATE", progress=0.0, room=room.name)
        if self._check_cancel("NAVIGATE", room.name): return
        nav = self._ensure_nav()
        if nav is None:
            self._fail("no_nav", room=room.name)
            task.status = "failed"
            task.result = {"reason": "no_nav"}
            return
        # 探测 server (wait_for_server) → 失败 no_nav
        if not nav.wait_for_server(timeout=2.0):
            self._fail("no_nav", room=room.name, msg="Nav2 action server 不在线")
            task.status = "failed"
            task.result = {"reason": "no_nav"}
            return

        # ---- 3. NAVIGATING (等到达入口) ----
        self._phase("NAVIGATING", progress=0.0, room=room.name,
                    distance_remaining=None)
        if self._check_cancel("NAVIGATING", room.name): return
        r = nav.send_goal_and_wait(
            room.nav_pose["x"], room.nav_pose["y"], room.nav_pose["yaw"],
            frame_id=room_map.frame_id)
        if not r.get("ok"):
            reason = r.get("reason", "nav_aborted")
            # 映射 reason (no_server→no_nav, 其余原样)
            if reason == "no_server":
                reason = "no_nav"
            self._fail(reason, room=room.name, status=r.get("status"),
                       stage="navigate_to_room")
            task.status = "failed"
            task.result = {"reason": reason, "raw": r}
            return
        if self._check_cancel("NAVIGATING", room.name): return

        # ---- 4. ARRIVED (生成航点序列) ----
        self._phase("ARRIVED", progress=0.0, room=room.name)
        if self._check_cancel("ARRIVED", room.name): return
        waypoints = self._plan_room_waypoints(room)
        total_wp = len(waypoints)
        with self._lock:
            self._current_total_wp = total_wp
        logger.info(f"[{mission_id}] 房间 '{room.name}' 入口已到, 生成 {total_wp} 个搜索航点")
        if total_wp == 0:
            # 房间退化: 直接进 REPORT
            self._phase("REPORT", progress=1.0, room=room.name,
                        info="无搜索航点 (房间面积过小?)")
            self._finalize_report(task, mission_id, room, total_wp=0,
                                  visited=0, detections_log=detections_log,
                                  start_time=start_time)
            return

        # ---- 5. SEARCH (逐航点) → DETECT (每航点读检测快照) ----
        skip_failed_wp = bool(os.environ.get("GO2W_SKIP_FAILED_WP"))
        visited = 0
        for i, wp in enumerate(waypoints):
            with self._lock:
                self._current_wp_idx = i
            progress = float(i) / float(max(1, total_wp))
            self._phase("SEARCH", progress=progress, room=room.name,
                        current_wp=i, total_wp=total_wp,
                        waypoint=(wp["x"], wp["y"]))
            if self._check_cancel("SEARCH", room.name): return

            # 发航点 Nav2 goal
            r = nav.send_goal_and_wait(
                wp["x"], wp["y"], wp.get("yaw", 0.0),
                frame_id=room_map.frame_id)
            if not r.get("ok"):
                reason = r.get("reason", "wp_nav_err")
                if reason == "no_server":
                    reason = "no_nav"
                # spec §11: 默认整任务失败 (不跳过); GO2W_SKIP_FAILED_WP=1 时跳过继续
                if skip_failed_wp and reason in ("aborted", "timeout", "wp_nav_err"):
                    logger.warning(f"[{mission_id}] 航点 {i} 失败 ({reason}), SKIP 继续下一个")
                    self._phase("SEARCH", progress=progress, room=room.name,
                                current_wp=i, total_wp=total_wp,
                                warning=f"航点 {i} 跳过 ({reason})")
                    continue
                self._fail(reason, room=room.name, status=r.get("status"),
                           stage=f"waypoint_{i}", wp=(wp["x"], wp["y"]))
                task.status = "failed"
                task.result = {"reason": reason, "wp_index": i, "raw": r}
                return
            visited += 1

            # ---- DETECT (读阶段B 检测快照) ----
            self._phase("DETECT", progress=float(i + 1) / float(max(1, total_wp)),
                        room=room.name, current_wp=i, total_wp=total_wp)
            if self._check_cancel("DETECT", room.name): return
            # 取狗当前位姿 (map 坐标) — 优先用航点位姿 (Nav2 已到), nx_web 注入时也可读 /odom
            robot_x, robot_y, robot_yaw = self._get_robot_pose(fallback=wp)
            self._snapshot_detections(room, target_classes,
                                      robot_x, robot_y, robot_yaw,
                                      found_list, detections_log, wp_index=i)
            with self._lock:
                self._current_targets_found = len(detections_log)
            # 短停 (让 YOLO 多采几帧, 与阶段A search 的 0.2s sleep 同款节奏)
            time.sleep(0.3)
            if self._check_cancel("DETECT", room.name): return

        # ---- 6. REPORT ----
        self._phase("REPORT", progress=1.0, room=room.name,
                    targets_found=len(detections_log))
        self._finalize_report(task, mission_id, room, total_wp=total_wp,
                              visited=visited, detections_log=detections_log,
                              start_time=start_time)

    def _run_product_person_search(self, task, mission_id: str, start_time: float,
                                   room_map: RoomMap, room: Room,
                                   target_classes: List[str], params: dict) -> None:
        if not self._product_search_available():
            self._fail("product_search_unavailable", room=room.name)
            task.status = "failed"
            task.result = {"reason": "product_search_unavailable"}
            return

        get_snapshot = getattr(self._ai, "get_person_detection_snapshot", None)
        if self._ai is None or not callable(get_snapshot):
            self._fail("no_yolo", room=room.name)
            task.status = "failed"
            task.result = {"reason": "no_yolo"}
            return

        target_classes = list(target_classes or [])
        if "person" not in target_classes:
            target_classes = ["person"]

        with self._lock:
            self._current_room_name = room.name
            self._person_markers = []

        self._phase("NAVIGATE", progress=0.0, room=room.name)
        if self._check_cancel("NAVIGATE", room.name):
            task.status = "failed"
            task.result = {"reason": "cancelled"}
            return

        nav = self._ensure_nav()
        if nav is None or not nav.wait_for_server(timeout=2.0):
            self._fail("no_nav", room=room.name)
            task.status = "failed"
            task.result = {"reason": "no_nav"}
            return

        self._phase("NAVIGATING", progress=0.0, room=room.name,
                    distance_remaining=None)
        entry = nav.send_goal_and_wait(
            room.nav_pose["x"], room.nav_pose["y"], room.nav_pose["yaw"],
            frame_id=room_map.frame_id)
        if not entry.get("ok"):
            reason = self._nav_failure_reason(entry)
            self._fail(reason, room=room.name, status=entry.get("status"),
                       stage="navigate_to_room")
            task.status = "failed"
            task.result = {"reason": reason, "raw": entry}
            return
        if self._check_cancel("NAVIGATING", room.name):
            task.status = "failed"
            task.result = {"reason": "cancelled"}
            return

        self._phase("ARRIVED", progress=0.0, room=room.name)
        max_views = self._positive_int(params.get("max_views"), 12)
        with self._lock:
            self._current_total_wp = max_views

        try:
            planner = ActiveSearchPlanner(
                spacing=self._active_search_spacing(room),
                obstacle_clearance=self._positive_float(
                    params.get("obstacle_clearance_m"), 0.5))
        except Exception as e:
            self._fail("invalid_search_area", room=room.name, msg=str(e))
            task.status = "failed"
            task.result = {"reason": "invalid_search_area", "error": str(e)}
            return

        store = PersonMissionStore(mission_id, static_root=self._static_root)
        visited = 0
        require_photos = bool(params.get("require_photos", False))
        use_lidar = bool(params.get("use_lidar_person_range", True))

        for view_idx in range(max_views):
            with self._lock:
                self._current_wp_idx = view_idx
            if self._check_cancel("ACTIVE_SEARCH", room.name):
                task.status = "failed"
                task.result = {"reason": "cancelled"}
                return

            robot_pose = self._get_robot_pose(fallback=room.nav_pose)
            try:
                candidates = planner.generate_candidates(
                    room.search_area, robot_pose, self._get_obstacle_points())
                candidate = planner.select_next_best(candidates, robot_pose)
            except Exception as e:
                self._fail("invalid_search_area", room=room.name, msg=str(e))
                task.status = "failed"
                task.result = {"reason": "invalid_search_area", "error": str(e)}
                return
            if candidate is None:
                break

            progress = float(view_idx) / float(max(1, max_views))
            self._phase("NEXT_BEST_VIEW", progress=progress, room=room.name,
                        current_wp=view_idx, total_wp=max_views,
                        waypoint=(candidate["x"], candidate["y"]),
                        score=candidate.get("score"))
            result = nav.send_goal_and_wait(
                candidate["x"], candidate["y"], candidate.get("yaw", 0.0),
                frame_id=room_map.frame_id)
            if not result.get("ok"):
                reason = self._nav_failure_reason(result)
                if reason == "no_nav":
                    self._fail(reason, room=room.name, status=result.get("status"),
                               stage=f"viewpoint_{view_idx}")
                    task.status = "failed"
                    task.result = {"reason": reason, "raw": result}
                    return
                planner.mark_blocked(candidate)
                self._phase("NEXT_BEST_VIEW", progress=progress, room=room.name,
                            current_wp=view_idx, total_wp=max_views,
                            warning=f"viewpoint {view_idx} skipped ({reason})")
                continue

            visited += 1
            planner.mark_visited(candidate)
            observe_pose = self._get_robot_pose(fallback=candidate)
            self._phase("DETECT", progress=float(view_idx + 1) / float(max(1, max_views)),
                        room=room.name, current_wp=view_idx, total_wp=max_views)
            self._observe_people_at_viewpoint(
                store, room.name, view_idx, observe_pose, target_classes,
                require_photos=require_photos, use_lidar=use_lidar)
            self._broadcast_person_markers(mission_id, store.markers())

        markers = store.markers()
        self._broadcast_person_markers(mission_id, markers)
        self._phase("REPORT", progress=1.0, room=room.name,
                    targets_found=len(markers))
        self._finalize_report(task, mission_id, room, total_wp=max_views,
                              visited=visited, detections_log=markers,
                              start_time=start_time)

    def _product_search_available(self) -> bool:
        return all((
            ActiveSearchPlanner is not None,
            DetectionFrame is not None,
            LaserScanSnapshot is not None,
            localize_person_detection is not None,
            PersonMissionStore is not None,
        ))

    def _observe_people_at_viewpoint(self, store, room_name: str, view_idx: int,
                                     robot_pose, target_classes: List[str],
                                     require_photos: bool = True,
                                     use_lidar: bool = True) -> int:
        get_snapshot = getattr(self._ai, "get_person_detection_snapshot", None)
        if not callable(get_snapshot):
            return 0
        try:
            snapshot = get_snapshot() or {}
        except Exception as e:
            logger.warning(f"get_person_detection_snapshot failed: {e}")
            return 0

        detections = snapshot.get("detections") or []
        if not detections:
            return 0

        scan = self._laser_scan_snapshot()
        if scan is None:
            return 0

        frame = snapshot.get("frame")
        frame_width, frame_height = self._snapshot_frame_size(snapshot, frame)
        if frame_width <= 0 or frame_height <= 0:
            return 0

        frame_info = DetectionFrame(
            width=frame_width,
            height=frame_height,
            camera_hfov_rad=self._camera_hfov_rad(),
        )
        robot_x, robot_y, robot_yaw = robot_pose
        allowed = set(target_classes or ["person"])
        added = 0
        for det in detections:
            if det.get("class") != "person":
                continue
            if allowed and "person" not in allowed:
                continue
            try:
                localized = localize_person_detection(
                    det, frame_info, scan, robot_x, robot_y, robot_yaw)
            except Exception as e:
                logger.warning(f"localize_person_detection failed: {e}")
                continue
            if use_lidar and localized.get("position_quality") != "range_lidar":
                continue
            localized.update({
                "robot_x": round(float(robot_x), 3),
                "robot_y": round(float(robot_y), 3),
                "robot_yaw": round(float(robot_yaw), 3),
                "room": room_name,
                "wp_index": int(view_idx),
                "view_index": int(view_idx),
                "timestamp": time.time(),
            })
            try:
                store.add_observation(
                    localized, frame=frame if require_photos else None)
            except Exception as e:
                logger.warning(f"person observation storage failed: {e}")
                continue
            added += 1
        return added

    def _laser_scan_snapshot(self):
        if self._node is None or LaserScanSnapshot is None:
            return None
        get_scan = getattr(self._node, "get_scan_snapshot", None)
        if not callable(get_scan):
            return None
        try:
            data = get_scan() or {}
            ranges = list(data.get("ranges") or [])
            if not ranges:
                return None
            return LaserScanSnapshot(
                angle_min=float(data.get("angle_min", 0.0)),
                angle_increment=float(data.get("angle_increment", 0.0)),
                ranges=ranges,
                range_min=float(data.get("range_min", 0.15)),
                range_max=float(data.get("range_max", 10.0)),
            )
        except Exception as e:
            logger.warning(f"get_scan_snapshot failed: {e}")
            return None

    def _snapshot_frame_size(self, snapshot: dict, frame) -> tuple:
        width = self._positive_int(snapshot.get("frame_width"), 0)
        height = self._positive_int(snapshot.get("frame_height"), 0)
        if (width <= 0 or height <= 0) and frame is not None:
            try:
                shape = getattr(frame, "shape", None)
                if shape is not None and len(shape) >= 2:
                    height = int(shape[0])
                    width = int(shape[1])
            except Exception:
                pass
        return width, height

    def _broadcast_person_markers(self, mission_id: str, markers: List[dict]) -> None:
        marker_list = list(markers or [])
        with self._lock:
            self._person_markers = marker_list
            self._current_targets_found = len(marker_list)
        self._safe_broadcast({
            "type": "person_markers",
            "data": {
                "mission_id": mission_id,
                "markers": marker_list,
            },
        })

    def _get_obstacle_points(self) -> List[tuple]:
        try:
            if self._node is None:
                return []
            with getattr(self._node, "_lock", threading.Lock()):
                ranges = list(getattr(self._node, "_scan_ranges", []) or [])
                angle_min = float(getattr(self._node, "_scan_angle_min", 0.0))
                angle_increment = float(getattr(self._node, "_scan_angle_increment", 0.0))
                range_min = float(getattr(self._node, "_scan_range_min", 0.15) or 0.15)
                range_max = float(getattr(self._node, "_scan_range_max", 10.0) or 10.0)
                robot_x = float(getattr(self._node, "_odom_x", 0.0))
                robot_y = float(getattr(self._node, "_odom_y", 0.0))
                robot_yaw = float(getattr(self._node, "_imu_yaw", 0.0))
            if not ranges or angle_increment <= 0.0:
                return []
            stride = max(1, len(ranges) // 720)
            points = []
            for index in range(0, len(ranges), stride):
                try:
                    range_m = float(ranges[index])
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(range_m) or range_m < range_min or range_m > range_max:
                    continue
                angle = robot_yaw + angle_min + index * angle_increment
                points.append((
                    robot_x + range_m * math.cos(angle),
                    robot_y + range_m * math.sin(angle),
                ))
            return points
        except Exception:
            return []

    def _active_search_spacing(self, room: Room) -> float:
        return self._positive_float(room.search_area.get("spacing"), 1.0)

    def _camera_hfov_rad(self) -> float:
        try:
            hfov_deg = float(os.environ.get("GO2W_CAMERA_HFOV", "70"))
            if math.isfinite(hfov_deg) and hfov_deg > 0.0:
                return math.radians(hfov_deg)
        except (TypeError, ValueError):
            pass
        return math.radians(70.0)

    def _nav_failure_reason(self, result: dict) -> str:
        reason = (result or {}).get("reason", "nav_aborted")
        return "no_nav" if reason == "no_server" else reason

    def _positive_int(self, value, default: int) -> int:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            parsed = int(default)
        return parsed if parsed > 0 else int(default)

    def _positive_float(self, value, default: float) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = float(default)
        if not math.isfinite(parsed) or parsed <= 0.0:
            return float(default)
        return parsed

    def _plan_room_waypoints(self, room: Room) -> List[dict]:
        """用 plan_lawnmower/plan_spiral 把房间 search_area 切成航点序列。
        spec 决策: planner 已在 nx_web_server 内联, 从 nx_web_server import 复用 (避免三处复制)。
        本模块 import 失败时 (纯逻辑测试) 用内置 fallback (等价公式)。
        """
        sa = room.search_area
        pattern = sa.get("pattern", "lawnmower")
        try:
            # 从 nx_web_server import (懒导入, 避免循环 import 在模块顶层)
            import nx_web_server as _nws
            if pattern == "spiral":
                return _nws.plan_spiral(sa["width"], sa["height"],
                                        sa.get("spacing", 2.5),
                                        sa["origin_x"], sa["origin_y"])
            return _nws.plan_lawnmower(sa["width"], sa["height"],
                                       sa.get("spacing", 2.5),
                                       sa["origin_x"], sa["origin_y"])
        except Exception as e:
            logger.debug(f"nx_web_server planner 不可用 ({e}), 用内置 fallback")
            return _fallback_planner(sa["width"], sa["height"],
                                     sa.get("spacing", 2.5),
                                     sa["origin_x"], sa["origin_y"],
                                     pattern=pattern)

    def _get_robot_pose(self, fallback: Optional[dict] = None):
        """取狗当前位姿 (map 坐标)。优先读 nx_web 注入的 node 订阅缓存, 失败用 fallback (航点)。
        """
        try:
            if self._node is not None:
                with getattr(self._node, "_lock", threading.Lock()):
                    x = float(getattr(self._node, "_odom_x", 0.0))
                    y = float(getattr(self._node, "_odom_y", 0.0))
                    yaw = float(getattr(self._node, "_imu_yaw", 0.0))
                if x != 0.0 or y != 0.0:
                    return x, y, yaw
        except Exception:
            pass
        if fallback is not None:
            return float(fallback.get("x", 0.0)), float(fallback.get("y", 0.0)), float(fallback.get("yaw", 0.0))
        return 0.0, 0.0, 0.0

    # ---- 辅助 ----
    def _phase(self, phase: str, **extra) -> None:
        """推送状态机进度: ws_broadcast({"type":"search_room","data":{phase, ...}})。
        spec §5 实现要点 2: 每阶段切换 ws_broadcast。
        """
        with self._lock:
            data = {
                "mission_id": self._current_mission_id,
                "room": self._current_room_name,
                "phase": phase,
                "current_wp": self._current_wp_idx,
                "total_wp": self._current_total_wp,
                "targets_found": self._current_targets_found,
                "timestamp": time.time(),
            }
        # progress 可能是 None (NAVIGATING 用 distance_remaining 而非 0-1)
        if "progress" in extra and extra["progress"] is not None:
            data["progress"] = float(extra.pop("progress"))
        elif phase in ("SELECT_ROOM", "NAVIGATE"):
            data["progress"] = 0.0
        elif phase == "NAVIGATING":
            data["progress"] = 0.0  # 由 distance_remaining 表达
        elif phase == "ARRIVED":
            data["progress"] = 0.0
        elif phase == "REPORT" or phase == "DONE":
            data["progress"] = 1.0
        # 把额外字段灌进 data (distance_remaining / eta_sec / waypoint / info / warning)
        for k, v in extra.items():
            if k in ("room",):
                continue  # room 单独处理
            data[k] = v
        self._safe_broadcast({"type": "search_room", "data": data})

    def _fail(self, reason: str, room: Optional[str] = None, **extra) -> None:
        """推送失败: ws_broadcast type=search_room, phase:FAILED, reason。
        spec §5 / §11: 失败子态全覆盖。
        """
        with self._lock:
            data = {
                "mission_id": self._current_mission_id,
                "room": room if room is not None else self._current_room_name,
                "phase": "FAILED",
                "reason": reason,
                "current_wp": self._current_wp_idx,
                "total_wp": self._current_total_wp,
                "targets_found": self._current_targets_found,
                "timestamp": time.time(),
            }
        for k, v in extra.items():
            data[k] = v
        logger.warning(f"[{data.get('mission_id')}] 房间搜索 FAILED: reason={reason} {extra}")
        self._safe_broadcast({"type": "search_room", "data": data})

    def _check_cancel(self, stage: str, room_name: str) -> bool:
        """检查 cancel 标志。True=已取消 (调用方应 return)。"""
        with self._lock:
            cancelled = self._cancelled
        if cancelled:
            self._fail("cancelled", room=room_name, stage=stage)
        return cancelled

    def _snapshot_detections(self, room: Room, target_classes: List[str],
                              robot_x: float, robot_y: float, robot_yaw: float,
                              found_list: List[str], detections_log: List[dict],
                              wp_index: int) -> None:
        """读阶段B ai_engine 最新检测快照, 过滤 target_classes, 去重加进 found_list/detections_log,
        增量推 type=search (复用阶段B 格式 {"found":[...]}), 记录发现位姿 (robot_x/y/yaw + 时间戳)。
        ai_engine=None (阶段A 退化) 时跳过检测, found_list 保持空。

        spec §10 实现要点: 不调 detector.detect (重复推理), 只读 get_detections_world 快照。
        spec §11 边界: target_classes 过滤后无匹配 → detections 空 (不算失败)。
        """
        if self._ai is None:
            return
        get_dets = getattr(self._ai, "get_detections_world", None)
        if get_dets is None:
            return
        try:
            dets = get_dets(robot_x, robot_y, robot_yaw) or []
        except Exception as e:
            logger.warning(f"读检测快照异常: {e}")
            return
        if not dets:
            return

        # 取置信度: ai_engine 的快照可能不含 confidence, 尝试读 _latest_dets
        conf_map = {}
        try:
            with getattr(self._ai, "_lock", threading.Lock()):
                raw = list(getattr(self._ai, "_latest_dets", []) or [])
            for d in raw:
                cls = d.get("class") or d.get("class_name") or "?"
                conf_map[cls] = float(d.get("confidence", d.get("score", 0.0)) or 0.0)
        except Exception:
            pass

        new_found = False
        ts = time.time()
        for d in dets:
            cls = d.get("class", "?")
            # target_classes 过滤 (room/task 级)
            if target_classes and cls not in target_classes:
                continue
            conf = conf_map.get(cls, 0.0)
            # 去重标签 (与阶段A _execute_search 一致: "class(xx%)")
            tag = f"{cls}({conf:.0%})" if conf > 0 else cls
            # 记录发现 (detections_log 不去重, found_list 去重)
            det_entry = {
                "class": cls,
                "confidence": round(conf, 3),
                "robot_x": round(robot_x, 2),
                "robot_y": round(robot_y, 2),
                "robot_yaw": round(robot_yaw, 3),
                "world_x": d.get("x"),
                "world_y": d.get("y"),
                "timestamp": ts,
                "wp_index": wp_index,
            }
            detections_log.append(det_entry)
            if tag not in found_list:
                found_list.append(tag)
                new_found = True
            logger.info(f"检测发现: {tag} @ ({robot_x:.2f},{robot_y:.2f}) wp={wp_index}")

        # 增量推 type=search (复用阶段B 格式)
        if new_found and found_list:
            self._safe_broadcast({"type": "search",
                                  "data": {"found": list(found_list)}})

    def _finalize_report(self, task, mission_id: str, room: Room,
                         total_wp: int, visited: int,
                         detections_log: List[dict], start_time: float) -> None:
        """REPORT 阶段: 生成 MissionReport + 推 type=mission_report + 标 task 完成。"""
        end_time = time.time()
        report = build_mission_report(
            mission_id=mission_id, room=room,
            waypoints_total=total_wp, waypoints_visited=visited,
            detections=detections_log, start_time=start_time, end_time=end_time)
        self._safe_broadcast({"type": "mission_report", "data": report})
        self._phase("DONE", progress=1.0, room=room.name,
                    targets_found=len(detections_log))
        task.status = "completed"
        task.result = report
        logger.info(f"[{mission_id}] 房间 '{room.name}' 搜索完成: "
                    f"wp={visited}/{total_wp} targets={len(detections_log)} "
                    f"dur={report['duration_sec']}s")

    def _safe_broadcast(self, data: dict) -> None:
        """转发到 ws_broadcast, 容错 (注入未完成时仅日志)。"""
        if self._ws is None:
            logger.debug(f"broadcast 跳过 (ws 未注入): {data.get('type')}")
            return
        try:
            self._ws(data)
        except Exception as e:
            logger.debug(f"ws_broadcast 异常: {e}")

    def cancel(self) -> None:
        """cancel_all / e_stop 时调: 标 _cancelled + nav.cancel_current()。
        spec §11 / §5 cancel 响应: worker 在阶段切换点检查 _cancelled 退出。
        """
        with self._lock:
            self._cancelled = True
        nav = self._nav
        if nav is not None:
            try:
                nav.cancel_current()
            except Exception as e:
                logger.warning(f"nav.cancel_current 异常: {e}")
        logger.info(f"RoomSearchOrchestrator.cancel 已调 (mission={self._current_mission_id})")


# ============================================================================
# 内置 fallback planner (纯逻辑测试时 nx_web_server 不可用, 公式等价 plan_lawnmower)
# ============================================================================
def _fallback_planner(width: float, height: float, spacing: float = 2.5,
                      origin_x: float = 0.0, origin_y: float = 0.0,
                      pattern: str = "lawnmower") -> List[dict]:
    """与 nx_web_server.plan_lawnmower/plan_spiral 公式等价的 fallback。
    仅在 nx_web_server import 失败时用 (纯逻辑测试场景)。
    """
    if spacing <= 0:
        spacing = 2.5
    if pattern == "spiral":
        cx = origin_x + width / 2.0
        cy = origin_y + height / 2.0
        max_radius = math.sqrt(width ** 2 + height ** 2) / 2.0
        num_turns = max(3, math.ceil(max_radius / spacing))
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
    # lawnmower (默认)
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
