#!/usr/bin/env python3
"""Go2W 阶段E — 房间级搜索编排器 (spec-stage-e §7.1)。

== 职责 ==
作为"组件"注入 nx_web_server.py 的 TaskManager (同进程, spec 决策 3 方案 b):
  1. RoomMap: 加载静态 YAML 房间地图 (config/rooms.yaml), 校验 schema + 房间匹配
  2. RoomSearchOrchestrator: 状态机驱动 (SELECT_ROOM→NAVIGATE→ARRIVED→SEARCH→DETECT→REPORT)
     每阶段切换 ws_broadcast type=search_room; 完成推 type=mission_report

== 线程模型 (spec 决策 4) ==
  - 主线程: HTTPServer.serve_forever
  - 线程1 (daemon): rclpy.spin(NxWebNode)         ← 主 spin (驱动订阅回调)
  - 线程X (TaskManager worker): 执行 search_room 时
       └─ RoomSearchOrchestrator.run(task)
            └─ 注入的 MissionNavigationPort → 进程唯一 NavigationGateway
  关键: action late acceptance / cancel / terminal ownership 由共享 gateway 保留
  关键: 编排器不创建 ROS action client，也不执行自己的 executor

== 红线 (spec §0 + §12 反模式) ==
  - 懒加载: __init__ 不加载 YAML；NavigationGateway 必须由主进程注入
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

from nx_mission_schema import MissionValidationError, SearchMissionRequest
from nx_camera_calibration import resolve_camera_calibration
from nx_exploration_manager import ExplorationManager
from nx_frontier_planner import (
    find_frontier_clusters as _planner_find_frontier_clusters,
    select_frontier_candidates as _planner_select_frontier_candidates,
)
from typing import Optional, List, Dict

logger = logging.getLogger("go2w.room_orch")

try:
    from nx_active_search import ActiveSearchPlanner
except Exception:
    ActiveSearchPlanner = None

try:
    from nx_coverage_metrics import compute_coverage as _compute_coverage
except Exception:
    _compute_coverage = None

try:
    from nx_person_localizer import (
        DetectionFrame,
        LaserScanSnapshot,
        PointCloudSnapshot,
        localize_person_detection,
        localize_target_detection,
    )
except Exception:
    DetectionFrame = None
    LaserScanSnapshot = None
    PointCloudSnapshot = None
    localize_person_detection = None
    localize_target_detection = None

try:
    from nx_person_mission import (
        PersonMissionStore,
        TargetMissionStore,
        load_latest_mission_report,
    )
except Exception:
    PersonMissionStore = None
    TargetMissionStore = None
    load_latest_mission_report = None

try:
    from nx_product_command import resolve_current_room
except Exception:
    resolve_current_room = None

# web/ 目录 (与 nx_web_server.py 同目录, 复用其内联的 plan_lawnmower/plan_spiral)
_WEB_DIR = os.path.dirname(os.path.abspath(__file__))

# rclpy/nav_msgs 接口 (懒导入, Windows 开发机无 rclpy 时也能静态检查)
_rclpy = None
_OccupancyGrid = None  # nav_msgs/OccupancyGrid (frontier 探索订阅 /map_frontier 用)
_QoSProfile = None
_ReliabilityPolicy = None
_DurabilityPolicy = None
_HistoryPolicy = None


def _import_ros():
    """懒导入 frontier 订阅所需的 rclpy、nav_msgs 和 QoS 类型。

    Nav2 action 客户端由主进程的 NavigationGateway 创建，编排器不导入或
    构造第二个客户端。
    """
    global _rclpy, _OccupancyGrid
    global _QoSProfile, _ReliabilityPolicy, _DurabilityPolicy, _HistoryPolicy
    if _rclpy is not None:
        return True
    # 必需: rclpy + nav_msgs (frontier 订阅 /map_frontier 用 OccupancyGrid)
    try:
        import rclpy as _r
        from rclpy.qos import (
            DurabilityPolicy as _DP,
            HistoryPolicy as _HP,
            QoSProfile as _QP,
            ReliabilityPolicy as _RP,
        )
        from nav_msgs.msg import OccupancyGrid as _OG
        _rclpy = _r
        _OccupancyGrid = _OG
        _QoSProfile = _QP
        _ReliabilityPolicy = _RP
        _DurabilityPolicy = _DP
        _HistoryPolicy = _HP
    except Exception as e:
        logger.debug(f"rclpy/nav_msgs 不可导入 (NX 部署外正常): {e}")
        return False
    return True


def _frontier_map_qos():
    """Match the padded map publisher's reliable/transient-local contract."""
    if not all((_QoSProfile, _ReliabilityPolicy,
                _DurabilityPolicy, _HistoryPolicy)):
        return 10
    return _QoSProfile(
        history=_HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=_ReliabilityPolicy.RELIABLE,
        durability=_DurabilityPolicy.TRANSIENT_LOCAL,
    )


# Runtime authority lives in the pure planner module.  The names remain
# exported for compatibility with existing offline callers.
_find_frontier_clusters = _planner_find_frontier_clusters


def select_frontier_candidates(map_msg, robot_pose, visited, **kwargs):
    kwargs.setdefault("cluster_finder", _find_frontier_clusters)
    return _planner_select_frontier_candidates(
        map_msg, robot_pose, visited, **kwargs)


def select_next_frontier(map_msg, robot_pose, visited, **kwargs):
    """Choose (a) nearest/(b) gain candidates using (c) cost-distance."""
    candidates = select_frontier_candidates(
        map_msg, robot_pose, visited, **kwargs)
    return candidates[0] if candidates else None


# ============================================================================
# Room / RoomMap — 房间地图 (YAML 加载, spec §6)
# ============================================================================
class Room:
    """单个房间定义 (从 rooms.yaml 一条 room 反序列化)。"""

    def __init__(self, name: str, aliases: List[str], nav_pose: dict,
                 search_area: dict, target_classes: Optional[List[str]] = None,
                 calibrated: bool = False):
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
        self.calibrated = bool(calibrated)

    def to_dict(self) -> dict:
        """序列化 (供 /api/rooms 或 mission_report.area 用)。"""
        return {
            "name": self.name,
            "aliases": list(self.aliases),
            "nav_pose": dict(self.nav_pose),
            "search_area": dict(self.search_area),
            "target_classes": list(self.target_classes),
            "calibrated": self.calibrated,
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
        calibrated = d.get("calibrated", False)
        if not isinstance(calibrated, bool):
            raise ValueError(f"room[{name}].calibrated 必须是 bool")

        return cls(name, aliases, nav_pose, search_area, target_classes,
                   calibrated=calibrated)


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
                 rooms_yaml_path: Optional[str] = None,
                 *, navigation_port=None,
                 observation_sync=None,
                 mission_root=None,
                 _test_allow_uncalibrated_rooms: bool = False):
        self._node = node                  # NxWebNode (只用于订阅/快照，不创建 action client)
        self._ai = ai_engine               # NxAiEngine (阶段B), 读 get_detections_world
        self._ws = ws_broadcast_fn         # ws_broadcast 注入 (与 nx_ai_node 同款)
        self._lock = threading.Lock()
        # 房间地图 (懒加载, 首次 run 时 load)
        self._rooms_yaml = rooms_yaml_path or os.environ.get(
            'GO2W_ROOMS_YAML', _DEFAULT_ROOMS_YAML)
        self._room_map: Optional[RoomMap] = None
        self._room_map_err: Optional[str] = None  # 加载失败原因 (供 /api/rooms 显示)
        # Nav2 client (懒创建, 首次 run 时)
        self._nav = navigation_port
        self._observation_sync = observation_sync
        # 当前 mission 状态 (cancel 检查 + 推进度用)
        self._current_mission_id: Optional[str] = None
        self._current_room_name: Optional[str] = None
        self._current_total_wp: int = 0
        self._current_wp_idx: int = 0
        self._current_targets_found: int = 0
        self._person_markers: List[dict] = []
        self._mission_state: Dict = {"phase": "idle"}
        configured_mission_root = (
            mission_root
            if mission_root is not None
            else os.environ.get("GO2W_MISSION_ROOT")
        )
        self._mission_root = (
            os.path.abspath(os.path.expanduser(str(configured_mission_root)))
            if str(configured_mission_root or "").strip()
            else None
        )
        self._last_report: Optional[Dict] = None
        self._static_root = os.path.join(_WEB_DIR, "static")
        if self._mission_root and callable(load_latest_mission_report):
            restored_report = load_latest_mission_report(self._mission_root)
            if restored_report is not None:
                restored_markers = restored_report.get("detections") or []
                self._last_report = dict(restored_report)
                self._person_markers = [
                    dict(marker) for marker in restored_markers
                    if isinstance(marker, dict)
                ]
                self._current_targets_found = len(self._person_markers)
        self._cancelled = False
        self._test_allow_uncalibrated_rooms = bool(
            _test_allow_uncalibrated_rooms)

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

    # ---- Shared Nav2 gateway facade ----
    def _ensure_nav(self):
        """Return the injected MissionNavigationPort; never create a client."""
        if self._nav is not None:
            return self._nav
        logger.error("Nav2 shared navigation gateway was not injected")
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
        mission_payload = params.get("mission_request")
        if mission_payload is not None:
            try:
                mission_request = SearchMissionRequest.from_dict(
                    mission_payload)
            except MissionValidationError as exc:
                self._fail("invalid_mission_request", room=None, msg=str(exc))
                task.status = "failed"
                task.result = {
                    "reason": "invalid_mission_request", "message": str(exc)}
                return
            extras = {
                key: params[key] for key in (
                    "max_frontiers", "max_frontier_plan_probes",
                    "max_frontier_rejections", "frontier_planning_timeout",
                    "max_plan_probes_per_cycle", "initial_radius_m",
                    "radius_step_m", "tile_size_m",
                    "stable_exhaustion_cycles",
                ) if key in params
            }
            params = mission_request.to_task_params()
            params.update(extras)
            task.params = params
        else:
            mission_request = None
        room_query = params.get("room", "")
        is_product_search = params.get("search_strategy") == "next_best_view"
        # 任务级 target_classes 覆盖 (room.target_classes 优先, 任务级次之)
        task_target_classes = params.get("target_classes", []) or []

        mission_id = (
            mission_request.request_id if mission_request is not None
            else uuid.uuid4().hex[:8])
        start_time = time.time()
        with self._lock:
            if getattr(task, "status", None) == "cancelled":
                self._cancelled = True
                return
            self._cancelled = False
            self._current_mission_id = mission_id
            self._current_room_name = room_query
            self._current_total_wp = 0
            self._current_wp_idx = 0
            self._current_targets_found = 0

        # ---- frontier 探索: 无预建图模式, 绕过 RoomMap/SELECT_ROOM ----
        # plan 2026-07-03 §3.3.3: 分发前移到 run() 入口, 不进 SELECT_ROOM/RoomMap 路径
        if params.get("search_strategy") == "frontier_explore":
            target_classes = self._normalize_target_classes(
                task_target_classes)
            target_scope = self._activate_detection_targets(target_classes)
            try:
                self._run_frontier_explore(task, mission_id, start_time, params)
            finally:
                self._restore_detection_targets(target_scope)
            return

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
                if is_product_search:
                    live_pose = self._get_live_robot_pose()
                    if live_pose is None:
                        self._fail("no_pose", room=room_query,
                                   msg="live robot pose required to resolve current room")
                        task.status = "failed"
                        task.result = {"reason": "no_pose", "query": room_query}
                        return
                    robot_x, robot_y, _ = live_pose
                else:
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
        if not room.calibrated and not self._test_allow_uncalibrated_rooms:
            self._fail(
                "room_uncalibrated",
                room=room.name,
                msg="calibrate nav_pose/search_area and set calibrated: true",
            )
            task.status = "failed"
            task.result = {"reason": "room_uncalibrated", "room": room.name}
            return
        with self._lock:
            self._current_room_name = room.name
        self._phase("SELECT_ROOM", progress=0.1, room=room.name,
                    info=f"匹配到房间 '{room.name}'")

        # 决定 target_classes: room 优先, 任务级次之, 空则全记
        target_classes = (
            task_target_classes if task_target_classes else room.target_classes)
        target_classes = self._normalize_target_classes(target_classes)
        if params.get("search_strategy") == "next_best_view":
            target_scope = self._activate_detection_targets(target_classes)
            try:
                self._run_product_person_search(
                    task, mission_id, start_time, room_map, room,
                    target_classes, params)
            finally:
                self._restore_detection_targets(target_scope)
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
            # 取狗当前位姿 (map 坐标) — 优先用 /localization_pose, 失败再用航点 fallback
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

        target_classes = self._normalize_target_classes(target_classes)
        get_snapshot = self._detection_snapshot_getter(target_classes)
        if self._ai is None or get_snapshot is None:
            self._fail("no_yolo", room=room.name)
            task.status = "failed"
            task.result = {"reason": "no_yolo"}
            return

        lidar_range_setting = params.get(
            "use_lidar_target_range",
            params.get("use_lidar_person_range"),
        )
        if self._param_explicitly_false(lidar_range_setting):
            self._fail("lidar_required", room=room.name)
            task.status = "failed"
            task.result = {"reason": "lidar_required"}
            return

        if self._laser_scan_snapshot() is None:
            self._fail("no_scan", room=room.name)
            task.status = "failed"
            task.result = {"reason": "no_scan"}
            return

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
                    params.get("obstacle_clearance_m"), 0.5),
                visual_range_m=self._positive_float(
                    params.get("visual_range_m"), 2.5))
        except Exception as e:
            self._fail("invalid_search_area", room=room.name, msg=str(e))
            task.status = "failed"
            task.result = {"reason": "invalid_search_area", "error": str(e)}
            return

        store = self._new_target_store(mission_id, target_classes[0])
        visited = 0
        require_photos = bool(params.get("require_photos", False))
        last_viewpoint_failure = None
        coverage_threshold = self._coverage_threshold(
            params.get("coverage_threshold"), 0.9)

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
                        score=candidate.get("score"),
                        room_area=dict(room.search_area),
                        candidate_viewpoints=[
                            {"x": item["x"], "y": item["y"]}
                            for item in candidates[:200]
                        ],
                        **planner.coverage_state())
            result = nav.send_goal_and_wait(
                candidate["x"], candidate["y"], candidate.get("yaw", 0.0),
                frame_id=room_map.frame_id)
            if not result.get("ok"):
                reason = self._nav_failure_reason(result)
                last_viewpoint_failure = reason
                if reason in ("no_nav", "cancelled"):
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
            observe_pose = self._get_live_robot_pose()
            if observe_pose is None:
                self._fail("no_pose", room=room.name, stage=f"viewpoint_{view_idx}",
                           msg="live robot pose required for person map coordinates")
                task.status = "failed"
                task.result = {"reason": "no_pose"}
                return
            self._phase("DETECT", progress=float(view_idx + 1) / float(max(1, max_views)),
                        room=room.name, current_wp=view_idx, total_wp=max_views)
            unresolved_before_observation = len(store.unresolved())
            observed = self._observe_people_at_viewpoint(
                store, room.name, view_idx, observe_pose, target_classes,
                require_photos=require_photos, use_lidar=True)
            if observed is None:
                self._fail("no_scan", room=room.name, stage=f"viewpoint_{view_idx}")
                task.status = "failed"
                task.result = {"reason": "no_scan"}
                return
            observation_warning = None
            if isinstance(observed, dict) and observed.get("reason"):
                reason = str(observed.get("reason"))
                if reason == "no_lidar_range":
                    resolved_count = self._positive_int(
                        observed.get("resolved_count"), 0)
                    store.resolve_unresolved(min(
                        resolved_count, unresolved_before_observation))
                    observation_warning = reason
                else:
                    self._fail(reason, room=room.name, stage=f"viewpoint_{view_idx}",
                               detections=observed.get("detections"))
                    task.status = "failed"
                    task.result = observed
                    return
            elif isinstance(observed, dict):
                resolved_count = self._positive_int(
                    observed.get("resolved_count"), 0)
                store.resolve_unresolved(min(
                    resolved_count, unresolved_before_observation))
            elif isinstance(observed, int) and observed > 0:
                store.resolve_unresolved(min(
                    observed, unresolved_before_observation))
            observation_source = (
                observed.get("source") if isinstance(observed, dict) else None)
            observation_valid = (
                bool(observed.get("observation_valid", True))
                if isinstance(observed, dict) else True
            )
            planner.mark_visited(
                candidate,
                camera_hfov_rad=self._camera_hfov_rad(observation_source),
                camera_yaw_offset_rad=self._camera_yaw_offset_rad(
                    observation_source),
                observation_valid=observation_valid,
            )
            coverage = planner.coverage_state()
            self._phase(
                "ACTIVE_SEARCH",
                progress=coverage["coverage_ratio"],
                room=room.name,
                current_wp=view_idx,
                total_wp=max_views,
                room_area=dict(room.search_area),
                coverage_threshold=coverage_threshold,
                warning=observation_warning,
                **coverage,
            )
            self._broadcast_person_markers(mission_id, store.markers())
            if (
                coverage["coverage_ratio"] >= coverage_threshold
                and not store.unresolved()
            ):
                break

        if visited == 0:
            reason = "no_viewpoint_reached"
            self._fail(reason, room=room.name, last_nav_failure=last_viewpoint_failure)
            task.status = "failed"
            task.result = {"reason": reason, "last_nav_failure": last_viewpoint_failure}
            return

        markers = store.markers()
        unresolved = store.unresolved()
        coverage = planner.coverage_state()
        self._broadcast_person_markers(mission_id, markers)
        if unresolved:
            reason = "no_lidar_range"
            self._fail(
                reason,
                room=room.name,
                detections=unresolved,
                coverage_ratio=coverage["coverage_ratio"],
            )
            task.status = "failed"
            task.result = {
                "reason": reason,
                "detections": unresolved,
                "resolved_detections": markers,
                "coverage_ratio": coverage["coverage_ratio"],
            }
            return
        if coverage["coverage_ratio"] < coverage_threshold:
            reason = "coverage_incomplete"
            self._fail(
                reason,
                room=room.name,
                coverage_ratio=coverage["coverage_ratio"],
                coverage_threshold=coverage_threshold,
                observed_cells=coverage["observed_cells"],
                total_cells=coverage["total_cells"],
            )
            task.status = "failed"
            task.result = {
                "reason": reason,
                "coverage_complete": False,
                "coverage_ratio": coverage["coverage_ratio"],
                "coverage_threshold": coverage_threshold,
                "observed_cells": coverage["observed_cells"],
                "total_cells": coverage["total_cells"],
                "detections": markers,
            }
            return
        self._phase("REPORT", progress=1.0, room=room.name,
                    targets_found=len(markers))
        self._finalize_report(task, mission_id, room, total_wp=max_views,
                              visited=visited, detections_log=markers,
                              start_time=start_time,
                              extra_result={
                                  "coverage_complete": True,
                                  "coverage_ratio": coverage["coverage_ratio"],
                                  "coverage_threshold": coverage_threshold,
                                  "observed_cells": coverage["observed_cells"],
                                  "total_cells": coverage["total_cells"],
                                  "visited_viewpoints": coverage["visited_viewpoints"],
                                  "visual_range_m": coverage["visual_range_m"],
                              })

    def _make_sentry_room(self, name: str = "__frontier__") -> Room:
        """构造最小哨兵 Room 供 frontier 探索 _finalize_report 用 (无真实 rooms.yaml 房间)。
        build_mission_report 已有 room is not None 守卫, search_area 放空 dict 对前端无意义。
        绕过 Room.__init__ (它强校验 search_area.width 等必填字段, frontier 无意义),
        直接设最小属性: build_mission_report 只读 room.name / room.search_area。
        """
        room = Room.__new__(Room)
        room.name = name
        room.aliases = []
        room.nav_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        room.search_area = {}
        room.target_classes = []
        room.calibrated = True
        return room

    def _run_frontier_explore(self, task, mission_id: str, start_time: float,
                              params: dict) -> None:
        """无预建图 frontier 探索搜人状态机 (plan 2026-07-03 §3.3.4)。

        状态流: INIT_SLAM → (FRONTIER_DETECT → NAVIGATING → DETECT)* → REPORT
        复用 _phase/_fail/_check_cancel/_finalize_report/_observe_people_at_viewpoint/
        _broadcast_person_markers/_get_live_robot_pose/_laser_scan_snapshot/共享导航端口。
        不依赖 RoomMap / rooms.yaml; room 名固定 "__frontier__" (哨兵 Room)。
        与 _run_product_person_search 同构, 便于前端复用 ws type=search_room 渲染。
        """
        # ---- 前置检查 ----
        # frontier 路径不走 _ensure_nav (NAVIGATE), 必须主动调 _import_ros 确保
        # rclpy + nav_msgs 加载; 否则 _OccupancyGrid 永远是初始 None (即使 NX 上
        # nav_msgs 可用), frontier_unavailable 误报 "OccupancyGrid type not loaded"。
        # 仅在 _OccupancyGrid is None 时调用 (测试 stub 后跳过, 避免重复 import)。
        if _OccupancyGrid is None and not _import_ros():
            self._fail("frontier_unavailable", room="__frontier__",
                       msg="rclpy/nav_msgs 不可导入 (NX 部署外正常)")
            task.status = "failed"
            task.result = {"reason": "frontier_unavailable"}
            return
        if not self._product_search_available():
            self._fail("frontier_unavailable", room="__frontier__")
            task.status = "failed"
            task.result = {"reason": "frontier_unavailable"}
            return
        if _OccupancyGrid is None:
            # _import_ros 已调但仍 None (理论不应发生; 防御性兜底)
            self._fail("frontier_unavailable", room="__frontier__",
                       msg="OccupancyGrid type not loaded (rclpy/nav_msgs missing)")
            task.status = "failed"
            task.result = {"reason": "frontier_unavailable"}
            return

        target_classes = self._normalize_target_classes(
            params.get("target_classes", []))
        get_snapshot = self._detection_snapshot_getter(target_classes)
        if self._ai is None or get_snapshot is None:
            self._fail("no_yolo", room="__frontier__")
            task.status = "failed"
            task.result = {"reason": "no_yolo"}
            return

        lidar_range_setting = params.get(
            "use_lidar_target_range",
            params.get("use_lidar_person_range"),
        )
        if self._param_explicitly_false(lidar_range_setting):
            self._fail("lidar_required", room="__frontier__")
            task.status = "failed"
            task.result = {"reason": "lidar_required"}
            return

        if self._laser_scan_snapshot() is None:
            self._fail("no_scan", room="__frontier__")
            task.status = "failed"
            task.result = {"reason": "no_scan"}
            return

        with self._lock:
            self._current_room_name = "__frontier__"
            self._person_markers = []

        # ---- INIT_SLAM: 等 /map_frontier 首帧 ----
        self._phase("INIT_SLAM", progress=0.0, room="__frontier__")
        if self._check_cancel("INIT_SLAM", "__frontier__"):
            task.status = "failed"
            task.result = {"reason": "cancelled"}
            return

        map_received = threading.Event()
        latest_map_box = [None]  # [0] = latest OccupancyGrid msg (回调写, 主循环读)
        map_lock = threading.Lock()
        sub_handle = None

        def _on_map_frontier(msg):
            with map_lock:
                latest_map_box[0] = msg
            map_received.set()

        try:
            # 创建订阅 (NxWebNode 上挂; 主 spin 线程驱动回调)
            sub_handle = self._node.create_subscription(
                _OccupancyGrid, "/map_frontier", _on_map_frontier,
                _frontier_map_qos())

            # 等首帧 (超时 10s → no_map)
            if not map_received.wait(timeout=10.0):
                self._fail("no_map", room="__frontier__",
                           msg="timeout waiting for /map_frontier first frame")
                task.status = "failed"
                task.result = {"reason": "no_map"}
                return

            # ensure Nav2 + wait_for_server
            nav = self._ensure_nav()
            if nav is None or not nav.wait_for_server(timeout=2.0):
                self._fail("no_nav", room="__frontier__")
                task.status = "failed"
                task.result = {"reason": "no_nav"}
                return
            wait_for_planner = getattr(nav, "wait_for_planner", None)
            compute_path = getattr(nav, "compute_path_to_pose", None)
            if (not callable(wait_for_planner) or not callable(compute_path)
                    or not wait_for_planner(timeout=2.0)):
                self._fail("no_planner", room="__frontier__")
                task.status = "failed"
                task.result = {"reason": "no_planner"}
                return

            sentry_room = self._make_sentry_room("__frontier__")

            # ---- 循环: frontier 驱动 Nav2 走 → 每停留点 DETECT ----
            max_frontiers = self._positive_int(params.get("max_frontiers"), 200)
            max_time = self._positive_float(params.get("max_time"), 1800.0)
            radius_value = params.get("max_radius_m")
            max_radius = (
                None if radius_value is None
                else self._positive_float(radius_value, 6.0))
            mission_origin = self._get_live_robot_pose()
            if mission_origin is None:
                self._fail("no_pose", room="__frontier__",
                           stage="mission_origin",
                           msg="live robot pose required to bound current-room search")
                task.status = "failed"
                task.result = {"reason": "no_pose"}
                return
            with self._lock:
                self._current_total_wp = max_frontiers

            store = self._new_target_store(mission_id, target_classes[0])
            require_photos = bool(params.get("require_photos", False))
            max_frontier_rejections = min(
                5,
                self._positive_int(params.get("max_frontier_rejections"), 2),
            )
            planning_timeout = self._positive_float(
                params.get("frontier_planning_timeout"), 3.0)
            max_plan_probes = self._positive_int(
                params.get(
                    "max_plan_probes_per_cycle",
                    params.get("max_frontier_plan_probes")),
                12,
            )
            initial_radius = self._positive_float(
                params.get("initial_radius_m"), 6.0)
            radius_step = self._positive_float(
                params.get("radius_step_m"), 6.0)
            tile_size = self._positive_float(
                params.get("tile_size_m"), 6.0)
            stable_exhaustion_cycles = self._positive_int(
                params.get("stable_exhaustion_cycles"), 3)
            mission_deadline = start_time + max_time
            canonical_request = params.get("mission_request") or {}
            exploration_mode = (
                "current_room"
                if canonical_request.get("room") == "current_room"
                or params.get("room") in {"__current__", "current_room"}
                else "whole_floor"
            )
            exploration = ExplorationManager(
                navigation_port=nav,
                mission_origin=mission_origin,
                observation_sync=self._observation_sync,
                mode=exploration_mode,
                room_radius_m=max_radius,
                room_polygon=params.get("room_polygon"),
                initial_radius_m=initial_radius,
                radius_step_m=radius_step,
                tile_size_m=tile_size,
                stable_exhaustion_cycles=stable_exhaustion_cycles,
                max_time_s=max_time,
                max_distance_m=params.get("max_distance_m"),
                battery_reserve_percent=self._positive_float(
                    params.get("battery_reserve_percent"), 20.0),
                max_failures_per_cell=max_frontier_rejections,
                max_blacklist_entries=self._positive_int(
                    params.get("max_frontier_blacklist"), 256),
                reject_map_edge=bool(params.get(
                    "reject_rolling_map_edge", True)),
                planning_timeout_s=planning_timeout,
                max_plan_probes=max_plan_probes,
                candidate_selector=lambda *args, **kwargs: (
                    select_frontier_candidates(*args, **kwargs)),
            )
            nav_attempts = 0
            waypoints_reached = 0
            completion_reason = "waypoint_budget_exhausted"

            # Observe at the start pose before asking for a frontier. A fully
            # mapped room can legitimately have zero frontiers while a person
            # is already visible from the command location.
            self._phase("DETECT", progress=0.0, room="__frontier__",
                        current_wp=0, total_wp=max_frontiers,
                        info="initial_viewpoint")
            unresolved_before_observation = len(store.unresolved())
            initial_observed = self._observe_people_at_viewpoint(
                store, "__frontier__", -1, mission_origin,
                target_classes, require_photos=require_photos,
                use_lidar=True)
            if initial_observed is None:
                self._fail("no_scan", room="__frontier__", stage="initial_viewpoint")
                task.status = "failed"
                task.result = {"reason": "no_scan"}
                return
            if (isinstance(initial_observed, dict)
                    and initial_observed.get("reason")):
                reason = str(initial_observed.get("reason"))
                if reason == "no_lidar_range":
                    resolved_count = self._positive_int(
                        initial_observed.get("resolved_count"), 0)
                    store.resolve_unresolved(min(
                        resolved_count, unresolved_before_observation))
                    self._phase(
                        "FRONTIER_DETECT", progress=0.0,
                        room="__frontier__", current_wp=0,
                        total_wp=max_frontiers,
                        warning="person seen without reliable lidar range; "
                                "continuing from another viewpoint",
                    )
                else:
                    self._fail(reason, room="__frontier__",
                               stage="initial_viewpoint",
                               detections=initial_observed.get("detections"))
                    task.status = "failed"
                    task.result = initial_observed
                    return
            elif isinstance(initial_observed, dict):
                resolved_count = self._positive_int(
                    initial_observed.get("resolved_count"), 0)
                store.resolve_unresolved(min(
                    resolved_count, unresolved_before_observation))
            elif isinstance(initial_observed, int) and initial_observed > 0:
                store.resolve_unresolved(min(
                    initial_observed, unresolved_before_observation))
            self._broadcast_person_markers(mission_id, store.markers())

            while nav_attempts < max_frontiers:
                iteration = nav_attempts
                with self._lock:
                    self._current_wp_idx = iteration

                if time.time() >= mission_deadline:
                    completion_reason = "time_budget_exhausted"
                    break
                if self._check_cancel("FRONTIER_DETECT", "__frontier__"):
                    task.status = "failed"
                    task.result = {"reason": "cancelled"}
                    return

                # 读最新 map 缓存
                with map_lock:
                    map_msg = latest_map_box[0]
                if map_msg is None:
                    break  # 无 map (不应发生, 首帧已收), 安全退出

                robot_pose = self._get_live_robot_pose()
                if robot_pose is None:
                    self._fail("no_pose", room="__frontier__",
                               stage=f"frontier_{iteration}",
                               msg="live robot pose required for frontier explore")
                    task.status = "failed"
                    task.result = {"reason": "no_pose"}
                    return

                progress = float(iteration) / float(max(1, max_frontiers))
                self._phase(
                    "FRONTIER_DETECT", progress=progress,
                    room="__frontier__", current_wp=iteration,
                    total_wp=max_frontiers, info="reachability_preflight")
                target = exploration.choose_next(map_msg, robot_pose)
                if target is None:
                    selection_reason = exploration.snapshot().get(
                        "last_selection_reason")
                    if selection_reason in {
                            "retry_pending",
                            "search_boundary_expanded",
                            "tile_transition_pending",
                            "stability_confirmation_pending"}:
                        continue
                    completion_reason = {
                        "information_gain_exhausted":
                            "reachable_frontiers_exhausted",
                        "reachable_frontiers_exhausted":
                            "reachable_frontiers_exhausted",
                    }.get(selection_reason, selection_reason or
                          "reachable_frontiers_exhausted")
                    break

                # NAVIGATING: 发 Nav2 goal
                self._phase("NAVIGATING", progress=progress,
                            room="__frontier__", current_wp=iteration,
                            total_wp=max_frontiers)
                if self._check_cancel("NAVIGATING", "__frontier__"):
                    task.status = "failed"
                    task.result = {"reason": "cancelled"}
                    return

                nav_attempts += 1
                # En-route observer: 导航期间 best-effort 时间对齐检测, 到达后 drain
                en_route_interval = self._positive_float(
                    params.get("en_route_sample_interval"), 0.4)
                en_route_cap = self._positive_int(
                    os.environ.get("GO2W_EN_ROUTE_MAX_SAMPLES"), 12)
                stop_event = threading.Event()
                en_route_holder = {}

                def _en_route_worker():
                    try:
                        en_route_holder["samples"] = self._observe_en_route(
                            stop_event, target_classes, en_route_interval,
                            max_samples=en_route_cap)
                    except Exception as exc:
                        logger.debug("en-route worker crashed: %s", exc)
                        en_route_holder["samples"] = []

                en_route_thread = threading.Thread(
                    target=_en_route_worker, daemon=True,
                    name=f"en-route-{mission_id}-{iteration}")
                en_route_thread.start()
                try:
                    result = nav.send_goal_and_wait(
                        target["x"], target["y"], target.get("yaw", 0.0),
                        frame_id="map")
                finally:
                    stop_event.set()
                    en_route_thread.join(timeout=2.0)
                en_route_samples = en_route_holder.get("samples") or []
                if en_route_samples:
                    self._ingest_en_route_samples(
                        en_route_samples, store, "__frontier__", require_photos)
                    self._broadcast_person_markers(mission_id, store.markers())
                if not result.get("ok"):
                    reason = self._nav_failure_reason(result)
                    if reason in ("no_nav", "cancelled"):
                        self._fail(reason, room="__frontier__",
                                   status=result.get("status"),
                                   stage=f"frontier_{iteration}")
                        task.status = "failed"
                        task.result = {"reason": reason, "raw": result}
                        return
                    exploration.mark_navigation_failed(reason, target)
                    self._phase("FRONTIER_DETECT", progress=progress,
                                room="__frontier__", current_wp=iteration,
                                total_wp=max_frontiers,
                                warning=f"frontier {iteration} skipped ({reason})")
                    continue

                exploration.mark_visited(target)
                waypoints_reached += 1

                # DETECT: 到达后调 _observe_people_at_viewpoint
                self._phase("DETECT",
                            progress=float(iteration + 1) / float(max(1, max_frontiers)),
                            room="__frontier__", current_wp=iteration,
                            total_wp=max_frontiers)
                if self._check_cancel("DETECT", "__frontier__"):
                    task.status = "failed"
                    task.result = {"reason": "cancelled"}
                    return

                observe_pose = self._get_live_robot_pose()
                if observe_pose is None:
                    self._fail("no_pose", room="__frontier__",
                               stage=f"frontier_{iteration}",
                               msg="live robot pose required for person map coordinates")
                    task.status = "failed"
                    task.result = {"reason": "no_pose"}
                    return

                unresolved_before_observation = len(store.unresolved())
                observed = self._observe_people_at_viewpoint(
                    store, "__frontier__", iteration, observe_pose,
                    target_classes, require_photos=require_photos,
                    use_lidar=True)
                if observed is None:
                    self._fail("no_scan", room="__frontier__",
                               stage=f"frontier_{iteration}")
                    task.status = "failed"
                    task.result = {"reason": "no_scan"}
                    return
                if isinstance(observed, dict) and observed.get("reason"):
                    reason = str(observed.get("reason"))
                    if reason == "no_lidar_range":
                        resolved_count = self._positive_int(
                            observed.get("resolved_count"), 0)
                        store.resolve_unresolved(min(
                            resolved_count, unresolved_before_observation))
                        self._phase(
                            "FRONTIER_DETECT", progress=progress,
                            room="__frontier__", current_wp=iteration,
                            total_wp=max_frontiers,
                            warning="person seen without reliable lidar range; "
                                    "continuing from another viewpoint",
                        )
                    else:
                        self._fail(reason, room="__frontier__",
                                   stage=f"frontier_{iteration}",
                                   detections=observed.get("detections"))
                        task.status = "failed"
                        task.result = observed
                        return
                elif isinstance(observed, dict):
                    resolved_count = self._positive_int(
                        observed.get("resolved_count"), 0)
                    store.resolve_unresolved(min(
                        resolved_count, unresolved_before_observation))
                elif isinstance(observed, int) and observed > 0:
                    store.resolve_unresolved(min(
                        observed, unresolved_before_observation))

                self._broadcast_person_markers(mission_id, store.markers())

                if time.time() >= mission_deadline:
                    completion_reason = "time_budget_exhausted"
                    break

            # ---- REPORT ----
            markers = store.markers()
            unresolved = store.unresolved()
            self._broadcast_person_markers(mission_id, markers)
            exploration_state = exploration.snapshot()
            # ROI 限定覆盖率 (review #3): mission_origin 圆或 room_polygon
            with map_lock:
                final_map = latest_map_box[0]
            room_polygon = params.get("room_polygon")
            active_radius = exploration_state.get("active_radius_m")
            coverage_radius = (
                active_radius if active_radius is not None else max_radius)
            coverage_roi = self._build_coverage_roi(
                room_polygon, mission_origin, coverage_radius)
            coverage_inflation = self._positive_float(
                os.environ.get("GO2W_COVERAGE_INFLATION_M"), 0.3)
            coverage_metrics = None
            if _compute_coverage is not None and final_map is not None:
                try:
                    coverage_metrics = _compute_coverage(
                        final_map, roi=coverage_roi,
                        mission_origin=tuple(mission_origin),
                        inflation_radius_m=coverage_inflation)
                except Exception as exc:
                    logger.warning("coverage computation failed: %s", type(exc).__name__)
                    logger.debug("coverage computation exception", exc_info=True)
                    coverage_metrics = None
            if coverage_metrics is None:
                coverage_metrics = {
                    "coverage_valid": False,
                    "roi": coverage_roi,
                    "free_cells": 0, "occupied_cells": 0,
                    "unknown_cells": 0, "total_cells": 0,
                    "explored_ratio": None,
                    "enclosed_unknown_regions": [],
                    "map_stamp": None,
                    "inflation_radius_m": coverage_inflation,
                }
            completion_status = self._derive_completion_status(
                completion_reason, coverage_metrics)
            blocked_frontiers = [
                {
                    **item,
                    "rejections": int(item.get("failures", 0)),
                }
                for item in exploration_state["blacklist"]
            ]
            frontier_result = {
                "completion_reason": completion_reason,
                "completion_status": completion_status,
                "waypoints_reached": waypoints_reached,
                "navigation_attempts": nav_attempts,
                "frontier_plan_probes": exploration_state["plan_probes"],
                "frontier_plan_rejections": exploration_state[
                    "plan_rejections"],
                "frontier_nav_failures": exploration_state[
                    "navigation_failures"],
                "blocked_frontiers": sorted(
                    blocked_frontiers,
                    key=lambda item: (item["x"], item["y"])),
                "time_budget_sec": max_time,
                "search_radius_m": coverage_radius,
                "active_radius_m": active_radius,
                "max_radius_m": exploration_state.get("max_radius_m"),
                "radius_step_m": exploration_state.get("radius_step_m"),
                "tile_size_m": exploration_state.get("tile_size_m"),
                "active_tile": exploration_state.get("active_tile"),
                "visited_tiles": exploration_state.get("visited_tiles", []),
                "exhaustion_streak": exploration_state.get(
                    "exhaustion_streak", 0),
                "stable_exhaustion_cycles": exploration_state.get(
                    "stable_exhaustion_cycles", stable_exhaustion_cycles),
                "exploration_state": exploration_state,
                "coverage_valid": coverage_metrics["coverage_valid"],
                "explored_ratio": coverage_metrics["explored_ratio"],
                "roi": coverage_metrics["roi"],
                "enclosed_unknown_regions": coverage_metrics[
                    "enclosed_unknown_regions"],
                "coverage_free_cells": coverage_metrics["free_cells"],
                "coverage_occupied_cells": coverage_metrics["occupied_cells"],
                "coverage_unknown_cells": coverage_metrics["unknown_cells"],
                "coverage_total_cells": coverage_metrics["total_cells"],
                "map_stamp": coverage_metrics["map_stamp"],
            }
            if unresolved:
                reason = "no_lidar_range"
                self._fail(
                    reason,
                    room="__frontier__",
                    detections=unresolved,
                    resolved_detections=markers,
                    **frontier_result,
                )
                task.status = "failed"
                task.result = {
                    "reason": reason,
                    "detections": unresolved,
                    "resolved_detections": markers,
                    **frontier_result,
                }
                return
            if completion_reason != "reachable_frontiers_exhausted":
                reason = "exploration_incomplete"
                self._fail(
                    reason,
                    room="__frontier__",
                    detections=markers,
                    targets_found=len(markers),
                    **frontier_result,
                )
                task.status = "failed"
                task.result = {
                    "reason": reason,
                    "mission_id": mission_id,
                    "room": "__frontier__",
                    "detections": markers,
                    "targets_found": len(markers),
                    **frontier_result,
                }
                return
            self._phase("REPORT", progress=1.0, room="__frontier__",
                        targets_found=len(markers))
            self._finalize_report(task, mission_id, sentry_room,
                                  total_wp=nav_attempts,
                                  visited=len(exploration_state[
                                      "visited_frontiers"]),
                                  detections_log=markers, start_time=start_time,
                                  extra_result=frontier_result)
        finally:
            # 清理订阅 (防重复 frontier 任务累积订阅泄漏)
            if sub_handle is not None:
                try:
                    self._node.destroy_subscription(sub_handle)
                except Exception as e:
                    logger.debug(f"destroy_subscription 异常 (可忽略): {e}")

    def _derive_completion_status(self, completion_reason, coverage_metrics):
        """Map completion_reason + coverage into the 4-state report status.

        review #5: 停止信号仍是 frontier 耗尽, 但完成状态必须真实反映覆盖率,
        不能只塞数值又按 frontier 耗尽宣告成功.
        """
        coverage_metrics = coverage_metrics or {}
        if not coverage_metrics.get("coverage_valid"):
            return "coverage_unverified"
        budget_reasons = {
            "time_budget_exhausted",
            "distance_budget_exhausted",
            "planning_budget_exhausted",
        }
        if completion_reason in budget_reasons:
            return "incomplete"
        threshold = self._positive_float(
            os.environ.get("GO2W_FRONTIER_COVERAGE_THRESHOLD"), 0.90)
        ratio = coverage_metrics.get("explored_ratio")
        enclosed = coverage_metrics.get("enclosed_unknown_regions") or []
        try:
            ratio_ok = ratio is not None and float(ratio) >= threshold
        except (TypeError, ValueError):
            ratio_ok = False
        if ratio_ok and not enclosed:
            return "completed"
        return "completed_with_gaps"

    def _product_search_available(self) -> bool:
        return all((
            ActiveSearchPlanner is not None,
            DetectionFrame is not None,
            LaserScanSnapshot is not None,
            localize_target_detection is not None,
            TargetMissionStore is not None,
        ))

    def _new_target_store(self, mission_id, default_class="person"):
        kwargs = {"default_class": default_class}
        if self._mission_root:
            kwargs["mission_root"] = self._mission_root
        else:
            kwargs["static_root"] = self._static_root
        return TargetMissionStore(mission_id, **kwargs)

    @staticmethod
    def _normalize_target_classes(target_classes) -> List[str]:
        normalized = []
        for value in target_classes or []:
            target_class = " ".join(str(value or "").strip().split()).lower()
            if target_class and target_class not in normalized:
                normalized.append(target_class)
        return normalized or ["person"]

    def _detection_snapshot_getter(self, target_classes):
        if self._ai is None:
            return None
        targets = self._normalize_target_classes(target_classes)
        generic = getattr(self._ai, "get_detection_snapshot", None)
        if callable(generic):
            return lambda: generic(targets)
        legacy = getattr(self._ai, "get_person_detection_snapshot", None)
        if targets == ["person"] and callable(legacy):
            return legacy
        return None

    def _activate_detection_targets(self, target_classes):
        setter = getattr(self._ai, "set_detection_targets", None)
        if not callable(setter):
            return None
        targets = self._normalize_target_classes(target_classes)
        try:
            previous = setter(targets)
        except Exception as exc:
            logger.warning("set_detection_targets failed: %s", exc)
            return None
        return setter, previous

    @staticmethod
    def _restore_detection_targets(target_scope) -> None:
        if target_scope is None:
            return
        setter, previous = target_scope
        try:
            setter(previous)
        except Exception as exc:
            logger.warning("restore detection targets failed: %s", exc)

    def _build_observation_bundle(self, snapshot, require_photos):
        """Read-only: form an observation_sync bundle for one detection snapshot.

        Shared by _observe_people_at_viewpoint (at-viewpoint, after fresh-wait)
        and _observe_en_route (in-flight worker). Does NOT write to the store.
        Safe to call from a worker thread: observation_sync carries its own
        RLock, and node snapshot readers are lock-guarded.

        Returns a dict with bundle/frame_info/frame/scan/pointcloud/robot_pose/
        source/captured_at/observation_valid/camera_calibration, or None if the
        bundle cannot be time-aligned within tolerance (caller must drop the
        sample — never fall back to ad-hoc pose+scan that misaligns in time).
        """
        snapshot = snapshot or {}
        source = str(snapshot.get("source") or "")
        try:
            captured_at = float(snapshot.get("timestamp"))
        except (TypeError, ValueError):
            captured_at = 0.0
        frame = snapshot.get("frame")
        frame_width, frame_height = self._snapshot_frame_size(snapshot, frame)
        try:
            calibration = resolve_camera_calibration(
                source, gimbal_yaw_rad=snapshot.get("gimbal_yaw_rad"))
        except Exception as exc:
            logger.warning("camera calibration failed: %s", exc)
            return None
        result = {
            "source": source,
            "captured_at": captured_at,
            "observation_valid": frame_width > 0 and frame_height > 0,
            "camera_calibration": calibration,
            "frame": frame,
            "frame_width": frame_width,
            "frame_height": frame_height,
        }
        detections = snapshot.get("detections") or []
        if not detections:
            result["bundle"] = None
            result["detections"] = []
            return result
        bundle = None
        if self._observation_sync is not None:
            try:
                if frame is not None:
                    self._observation_sync.add_frame(
                        stamp=captured_at, frame=frame)
                self._observation_sync.add_detection(
                    stamp=captured_at, detection=snapshot)
                # Self-feed pose+scan into the sync using REAL stamps so that
                # bundle_for_detection can time-align them with the detection's
                # captured_at. CRITICAL: do NOT fake captured_at as the pose/
                # scan stamp — on a moving robot the current pose != the pose
                # at captured_at (an older inference frame). ObservationSynchronizer
                # ._pose_at returns exact-stamp matches first, so a faked entry
                # would shadow the real interpolated pose from nx_web_server's
                # subscription feeds and misalign the person's map coordinate.
                #
                # pose: current wall clock for the current pose (≈ now).
                # scan: the scan snapshot's own timestamp (from the sensor).
                # If captured_at is too stale (outside tolerance of all real
                # samples), bundle_for_detection returns None and the caller
                # drops the sample — which is the CORRECT behavior for a stale
                # frame.审核 #1: 移动中时间对齐检测, 不漏标 + 不误标。
                live_pose = self._get_live_robot_pose()
                if live_pose is not None:
                    px, py, pyaw = live_pose
                    self._observation_sync.add_pose(
                        stamp=time.time(), x=px, y=py, yaw=pyaw)
                # Scan: fetch the raw dict from the node first so we can
                # self-feed with the scan's OWN timestamp (审核 #1: real-stamp
                # self-feed, not the detection's captured_at). _laser_scan_snapshot
                # wraps the data in a LaserScanSnapshot dataclass that drops the
                # timestamp, so we read the node snapshot directly here.
                live_scan = None
                scan_stamp = None
                if self._node is not None:
                    _get_scan = getattr(self._node, "get_scan_snapshot", None)
                    if callable(_get_scan):
                        try:
                            raw_scan = _get_scan() or {}
                            scan_stamp = raw_scan.get("timestamp")
                            if scan_stamp is not None:
                                try:
                                    scan_stamp = float(scan_stamp)
                                except (TypeError, ValueError):
                                    scan_stamp = None
                            live_scan = self._laser_scan_snapshot()
                        except Exception as exc:
                            logger.debug("en-route scan snapshot failed: %s", exc)
                            live_scan = None
                if live_scan is not None:
                    if scan_stamp is None or not math.isfinite(scan_stamp):
                        # No usable stamp from the node → cannot safely
                        # time-align; drop the scan rather than fake a stamp.
                        pass
                    else:
                        self._observation_sync.add_scan(
                            stamp=scan_stamp, scan=live_scan)
                # Cloud: PointCloudSnapshot is a frozen dataclass with only
                # `points` and `frame_id` — NO timestamp field. We feed it
                # into the sync with the current wall-clock stamp, which is
                # the honest stamp for a just-read snapshot (the dataclass
                # carries no stamp of its own, so time.time() reflects when
                # the orchestrator read it; this is NOT stamp fabrication,
                # which v1 did with the detection's captured_at).
                current_cloud = self._pointcloud_snapshot()
                if current_cloud is not None:
                    self._observation_sync.add_cloud(
                        stamp=time.time(), cloud=current_cloud)
                tolerance = self._positive_float(
                    os.environ.get("GO2W_OBSERVATION_SYNC_TOLERANCE_SEC"), 0.20)
                bundle = self._observation_sync.bundle_for_detection(
                    stamp=captured_at, tolerance=tolerance)
            except Exception as exc:
                logger.warning("observation synchronization failed: %s", exc)
                bundle = None
        if bundle is None:
            # 关键: 拿不到对齐 bundle → 返回 None, 调用方丢弃样本
            # 绝不退化为 "存 pose + 事后读最新 scan" (时间错位)
            return None
        pointcloud = bundle.cloud.value if bundle.cloud is not None else None
        result.update({
            "bundle": bundle,
            "scan": bundle.scan.value,
            "pointcloud": pointcloud,
            "robot_pose": (bundle.pose.x, bundle.pose.y, bundle.pose.yaw),
            "capture_stamp": bundle.capture_stamp,
            "pose_stamp": bundle.pose.stamp,
            "scan_stamp": bundle.scan.stamp,
            "pose_delta_s": bundle.pose_delta_s,
            "scan_delta_s": bundle.scan_delta_s,
            "localization_quality": "timestamp_interpolated",
            "detections": detections,
        })
        return result

    def _localize_en_route_detections(self, bundle_result, target_classes):
        """Read-only localize of en-route detections. Only range_lidar kept.

        bearing_only observations are left for the at-viewpoint steady-state
        sample (they need a second viewpoint to triangulate). Returns a list
        of localized dicts ready for store.add_observation. Safe from a worker
        thread: no store writes, no shared mutable state beyond bundle_result.
        """
        bundle_result = bundle_result or {}
        detections = bundle_result.get("detections") or []
        if not detections or bundle_result.get("bundle") is None:
            return []
        scan = bundle_result.get("scan")
        if scan is None:
            return []
        pointcloud = bundle_result.get("pointcloud")
        robot_x, robot_y, robot_yaw = bundle_result["robot_pose"]
        calibration = bundle_result["camera_calibration"]
        frame_width = bundle_result["frame_width"]
        frame_height = bundle_result["frame_height"]
        if frame_width <= 0 or frame_height <= 0:
            return []
        if localize_target_detection is None:
            return []
        frame_info = DetectionFrame(
            width=frame_width,
            height=frame_height,
            camera_hfov_rad=math.radians(calibration["hfov_deg"]),
            camera_yaw_offset_rad=math.radians(calibration["yaw_offset_deg"]),
            gimbal_yaw_rad=math.radians(calibration["gimbal_yaw_deg"] or 0.0),
        )
        localization = self._localization_health()
        try:
            robot_z = float(localization.get("z", 0.0))
        except (TypeError, ValueError):
            robot_z = 0.0
        if not math.isfinite(robot_z):
            robot_z = 0.0
        allowed = set(self._normalize_target_classes(target_classes))
        results = []
        for det in detections:
            if det.get("class") not in allowed:
                continue
            try:
                localized = localize_target_detection(
                    det, frame_info, scan, robot_x, robot_y, robot_yaw,
                    robot_z=robot_z, pointcloud=pointcloud)
            except Exception as exc:
                logger.debug("en-route localize failed: %s", exc)
                continue
            if localized.get("position_quality") != "range_lidar":
                continue
            localized.update({
                "robot_x": round(float(robot_x), 3),
                "robot_y": round(float(robot_y), 3),
                "robot_yaw": round(float(robot_yaw), 3),
                "source": "en_route",
                "detector_source": bundle_result.get("source"),
                "timestamp": bundle_result.get("capture_stamp", time.time()),
            })
            results.append(localized)
        return results

    def _observe_en_route(self, stop_event, target_classes, sample_interval=0.4,
                          max_samples=12, max_duration_s=None):
        """Worker thread: best-effort person detection while navigating.

        Reads ai detection snapshot cache; for each NEW inference frame
        (deduped by captured_at) forms a time-aligned observation_sync bundle
        and localizes range_lidar detections. Does NOT write to the store.
        Returns a bounded list of samples
        [{localized_list, frame, captured_at, source}, ...] for the main
        thread to ingest serially after the goal is reached.

        Dedup rule: same captured_at (inference frame stamp) is processed at
        most once, so a cached frame polled many times during one navigation
        leg is not copied repeatedly (avoiding the 100x 720p heap pressure
        flagged in review).

        Termination: rolling window — the loop runs until ``stop_event`` is
        set OR ``max_duration_s`` elapses (if not None). The sample list is
        capped at ``max_samples``; when full, the OLDEST sample is dropped
        (pop(0)) and the loop CONTINUES, so the most recent ``max_samples``
        observations are always kept. This is critical for production: a
        single Nav2 goal can run up to 40s — terminating after the first
        ``max_samples`` (~4.8s at interval=0.4, cap=12) would miss the
        remaining ~35s of detection opportunities.

        ``max_duration_s``: optional safety deadline. Production callers
        pass None (loop until stop_event is set by the frontier loop after
        send_goal_and_wait returns). Synchronous unit-test callers pass a
        small value so the call self-terminates without depending on
        stop_event.
        """
        samples = []
        get_snapshot = self._detection_snapshot_getter(target_classes)
        if get_snapshot is None:
            return samples
        interval = max(0.05, float(sample_interval))
        cap = max(1, int(max_samples))
        if max_duration_s is not None:
            deadline = time.monotonic() + float(max_duration_s)
        else:
            deadline = None
        seen_stamps = set()
        while not stop_event.is_set():
            if deadline is not None and time.monotonic() >= deadline:
                break
            try:
                try:
                    snapshot = get_snapshot() or {}
                except Exception as exc:
                    logger.debug("en-route snapshot read failed: %s", exc)
                    snapshot = {}
                try:
                    captured_at = float(snapshot.get("timestamp"))
                except (TypeError, ValueError):
                    captured_at = 0.0
                if captured_at > 0.0 and captured_at not in seen_stamps:
                    seen_stamps.add(captured_at)
                    try:
                        bundle_result = self._build_observation_bundle(snapshot, False)
                    except Exception as exc:
                        logger.debug("en-route bundle build failed: %s", exc)
                        bundle_result = None
                    if bundle_result is not None and bundle_result.get("bundle") is not None:
                        localized = self._localize_en_route_detections(
                            bundle_result, target_classes)
                        if localized:
                            # Rolling window: drop oldest when full, keep looping.
                            # Newer samples reflect the robot's current position
                            # better than stale samples captured at leg start.
                            if len(samples) >= cap:
                                samples.pop(0)
                            samples.append({
                                "localized_list": localized,
                                "frame": bundle_result.get("frame"),
                                "captured_at": captured_at,
                                "source": bundle_result.get("source"),
                            })
                if stop_event.wait(interval):
                    break
            except Exception:
                # Last-resort net: keep the worker alive across iterations
                # even if an unexpected exception escapes the per-operation
                # handlers above. Do NOT re-raise — the daemon thread must
                # stay alive so the main loop can keep collecting samples.
                logger.exception("en-route observer iteration failed")
        return samples

    def _ingest_en_route_samples(self, samples, store, room_name, require_photos):
        """Main thread: serially commit en-route localized samples to the store.

        Called after the worker thread is joined. Each sample carries a list
        of already-localized range_lidar observations; we only call
        store.add_observation (the store's own dedup + media policy applies).
        Returns the number of observations committed.
        """
        committed = 0
        for sample in samples or []:
            localized_list = sample.get("localized_list") or []
            frame = sample.get("frame") if require_photos else None
            room = str(room_name or "__frontier__")
            for localized in localized_list:
                localized.setdefault("room", room)
                try:
                    store.add_observation(localized, frame=frame)
                    committed += 1
                except Exception as exc:
                    logger.debug("en-route store commit failed: %s", exc)
        return committed

    def _observe_people_at_viewpoint(self, store, room_name: str, view_idx: int,
                                     robot_pose, target_classes: List[str],
                                     require_photos: bool = True,
                                     use_lidar: bool = True):
        target_classes = self._normalize_target_classes(target_classes)
        get_snapshot = self._detection_snapshot_getter(target_classes)
        if get_snapshot is None:
            return {
                "resolved_count": 0,
                "source": None,
                "observation_valid": False,
            }
        max_frame_age_sec = self._positive_float(
            os.environ.get("GO2W_DETECTION_MAX_AGE_SEC"), 2.0)
        # C13/YOLO inference is intentionally slower than the video stream and
        # can take several seconds per frame on the NX.  A viewpoint reached
        # between inference frames must wait for the next fresh observation;
        # immediately rejecting the cached frame makes room search fail at
        # random even though the camera and detector are healthy.
        fresh_wait_sec = self._positive_float(
            os.environ.get("GO2W_DETECTION_WAIT_SEC"), 12.0)
        fresh_deadline = time.monotonic() + fresh_wait_sec
        wait_started = time.monotonic()
        while True:
            try:
                snapshot = get_snapshot() or {}
            except Exception as e:
                logger.warning(f"get_detection_snapshot failed: {e}")
                return {
                    "resolved_count": 0,
                    "source": None,
                    "observation_valid": False,
                }

            source = snapshot.get("source")
            if str(source or "").strip().lower() == "mock":
                return {
                    "reason": "mock_detection_source",
                    "source": "mock",
                    "observation_valid": False,
                    "resolved_count": 0,
                }
            try:
                captured_at = float(snapshot.get("timestamp"))
                frame_age_sec = time.time() - captured_at
            except (TypeError, ValueError):
                captured_at = 0.0
                frame_age_sec = float("inf")
            stale = (
                not math.isfinite(captured_at)
                or captured_at <= 0.0
                or not math.isfinite(frame_age_sec)
                or frame_age_sec < -0.5
                or frame_age_sec > max_frame_age_sec
            )
            if not stale:
                break
            with self._lock:
                cancelled = self._cancelled
            if cancelled:
                return {
                    "reason": "cancelled",
                    "source": source,
                    "observation_valid": False,
                }
            remaining = fresh_deadline - time.monotonic()
            if remaining <= 0.0:
                return {
                    "reason": "stale_detection_frame",
                    "frame_age_sec": frame_age_sec,
                    "max_frame_age_sec": max_frame_age_sec,
                    "waited_sec": time.monotonic() - wait_started,
                    "source": source,
                    "observation_valid": False,
                }
            time.sleep(min(0.1, remaining))

        frame = snapshot.get("frame")
        frame_width, frame_height = self._snapshot_frame_size(snapshot, frame)
        calibration = resolve_camera_calibration(
            source,
            gimbal_yaw_rad=snapshot.get("gimbal_yaw_rad"),
        )
        observation_meta = {
            "source": source,
            "observation_valid": frame_width > 0 and frame_height > 0,
            "camera_calibration": calibration,
        }
        detections = snapshot.get("detections") or []
        if not detections:
            return {**observation_meta, "resolved_count": 0}

        # Test-protected fall-back: when observation_sync is not injected
        # (e.g. Windows dev tests, or older NX runs without the sync layer),
        # use the legacy ad-hoc path: read scan + pointcloud directly. The
        # en-route worker never enters this branch because it requires
        # observation_sync (see _build_observation_bundle → None handling).
        if self._observation_sync is None:
            scan = self._laser_scan_snapshot()
            if scan is None:
                return None
            pointcloud = self._pointcloud_snapshot()
        else:
            bundle_result = self._build_observation_bundle(
                snapshot, require_photos)
            if bundle_result is None:
                return {
                    "source": source,
                    "observation_valid": False,
                    "reason": "unsynchronized_observation",
                    "capture_stamp": captured_at,
                }
            observation_meta = {
                "source": bundle_result["source"],
                "observation_valid": bundle_result["observation_valid"],
                "camera_calibration": bundle_result["camera_calibration"],
            }
            if bundle_result.get("bundle") is None:
                # No detection or empty detections: bundle constructor returns
                # a no-detection result; behave as the legacy "no detections"
                # path.
                return {**observation_meta, "resolved_count": 0}
            observation_meta.update({
                "capture_stamp": bundle_result["capture_stamp"],
                "pose_stamp": bundle_result["pose_stamp"],
                "scan_stamp": bundle_result["scan_stamp"],
                "pose_delta_s": bundle_result["pose_delta_s"],
                "scan_delta_s": bundle_result["scan_delta_s"],
                "cloud_stamp": (
                    bundle_result["bundle"].cloud.stamp
                    if bundle_result["bundle"].cloud is not None else None),
                "cloud_delta_s": bundle_result["bundle"].cloud_delta_s,
                "localization_quality": bundle_result["localization_quality"],
            })
            scan = bundle_result["scan"]
            pointcloud = bundle_result["pointcloud"]
            robot_pose = bundle_result["robot_pose"]
            frame = bundle_result["frame"]
            frame_width = bundle_result["frame_width"]
            frame_height = bundle_result["frame_height"]
            calibration = bundle_result["camera_calibration"]
            detections = bundle_result["detections"]

        if frame_width <= 0 or frame_height <= 0:
            if require_photos:
                return {
                    **observation_meta,
                    "reason": "photo_artifact_failed",
                    "error": "detection frame dimensions missing",
                }
            return {**observation_meta, "resolved_count": 0}
        if require_photos and frame is None:
            return {
                **observation_meta,
                "reason": "photo_artifact_failed",
                "error": "detection frame missing",
            }

        frame_info = DetectionFrame(
            width=frame_width,
            height=frame_height,
            camera_hfov_rad=math.radians(calibration["hfov_deg"]),
            camera_yaw_offset_rad=math.radians(
                calibration["yaw_offset_deg"]),
            gimbal_yaw_rad=math.radians(
                calibration["gimbal_yaw_deg"] or 0.0),
        )
        robot_x, robot_y, robot_yaw = robot_pose
        localization = self._localization_health()
        try:
            robot_z = float(localization.get("z", 0.0))
        except (TypeError, ValueError):
            robot_z = 0.0
        if not math.isfinite(robot_z):
            robot_z = 0.0
        allowed = set(target_classes)
        added = 0
        no_lidar_context = []
        for det in detections:
            if det.get("class") not in allowed:
                continue
            try:
                localized = localize_target_detection(
                    det, frame_info, scan, robot_x, robot_y, robot_yaw,
                    robot_z=robot_z, pointcloud=pointcloud)
            except Exception as e:
                logger.warning(f"localize_target_detection failed: {e}")
                continue
            localized.update({
                **observation_meta,
                "robot_x": round(float(robot_x), 3),
                "robot_y": round(float(robot_y), 3),
                "robot_yaw": round(float(robot_yaw), 3),
                "room": room_name,
                "wp_index": int(view_idx),
                "view_index": int(view_idx),
                "timestamp": captured_at,
                "detector_source": source,
            })
            if use_lidar and localized.get("position_quality") != "range_lidar":
                try:
                    unresolved = store.add_unresolved_observation(
                        localized, frame=frame if require_photos else None)
                except Exception as e:
                    logger.warning(f"unresolved target artifact storage failed: {e}")
                    if require_photos:
                        return {
                            **observation_meta,
                            "reason": "photo_artifact_failed",
                            "error": str(e),
                        }
                    unresolved = localized
                no_lidar_context.append(unresolved)
                continue
            try:
                marker = store.add_observation(
                    localized, frame=frame if require_photos else None)
            except Exception as e:
                logger.warning(f"target observation storage failed: {e}")
                if require_photos:
                    return {
                        **observation_meta,
                        "reason": "photo_artifact_failed",
                        "error": str(e),
                        "detection": {
                            "class": det.get("class"),
                            "confidence": det.get("confidence"),
                            "bbox": det.get("bbox"),
                        },
                    }
                continue
            if require_photos and (not marker.get("photo_url") or not marker.get("crop_url")):
                return {
                    **observation_meta,
                    "reason": "photo_artifact_failed",
                    "error": "required annotated photo or crop missing",
                }
            added += 1
        if use_lidar and no_lidar_context:
            return {
                **observation_meta,
                "reason": "no_lidar_range",
                "detections": no_lidar_context,
                "resolved_count": added,
            }
        return {**observation_meta, "resolved_count": added}

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
            try:
                timestamp = float(data.get("timestamp"))
            except (TypeError, ValueError):
                return None
            if not math.isfinite(timestamp) or timestamp <= 0.0:
                return None
            try:
                age_sec = (
                    float(data.get("age_sec"))
                    if data.get("age_sec") is not None
                    else time.time() - timestamp
                )
            except (TypeError, ValueError):
                return None
            if not math.isfinite(age_sec) or age_sec < 0.0:
                return None
            max_age_sec = self._positive_float(
                os.environ.get("GO2W_SCAN_MAX_AGE_SEC"), 2.0)
            if age_sec > max_age_sec:
                return None
            angle_increment = float(data.get("angle_increment", 0.0))
            range_min = float(data.get("range_min", 0.15))
            range_max = float(data.get("range_max", 10.0))
            if angle_increment <= 0.0 or range_min <= 0.0 or range_max <= range_min:
                return None
            has_valid_range = False
            for raw_range in ranges:
                try:
                    range_m = float(raw_range)
                except (TypeError, ValueError):
                    continue
                if math.isfinite(range_m) and range_min <= range_m <= range_max:
                    has_valid_range = True
                    break
            if not has_valid_range:
                return None
            return LaserScanSnapshot(
                angle_min=float(data.get("angle_min", 0.0)),
                angle_increment=angle_increment,
                ranges=ranges,
                range_min=range_min,
                range_max=range_max,
            )
        except Exception as e:
            logger.warning(f"get_scan_snapshot failed: {e}")
            return None

    def _pointcloud_snapshot(self):
        """Return a fresh base_link MID360 cloud for optional height evidence."""
        if self._node is None or PointCloudSnapshot is None:
            return None
        getter = getattr(self._node, "get_pointcloud_snapshot", None)
        if not callable(getter):
            return None
        try:
            data = getter() or {}
            if str(data.get("frame_id") or "") != "base_link":
                return None
            points = data.get("points")
            if points is None or len(points) == 0:
                return None
            timestamp = float(data.get("timestamp"))
            age_sec = (
                float(data.get("age_sec"))
                if data.get("age_sec") is not None
                else time.time() - timestamp
            )
            max_age_sec = self._positive_float(
                os.environ.get("GO2W_POINTCLOUD_MAX_AGE_SEC"), 1.0)
            if (
                not math.isfinite(timestamp) or timestamp <= 0.0
                or not math.isfinite(age_sec) or age_sec < 0.0
                or age_sec > max_age_sec
            ):
                return None
            return PointCloudSnapshot(points=points, frame_id="base_link")
        except Exception as e:
            logger.warning(f"get_pointcloud_snapshot failed: {e}")
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
            "type": "target_markers",
            "data": {
                "mission_id": mission_id,
                "markers": marker_list,
            },
        })
        if marker_list and any(
                marker.get("class", "person") != "person"
                for marker in marker_list):
            return
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
            health = self._localization_health()
            if not health.get("healthy"):
                return []
            with getattr(self._node, "_lock", threading.Lock()):
                ranges = list(getattr(self._node, "_scan_ranges", []) or [])
                angle_min = float(getattr(self._node, "_scan_angle_min", 0.0))
                angle_increment = float(getattr(self._node, "_scan_angle_increment", 0.0))
                range_min = float(getattr(self._node, "_scan_range_min", 0.15) or 0.15)
                range_max = float(getattr(self._node, "_scan_range_max", 10.0) or 10.0)
                robot_x = float(getattr(self._node, "_map_x", 0.0))
                robot_y = float(getattr(self._node, "_map_y", 0.0))
                robot_yaw = float(getattr(self._node, "_map_yaw", 0.0))
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

    def _camera_hfov_rad(self, source=None) -> float:
        return math.radians(
            resolve_camera_calibration(source)["hfov_deg"])

    def _camera_yaw_offset_rad(self, source=None) -> float:
        return math.radians(
            resolve_camera_calibration(source)["effective_yaw_offset_deg"])

    def _camera_source_key(self, source) -> str:
        return "".join(
            character if character.isalnum() else "_"
            for character in str(source or "").upper()
        ).strip("_")

    def _nav_failure_reason(self, result: dict) -> str:
        reason = (result or {}).get("reason", "nav_aborted")
        if reason == "no_server":
            return "no_nav"
        if reason in ("cancelled", "canceled"):
            return "cancelled"
        return reason

    def _param_explicitly_false(self, value) -> bool:
        if value is False:
            return True
        if isinstance(value, str):
            return value.strip().lower() in ("false", "0", "no", "off")
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            try:
                return float(value) == 0.0
            except (TypeError, ValueError):
                return False
        return False

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

    def _build_coverage_roi(self, room_polygon, mission_origin, max_radius):
        """Build the coverage ROI dict, preferring a validated room_polygon and
        falling back to a mission_origin-centered circle on any validation
        failure. Never raises — REPORT must not abort on a bad polygon.
        """
        reason = None
        if not isinstance(room_polygon, list):
            reason = "not_list"
        elif not (3 <= len(room_polygon) <= 100):
            reason = "length_out_of_range"
        else:
            points = []
            for p in room_polygon:
                if not (isinstance(p, (list, tuple)) and len(p) == 2):
                    reason = "point_not_pair"
                    break
                try:
                    a, b = float(p[0]), float(p[1])
                except (TypeError, ValueError):
                    reason = "point_not_numeric"
                    break
                if not (math.isfinite(a) and math.isfinite(b)):
                    reason = "point_not_finite"
                    break
                points.append((a, b))
            if reason is None:
                return {
                    "type": "polygon",
                    "points": points,
                }
        logger.warning(
            "invalid room_polygon, falling back to circle ROI: %s", reason)
        return {
            "type": "circle",
            "center": [float(mission_origin[0]), float(mission_origin[1])],
            "radius": float(max_radius) if max_radius is not None else 6.0,
        }

    def _coverage_threshold(self, value, default: float) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = float(default)
        if not math.isfinite(parsed) or parsed <= 0.0 or parsed > 1.0:
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
                health = self._localization_health()
                if health.get("healthy"):
                    return float(health["x"]), float(health["y"]), float(health["yaw"])
        except Exception:
            pass
        if fallback is not None:
            return float(fallback.get("x", 0.0)), float(fallback.get("y", 0.0)), float(fallback.get("yaw", 0.0))
        return 0.0, 0.0, 0.0

    # ---- 辅助 ----
    def _localization_health(self):
        """Read NxWebNode localization, with support for legacy injected test adapters."""
        if self._node is None:
            return {"healthy": False, "reason": "no_node"}
        getter = getattr(self._node, "get_localization_health", None)
        if callable(getter):
            return getter()

        # Older injected adapters do not subscribe ROS topics themselves. Keep
        # them usable while the production NxWebNode always takes this method's
        # /localization_pose path above.
        with getattr(self._node, "_lock", threading.Lock()):
            count = int(getattr(self._node, "_odom_count", 0) or 0)
            received = float(getattr(self._node, "_odom_t", 0.0) or 0.0)
            x = float(getattr(self._node, "_odom_x", 0.0))
            y = float(getattr(self._node, "_odom_y", 0.0))
            z = float(getattr(self._node, "_map_z", 0.0))
            yaw = float(getattr(self._node, "_map_yaw", 0.0))
        max_age = self._positive_float(os.environ.get("GO2W_ODOM_MAX_AGE_SEC"), 2.0)
        age = time.time() - received if received > 0.0 else None
        healthy = (
            count > 0
            and all(math.isfinite(value) for value in (x, y, yaw, received))
            and age is not None
            and math.isfinite(age)
            and 0.0 <= age <= max_age
        )
        return {
            "healthy": healthy,
            "reason": "ok" if healthy else "legacy_pose_unavailable",
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "age_sec": age,
        }

    def _get_live_robot_pose(self):
        try:
            if self._node is None:
                return None
            health = self._localization_health()
            if not health.get("healthy"):
                return None
            return float(health["x"]), float(health["y"]), float(health["yaw"])
        except Exception:
            return None

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
        with self._lock:
            self._mission_state = dict(data)
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
        with self._lock:
            self._mission_state = dict(data)
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
                         detections_log: List[dict], start_time: float,
                         extra_result: Optional[dict] = None) -> None:
        """REPORT 阶段: 生成 MissionReport + 推 type=mission_report + 标 task 完成。"""
        end_time = time.time()
        evidence_store = None
        result_path = ""
        if self._mission_root and TargetMissionStore is not None:
            evidence_store = self._new_target_store(mission_id)
            result_path = (
                f"/missions/{evidence_store.mission_slug}/report.json")
        report = build_mission_report(
            mission_id=mission_id, room=room,
            waypoints_total=total_wp, waypoints_visited=visited,
            detections=detections_log, start_time=start_time, end_time=end_time,
            result_path=result_path)
        if extra_result:
            report.update(dict(extra_result))
        if evidence_store is not None:
            try:
                evidence_store.save_report(report)
            except Exception as exc:
                logger.error("mission report persistence failed: %s", exc)
                self._fail(
                    "mission_artifact_failed", room=room.name, msg=str(exc))
                task.status = "failed"
                task.result = {
                    "reason": "mission_artifact_failed", "error": str(exc)}
                return
        with self._lock:
            self._last_report = dict(report)
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

    def wait_drained(self, timeout: float) -> bool:
        """Wait only for Nav2 ownership; TaskManager separately owns the worker."""
        nav = self._nav
        if nav is None:
            return True
        waiter = getattr(nav, "wait_drained", None)
        if not callable(waiter):
            return False
        try:
            return bool(waiter(timeout))
        except Exception:
            return False

    def get_navigation_state(self) -> Dict:
        nav = self._nav
        if nav is None:
            state = {"phase": "idle", "drained": True, "in_flight": False}
        else:
            getter = getattr(nav, "get_state", None)
            if not callable(getter):
                state = {"phase": "unknown", "drained": False, "in_flight": True}
            else:
                try:
                    state = dict(getter())
                except Exception:
                    state = {"phase": "unknown", "drained": False, "in_flight": True}
        with self._lock:
            mission_state = dict(self._mission_state)
            markers = list(self._person_markers)
            last_report = (
                None if self._last_report is None else dict(self._last_report))
            state.update({
                "search_phase": mission_state.get("phase", "idle"),
                "mission_id": self._current_mission_id,
                "room": self._current_room_name,
                "current_wp": self._current_wp_idx,
                "total_wp": self._current_total_wp,
                "targets_found": self._current_targets_found,
                "target_markers": markers,
                "person_markers": markers,
                "last_report": last_report,
                "search_state": mission_state,
            })
        return state


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
