"""test_frontier_explore.py — frontier 探索搜人状态机测试 (plan 2026-07-03 §3.4)。

测试范围 (Windows 开发机, 无 rclpy):
  1. select_next_frontier 占位行为锁定 (learning-mode 边界)
  2. _run_frontier_explore 状态机骨架:
     - 分发 (search_strategy=frontier_explore)
     - INIT_SLAM 超时 (no map) → failed
     - 零 frontier 正常结束 → completed
     - cancel 响应 → failed
     - _ensure_room_map 不被调用 (绕过 RoomMap)

复用 test_product_room_orchestrator.py 的 FakeTask/FakeAi/FakeNode/FakeNav 模式。
不依赖 rclpy: _OccupancyGrid is None 时 orchestrator 前置 fail, 走 fail 断言路径;
想测正常路径用 monkeypatch 把 _OccupancyGrid 设成 stub 类。
"""
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

import nx_room_orchestrator as orch_module  # noqa: E402
from nx_room_orchestrator import (  # noqa: E402
    RoomSearchOrchestrator,
    select_frontier_candidates,
    select_next_frontier,
)


# ============================================================================
# Reusable fakes (与 test_product_room_orchestrator.py 同款)
# ============================================================================
class FakeTask:
    def __init__(self):
        self.type = "search_room"
        self.params = {
            "room": "__frontier__",
            "target_classes": ["person"],
            "require_photos": False,
            "mark_on_map": True,
            "search_strategy": "frontier_explore",
            "use_lidar_person_range": True,
            "max_frontiers": 5,
            "max_time": 30,
        }
        self.status = "pending"
        self.result = None


class FakeAi:
    def get_person_detection_snapshot(self):
        return {
            "frame": np.zeros((100, 100, 3), dtype=np.uint8),
            "frame_width": 100,
            "frame_height": 100,
            "timestamp": time.time(),
            "source": "c13_vis",
            "detections": [
                {
                    "class": "person",
                    "confidence": 0.91,
                    "bbox": [45, 10, 55, 90],
                    "frame_width": 100,
                    "frame_height": 100,
                    "source": "yolo",
                }
            ],
        }


class GenericTargetAi(FakeAi):
    def __init__(self):
        self.target_calls = []
        self._targets = None

    def set_detection_targets(self, target_classes=None):
        previous = self._targets
        self._targets = (
            None if target_classes is None else list(target_classes)
        )
        self.target_calls.append(self._targets)
        return previous

    def get_detection_snapshot(self, target_classes=None):
        snapshot = self.get_person_detection_snapshot()
        requested = list(target_classes or [])
        snapshot["target_classes"] = requested
        snapshot["detections"] = [{
            **snapshot["detections"][0],
            "class": "dining table",
        }]
        return snapshot


class FakeNode:
    """NxWebNode stub: 支持订阅缓存 create_subscription/destroy_subscription + scan/odom。"""

    def __init__(self, scan_timestamp=None, emit_map=False):
        self._lock = threading.RLock()
        self._odom_x = 0.2
        self._odom_y = 0.2
        self._imu_yaw = 0.0
        self._odom_count = 1
        self._odom_t = time.time()
        self._scan_timestamp = time.time() if scan_timestamp is None else scan_timestamp
        self._emit_map = emit_map
        self._subscriptions = []

    def get_scan_snapshot(self):
        ranges = [0.0] * 360
        ranges[180] = 2.0
        return {
            "ranges": ranges,
            "angle_min": -math.pi,
            "angle_increment": math.pi / 180.0,
            "range_min": 0.15,
            "range_max": 10.0,
            "timestamp": self._scan_timestamp,
        }

    def create_subscription(self, msg_type, topic, callback, qos):
        """Stub: 如果 emit_map=True, 立即触发回调 (模拟首帧 map 到达)。"""
        handle = {"msg_type": msg_type, "topic": topic, "callback": callback, "qos": qos}
        self._subscriptions.append(handle)
        if self._emit_map and topic == "/map_frontier":
            # 模拟 SLAM 首帧 map (异步, 不阻塞 create_subscription)
            def _emit():
                time.sleep(0.05)
                callback(_make_fake_map())
            t = threading.Thread(target=_emit, daemon=True)
            t.start()
        return handle

    def destroy_subscription(self, handle):
        try:
            self._subscriptions.remove(handle)
        except ValueError:
            pass


class NoMapNode(FakeNode):
    """不发 map 帧 (INIT_SLAM 超时路径)。"""

    def __init__(self):
        super().__init__(emit_map=False)


class FakeNav:
    def __init__(self, results=None, path_results=None):
        self.calls = []
        self.plan_calls = []
        self._results = list(results or [])
        self._path_results = list(path_results or [])

    def wait_for_server(self, timeout=2.0):
        return True

    def wait_for_planner(self, timeout=2.0):
        return True

    def compute_path_to_pose(self, x, y, yaw, frame_id="map", timeout=None):
        self.plan_calls.append((float(x), float(y), float(yaw), frame_id))
        if self._path_results:
            return self._path_results.pop(0)
        return {"ok": True, "status": 4, "poses": 2, "path_length": 1.0}

    def send_goal_and_wait(self, x, y, yaw, frame_id="map"):
        self.calls.append((float(x), float(y), float(yaw), frame_id))
        if self._results:
            return self._results.pop(0)
        return {"ok": True, "status": 4}

    def cancel_current(self):
        return True


# ============================================================================
# Fake OccupancyGrid (模拟 nav_msgs/OccupancyGrid 的最小属性)
# ============================================================================
class _FakeMapInfo:
    def __init__(self, resolution=0.05, width=10, height=10):
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)
        self.origin = type("_Origin", (), {})()
        self.origin.position = type("_Pos", (), {})()
        self.origin.position.x = 0.0
        self.origin.position.y = 0.0
        self.origin.position.z = 0.0
        self.origin.orientation = type("_Ori", (), {})()
        self.origin.orientation.w = 1.0
        self.origin.orientation.x = 0.0
        self.origin.orientation.y = 0.0
        self.origin.orientation.z = 0.0


def _make_fake_map(width=10, height=10, fill_value=0):
    """构造最小 OccupancyGrid stub (data 全 fill_value, 默认全自由=0)。"""
    msg = type("_FakeOccupancyGrid", (), {})()
    msg.header = type("_Header", (), {})()
    msg.header.frame_id = "map"
    msg.info = _FakeMapInfo(resolution=0.05, width=width, height=height)
    msg.data = [fill_value] * (width * height)
    return msg


# ============================================================================
# Orchestrator factory
# ============================================================================
def _ensure_person_deps():
    """前置检查: ActiveSearchPlanner/DetectionFrame/PersonMissionStore 等依赖是否就绪。
    frontier 路径复用 _product_search_available() 守卫, 缺任一会 frontier_unavailable。
    """
    deps_ok = all([
        orch_module.ActiveSearchPlanner is not None,
        orch_module.DetectionFrame is not None,
        orch_module.LaserScanSnapshot is not None,
        orch_module.localize_person_detection is not None,
        orch_module.PersonMissionStore is not None,
    ])
    return deps_ok


def make_orchestrator(events, nav, node=None, ai_engine=None):
    orchestrator = RoomSearchOrchestrator(
        node=node or FakeNode(emit_map=True),
        ai_engine=ai_engine or FakeAi(),
        ws_broadcast_fn=events.append,
        rooms_yaml_path=str(Path(__file__).parent / "_nonexistent_rooms.yaml"),
    )
    orchestrator._nav = nav
    orchestrator._static_root = Path(__file__).resolve().parent
    return orchestrator


# ============================================================================
# 测试 1: select_next_frontier 无簇容错 (占位语义已替换为 cost-distance 实现)
# ============================================================================
def test_select_next_frontier_returns_none_when_no_clusters():
    """全已知 map (data 全 0, 无 -1 → 无 frontier cell → 无簇) → 返回 None。"""
    fake_map = _make_fake_map()  # fill_value=0 默认, 无未知 cell
    assert select_next_frontier(fake_map, (0.0, 0.0, 0.0), []) is None


def test_select_next_frontier_signature_stable():
    """调用签名 (map_msg, robot_pose, visited) 不抛异常 (参数稳定)。"""
    # None map + 空 visited 也不抛
    assert select_next_frontier(None, (0, 0, 0), []) is None
    # 真实 stub map
    fake_map = _make_fake_map(fill_value=-1)  # 全未知
    assert select_next_frontier(fake_map, (1.5, 2.5, 0.3), [{"x": 1.0, "y": 1.0}]) is None


def test_select_next_frontier_docstring_lists_three_methods():
    """docstring 必须列出 (a)/(b)/(c) 三种 valid 方法 (文档契约)。"""
    doc = select_next_frontier.__doc__ or ""
    assert "(a)" in doc
    assert "(b)" in doc
    assert "(c)" in doc


# ============================================================================
# 测试 1b: _find_frontier_clusters 脚手架 (不测评分, 只测簇提取)
# ============================================================================
def _make_half_unknown_map():
    """10x10 grid, 左半 (col 0-4) 自由=0, 右半 (col 5-9) 未知=-1。
    边界 frontier 落在 col=4 (自由, 右邻 col=5 未知)。
    origin=(0,0), resolution=0.1 → cell center (r,4) world=(0.45, r*0.1+0.05)。
    用 SimpleNamespace, 不依赖 rclpy/nav_msgs。
    """
    from types import SimpleNamespace
    width, height = 10, 10
    data = []
    for r in range(height):
        for c in range(width):
            data.append(0 if c < 5 else -1)
    msg = SimpleNamespace(
        info=SimpleNamespace(
            resolution=0.1,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=data,
    )
    return msg


def test_find_frontier_clusters_detects_boundary():
    """_find_frontier_clusters 在 左自由/右未知 grid 上返回非空簇, 每个簇含 4 字段。"""
    from nx_room_orchestrator import _find_frontier_clusters
    msg = _make_half_unknown_map()
    clusters = _find_frontier_clusters(msg, (0.0, 0.0, 0.0), [],
                                       min_cluster_size=3, revisit_radius=1.0)
    assert clusters, "应至少检测到一个 frontier 簇"
    for cl in clusters:
        # 必备 4 字段
        assert "center_cell" in cl
        assert "center_world" in cl
        assert "size" in cl
        assert "distance" in cl
        # 字段类型/形状
        assert isinstance(cl["center_cell"], tuple) and len(cl["center_cell"]) == 2
        assert isinstance(cl["center_world"], tuple) and len(cl["center_world"]) == 2
        assert isinstance(cl["size"], int) and cl["size"] >= 3
        assert isinstance(cl["distance"], float) and cl["distance"] >= 0.0
        # 边界 cell 的 col 应是 4 (左自由最右列, 右邻未知)
        # 簇中心 col 不会超过 4
        assert cl["center_cell"][1] <= 4


def test_frontier_cluster_goal_uses_nearest_reachable_cell_not_giant_centroid():
    """A rolling-costmap boundary is one giant cluster; target its near edge."""
    from nx_room_orchestrator import _find_frontier_clusters
    msg = _make_half_unknown_map()

    clusters = _find_frontier_clusters(
        msg, (0.0, 0.0, 0.0), [],
        min_cluster_size=3, revisit_radius=1.0)

    assert len(clusters) == 1
    assert clusters[0]["center_cell"] == (0, 4)
    assert clusters[0]["center_world"] == pytest.approx((0.45, 0.05))
    assert clusters[0]["distance"] == pytest.approx(math.hypot(0.45, 0.05))


def test_frontier_cell_center_respects_rotated_map_origin():
    """OccupancyGrid origin is a full pose, not an axis-aligned translation."""
    from nx_room_orchestrator import _find_frontier_clusters

    msg = _make_half_unknown_map()
    msg.info.origin.position.x = 10.0
    msg.info.origin.position.y = 20.0
    msg.info.origin.orientation.z = math.sin(math.pi / 4.0)
    msg.info.origin.orientation.w = math.cos(math.pi / 4.0)

    clusters = _find_frontier_clusters(
        msg, (10.0, 20.0, 0.0), [], min_cluster_size=3,
        revisit_radius=1.0)

    assert len(clusters) == 1
    assert clusters[0]["center_cell"] == (0, 4)
    assert clusters[0]["center_world"] == pytest.approx((9.95, 20.45))


def test_find_frontier_clusters_filters_visited():
    """visited 含 frontier 附近点 → revisit_radius 内的簇被过滤。"""
    from nx_room_orchestrator import _find_frontier_clusters
    msg = _make_half_unknown_map()
    # 先空 visited 拿到基线簇列表
    baseline = _find_frontier_clusters(msg, (0.0, 0.0, 0.0), [],
                                       min_cluster_size=3, revisit_radius=1.0)
    assert baseline, "基线应有簇"
    # 取基线第一个簇的 world 坐标作为 visited 点, revisit_radius=1.0 应过滤掉它
    bx, by = baseline[0]["center_world"]
    visited = [{"x": bx, "y": by}]
    filtered = _find_frontier_clusters(msg, (0.0, 0.0, 0.0), visited,
                                       min_cluster_size=3, revisit_radius=1.0)
    # 过滤后簇数应严格小于基线 (至少过滤掉一个)
    assert len(filtered) < len(baseline), (
        f"visited 过滤无效: baseline={len(baseline)} filtered={len(filtered)}")
    # 剩余簇都不应在 revisit_radius 内
    for cl in filtered:
        for vp in visited:
            d = math.hypot(cl["center_world"][0] - vp["x"],
                           cl["center_world"][1] - vp["y"])
            assert d >= 1.0, f"簇 {cl['center_world']} 仍在 visited 点 {vp} 的半径内"


# ============================================================================
# 测试 2: _run_frontier_explore 状态机骨架
# ============================================================================
def test_run_frontier_explore_dispatches_on_strategy(tmp_path, monkeypatch):
    """search_strategy=frontier_explore 走 frontier 分支, 不进 SELECT_ROOM/RoomMap。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps (ActiveSearch/PersonLocalizer) not importable")

    # stub _OccupancyGrid 让前置检查通过 (Windows 无 nav_msgs)
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(events, nav)
    # spy: _ensure_room_map 不应被调用
    orchestrator._ensure_room_map = lambda: pytest.fail(
        "_ensure_room_map should not be called in frontier mode")
    task = FakeTask()

    orchestrator.run(task)

    # learning-mode: select_next_frontier 返回 None → 循环 0 次 → completed
    assert task.status == "completed"
    assert task.result is not None
    assert task.result["waypoints_visited"] == 0
    assert task.result["targets_found"] == 1
    # phase 推送应包含 INIT_SLAM / FRONTIER_DETECT / REPORT
    phases = [e["data"]["phase"] for e in events if e.get("type") == "search_room"]
    assert "INIT_SLAM" in phases
    assert "REPORT" in phases


def test_frontier_explore_handles_no_map_timeout(monkeypatch):
    """FakeNode 不发 map → INIT_SLAM 超时 → failed, reason=no_map。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    events = []
    nav = FakeNav()
    # NoMapNode: create_subscription 不触发回调 → map_received.wait(10s) 超时
    orchestrator = make_orchestrator(events, nav, node=NoMapNode())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_map"


def test_frontier_explore_zero_frontiers_finalize_report(monkeypatch):
    """mock map 全已知 (data 全 0) + select_next_frontier 返回 None → 循环 0 次 → completed。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(events, nav)
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result is not None
    assert task.result["waypoints_visited"] == 0
    # mission_report 推送存在
    reports = [e for e in events if e.get("type") == "mission_report"]
    assert reports
    # room 名是 __frontier__
    assert task.result["room"] == "__frontier__"
    # search_area 空 dict (哨兵 Room)
    assert task.result["area"] == {}


def test_frontier_table_target_is_used_for_detection_and_restored(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    events = []
    ai = GenericTargetAi()
    orchestrator = make_orchestrator(events, FakeNav(), ai_engine=ai)
    task = FakeTask()
    task.params["target_classes"] = ["dining table"]
    task.params.pop("use_lidar_person_range")
    task.params["use_lidar_target_range"] = True

    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["targets_found"] == 1
    assert task.result["detections"][0]["class"] == "dining table"
    assert ai.target_calls == [["dining table"], None]
    marker_events = [
        event for event in events if event.get("type") == "target_markers"]
    assert marker_events
    assert marker_events[-1]["data"]["markers"][0]["class"] == "dining table"


def test_frontier_explore_cancel_mid_loop(monkeypatch):
    """_cancelled=True 后 run → failed, reason=cancelled。
    run() 入口会重置 _cancelled=False, 所以必须用 ws_broadcast 回调在
    INIT_SLAM phase 推送后设置 _cancelled=True (模拟用户中途 cancel)。
    """
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    events = []
    nav = FakeNav()

    def cancel_on_init_slam(data):
        """ws_broadcast hook: 第一次收到 INIT_SLAM 后触发 cancel。"""
        events.append(data)
        if (data.get("type") == "search_room"
                and data.get("data", {}).get("phase") == "INIT_SLAM"):
            # 在 INIT_SLAM 推送后设 cancelled (下一次 _check_cancel 会捕获)
            orchestrator._cancelled = True

    orchestrator = make_orchestrator(events, nav)
    orchestrator._ws = cancel_on_init_slam
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "cancelled"


def test_frontier_explore_fails_when_occupancy_grid_unavailable():
    """_OccupancyGrid is None (Windows 无 nav_msgs) → failed, reason=frontier_unavailable。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(events, nav)
    # 确保 _OccupancyGrid is None (不 stub)
    original = orch_module._OccupancyGrid
    try:
        orch_module._OccupancyGrid = None
        task = FakeTask()
        orchestrator.run(task)
        assert task.status == "failed"
        assert task.result["reason"] == "frontier_unavailable"
    finally:
        orch_module._OccupancyGrid = original


def test_frontier_explore_subscription_destroyed_after_run(monkeypatch):
    """_run_frontier_explore 退出后订阅被 destroy (防累积泄漏)。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    events = []
    nav = FakeNav()
    node = FakeNode(emit_map=True)
    orchestrator = make_orchestrator(events, nav, node=node)
    task = FakeTask()

    orchestrator.run(task)

    # 订阅应在 finally 里被 destroy (FakeNode 记录 subscriptions, destroy 后移除)
    subs = [s for s in node._subscriptions if s["topic"] == "/map_frontier"]
    assert subs == [], "frontier /map_frontier subscription not destroyed after run"


def test_make_sentry_room_returns_minimal_room():
    """_make_sentry_room 返回 Room, search_area 空 dict, name=__frontier__。"""
    orchestrator = RoomSearchOrchestrator.__new__(RoomSearchOrchestrator)
    room = orchestrator._make_sentry_room()
    assert room.name == "__frontier__"
    assert room.search_area == {}
    assert room.target_classes == []
    assert room.aliases == []


# ============================================================================
# 测试 3: cost-distance 评分 (默认实现: alpha=1.0)
# 用 monkeypatch 替换 _find_frontier_clusters 直接控制簇列表, 隔离评分逻辑。
# ============================================================================
def test_select_next_frontier_picks_highest_score(monkeypatch):
    """3 个簇: 大但远 / 小但近 / 中等 — 应选 score 最高的 (大但远)。"""
    # alpha=1.0: score = size / (1 + distance)
    fake_clusters = [
        # score=20/6≈3.33 (最高)
        {"center_cell": (0, 0), "center_world": (5.0, 0.0), "size": 20, "distance": 5.0},
        # score=3/2=1.5
        {"center_cell": (0, 0), "center_world": (1.0, 0.0), "size": 3, "distance": 1.0},
        # score=8/3≈2.67
        {"center_cell": (0, 0), "center_world": (2.0, 0.0), "size": 8, "distance": 2.0},
    ]
    monkeypatch.setattr(orch_module, "_find_frontier_clusters",
                        lambda *a, **k: fake_clusters)
    result = select_next_frontier(None, (0.0, 0.0, 0.0), [])
    assert result is not None
    assert result["x"] == 5.0  # 最高 score 是第一个
    assert result["y"] == 0.0
    assert result["size"] == 20
    assert abs(result["score"] - 20.0 / 6.0) < 1e-6


def test_select_frontier_candidates_returns_deterministic_score_order(monkeypatch):
    fake_clusters = [
        {"center_cell": (2, 2), "center_world": (2.0, 0.0),
         "size": 8, "distance": 2.0},
        {"center_cell": (1, 1), "center_world": (5.0, 0.0),
         "size": 20, "distance": 5.0},
        {"center_cell": (0, 0), "center_world": (1.0, 0.0),
         "size": 3, "distance": 1.0},
    ]
    monkeypatch.setattr(
        orch_module, "_find_frontier_clusters", lambda *a, **k: fake_clusters)

    candidates = select_frontier_candidates(None, (0.0, 0.0, 0.0), [])

    assert [candidate["x"] for candidate in candidates] == [5.0, 2.0, 1.0]


def test_frontier_explore_preflights_and_skips_unreachable_candidate(monkeypatch):
    """A high-score blocked frontier must never be sent as a motion goal."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    first = {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 20, "score": 5.0}
    second = {"x": 0.0, "y": 2.0, "yaw": math.pi / 2.0,
              "size": 10, "score": 3.0}

    def candidates(_map, _pose, visited, **_kwargs):
        return [] if visited else [first, second]

    monkeypatch.setattr(orch_module, "select_frontier_candidates", candidates)
    nav = FakeNav(path_results=[
        {"ok": False, "reason": "unreachable", "status": 6},
        {"ok": True, "status": 4, "poses": 12, "path_length": 2.4},
    ])
    task = FakeTask()
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "completed"
    assert [call[:2] for call in nav.plan_calls] == [(3.0, 0.0), (0.0, 2.0)]
    assert [call[:2] for call in nav.calls] == [(0.0, 2.0)]
    assert task.result["completion_reason"] == "reachable_frontiers_exhausted"
    assert task.result["frontier_plan_rejections"] == 1
    assert task.result["waypoints_reached"] == 1


def test_frontier_explore_all_unreachable_is_bounded(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    candidates = [
        {"x": 2.0, "y": 0.0, "yaw": 0.0, "size": 8, "score": 4.0},
        {"x": 0.0, "y": 2.0, "yaw": 1.57, "size": 6, "score": 3.0},
    ]
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates",
        lambda *_a, **_k: list(candidates))
    nav = FakeNav(path_results=[
        {"ok": False, "reason": "unreachable", "status": 6}
        for _ in range(4)
    ])
    task = FakeTask()
    task.params["max_frontier_rejections"] = 2
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "completed"
    assert len(nav.plan_calls) == 4
    assert nav.calls == []
    assert task.result["completion_reason"] == "reachable_frontiers_exhausted"
    assert task.result["frontier_plan_rejections"] == 4
    assert len(task.result["blocked_frontiers"]) == 2


def test_frontier_explore_checks_time_budget_before_planning(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates",
        lambda *_a, **_k: [
            {"x": 1.0, "y": 0.0, "yaw": 0.0, "size": 4, "score": 2.0}
        ])
    nav = FakeNav()
    task = FakeTask()
    task.params["max_time"] = 1e-9
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "exploration_incomplete"
    assert nav.plan_calls == []
    assert nav.calls == []
    assert task.result["completion_reason"] == "time_budget_exhausted"


def test_frontier_explore_planning_budget_does_not_claim_completion(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates",
        lambda *_a, **_k: [
            {"x": 1.0, "y": 0.0, "yaw": 0.0, "size": 4, "score": 2.0}
        ])
    nav = FakeNav(path_results=[
        {"ok": False, "reason": "unreachable", "status": 6}
    ])
    task = FakeTask()
    task.params["max_frontier_plan_probes"] = 1
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "exploration_incomplete"
    assert task.result["completion_reason"] == "planning_budget_exhausted"
    assert nav.calls == []


def test_frontier_explore_waypoint_budget_does_not_claim_completion(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates",
        lambda *_a, **_k: [
            {"x": 1.0, "y": 0.0, "yaw": 0.0, "size": 4, "score": 2.0}
        ])
    nav = FakeNav()
    task = FakeTask()
    task.params["max_frontiers"] = 1
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "exploration_incomplete"
    assert task.result["completion_reason"] == "waypoint_budget_exhausted"
    assert len(nav.calls) == 1


def test_frontier_explore_retries_after_temporary_no_lidar_range(
        monkeypatch, tmp_path):
    """A bearing-only sighting must not abort exploration before another view."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    def candidates(_map, _pose, visited, **_kwargs):
        if visited:
            return []
        return [
            {"x": 1.0, "y": 0.0, "yaw": 0.0, "size": 4, "score": 2.0}
        ]

    monkeypatch.setattr(orch_module, "select_frontier_candidates", candidates)
    orchestrator = make_orchestrator([], FakeNav())
    orchestrator._static_root = tmp_path
    observe_calls = 0

    def scripted_observe(store, room_name, view_idx, robot_pose, target_classes,
                         require_photos=False, use_lidar=True):
        nonlocal observe_calls
        observe_calls += 1
        common = {
            "class": "person",
            "confidence": 0.9,
            "bbox": [40, 10, 60, 90],
            "room": room_name,
            "view_index": view_idx,
        }
        if observe_calls == 1:
            unresolved = store.add_unresolved_observation({
                **common,
                "position_quality": "bearing_only",
                "world_x": None,
                "world_y": None,
            })
            return {
                "reason": "no_lidar_range",
                "detections": [unresolved],
                "resolved_count": 0,
            }
        store.add_observation({
            **common,
            "position_quality": "range_lidar",
            "world_x": 1.2,
            "world_y": 0.2,
        })
        return {"resolved_count": 1}

    orchestrator._observe_people_at_viewpoint = scripted_observe
    task = FakeTask()
    task.params["max_frontiers"] = 2

    orchestrator.run(task)

    assert observe_calls == 2
    assert task.status == "completed"
    assert task.result["targets_found"] == 1
    assert task.result["completion_reason"] == "reachable_frontiers_exhausted"


def test_frontier_explore_reports_unresolved_only_after_search_exhausted(
        monkeypatch, tmp_path):
    """An unresolved person is a terminal failure only after frontier search ends."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])
    orchestrator = make_orchestrator([], FakeNav())
    orchestrator._static_root = tmp_path

    def unresolved_observe(store, room_name, view_idx, robot_pose, target_classes,
                           require_photos=False, use_lidar=True):
        unresolved = store.add_unresolved_observation({
            "class": "person",
            "confidence": 0.9,
            "bbox": [40, 10, 60, 90],
            "room": room_name,
            "view_index": view_idx,
            "position_quality": "bearing_only",
            "world_x": None,
            "world_y": None,
        })
        return {
            "reason": "no_lidar_range",
            "detections": [unresolved],
            "resolved_count": 0,
        }

    orchestrator._observe_people_at_viewpoint = unresolved_observe
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_lidar_range"
    assert task.result["completion_reason"] == "reachable_frontiers_exhausted"
    assert len(task.result["detections"]) == 1
    assert task.result["resolved_detections"] == []


def test_select_next_frontier_zero_distance_no_divbyzero(monkeypatch):
    """簇恰在机器人位置: distance=0 → score=size/(1+0)=size, 不除零。"""
    fake = [{"center_cell": (0, 0), "center_world": (0.0, 0.0),
             "size": 5, "distance": 0.0}]
    monkeypatch.setattr(orch_module, "_find_frontier_clusters",
                        lambda *a, **k: fake)
    result = select_next_frontier(None, (0.0, 0.0, 0.0), [])
    assert result is not None
    assert result["score"] == 5.0  # 5/(1+0)=5


def test_select_next_frontier_respects_current_room_mission_radius(monkeypatch):
    fake_clusters = [
        {"center_cell": (0, 0), "center_world": (5.0, 0.0),
         "size": 100, "distance": 1.0},
        {"center_cell": (0, 0), "center_world": (2.0, 0.0),
         "size": 5, "distance": 2.0},
    ]
    monkeypatch.setattr(
        orch_module, "_find_frontier_clusters", lambda *a, **k: fake_clusters)

    result = select_next_frontier(
        None, (4.0, 0.0, 0.0), [], origin_pose=(0.0, 0.0, 0.0),
        max_radius=3.0)

    assert result is not None
    assert result["x"] == 2.0


def test_select_next_frontier_yaw_points_to_cluster(monkeypatch):
    """yaw 指向被选簇中心: atan2(by-robot_y, bx-robot_x)。"""
    fake = [{"center_cell": (0, 0), "center_world": (1.0, 1.0),
             "size": 4, "distance": math.hypot(1.0, 1.0)}]
    monkeypatch.setattr(orch_module, "_find_frontier_clusters",
                        lambda *a, **k: fake)
    result = select_next_frontier(None, (0.0, 0.0, 0.0), [])
    assert result is not None
    expected_yaw = math.atan2(1.0 - 0.0, 1.0 - 0.0)
    assert abs(result["yaw"] - expected_yaw) < 1e-6


# ============================================================================
# 测试 3b: GO2W_FRONTIER_ALPHA env 覆盖与防御 (H1/H2 回归)
# ============================================================================
def test_select_next_frontier_alpha_env_override(monkeypatch):
    """合法 env 覆盖: alpha=0.5 改变 score。"""
    fake = [{"center_cell": (0, 0), "center_world": (2.0, 0.0),
             "size": 10, "distance": 2.0}]
    monkeypatch.setattr(orch_module, "_find_frontier_clusters",
                        lambda *a, **k: fake)
    monkeypatch.setenv("GO2W_FRONTIER_ALPHA", "0.5")
    result = select_next_frontier(None, (0.0, 0.0, 0.0), [])
    # score = 10 / (1 + 2*0.5) = 10/2 = 5.0
    assert result is not None
    assert abs(result["score"] - 5.0) < 1e-6


def test_select_next_frontier_alpha_invalid_falls_back(monkeypatch):
    """非法 env (abc) → 回退默认 1.0, 不崩 (H1 防御)。"""
    fake = [{"center_cell": (0, 0), "center_world": (1.0, 0.0),
             "size": 3, "distance": 1.0}]
    monkeypatch.setattr(orch_module, "_find_frontier_clusters",
                        lambda *a, **k: fake)
    monkeypatch.setenv("GO2W_FRONTIER_ALPHA", "abc")
    result = select_next_frontier(None, (0.0, 0.0, 0.0), [])
    # 回退 alpha=1.0 → score = 3/(1+1*1) = 1.5
    assert result is not None
    assert abs(result["score"] - 1.5) < 1e-6


def test_select_next_frontier_alpha_negative_falls_back(monkeypatch):
    """负值 env → 回退默认 1.0, 不崩不反转 (H2 防御)。"""
    fake = [{"center_cell": (0, 0), "center_world": (1.0, 0.0),
             "size": 3, "distance": 1.0}]
    monkeypatch.setattr(orch_module, "_find_frontier_clusters",
                        lambda *a, **k: fake)
    monkeypatch.setenv("GO2W_FRONTIER_ALPHA", "-1.0")
    result = select_next_frontier(None, (0.0, 0.0, 0.0), [])
    assert result is not None
    assert abs(result["score"] - 1.5) < 1e-6  # 回退 1.0
