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
        # Fresh timestamp per call: simulates a live LiDAR stream. Required
        # because _build_observation_bundle now self-feeds the scan into
        # observation_sync with the scan's OWN stamp (审核 #1: real-stamp
        # self-feed), and bundle_for_detection time-aligns within 0.20s
        # tolerance — a fixed stale stamp would (correctly) be dropped.
        return {
            "ranges": ranges,
            "angle_min": -math.pi,
            "angle_increment": math.pi / 180.0,
            "range_min": 0.15,
            "range_max": 10.0,
            "timestamp": time.time(),
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
        self.begin_mission_calls = 0
        self._results = list(results or [])
        self._path_results = list(path_results or [])

    def begin_mission(self):
        self.begin_mission_calls += 1

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

    def cancel_current(self, reason="mission_cancel"):
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


def make_orchestrator(events, nav, node=None, ai_engine=None, with_observation_sync=False):
    kwargs = dict(
        node=node or FakeNode(emit_map=True),
        ai_engine=ai_engine or FakeAi(),
        ws_broadcast_fn=events.append,
        rooms_yaml_path=str(Path(__file__).parent / "_nonexistent_rooms.yaml"),
    )
    if with_observation_sync:
        # En-route worker requires observation_sync to time-align pose/scan/
        # detection into a single bundle. Tests that exercise en-route set
        # this flag; legacy at-viewpoint tests keep the None path (preserves
        # the historical fall-back test coverage).
        from nx_observation_sync import ObservationSynchronizer
        kwargs["observation_sync"] = ObservationSynchronizer()
    orchestrator = RoomSearchOrchestrator(**kwargs)
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


def _make_grid(width, height, data, resolution=0.1):
    """Build an OccupancyGrid-shaped object for pure planner tests."""
    from types import SimpleNamespace

    return SimpleNamespace(
        info=SimpleNamespace(
            resolution=float(resolution),
            width=int(width),
            height=int(height),
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=list(data),
    )


def test_long_connected_frontier_keeps_unvisited_goal_candidates():
    """Visiting one point must not suppress a room-sized frontier boundary."""
    from nx_frontier_planner import find_frontier_clusters

    width, height = 40, 30
    data = [
        0 if col < 20 else -1
        for row in range(height)
        for col in range(width)
    ]
    grid = _make_grid(width, height, data)
    robot_pose = (0.5, 1.5, 0.0)

    baseline = find_frontier_clusters(
        grid, robot_pose, [], min_cluster_size=3,
        revisit_radius=0.5, frontier_spacing_m=0.6)

    assert len(baseline) >= 3
    visited = [{
        "x": baseline[0]["center_world"][0],
        "y": baseline[0]["center_world"][1],
    }]
    remaining = find_frontier_clusters(
        grid, robot_pose, visited, min_cluster_size=3,
        revisit_radius=0.5, frontier_spacing_m=0.6)

    assert remaining
    assert max(item["center_world"][1] for item in remaining) > 2.0


def test_frontier_extraction_ignores_disconnected_free_island():
    """Only the robot's reachable known-free component may create goals."""
    from nx_frontier_planner import find_frontier_clusters

    width, height = 60, 30
    data = [-1] * (width * height)
    for row in range(5, 20):
        for col in range(5, 20):
            data[row * width + col] = 0
        for col in range(40, 55):
            data[row * width + col] = 0
    grid = _make_grid(width, height, data)

    candidates = find_frontier_clusters(
        grid, (1.0, 1.0, 0.0), [], min_cluster_size=1,
        revisit_radius=0.2, frontier_spacing_m=0.5)

    assert candidates
    assert all(item["center_world"][0] < 3.0 for item in candidates)


def test_noisy_large_frontier_selection_stays_within_cpu_budget():
    """A valid noisy map must not turn local gain into an O(K*N) stall."""
    from nx_frontier_planner import find_frontier_clusters

    width = height = 1000
    data = [
        -1 if row % 2 == 0 and col % 2 == 0 else 0
        for row in range(height)
        for col in range(width)
    ]
    grid = _make_grid(width, height, data, resolution=0.05)
    started = time.perf_counter()

    candidates = find_frontier_clusters(
        grid, (25.075, 25.025, 0.0), [], min_cluster_size=3,
        revisit_radius=1.0, frontier_spacing_m=1.5,
        max_candidates_per_cluster=64)

    elapsed = time.perf_counter() - started
    assert candidates
    assert len(candidates) <= 64
    assert elapsed < 5.0


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


def test_occupancy_frontier_candidates_request_safe_standoff():
    candidates = select_frontier_candidates(
        _make_half_unknown_map(), (0.0, 0.0, 0.0), [],
        min_cluster_size=3, revisit_radius=1.0,
    )

    assert candidates
    assert all(candidate["prefer_standoff"] is True for candidate in candidates)


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
    assert nav.begin_mission_calls == 1
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


def test_current_room_frontier_explore_feeds_lidar_c13_visibility_to_state(
        monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(
        orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    created = []

    class RecordingVisibilityTracker:
        def __init__(self, **kwargs):
            self.kwargs = dict(kwargs)
            self.observations = []
            created.append(self)

        def observe(self, map_msg, robot_pose, scan_snapshot):
            self.observations.append((map_msg, tuple(robot_pose), scan_snapshot))
            return self.snapshot(map_msg)

        def rank_candidates(self, _map_msg, _robot_pose, candidates):
            return list(candidates)

        def coverage_candidates(
                self, _map_msg, _robot_pose, _visited, *, limit=32):
            del limit
            return []

        def snapshot(self, _map_msg=None):
            return {
                "observed_cells": [{"x": 0.25, "y": 0.25}],
                "visual_coverage_ratio": 1.0,
                "coverage_cell_size_m": 0.5,
                "visual_range_m": 8.0,
                "scan_usable": True,
                "forward_clearance_m": 7.5,
                "scene_complexity": 0.05,
                "adaptive_step_m": 5.6,
            }

    monkeypatch.setattr(
        orch_module, "VisibilityCoverageTracker", RecordingVisibilityTracker)
    events = []
    task = FakeTask()
    task.params.update({
        "room": "__current__",
        "max_radius_m": 6.0,
        "initial_radius_m": 6.0,
        "stable_exhaustion_cycles": 1,
        "exclude_entrance_rear": False,
    })
    orchestrator = make_orchestrator(events, FakeNav())

    orchestrator.run(task)

    assert task.status == "completed"
    assert len(created) == 1
    tracker = created[0]
    assert tracker.kwargs["camera_hfov_rad"] == pytest.approx(
        math.radians(77.4))
    assert tracker.kwargs["visual_range_m"] == pytest.approx(8.0)
    assert tracker.observations
    scan = tracker.observations[0][2]
    assert scan["age_sec"] >= 0.0
    assert task.result["visual_coverage_ratio"] == pytest.approx(1.0)
    assert task.result["adaptive_step_m"] == pytest.approx(5.6)
    live_states = [
        event["data"] for event in events
        if event.get("type") == "search_room"
        and event.get("data", {}).get("observed_cells")
    ]
    assert live_states
    assert live_states[-1]["coverage_cell_size_m"] == pytest.approx(0.5)


def test_current_room_infers_and_reports_the_unique_entrance_gate(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(
        orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    inferred = []
    gate = {
        "center_x": 0.2,
        "center_y": 0.2,
        "yaw": 0.0,
        "width_m": 1.0,
    }

    def infer_gate(map_msg, mission_origin, **kwargs):
        inferred.append((map_msg, tuple(mission_origin), dict(kwargs)))
        return dict(gate)

    class CompleteVisibilityTracker:
        def __init__(self, **_kwargs):
            pass

        def observe(self, map_msg, _robot_pose, _scan_snapshot):
            return self.snapshot(map_msg)

        def rank_candidates(self, _map_msg, _robot_pose, candidates):
            return list(candidates)

        def snapshot(self, map_msg=None):
            observed = []
            if map_msg is not None:
                width = map_msg.info.width
                resolution = map_msg.info.resolution
                observed = [
                    {
                        "x": ((index % width) + 0.5) * resolution,
                        "y": ((index // width) + 0.5) * resolution,
                    }
                    for index, value in enumerate(map_msg.data)
                    if value == 0
                ]
            return {
                "observed_cells": observed,
                "observed_cell_count": len(observed),
                "visual_coverage_ratio": 1.0,
            }

    monkeypatch.setattr(orch_module, "infer_entrance_gate", infer_gate)
    monkeypatch.setattr(
        orch_module, "VisibilityCoverageTracker", CompleteVisibilityTracker)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])
    task = FakeTask()
    task.params.update({
        "room": "current_room",
        "stable_exhaustion_cycles": 1,
        "exclude_entrance_rear": True,
    })
    events = []
    orchestrator = make_orchestrator(events, FakeNav())

    orchestrator.run(task)

    assert inferred
    assert task.status == "completed"
    assert task.result["exploration_state"]["entrance_gate"] == gate
    assert task.result["entrance_gate"] == gate
    assert task.result["traversable_opening_count"] == 0
    assert task.result["explainable_coverage_ratio"] == pytest.approx(1.0)
    live_states = [
        event["data"] for event in events
        if event.get("type") == "search_room"
        and event.get("data", {}).get("entrance_gate") == gate
    ]
    assert live_states
    assert live_states[-1]["traversable_opening_count"] == 0
    assert live_states[-1]["explainable_coverage_ratio"] == pytest.approx(1.0)


def test_current_room_fails_closed_when_entrance_gate_is_unconfirmed(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(
        orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "infer_entrance_gate", lambda *_a, **_k: None)
    task = FakeTask()
    task.params.update({
        "room": "current_room",
        "exclude_entrance_rear": True,
    })
    nav = FakeNav()
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "entrance_gate_unconfirmed"
    assert nav.calls == []


def test_current_room_starts_without_an_entrance_gate_by_default(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(
        orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    def unexpected_gate_inference(*_args, **_kwargs):
        raise AssertionError("default current-room search must not infer an entrance gate")

    class CompleteVisibilityTracker:
        def __init__(self, **_kwargs):
            pass

        def observe(self, map_msg, _robot_pose, _scan_snapshot):
            return self.snapshot(map_msg)

        def rank_candidates(self, _map_msg, _robot_pose, candidates):
            return list(candidates)

        def snapshot(self, map_msg=None):
            observed = []
            if map_msg is not None:
                width = map_msg.info.width
                resolution = map_msg.info.resolution
                observed = [
                    {
                        "x": ((index % width) + 0.5) * resolution,
                        "y": ((index // width) + 0.5) * resolution,
                    }
                    for index, value in enumerate(map_msg.data)
                    if value == 0
                ]
            return {
                "observed_cells": observed,
                "observed_cell_count": len(observed),
                "visual_coverage_ratio": 1.0,
            }

    monkeypatch.setattr(
        orch_module, "infer_entrance_gate", unexpected_gate_inference)
    monkeypatch.setattr(
        orch_module, "VisibilityCoverageTracker", CompleteVisibilityTracker)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])
    task = FakeTask()
    task.params.update({
        "room": "current_room",
        "stable_exhaustion_cycles": 1,
    })
    nav = FakeNav()
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["entrance_gate"] is None
    assert task.result["exploration_state"]["entrance_gate"] is None
    assert task.result["global_search"]["valid"] is True
    assert task.result["global_search"]["entrance_excluded_edge_count"] == 0


def test_visibility_scan_snapshot_preserves_sensor_geometry_and_freshness():
    orchestrator = make_orchestrator([], FakeNav(), node=FakeNode())

    snapshot = orchestrator._visibility_scan_snapshot()

    assert len(snapshot["ranges"]) == 360
    assert snapshot["angle_min"] == pytest.approx(-math.pi)
    assert snapshot["angle_increment"] == pytest.approx(math.pi / 180.0)
    assert 0.0 <= snapshot["age_sec"] < 1.0


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


def test_new_frontier_mission_does_not_publish_previous_mask(monkeypatch):
    """Live coverage must not alternate with the preceding mission report."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    monkeypatch.setattr(
        orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    events = []
    states_during_init = []
    orchestrator = make_orchestrator(events, FakeNav())
    orchestrator._last_report = {
        "mission_id": "previous-mission",
        "observed_cells": [{"x": -9.0, "y": -9.0}],
    }

    def capture_new_mission_state(data):
        events.append(data)
        if (data.get("type") == "search_room"
                and data.get("data", {}).get("phase") == "INIT_SLAM"):
            states_during_init.append(orchestrator.get_navigation_state())
            orchestrator.cancel()

    orchestrator._ws = capture_new_mission_state
    orchestrator.run(FakeTask())

    assert states_during_init
    assert states_during_init[0]["last_report"] is None


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


def test_select_frontier_candidates_supports_legacy_cluster_callback_signature():
    from nx_frontier_planner import select_frontier_candidates as select

    def legacy_cluster_finder(
            map_msg, robot_pose, visited, min_cluster_size, revisit_radius):
        del map_msg, robot_pose, visited, min_cluster_size, revisit_radius
        return [{
            "center_cell": (1, 2),
            "center_world": (2.0, 1.0),
            "size": 5,
            "distance": math.sqrt(5.0),
        }]

    candidates = select(
        None, (0.0, 0.0, 0.0), [],
        cluster_finder=legacy_cluster_finder,
        frontier_spacing_m=0.8)

    assert len(candidates) == 1
    assert candidates[0]["x"] == pytest.approx(2.0)


def test_frontier_explore_starts_nearest_reachable_candidate_without_probe_sweep(monkeypatch):
    """Nearest reachable frontier starts without probing a farther alternative."""
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
        {"ok": True, "status": 4, "poses": 12, "path_length": 2.4},
    ])
    task = FakeTask()
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "completed"
    assert [call[:2] for call in nav.plan_calls] == [(0.0, 2.0)]
    assert [call[:2] for call in nav.calls] == [(0.0, 2.0)]
    assert task.result["completion_reason"] == "reachable_frontiers_exhausted"
    assert task.result["frontier_plan_rejections"] == 0
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


def test_frontier_explore_plan_probe_cap_is_per_cycle_not_mission(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    blocked = {
        "x": 1.0, "y": 0.0, "yaw": 0.0, "size": 4, "score": 2.0,
        "center_cell": (0, 1), "distance": 1.0,
        "information_gain": 4.0,
    }
    reachable = {
        "x": 2.0, "y": 0.0, "yaw": 0.0, "size": 3, "score": 1.0,
        "center_cell": (0, 2), "distance": 2.0,
        "information_gain": 3.0,
    }

    def candidates(_map, _pose, visited, **_kwargs):
        return [] if visited else [dict(blocked), dict(reachable)]

    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", candidates)
    nav = FakeNav(path_results=[
        {"ok": False, "reason": "unreachable", "status": 6},
        {"ok": True, "status": 4, "poses": 3, "path_length": 2.0},
    ])
    task = FakeTask()
    task.params["max_frontier_plan_probes"] = 1
    task.params["max_plan_probes_per_cycle"] = 1
    task.params["max_frontier_rejections"] = 1
    task.params["stable_exhaustion_cycles"] = 1
    orchestrator = make_orchestrator([], nav)

    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["completion_reason"] == "reachable_frontiers_exhausted"
    assert [call[:2] for call in nav.plan_calls] == [(1.0, 0.0), (2.0, 0.0)]
    assert [call[:2] for call in nav.calls] == [(2.0, 0.0)]
    assert task.result["frontier_plan_probes"] == 2


def test_frontier_explore_reports_dynamic_radius_and_tiles(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    seen_radii = []
    seen_spacings = []

    def candidates(_map, _pose, visited, **kwargs):
        radius = kwargs.get("max_radius")
        seen_radii.append(radius)
        seen_spacings.append(kwargs.get("frontier_spacing_m"))
        if visited:
            return []
        if radius is not None and radius >= 12.0:
            return [{
                "x": 8.0, "y": 0.0, "yaw": 0.0, "size": 4,
                "center_cell": (0, 8), "distance": 8.0,
                "information_gain": 4.0, "score": 1.0,
            }]
        return []

    monkeypatch.setattr(orch_module, "select_frontier_candidates", candidates)
    task = FakeTask()
    task.params.update({
        "room": "__current__",
        "max_radius_m": 12.0,
        "initial_radius_m": 6.0,
        "radius_step_m": 6.0,
        "tile_size_m": 6.0,
        "frontier_spacing_m": 0.8,
        "stable_exhaustion_cycles": 1,
        "visibility_aware_exploration": False,
        "exclude_entrance_rear": False,
    })
    orchestrator = make_orchestrator([], FakeNav())

    orchestrator.run(task)

    assert task.status == "completed"
    assert seen_radii[:2] == [6.0, 12.0]
    assert task.result["active_radius_m"] == pytest.approx(12.0)
    assert task.result["max_radius_m"] == pytest.approx(12.0)
    assert task.result["tile_size_m"] == pytest.approx(6.0)
    assert seen_spacings and all(
        spacing == pytest.approx(0.8) for spacing in seen_spacings)
    assert task.result["exploration_state"]["frontier_spacing_m"] == pytest.approx(0.8)
    assert [1, 0] in task.result["visited_tiles"]


def test_frontier_explore_waits_for_stable_exhaustion(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    calls = []

    def candidates(*_args, **_kwargs):
        calls.append(1)
        return []

    monkeypatch.setattr(orch_module, "select_frontier_candidates", candidates)
    task = FakeTask()
    task.params.update({
        "room": "__current__",
        "max_radius_m": 6.0,
        "initial_radius_m": 6.0,
        "stable_exhaustion_cycles": 3,
        "visibility_aware_exploration": False,
        "exclude_entrance_rear": False,
    })
    orchestrator = make_orchestrator([], FakeNav())

    orchestrator.run(task)

    assert task.status == "completed"
    assert len(calls) == 3
    assert task.result["exhaustion_streak"] == 3


def test_large_room_defaults_do_not_stop_at_fifteen_waypoints(monkeypatch):
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    def candidates(_map, _pose, visited, **_kwargs):
        if len(visited) >= 16:
            return []
        x = float(len(visited) + 1)
        return [{
            "x": x, "y": 0.0, "yaw": 0.0, "size": 4,
            "center_cell": (0, int(x)), "distance": 1.0,
            "information_gain": 4.0, "score": 1.0,
        }]

    monkeypatch.setattr(orch_module, "select_frontier_candidates", candidates)
    from nx_mission_schema import SearchMissionRequest
    task = FakeTask()
    task.params = SearchMissionRequest.current_room(
        ["person"], request_id="large-room-16").to_task_params()
    task.params["stable_exhaustion_cycles"] = 1
    task.params["exclude_entrance_rear"] = False
    orchestrator = make_orchestrator([], FakeNav())

    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["waypoints_reached"] == 16
    assert task.result["completion_reason"] == "reachable_frontiers_exhausted"


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


# ============================================================================
# 测试 4: budget 锁定 + en-route observer (Task 1: 导航期间连续 YOLO 标人)
# ============================================================================
@pytest.mark.parametrize(
    ("env_value", "expected_timeout"),
    [(None, 90.0), ("12.5", 12.5)],
)
def test_send_goal_timeout_uses_configurable_90s_default(
        monkeypatch, env_value, expected_timeout):
    """Long frontiers get 90s by default and commissioning can override it."""
    from nx_navigation_gateway import MissionNavigationPort, NavigationGateway

    if env_value is None:
        monkeypatch.delenv("GO2W_FRONTIER_NAV_TIMEOUT", raising=False)
    else:
        monkeypatch.setenv("GO2W_FRONTIER_NAV_TIMEOUT", env_value)

    class NeverTerminalActionPort:
        def __init__(self):
            self.canceled = []
            self.state = {"status": "idle", "drained": True, "healthy": True}

        def submit(self, _x, _y, _yaw):
            self.state = {
                "status": "active",
                "drained": False,
                "healthy": True,
                "generation": 1,
            }
            return {"ok": True, "generation": 1}

        def cancel(self, reason):
            self.canceled.append(reason)
            return True

        def tick(self):
            return dict(self.state)

        def get_state(self):
            return dict(self.state)

    clock = [0.0]
    action = NeverTerminalActionPort()
    gateway = NavigationGateway(
        action_port=action,
        monotonic=lambda: clock[0],
        sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds),
        poll_interval=200.0,
    )

    result = MissionNavigationPort(gateway).send_goal_and_wait(36.0, 0.0, 0.0)

    assert result == {"ok": False, "reason": "timeout"}
    assert clock[0] == pytest.approx(expected_timeout)
    assert action.canceled == ["navigation_timeout"]


def test_frontier_explore_max_time_defaults_1800s(monkeypatch):
    """大房间探索默认给 30 分钟，同时仍受电量和取消门禁约束。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])
    orchestrator = make_orchestrator([], FakeNav())
    task = FakeTask()
    task.params.pop("max_time", None)  # 用默认值

    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["time_budget_sec"] == 1800


def test_frontier_explore_observes_people_en_route(monkeypatch, tmp_path):
    """导航期间 observer 应采到 FakeAi 缓存里的人, 到达后 ingest 进 store。

    FakeNav.send_goal_and_wait 阻塞 0.3s 模拟导航耗时, 期间 observer 线程
    每 0.05s 读一次 detection snapshot。用 monkeypatch spy _ingest_en_route_samples
    验证 en-route 路径独立于到点稳态采样被真的走通 (不污染生产类计数器)。
    """
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    def candidates(_map, _pose, visited, **_kwargs):
        if visited:
            return []
        return [{"x": 1.0, "y": 0.0, "yaw": 0.0, "size": 4, "score": 2.0}]
    monkeypatch.setattr(orch_module, "select_frontier_candidates", candidates)

    class SlowNav(FakeNav):
        def send_goal_and_wait(self, x, y, yaw, frame_id="map"):
            time.sleep(0.3)  # 让 observer 线程有机会采样
            return super().send_goal_and_wait(x, y, yaw, frame_id=frame_id)

    orch = make_orchestrator([], SlowNav(), ai_engine=FakeAi(),
                              with_observation_sync=True)
    orch._static_root = tmp_path
    task = FakeTask()
    # max_frontiers=2: 1st frontier reached, 2nd iter queries candidates with
    # visited=[{x:1.0}] → [] → target=None → break → completion_reason=
    # "reachable_frontiers_exhausted" → completed. With max_frontiers=1 the
    # loop exits with "waypoint_budget_exhausted" → exploration_incomplete.
    task.params["max_frontiers"] = 2
    task.params["en_route_sample_interval"] = 0.05  # 快速采样

    # spy: 替换 ingest 方法, 不碰生产类属性
    ingest_calls = []
    real_ingest = orch._ingest_en_route_samples if hasattr(
        orch, "_ingest_en_route_samples") else None

    def _spy(samples, store, room_name, require_photos):
        ingest_calls.append(len(samples))
        if real_ingest is not None:
            return real_ingest(samples, store, room_name, require_photos)
        return 0
    monkeypatch.setattr(orch, "_ingest_en_route_samples", _spy)

    orch.run(task)

    assert task.status == "completed"
    assert sum(ingest_calls) >= 1, (
        "en-route ingest 未触发 — observer 路径未走通 (spy 应捕获 ≥1 个 sample)")


def test_en_route_observer_dedups_by_capture_stamp(monkeypatch, tmp_path):
    """同一推理帧 (captured_at 相同) 不应被 observer 重复入队。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    fixed_stamp = time.time()
    class StickyAi(FakeAi):
        def get_person_detection_snapshot(self):
            snap = super().get_person_detection_snapshot()
            snap["timestamp"] = fixed_stamp  # 同一帧时间戳不变
            return snap

    orch = make_orchestrator([], FakeNav(), ai_engine=StickyAi(),
                              with_observation_sync=True)
    orch._static_root = tmp_path
    stop_event = threading.Event()
    # max_duration_s=0.3: synchronous call self-terminates (production callers
    # pass None and rely on stop_event being set after send_goal_and_wait).
    # sample_interval=0.05 → ~6 iterations, but same stamp dedups to 1 sample.
    samples = orch._observe_en_route(
        stop_event, ["person"], sample_interval=0.05, max_samples=12,
        max_duration_s=0.3)
    stop_event.set()
    # 即使循环多次, captured_at 去重后 samples 里同一 stamp 最多 1 个
    stamps = [s["captured_at"] for s in samples]
    assert len(stamps) == len(set(stamps)), "同帧被重复入队 (去重失效)"


def test_frontier_progress_monitor_cancels_a_diverging_navigation():
    orch = make_orchestrator([], FakeNav())
    stop_event = threading.Event()
    holder = {}

    class DivergingExploration:
        def observe_navigation_pose(self, pose):
            assert pose == (3.0, 2.0, 0.5)
            return {
                "ok": False,
                "reason": "navigation_diverging",
                "distance_to_goal_m": 4.2,
                "allowed_distance_m": 1.8,
            }

    class CancelingNav:
        def __init__(self):
            self.cancel_reasons = []

        def cancel_current(self, reason="mission_cancel"):
            self.cancel_reasons.append(reason)
            return True

    nav = CancelingNav()
    orch._get_live_robot_pose = lambda: (3.0, 2.0, 0.5)

    orch._monitor_frontier_navigation_progress(
        stop_event, DivergingExploration(), nav, holder, 0.01)

    assert nav.cancel_reasons == ["navigation_diverging"]
    assert holder["failure"]["reason"] == "navigation_diverging"


def test_frontier_progress_monitor_cancels_goal_that_becomes_unreachable():
    orch = make_orchestrator([], FakeNav())
    stop_event = threading.Event()
    holder = {}

    class DynamicallyBlockedExploration:
        def observe_navigation_pose(self, _pose):
            return {"ok": True}

        def revalidate_current_goal(self):
            return {
                "ok": False,
                "reason": "goal_became_unreachable",
                "goal_revalidation_failures": 2,
            }

    class CancelingNav:
        def __init__(self):
            self.cancel_reasons = []

        def cancel_current(self, reason="mission_cancel"):
            self.cancel_reasons.append(reason)
            return True

    nav = CancelingNav()
    orch._get_live_robot_pose = lambda: (0.0, 0.0, 0.0)

    orch._monitor_frontier_navigation_progress(
        stop_event, DynamicallyBlockedExploration(), nav, holder, 0.01,
        path_revalidation_interval=0.01)

    assert nav.cancel_reasons == ["goal_became_unreachable"]
    assert holder["failure"]["reason"] == "goal_became_unreachable"


def test_en_route_observer_bounded_queue(monkeypatch, tmp_path):
    """worker queue 不超过 max_samples (防 100 张 frame 堆积)。"""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    class FreshStampAi(FakeAi):
        def __init__(self):
            self._n = 0
        def get_person_detection_snapshot(self):
            self._n += 1
            snap = super().get_person_detection_snapshot()
            # Fresh near-now stamp (strictly increasing) so that real-stamp
            # self-feed in _build_observation_bundle stays inside tolerance
            # and the bundle succeeds. Older code faked captured_at for the
            # pose/scan stamp; Fix 2 requires a real timestamp.
            snap["timestamp"] = time.time() + self._n * 0.01
            return snap

    orch = make_orchestrator([], FakeNav(), ai_engine=FreshStampAi(),
                              with_observation_sync=True)
    orch._static_root = tmp_path
    stop_event = threading.Event()
    # max_duration_s=0.3: synchronous self-terminate. Production callers
    # pass None and rely on stop_event from the frontier loop.
    samples = orch._observe_en_route(
        stop_event, ["person"], sample_interval=0.02, max_samples=4,
        max_duration_s=0.3)
    stop_event.set()
    assert len(samples) <= 4, f"bounded queue 失效: {len(samples)} > 4"


def test_en_route_observer_rolls_window_and_runs_until_stop_event(monkeypatch, tmp_path):
    """Regression: rolling window + runs until stop_event (production path).

    Spawn _observe_en_route with max_duration_s=None (production default).
    Fresh stamps each call → samples roll when full. Main thread sleeps 0.3s,
    sets stop_event, joins. Asserts (a) len(samples) == cap (held, not grown),
    (b) the 1st captured_at < the last (proves pop(0) + new arrival happened).
    Locks production semantics against early-termination regression.
    """
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)

    class FreshStampAi(FakeAi):
        def __init__(self):
            self._n = 0
        def get_person_detection_snapshot(self):
            self._n += 1
            snap = super().get_person_detection_snapshot()
            # Fresh, strictly-increasing stamp within ~0.1s of wall clock so
            # that real-stamp self-feed (pose@time.time, scan@scan_stamp)
            # stays inside bundle_for_detection tolerance (0.20s). Older
            # future-stamp variants (time.time()+n) were unaligned by >1s.
            snap["timestamp"] = time.time() + self._n * 0.01
            return snap

    orch = make_orchestrator([], FakeNav(), ai_engine=FreshStampAi(),
                              with_observation_sync=True)
    orch._static_root = tmp_path
    stop_event = threading.Event()
    samples = []
    worker_error = []

    def _worker():
        try:
            samples.extend(orch._observe_en_route(
                stop_event, ["person"], sample_interval=0.05, max_samples=3,
                max_duration_s=None))
        except Exception as exc:  # pragma: no cover - surface unexpected failures
            worker_error.append(exc)

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    # Let the worker run long enough to overflow cap=3 many times over.
    time.sleep(0.3)
    stop_event.set()
    t.join(timeout=2.0)

    assert not worker_error, f"worker crashed: {worker_error}"
    assert not t.is_alive(), "worker did not terminate after stop_event.set()"
    # Cap held: rolling window never grows beyond max_samples.
    assert len(samples) == 3, (
        f"expected cap=3 held under rolling, got {len(samples)} "
        "(rolling-window regression: worker terminated early or overgrew cap)")
    # Rolling proof: oldest was dropped, newest kept → monotonic increase.
    stamps = [s["captured_at"] for s in samples]
    assert len(stamps) == len(set(stamps)), "dedup regressed"
    assert stamps[0] < stamps[-1], (
        f"rolling-window did not drop oldest: stamps[0]={stamps[0]} "
        f"not < stamps[-1]={stamps[-1]} (samples are not fresh; pop(0) failed)")


def test_build_observation_bundle_feeds_pointcloud_dataclass(monkeypatch, tmp_path):
    """I1 regression: en-route bundle must carry pointcloud depth evidence.

    `_pointcloud_snapshot()` returns a PointCloudSnapshot dataclass (no
    timestamp field). `_build_observation_bundle` should feed it into the
    sync with `stamp=time.time()` (the honest read time), NOT skip the
    feed. Asserts `bundle_result["bundle"].cloud is not None` when a real
    PointCloudSnapshot is available.
    """
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    from nx_person_localizer import PointCloudSnapshot

    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    orch = make_orchestrator([], FakeNav(), ai_engine=FakeAi(),
                              with_observation_sync=True)
    orch._static_root = tmp_path

    fake_cloud = PointCloudSnapshot(
        points=[[0.5, 0.0, 0.5], [0.6, 0.0, 0.6], [0.7, 0.0, 0.7]],
        frame_id="base_link",
    )
    monkeypatch.setattr(orch, "_pointcloud_snapshot", lambda: fake_cloud)

    snapshot = FakeAi().get_person_detection_snapshot()
    bundle_result = orch._build_observation_bundle(snapshot, False)

    assert bundle_result is not None, "bundle build returned None"
    bundle = bundle_result.get("bundle")
    assert bundle is not None, "bundle_for_detection returned None"
    assert bundle.cloud is not None, (
        "PointCloudSnapshot was not fed into observation_sync (I1 regression: "
        "cloud depth evidence lost)")
    assert bundle.cloud.value is fake_cloud, (
        f"expected the fed PointCloudSnapshot back, got {bundle.cloud.value!r}")


# ============================================================================
# 测试 5: REPORT 段 ROI coverage + 4-state completion_status (Task 3)
# ============================================================================
def test_frontier_explore_report_completed_when_full_coverage(monkeypatch):
    """全自由 map + ROI → completion_status=completed, explored_ratio=1.0."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])

    orchestrator = make_orchestrator([], FakeNav())
    task = FakeTask()
    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["completion_status"] == "completed"
    assert task.result["coverage_valid"] is True
    assert task.result["explored_ratio"] == 1.0
    assert task.result["enclosed_unknown_regions"] == []


def test_completion_status_prefers_bounded_room_coverage():
    """地图外未知 padding 不应压低已闭合房间的完成状态。"""
    orchestrator = make_orchestrator([], FakeNav())
    status = orchestrator._derive_completion_status(
        "reachable_frontiers_exhausted",
        {
            "coverage_valid": True,
            "roi": {"type": "polygon"},
            "explored_ratio": 0.2,
            "bounded_explored_ratio": 1.0,
            "enclosed_unknown_regions": [],
        },
    )
    assert status == "completed"


def test_completion_status_is_incomplete_while_a_traversable_opening_is_blocked():
    orchestrator = make_orchestrator([], FakeNav())

    status = orchestrator._derive_completion_status(
        "traversable_opening_blocked",
        {
            "coverage_valid": True,
            "roi": {"type": "circle"},
            "explored_ratio": 1.0,
            "bounded_explored_ratio": 1.0,
            "enclosed_unknown_regions": [],
        },
    )

    assert status == "incomplete"


def test_dynamic_circle_roi_cannot_complete_with_most_area_still_unknown():
    orchestrator = make_orchestrator([], FakeNav())
    status = orchestrator._derive_completion_status(
        "reachable_frontiers_exhausted",
        {
            "coverage_valid": True,
            "roi": {"type": "circle"},
            "explored_ratio": 0.011631,
            "bounded_explored_ratio": 0.978814,
            "exterior_unknown_cells": 58876,
            "enclosed_unknown_regions": [],
        },
    )

    assert status == "incomplete"


def test_frontier_explore_report_completed_with_gaps_for_walled_pocket(monkeypatch):
    """被障碍围死的未知区 → completion_status=completed_with_gaps."""
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")

    width = height = 7
    data = []
    for r in range(height):
        for c in range(width):
            if 2 <= r <= 4 and 2 <= c <= 4:
                data.append(-1)
            elif 1 <= r <= 5 and 1 <= c <= 5:
                data.append(100)
            else:
                data.append(0)
    pocket_map = _make_fake_map(width=width, height=height, fill_value=0)
    pocket_map.data = data

    class PocketNode(FakeNode):
        def __init__(self):
            super().__init__(emit_map=False)
            self._pocket = pocket_map

        def create_subscription(self, msg_type, topic, callback, qos):
            handle = super().create_subscription(msg_type, topic, callback, qos)
            if topic == "/map_frontier":
                def _emit():
                    time.sleep(0.05)
                    callback(self._pocket)
                threading.Thread(target=_emit, daemon=True).start()
            return handle

    monkeypatch.setattr(orch_module, "_OccupancyGrid", pocket_map.__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])

    orchestrator = make_orchestrator([], FakeNav(), node=PocketNode())
    task = FakeTask()
    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["completion_status"] == "completed_with_gaps"
    assert len(task.result["enclosed_unknown_regions"]) == 1


def test_frontier_explore_report_coverage_unverified_without_map(monkeypatch):
    """地图不可用 (coverage None) → completion_status=coverage_unverified.

    用 monkeypatch 让 _compute_coverage 返回 None (模拟地图无效 / 几何不可解析)。
    """
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])
    monkeypatch.setattr(orch_module, "_compute_coverage", lambda *a, **k: None)

    orchestrator = make_orchestrator([], FakeNav())
    task = FakeTask()
    orchestrator.run(task)

    assert task.status == "completed"
    assert task.result["completion_status"] == "coverage_unverified"
    assert "explored_ratio" in task.result
    assert task.result["explored_ratio"] is None


def test_frontier_explore_invalid_room_polygon_falls_back_to_circle(monkeypatch):
    """REPORT 段畸形的 room_polygon 必须静默回退到 circle ROI.

    验证 Finding 1 的防御性回退: _build_coverage_roi 在输入为非列表 / 元素
    不合法 / 元素数量越界 / 含 NaN 等情况下都不得抛出, 必须返回 circle ROI。
    REPORT 段 (建 coverage_roi 时) 用此 helper, 因此 helper 行为即覆盖该路径。
    """
    if not _ensure_person_deps():
        pytest.skip("frontier deps not importable")
    monkeypatch.setattr(orch_module, "_OccupancyGrid", _make_fake_map().__class__)
    monkeypatch.setattr(
        orch_module, "select_frontier_candidates", lambda *_a, **_k: [])

    orchestrator = make_orchestrator([], FakeNav())

    # 直接单元测试 helper: 每种非法输入都回退到 circle
    roi = orchestrator._build_coverage_roi(
        room_polygon="not-a-list",
        mission_origin=(1.0, 2.0),
        max_radius=5.0)
    assert roi["type"] == "circle"
    assert roi["center"] == [1.0, 2.0]
    assert roi["radius"] == 5.0

    # 过短 / 过长
    assert orchestrator._build_coverage_roi(
        [(0, 0), (1, 1)], (0, 0), None)["type"] == "circle"
    assert orchestrator._build_coverage_roi(
        [(0, 0)] * 101, (0, 0), None)["type"] == "circle"

    # 元素非 pair / 非数值 / 非有限
    assert orchestrator._build_coverage_roi(
        [(0, 0), (1, "x"), (2, 2)], (0, 0), None)["type"] == "circle"
    assert orchestrator._build_coverage_roi(
        [(0, 0), (float("nan"), 1), (2, 2)], (0, 0), None)["type"] == "circle"

    # 合法 polygon 应原样返回
    valid = orchestrator._build_coverage_roi(
        [(0, 0), (1, 0), (1, 1), (0, 1)], (0, 0), None)
    assert valid["type"] == "polygon"
    assert valid["points"] == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
