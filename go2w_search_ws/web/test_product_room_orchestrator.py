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

from nx_room_orchestrator import RoomSearchOrchestrator  # noqa: E402
from nx_person_mission import TargetMissionStore  # noqa: E402


ROOM_NAME = "客厅"


def test_orchestrator_restores_latest_persisted_map_markers(tmp_path):
    store = TargetMissionStore("mission-restored", mission_root=tmp_path)
    marker = {
        "id": "person_001", "class": "person",
        "world_x": 1.2, "world_y": 2.3,
        "photo_url": "/missions/mission-restored/person_001_annotated.jpg",
    }
    report = {
        "mission_id": "mission-restored",
        "room": "__frontier__",
        "status": "completed",
        "end_time": 123.0,
        "detections": [marker],
    }
    store.save_report(report)

    orchestrator = RoomSearchOrchestrator(mission_root=tmp_path)
    state = orchestrator.get_navigation_state()

    assert state["last_report"] == report
    assert state["target_markers"] == [marker]
    assert state["person_markers"] == [marker]


class FakeTask:
    def __init__(self):
        self.type = "search_room"
        self.params = {
            "room": ROOM_NAME,
            "target_classes": ["person"],
            "require_photos": True,
            "mark_on_map": True,
            "search_strategy": "next_best_view",
            "use_lidar_person_range": True,
            "max_views": 1,
            "coverage_threshold": 0.25,
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


class InvalidPhotoAi(FakeAi):
    def get_person_detection_snapshot(self):
        snapshot = super().get_person_detection_snapshot()
        snapshot["frame"] = np.array([1], dtype=np.uint8)
        return snapshot


class StaleDetectionAi(FakeAi):
    def get_person_detection_snapshot(self):
        snapshot = super().get_person_detection_snapshot()
        snapshot["timestamp"] = time.time() - 10.0
        return snapshot


class MockDetectionAi(FakeAi):
    def get_person_detection_snapshot(self):
        snapshot = super().get_person_detection_snapshot()
        snapshot["source"] = "mock"
        for detection in snapshot["detections"]:
            detection["source"] = "mock"
        return snapshot


class RefreshingDetectionAi(FakeAi):
    def __init__(self):
        self.calls = 0

    def get_person_detection_snapshot(self):
        self.calls += 1
        snapshot = super().get_person_detection_snapshot()
        if self.calls == 1:
            snapshot["timestamp"] = time.time() - 10.0
        return snapshot


class FakeNode:
    def __init__(self, scan_timestamp=None):
        self._lock = threading.RLock()
        self._odom_x = 0.2
        self._odom_y = 0.2
        self._imu_yaw = 0.0
        self._odom_count = 1
        self._odom_t = time.time()
        self._scan_timestamp = time.time() if scan_timestamp is None else scan_timestamp

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

    def get_pointcloud_snapshot(self):
        return {
            "frame_id": "base_link",
            "points": [
                (1.98, -0.02, -0.1),
                (2.00, 0.00, 0.5),
                (2.02, 0.02, 1.1),
            ],
            "timestamp": time.time(),
            "age_sec": 0.0,
        }


class NoPoseNode(FakeNode):
    def __init__(self):
        super().__init__()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._imu_yaw = 0.0
        self._odom_count = 0


class StaleScanNode(FakeNode):
    def __init__(self):
        super().__init__(scan_timestamp=time.time() - 10.0)


class StaleOdomNode(FakeNode):
    def __init__(self):
        super().__init__()
        self._odom_t = time.time() - 10.0


class InvalidForwardRangeNode(FakeNode):
    def get_scan_snapshot(self):
        snapshot = super().get_scan_snapshot()
        snapshot["ranges"][90] = 2.0
        snapshot["ranges"][180] = 0.0
        return snapshot


class RetryLidarNode(FakeNode):
    def __init__(self):
        super().__init__()
        self.scan_calls = 0

    def get_scan_snapshot(self):
        self.scan_calls += 1
        snapshot = super().get_scan_snapshot()
        snapshot["ranges"][90] = 2.0  # keep the scan valid away from camera bearing
        snapshot["ranges"][180] = 2.0 if self.scan_calls >= 3 else 0.0
        return snapshot


class LeftBearingRangeNode(FakeNode):
    def get_scan_snapshot(self):
        snapshot = super().get_scan_snapshot()
        snapshot["ranges"][180] = 0.0
        snapshot["ranges"][270] = 2.0
        return snapshot


class MissingScanNode:
    def get_scan_snapshot(self):
        return {}


class FakeNav:
    def __init__(self, results=None):
        self.calls = []
        self._results = list(results or [])

    def wait_for_server(self, timeout=2.0):
        return True

    def send_goal_and_wait(self, x, y, yaw, frame_id="map"):
        self.calls.append((float(x), float(y), float(yaw), frame_id))
        if self._results:
            return self._results.pop(0)
        return {"ok": True, "status": 4}

    def cancel_current(self):
        return True


def write_rooms_yaml(tmp_path, calibrated=True):
    rooms_yaml = tmp_path / "rooms.yaml"
    rooms_yaml.write_text(
        f"""
frame_id: map
version: "1.0"
rooms:
  - name: {ROOM_NAME}
    calibrated: {str(bool(calibrated)).lower()}
    aliases: []
    nav_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
    search_area:
      origin_x: 0.0
      origin_y: 0.0
      width: 1.0
      height: 1.0
      spacing: 1.0
    target_classes:
      - person
""",
        encoding="utf-8",
    )
    return rooms_yaml


def make_orchestrator(tmp_path, events, nav, node=None, ai_engine=None,
                      calibrated=True):
    rooms_yaml = write_rooms_yaml(tmp_path, calibrated=calibrated)
    orchestrator = RoomSearchOrchestrator(
        node=node or FakeNode(),
        ai_engine=ai_engine or FakeAi(),
        ws_broadcast_fn=events.append,
        rooms_yaml_path=str(rooms_yaml),
    )
    orchestrator._nav = nav
    orchestrator._static_root = tmp_path
    return orchestrator


def test_product_search_creates_lidar_person_marker_and_report(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav)
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "completed"
    assert len(nav.calls) >= 2
    report = task.result
    assert report["targets_found"] == 1
    detection = report["detections"][0]
    assert detection["position_quality"] == "range_lidar"
    assert detection["position_dimension"] == 3
    assert detection["world_z"] == pytest.approx(0.5)
    assert detection["height_source"] == "mid360_pointcloud"
    assert detection["photo_url"].endswith("person_001_annotated.jpg")
    marker_events = [event for event in events if event.get("type") == "person_markers"]
    assert marker_events
    marker = marker_events[-1]["data"]["markers"][0]
    assert marker["position_quality"] == "range_lidar"
    assert marker["photo_url"].endswith("person_001_annotated.jpg")


def test_product_search_rejects_mock_video_detections(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(
        tmp_path, events, nav, ai_engine=MockDetectionAi())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "mock_detection_source"
    assert task.result["source"] == "mock"
    assert task.result["observation_valid"] is False


def test_product_search_refuses_uncalibrated_room_before_navigation(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(
        tmp_path, events, nav, calibrated=False)
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "room_uncalibrated"
    assert task.result["room"] == ROOM_NAME
    assert nav.calls == []


def test_product_search_stops_when_observed_coverage_reaches_threshold(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav)
    task = FakeTask()
    task.params.update({"max_views": 8, "coverage_threshold": 0.9})

    orchestrator.run(task)

    assert task.status == "completed"
    assert len(nav.calls) > 2  # a camera wedge cannot cover the room from one view
    assert task.result["coverage_complete"] is True
    assert task.result["coverage_ratio"] == 1.0
    coverage_events = [
        event for event in events
        if event.get("type") == "search_room"
        and event.get("data", {}).get("phase") == "ACTIVE_SEARCH"
    ]
    assert coverage_events
    assert coverage_events[-1]["data"]["coverage_ratio"] == 1.0
    assert coverage_events[-1]["data"]["room_area"]["width"] == 1.0


def test_product_search_fails_instead_of_claiming_incomplete_coverage(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav)
    task = FakeTask()
    task.params.update({
        "max_views": 1,
        "visual_range_m": 0.1,
        "coverage_threshold": 0.9,
    })

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "coverage_incomplete"
    assert task.result["coverage_ratio"] == 0.25
    assert task.result["coverage_complete"] is False
    assert task.result["detections"][0]["photo_url"].endswith("person_001_annotated.jpg")
    assert not [event for event in events if event.get("type") == "mission_report"]


def test_product_search_fails_when_all_viewpoint_nav_goals_abort(tmp_path):
    events = []
    nav = FakeNav(results=[
        {"ok": True, "status": 4},
        {"ok": False, "reason": "aborted", "status": 6},
        {"ok": False, "reason": "aborted", "status": 6},
    ])
    orchestrator = make_orchestrator(tmp_path, events, nav)
    task = FakeTask()
    task.params["max_views"] = 2

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] in {"aborted", "nav_aborted", "no_viewpoint_reached"}
    assert not [event for event in events if event.get("type") == "mission_report"]


def test_product_search_rejects_disabled_lidar_range(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav)
    task = FakeTask()
    task.params["use_lidar_person_range"] = False

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "lidar_required"


def test_product_search_fails_when_scan_snapshot_missing(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav, node=MissingScanNode())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_scan"


def test_product_search_fails_when_required_photo_artifact_cannot_be_saved(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(
        tmp_path, events, nav, ai_engine=InvalidPhotoAi())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "photo_artifact_failed"
    assert not [event for event in events if event.get("type") == "mission_report"]


def test_product_search_rejects_stale_detection_frame_for_map_localization(
        tmp_path, monkeypatch):
    monkeypatch.setenv("GO2W_DETECTION_WAIT_SEC", "0.01")
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(
        tmp_path, events, nav, ai_engine=StaleDetectionAi())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "stale_detection_frame"
    assert not [event for event in events if event.get("type") == "mission_report"]


def test_product_search_waits_for_next_fresh_detection_frame(tmp_path, monkeypatch):
    monkeypatch.setenv("GO2W_DETECTION_WAIT_SEC", "0.5")
    events = []
    nav = FakeNav()
    ai = RefreshingDetectionAi()
    orchestrator = make_orchestrator(tmp_path, events, nav, ai_engine=ai)
    task = FakeTask()

    orchestrator.run(task)

    assert ai.calls >= 2
    assert task.status == "completed"
    assert task.result["targets_found"] == 1


def test_product_current_room_fails_no_pose_without_live_odom(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav, node=NoPoseNode())
    task = FakeTask()
    task.params["room"] = "__current__"

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_pose"
    assert nav.calls == []


def test_product_current_room_fails_no_pose_with_stale_odom(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav, node=StaleOdomNode())
    task = FakeTask()
    task.params["room"] = "__current__"

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_pose"
    assert nav.calls == []


def test_product_search_fails_no_pose_when_observing_without_live_odom(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav, node=NoPoseNode())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_pose"


def test_product_search_fails_no_pose_when_observing_with_stale_odom(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav, node=StaleOdomNode())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_pose"


def test_product_search_fails_when_scan_snapshot_stale(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav, node=StaleScanNode())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_scan"


def test_product_search_fails_no_lidar_range_when_person_has_no_valid_range(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav, node=InvalidForwardRangeNode())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_lidar_range"
    assert task.result["detections"][0]["position_quality"] == "bearing_only"
    assert task.result["detections"][0]["photo_url"].endswith("unresolved_001_annotated.jpg")
    assert task.result["detections"][0]["crop_url"].endswith("unresolved_001_crop.jpg")


def test_product_search_retries_another_viewpoint_after_temporary_no_lidar_range(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(
        tmp_path, events, nav, node=RetryLidarNode())
    task = FakeTask()
    task.params.update({
        "max_views": 2,
        "visual_range_m": 0.1,
        "coverage_threshold": 0.25,
    })

    orchestrator.run(task)

    assert task.status == "completed"
    assert len(nav.calls) == 3  # room entry + retry at a second viewpoint
    assert task.result["targets_found"] == 1
    assert task.result["detections"][0]["position_quality"] == "range_lidar"
    assert not [
        event for event in events
        if event.get("type") == "search_room"
        and event.get("data", {}).get("reason") == "no_lidar_range"
    ]
    unresolved_photos = list((tmp_path / "missions").glob("*/unresolved_001_annotated.jpg"))
    assert unresolved_photos


def test_product_search_resolves_only_prior_unresolved_people_in_mixed_frame(tmp_path):
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(tmp_path, events, nav)
    observe_calls = 0
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

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
            }, frame=frame)
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
        }, frame=frame)
        current_unresolved = store.add_unresolved_observation({
            **common,
            "confidence": 0.8,
            "bbox": [70, 10, 90, 90],
            "position_quality": "bearing_only",
            "world_x": None,
            "world_y": None,
        }, frame=frame)
        return {
            "reason": "no_lidar_range",
            "detections": [current_unresolved],
            "resolved_count": 1,
        }

    orchestrator._observe_people_at_viewpoint = scripted_observe
    task = FakeTask()
    task.params.update({
        "max_views": 2,
        "visual_range_m": 0.1,
        "coverage_threshold": 0.25,
    })

    orchestrator.run(task)

    assert task.status == "failed"
    assert task.result["reason"] == "no_lidar_range"
    assert len(task.result["resolved_detections"]) == 1
    assert [item["id"] for item in task.result["detections"]] == ["unresolved_002"]


def test_product_search_applies_camera_source_yaw_extrinsic(monkeypatch, tmp_path):
    monkeypatch.setenv("GO2W_CAMERA_YAW_OFFSET_C13_VIS_DEG", "90")
    events = []
    nav = FakeNav()
    orchestrator = make_orchestrator(
        tmp_path, events, nav, node=LeftBearingRangeNode())
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "completed"
    marker = task.result["detections"][0]
    assert marker["world_x"] == pytest.approx(0.2, abs=0.02)
    assert marker["world_y"] == pytest.approx(2.2, abs=0.02)


def test_room_navigation_state_exposes_search_phase_markers_and_last_report(tmp_path):
    events = []
    orchestrator = make_orchestrator(tmp_path, events, FakeNav())
    orchestrator._current_mission_id = "mission-1"
    orchestrator._current_room_name = "__frontier__"
    orchestrator._phase("DETECT", progress=0.5, current_wp=2)
    marker = {
        "id": "person_001",
        "world_x": 1.2,
        "world_y": 0.3,
        "photo_url": "/missions/mission-1/person_001_annotated.jpg",
    }
    orchestrator._broadcast_person_markers("mission-1", [marker])
    orchestrator._last_report = {"mission_id": "mission-0", "status": "completed"}

    state = orchestrator.get_navigation_state()

    assert state["search_phase"] == "DETECT"
    assert state["mission_id"] == "mission-1"
    assert state["room"] == "__frontier__"
    assert state["person_markers"] == [marker]
    assert state["target_markers"] == [marker]
    assert state["last_report"]["mission_id"] == "mission-0"
