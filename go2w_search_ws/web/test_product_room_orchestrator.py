import math
import sys
from pathlib import Path

import numpy as np


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_room_orchestrator import RoomSearchOrchestrator  # noqa: E402


class FakeTask:
    def __init__(self):
        self.type = "search_room"
        self.params = {
            "room": "客厅",
            "target_classes": ["person"],
            "require_photos": True,
            "mark_on_map": True,
            "search_strategy": "next_best_view",
            "use_lidar_person_range": True,
            "max_views": 1,
        }
        self.status = "pending"
        self.result = None


class FakeAi:
    def get_person_detection_snapshot(self):
        return {
            "frame": np.zeros((100, 100, 3), dtype=np.uint8),
            "frame_width": 100,
            "frame_height": 100,
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


class FakeNode:
    def get_scan_snapshot(self):
        ranges = [0.0] * 360
        ranges[180] = 2.0
        return {
            "ranges": ranges,
            "angle_min": -math.pi,
            "angle_increment": math.pi / 180.0,
            "range_min": 0.15,
            "range_max": 10.0,
        }


class FakeNav:
    def __init__(self):
        self.calls = []

    def wait_for_server(self, timeout=2.0):
        return True

    def send_goal_and_wait(self, x, y, yaw, frame_id="map"):
        self.calls.append((float(x), float(y), float(yaw), frame_id))
        return {"ok": True, "status": 4}

    def cancel_current(self):
        return True


def test_product_search_creates_lidar_person_marker_and_report(tmp_path):
    events = []
    nav = FakeNav()
    rooms_yaml = tmp_path / "rooms.yaml"
    rooms_yaml.write_text(
        """
frame_id: map
version: "1.0"
rooms:
  - name: 客厅
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
    orchestrator = RoomSearchOrchestrator(
        node=FakeNode(),
        ai_engine=FakeAi(),
        ws_broadcast_fn=events.append,
        rooms_yaml_path=str(rooms_yaml),
    )
    orchestrator._nav = nav
    orchestrator._static_root = tmp_path
    task = FakeTask()

    orchestrator.run(task)

    assert task.status == "completed"
    assert len(nav.calls) >= 2
    report = task.result
    assert report["targets_found"] == 1
    detection = report["detections"][0]
    assert detection["position_quality"] == "range_lidar"
    assert detection["photo_url"].endswith("person_001_annotated.jpg")
    marker_events = [event for event in events if event.get("type") == "person_markers"]
    assert marker_events
    marker = marker_events[-1]["data"]["markers"][0]
    assert marker["position_quality"] == "range_lidar"
    assert marker["photo_url"].endswith("person_001_annotated.jpg")
