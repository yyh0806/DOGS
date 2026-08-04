# Product Room Person Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the product-grade "search this room and mark all people" loop using 2D SLAM/Nav2, YOLO-only person detection, LiDAR-backed person localization, photo artifacts, and map markers.

**Architecture:** Keep `nx_web_server.py` as the process host and `RoomSearchOrchestrator` as the mission entry point. Move new product logic into focused pure-Python modules under `web/`: command parsing, active-search planning, person localization, and person mission state. Integrate those modules into the existing TaskManager and WebSocket broadcast path after they are covered by offline tests.

**Tech Stack:** ROS2 Humble, slam_toolbox, Nav2 slim, Python 3, rclpy, YOLO via existing `NxAiEngine`, OpenCV for photo artifacts, JavaScript Canvas for map markers.

---

## Scope

This plan implements the first product loop, not the future FAST_LIO route. LocateAnything remains excluded from the mission path. Existing LocateAnything files in the current dirty worktree are not touched by these tasks.

## File Structure

- Create `go2w_search_ws/web/nx_product_command.py`: deterministic parser for product person-search commands and current-room resolution helpers.
- Create `go2w_search_ws/web/nx_person_localizer.py`: pure LiDAR range and person map-coordinate localization.
- Create `go2w_search_ws/web/nx_person_mission.py`: person observation deduplication, marker metadata, photo/crop/JSON artifact saving.
- Create `go2w_search_ws/web/nx_active_search.py`: room-bounded next-best-view candidate generation and scoring.
- Modify `go2w_search_ws/web/nx_ai_node.py`: expose a YOLO-only frame/detection snapshot for mission code.
- Modify `go2w_search_ws/web/nx_web_server.py`: expose full LaserScan snapshots, use deterministic command parser, include person markers in status or dedicated WebSocket events.
- Modify `go2w_search_ws/web/nx_room_orchestrator.py`: add `search_strategy=next_best_view` product mission branch while preserving current stage-E behavior as fallback.
- Modify `go2w_search_ws/web/static/map.js`: draw person markers, viewpoint candidates, and observed region hints.
- Modify `go2w_search_ws/web/static/panel.html`: handle person marker events and display mission reports/photos.
- Create/modify tests:
  - `go2w_search_ws/web/test_product_command.py`
  - `go2w_search_ws/web/test_person_localizer.py`
  - `go2w_search_ws/web/test_person_mission.py`
  - `go2w_search_ws/web/test_active_search.py`
  - `go2w_search_ws/web/test_product_room_orchestrator.py`
  - `go2w_search_ws/web/test_map_contract.js`

## Task 1: Product Command Parser And Current-Room Resolution

**Files:**
- Create: `go2w_search_ws/web/nx_product_command.py`
- Create: `go2w_search_ws/web/test_product_command.py`
- Modify: `go2w_search_ws/web/nx_web_server.py:621-728`

- [ ] **Step 1: Write failing parser tests**

Create `go2w_search_ws/web/test_product_command.py`:

```python
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_product_command import (
    parse_product_command,
    resolve_current_room,
)


ROOMS = [
    {
        "name": "客厅",
        "aliases": ["living room"],
        "nav_pose": {"x": 2.5, "y": 1.8, "yaw": 0.0},
        "search_area": {"origin_x": 1.0, "origin_y": 0.5, "width": 5.0, "height": 4.0},
    },
    {
        "name": "卧室",
        "aliases": ["bedroom"],
        "nav_pose": {"x": -1.2, "y": 3.4, "yaw": 1.5708},
        "search_area": {"origin_x": -3.0, "origin_y": 2.0, "width": 4.0, "height": 3.5},
    },
]


def test_parse_current_room_all_people_command():
    result = parse_product_command("去搜索这个房间，把所有人标注出来")

    assert result["response"] == "搜索当前房间并标注所有人"
    assert result["tasks"] == [{
        "type": "search_room",
        "priority": 8,
        "params": {
            "room": "__current__",
            "target_classes": ["person"],
            "require_photos": True,
            "mark_on_map": True,
            "search_strategy": "next_best_view",
            "use_lidar_person_range": True,
        },
    }]


def test_parse_named_room_people_command():
    result = parse_product_command("去卧室找人")

    assert result["tasks"][0]["type"] == "search_room"
    assert result["tasks"][0]["priority"] == 8
    assert result["tasks"][0]["params"]["room"] == "卧室"
    assert result["tasks"][0]["params"]["target_classes"] == ["person"]


def test_non_product_command_returns_none():
    assert parse_product_command("前进两米") is None


def test_resolve_current_room_by_containing_area():
    room = resolve_current_room(2.0, 1.0, ROOMS)

    assert room == "客厅"


def test_resolve_current_room_by_nearest_nav_pose_when_outside_area():
    room = resolve_current_room(-1.0, 1.5, ROOMS)

    assert room == "卧室"


def test_resolve_current_room_returns_none_for_empty_rooms():
    assert resolve_current_room(0.0, 0.0, []) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_product_command.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'nx_product_command'`.

- [ ] **Step 3: Implement parser module**

Create `go2w_search_ws/web/nx_product_command.py`:

```python
"""Deterministic product command parsing for room person search."""

import math
import re
from typing import Iterable, Optional


PERSON_WORDS = ("人", "人员", "所有人", "找人")
SEARCH_WORDS = ("搜索", "搜", "找", "寻找")
CURRENT_ROOM_WORDS = ("这个房间", "当前房间", "本房间", "这里")


def _base_person_search_task(room: str) -> dict:
    return {
        "type": "search_room",
        "priority": 8,
        "params": {
            "room": room,
            "target_classes": ["person"],
            "require_photos": True,
            "mark_on_map": True,
            "search_strategy": "next_best_view",
            "use_lidar_person_range": True,
        },
    }


def parse_product_command(text: str) -> Optional[dict]:
    text = (text or "").strip()
    if not text:
        return None
    has_search = any(word in text for word in SEARCH_WORDS)
    has_person = any(word in text for word in PERSON_WORDS)
    has_mark = "标注" in text or "标出来" in text or "标记" in text
    if not (has_search and (has_person or has_mark)):
        return None

    room = "__current__"
    named_room = _extract_named_room(text)
    if named_room:
        room = named_room

    response = "搜索当前房间并标注所有人" if room == "__current__" else f"搜索{room}并标注所有人"
    return {
        "understanding": text,
        "response": response,
        "tasks": [_base_person_search_task(room)],
    }


def _extract_named_room(text: str) -> Optional[str]:
    for word in CURRENT_ROOM_WORDS:
        if word in text:
            return None
    match = re.search(r"(客厅|卧室|厨房|书房|走廊|阳台|卫生间|厕所)", text)
    return match.group(1) if match else None


def resolve_current_room(robot_x: float, robot_y: float, rooms_detail: Iterable[dict]) -> Optional[str]:
    rooms = list(rooms_detail or [])
    if not rooms:
        return None
    containing = [
        room for room in rooms
        if _point_in_search_area(robot_x, robot_y, room.get("search_area", {}))
    ]
    if containing:
        return str(containing[0].get("name", "")) or None

    def nav_dist(room: dict) -> float:
        pose = room.get("nav_pose", {})
        return math.hypot(float(pose.get("x", 0.0)) - robot_x, float(pose.get("y", 0.0)) - robot_y)

    nearest = min(rooms, key=nav_dist)
    return str(nearest.get("name", "")) or None


def _point_in_search_area(x: float, y: float, search_area: dict) -> bool:
    ox = float(search_area.get("origin_x", 0.0))
    oy = float(search_area.get("origin_y", 0.0))
    width = float(search_area.get("width", 0.0))
    height = float(search_area.get("height", 0.0))
    return ox <= x <= ox + width and oy <= y <= oy + height
```

- [ ] **Step 4: Run parser tests**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_product_command.py -v
```

Expected: PASS, 6 tests pass.

- [ ] **Step 5: Integrate deterministic parser into TaskManager**

Modify `go2w_search_ws/web/nx_web_server.py`:

Add import near the other local imports:

```python
try:
    from nx_product_command import parse_product_command
except Exception:
    parse_product_command = None
```

In `TaskManager._process_command_bg`, before the VLM branch, add:

```python
            product_result = parse_product_command(text) if parse_product_command else None
            if product_result is not None:
                result = product_result
            else:
                result = self._vlm_parse_command(text) if (self.vlm and getattr(self.vlm, 'loaded', False)) \
                    else self._fallback_parse(text)
```

Then remove the existing two-line assignment that directly sets `result = self._vlm_parse_command(...)`.

In `_vlm_parse_command` prompt, add `search_room` to supported task types and replace the "搜索这个房间" example with:

```text
- search_room: {"room":"__current__|房间名", "target_classes":["person"], "require_photos":true, "mark_on_map":true, "search_strategy":"next_best_view", "use_lidar_person_range":true}

输入"搜索这个房间，把所有人标注出来"
输出: {"tasks":[{"type":"search_room","priority":8,"params":{"room":"__current__","target_classes":["person"],"require_photos":true,"mark_on_map":true,"search_strategy":"next_best_view","use_lidar_person_range":true}}]}
```

- [ ] **Step 6: Run parser tests and static command-path check**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_product_command.py -v
Select-String -Path web/nx_web_server.py -Pattern "parse_product_command|search_strategy|next_best_view"
```

Expected: tests pass, and PowerShell prints at least three matches from `nx_web_server.py`.

- [ ] **Step 7: Commit**

```bash
git add web/nx_product_command.py web/test_product_command.py web/nx_web_server.py
git commit -m "feat: parse product room person search commands"
```

## Task 2: LiDAR-Backed Person Localization

**Files:**
- Create: `go2w_search_ws/web/nx_person_localizer.py`
- Create: `go2w_search_ws/web/test_person_localizer.py`
- Modify integration points only after this task passes.

- [ ] **Step 1: Write failing localization tests**

Create `go2w_search_ws/web/test_person_localizer.py`:

```python
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_person_localizer import (
    DetectionFrame,
    LaserScanSnapshot,
    localize_person_detection,
)


def test_center_bbox_uses_forward_lidar_range():
    det = {"class": "person", "confidence": 0.9, "bbox": [590, 100, 690, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)
    scan.ranges[180] = 2.5

    result = localize_person_detection(det, frame, scan, robot_x=1.0, robot_y=2.0, robot_yaw=0.0)

    assert result["position_quality"] == "range_lidar"
    assert result["range_source"] == "lidar"
    assert abs(result["world_x"] - 3.5) < 0.05
    assert abs(result["world_y"] - 2.0) < 0.05


def test_invalid_scan_returns_bearing_only():
    det = {"class": "person", "confidence": 0.9, "bbox": [590, 100, 690, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)

    result = localize_person_detection(det, frame, scan, robot_x=1.0, robot_y=2.0, robot_yaw=0.0)

    assert result["position_quality"] == "bearing_only"
    assert result["range_source"] == "unresolved"
    assert result["world_x"] is None
    assert result["world_y"] is None


def test_right_side_bbox_rotates_with_robot_yaw():
    det = {"class": "person", "confidence": 0.8, "bbox": [960, 100, 1060, 500]}
    frame = DetectionFrame(width=1280, height=720, camera_hfov_rad=math.radians(70.0))
    scan = LaserScanSnapshot(angle_min=-math.pi, angle_increment=math.pi / 180.0, ranges=[0.0] * 360)
    scan.ranges[200] = 3.0

    result = localize_person_detection(det, frame, scan, robot_x=0.0, robot_y=0.0, robot_yaw=math.pi / 2)

    assert result["position_quality"] == "range_lidar"
    assert result["world_y"] > 2.0
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_person_localizer.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'nx_person_localizer'`.

- [ ] **Step 3: Implement LiDAR localizer**

Create `go2w_search_ws/web/nx_person_localizer.py`:

```python
"""YOLO bbox bearing + LaserScan range localization."""

import math
from dataclasses import dataclass
from statistics import median
from typing import Sequence


@dataclass(frozen=True)
class DetectionFrame:
    width: int
    height: int
    camera_hfov_rad: float
    camera_yaw_offset_rad: float = 0.0
    gimbal_yaw_rad: float = 0.0


@dataclass(frozen=True)
class LaserScanSnapshot:
    angle_min: float
    angle_increment: float
    ranges: Sequence[float]
    range_min: float = 0.15
    range_max: float = 10.0


def localize_person_detection(
    detection: dict,
    frame: DetectionFrame,
    scan: LaserScanSnapshot,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    window_rad: float = math.radians(5.0),
) -> dict:
    bbox = detection.get("bbox") or []
    if len(bbox) < 4 or frame.width <= 0:
        return _bearing_only(detection, None)
    x1, _, x2, _ = [float(v) for v in bbox[:4]]
    cx_norm = ((x1 + x2) / 2.0) / float(frame.width)
    camera_angle = (cx_norm - 0.5) * frame.camera_hfov_rad
    bearing_base = frame.camera_yaw_offset_rad + frame.gimbal_yaw_rad + camera_angle
    lidar_range = range_at_bearing(scan, bearing_base, window_rad=window_rad)
    bearing_map = robot_yaw + bearing_base
    result = dict(detection)
    result["bearing_base"] = round(bearing_base, 4)
    result["bearing_map"] = round(bearing_map, 4)
    if lidar_range is None:
        result.update({
            "range_m": None,
            "range_source": "unresolved",
            "position_quality": "bearing_only",
            "world_x": None,
            "world_y": None,
        })
        return result
    result.update({
        "range_m": round(lidar_range, 3),
        "range_source": "lidar",
        "position_quality": "range_lidar",
        "world_x": round(float(robot_x) + lidar_range * math.cos(bearing_map), 3),
        "world_y": round(float(robot_y) + lidar_range * math.sin(bearing_map), 3),
    })
    return result


def range_at_bearing(scan: LaserScanSnapshot, bearing_rad: float, window_rad: float) -> float | None:
    if not scan.ranges or scan.angle_increment == 0:
        return None
    values = []
    for i, raw in enumerate(scan.ranges):
        try:
            r = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        ray_angle = scan.angle_min + i * scan.angle_increment
        if abs(_angle_diff(ray_angle, bearing_rad)) <= window_rad:
            values.append(r)
    return float(median(values)) if values else None


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _bearing_only(detection: dict, bearing_base: float | None) -> dict:
    result = dict(detection)
    result.update({
        "bearing_base": bearing_base,
        "range_m": None,
        "range_source": "unresolved",
        "position_quality": "bearing_only",
        "world_x": None,
        "world_y": None,
    })
    return result
```

- [ ] **Step 4: Run localization tests**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_person_localizer.py -v
```

Expected: PASS, 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add web/nx_person_localizer.py web/test_person_localizer.py
git commit -m "feat: localize people with lidar range"
```

## Task 3: Person Mission State, Deduplication, And Photo Artifacts

**Files:**
- Create: `go2w_search_ws/web/nx_person_mission.py`
- Create: `go2w_search_ws/web/test_person_mission.py`

- [ ] **Step 1: Write failing mission-state tests**

Create `go2w_search_ws/web/test_person_mission.py`:

```python
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_person_mission import PersonMissionStore


def test_deduplicates_nearby_lidar_observations():
    store = PersonMissionStore("m123")
    first = store.add_observation({
        "class": "person",
        "confidence": 0.7,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [10, 10, 50, 80],
    })
    second = store.add_observation({
        "class": "person",
        "confidence": 0.9,
        "world_x": 1.4,
        "world_y": 2.1,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [12, 12, 52, 82],
    })

    assert first["id"] == "person_001"
    assert second["id"] == "person_001"
    assert len(store.markers()) == 1
    assert store.markers()[0]["confidence"] == 0.9


def test_keeps_separate_people_when_far_apart():
    store = PersonMissionStore("m123")
    store.add_observation({"class": "person", "confidence": 0.7, "world_x": 1.0, "world_y": 2.0, "position_quality": "range_lidar"})
    store.add_observation({"class": "person", "confidence": 0.8, "world_x": 4.0, "world_y": 2.0, "position_quality": "range_lidar"})

    assert [m["id"] for m in store.markers()] == ["person_001", "person_002"]


def test_saves_artifacts_for_confirmed_person():
    with tempfile.TemporaryDirectory() as td:
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        store = PersonMissionStore("m123", static_root=td)
        marker = store.add_observation({
            "class": "person",
            "confidence": 0.95,
            "world_x": 1.0,
            "world_y": 2.0,
            "position_quality": "range_lidar",
            "range_source": "lidar",
            "bbox": [10, 10, 50, 80],
            "frame_width": 100,
            "frame_height": 100,
        }, frame=frame)

        assert marker["photo_url"].endswith("/missions/m123/person_001_annotated.jpg")
        assert os.path.exists(os.path.join(td, "missions", "m123", "person_001.json"))
        assert os.path.exists(os.path.join(td, "missions", "m123", "person_001_crop.jpg"))
```

- [ ] **Step 2: Run tests to verify failure**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_person_mission.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'nx_person_mission'`.

- [ ] **Step 3: Implement person mission store**

Create `go2w_search_ws/web/nx_person_mission.py`:

```python
"""Person marker state and mission media artifact storage."""

import json
import math
import os
import time
from pathlib import Path


class PersonMissionStore:
    def __init__(self, mission_id: str, static_root: str | None = None, merge_distance_m: float = 0.7):
        self.mission_id = str(mission_id)
        self.static_root = Path(static_root or Path(__file__).resolve().parent / "static")
        self.merge_distance_m = float(merge_distance_m)
        self._markers: list[dict] = []

    def add_observation(self, observation: dict, frame=None) -> dict:
        obs = dict(observation)
        existing = self._find_existing(obs)
        if existing is None:
            marker = self._new_marker(obs)
            self._markers.append(marker)
        else:
            marker = self._merge(existing, obs)
        if frame is not None and marker.get("position_quality") == "range_lidar":
            self._save_artifacts(marker, obs, frame)
        return dict(marker)

    def markers(self) -> list[dict]:
        return [dict(m) for m in self._markers]

    def _find_existing(self, obs: dict) -> dict | None:
        if obs.get("position_quality") != "range_lidar":
            return None
        ox = obs.get("world_x")
        oy = obs.get("world_y")
        if ox is None or oy is None:
            return None
        for marker in self._markers:
            mx = marker.get("world_x")
            my = marker.get("world_y")
            if mx is None or my is None:
                continue
            if math.hypot(float(mx) - float(ox), float(my) - float(oy)) <= self.merge_distance_m:
                return marker
        return None

    def _new_marker(self, obs: dict) -> dict:
        marker_id = f"person_{len(self._markers) + 1:03d}"
        marker = {
            "id": marker_id,
            "class": "person",
            "confidence": float(obs.get("confidence", 0.0)),
            "world_x": obs.get("world_x"),
            "world_y": obs.get("world_y"),
            "robot_x": obs.get("robot_x"),
            "robot_y": obs.get("robot_y"),
            "robot_yaw": obs.get("robot_yaw"),
            "bbox": list(obs.get("bbox", [])),
            "range_source": obs.get("range_source", "unresolved"),
            "position_quality": obs.get("position_quality", "bearing_only"),
            "timestamp": float(obs.get("timestamp", time.time())),
        }
        return marker

    def _merge(self, marker: dict, obs: dict) -> dict:
        obs_conf = float(obs.get("confidence", 0.0))
        old_conf = float(marker.get("confidence", 0.0))
        if obs.get("world_x") is not None and marker.get("world_x") is not None:
            marker["world_x"] = round((float(marker["world_x"]) * old_conf + float(obs["world_x"]) * obs_conf) / max(old_conf + obs_conf, 1e-6), 3)
        if obs.get("world_y") is not None and marker.get("world_y") is not None:
            marker["world_y"] = round((float(marker["world_y"]) * old_conf + float(obs["world_y"]) * obs_conf) / max(old_conf + obs_conf, 1e-6), 3)
        if obs_conf >= old_conf:
            marker["confidence"] = obs_conf
            marker["bbox"] = list(obs.get("bbox", marker.get("bbox", [])))
            marker["timestamp"] = float(obs.get("timestamp", time.time()))
        return marker

    def _save_artifacts(self, marker: dict, obs: dict, frame) -> None:
        import cv2

        mission_dir = self.static_root / "missions" / self.mission_id
        mission_dir.mkdir(parents=True, exist_ok=True)
        marker_id = marker["id"]
        raw_path = mission_dir / f"{marker_id}_raw.jpg"
        annotated_path = mission_dir / f"{marker_id}_annotated.jpg"
        crop_path = mission_dir / f"{marker_id}_crop.jpg"
        json_path = mission_dir / f"{marker_id}.json"

        cv2.imwrite(str(raw_path), frame)
        annotated = frame.copy()
        bbox = [int(v) for v in obs.get("bbox", [])[:4]]
        if len(bbox) == 4:
            x1, y1, x2, y2 = _clamp_bbox(bbox, frame.shape[1], frame.shape[0])
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            crop = frame[y1:y2, x1:x2]
            if crop.size:
                cv2.imwrite(str(crop_path), crop)
        cv2.imwrite(str(annotated_path), annotated)

        marker["photo_url"] = f"/missions/{self.mission_id}/{marker_id}_annotated.jpg"
        marker["crop_url"] = f"/missions/{self.mission_id}/{marker_id}_crop.jpg"
        marker["raw_url"] = f"/missions/{self.mission_id}/{marker_id}_raw.jpg"
        json_path.write_text(json.dumps(marker, ensure_ascii=False, indent=2), encoding="utf-8")


def _clamp_bbox(bbox: list[int], width: int, height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    x1 = max(0, min(width, x1))
    x2 = max(0, min(width, x2))
    y1 = max(0, min(height, y1))
    y2 = max(0, min(height, y2))
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)
```

- [ ] **Step 4: Run tests**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_person_mission.py -v
```

Expected: PASS, 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add web/nx_person_mission.py web/test_person_mission.py
git commit -m "feat: store person mission markers and photos"
```

## Task 4: Active Search Planner

**Files:**
- Create: `go2w_search_ws/web/nx_active_search.py`
- Create: `go2w_search_ws/web/test_active_search.py`

- [ ] **Step 1: Write failing active-search tests**

Create `go2w_search_ws/web/test_active_search.py`:

```python
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_active_search import ActiveSearchPlanner


def test_generates_candidates_inside_room_only():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.4)
    room_area = {"origin_x": 0.0, "origin_y": 0.0, "width": 3.0, "height": 2.0}

    candidates = planner.generate_candidates(room_area, robot_pose=(0.0, 0.0, 0.0), obstacles=[])

    assert candidates
    assert all(0.0 <= c["x"] <= 3.0 and 0.0 <= c["y"] <= 2.0 for c in candidates)


def test_filters_candidates_near_obstacle():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.6)
    room_area = {"origin_x": 0.0, "origin_y": 0.0, "width": 3.0, "height": 2.0}

    candidates = planner.generate_candidates(room_area, robot_pose=(0.0, 0.0, 0.0), obstacles=[[1.0, 1.0]])

    assert all(not (abs(c["x"] - 1.0) < 0.6 and abs(c["y"] - 1.0) < 0.6) for c in candidates)


def test_selects_unobserved_candidate_with_lower_path_cost_when_scores_tie():
    planner = ActiveSearchPlanner(spacing=1.0, obstacle_clearance=0.2)
    candidates = [
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "information_gain": 4.0, "visual_coverage_gain": 4.0, "obstacle_risk_cost": 0.0, "repeated_observation_penalty": 0.0},
        {"x": 1.0, "y": 0.0, "yaw": 0.0, "information_gain": 4.0, "visual_coverage_gain": 4.0, "obstacle_risk_cost": 0.0, "repeated_observation_penalty": 0.0},
    ]

    selected = planner.select_next_best(candidates, robot_pose=(0.0, 0.0, 0.0))

    assert selected["x"] == 1.0
```

- [ ] **Step 2: Run tests to verify failure**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_active_search.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'nx_active_search'`.

- [ ] **Step 3: Implement planner**

Create `go2w_search_ws/web/nx_active_search.py`:

```python
"""Room-bounded next-best-view active search planning."""

import math


class ActiveSearchPlanner:
    def __init__(self, spacing: float = 1.0, obstacle_clearance: float = 0.5):
        self.spacing = float(spacing)
        self.obstacle_clearance = float(obstacle_clearance)
        self._blocked = set()
        self._visited = set()

    def mark_blocked(self, candidate: dict) -> None:
        self._blocked.add(self._key(candidate))

    def mark_visited(self, candidate: dict) -> None:
        self._visited.add(self._key(candidate))

    def generate_candidates(self, room_area: dict, robot_pose: tuple[float, float, float], obstacles: list[list[float]]) -> list[dict]:
        ox = float(room_area["origin_x"])
        oy = float(room_area["origin_y"])
        width = float(room_area["width"])
        height = float(room_area["height"])
        candidates = []
        x = ox
        while x <= ox + width + 1e-6:
            y = oy
            while y <= oy + height + 1e-6:
                candidate = {
                    "x": round(x, 2),
                    "y": round(y, 2),
                    "yaw": 0.0,
                    "information_gain": self._edge_gain(x, y, ox, oy, width, height),
                    "visual_coverage_gain": self._center_gain(x, y, ox, oy, width, height),
                    "obstacle_risk_cost": self._obstacle_risk(x, y, obstacles),
                    "repeated_observation_penalty": 1.0 if (round(x, 2), round(y, 2)) in self._visited else 0.0,
                }
                if self._is_safe(candidate, obstacles) and self._key(candidate) not in self._blocked:
                    candidates.append(candidate)
                y += self.spacing
            x += self.spacing
        return candidates

    def select_next_best(self, candidates: list[dict], robot_pose: tuple[float, float, float]) -> dict | None:
        if not candidates:
            return None
        rx, ry, _ = robot_pose
        scored = []
        for c in candidates:
            path_cost = math.hypot(float(c["x"]) - rx, float(c["y"]) - ry)
            score = (
                float(c.get("information_gain", 0.0))
                + float(c.get("visual_coverage_gain", 0.0))
                - path_cost
                - float(c.get("obstacle_risk_cost", 0.0))
                - float(c.get("repeated_observation_penalty", 0.0))
            )
            scored.append((score, -path_cost, c))
        scored.sort(key=lambda item: (item[0], item[1]), reverse=True)
        selected = dict(scored[0][2])
        selected["score"] = round(scored[0][0], 3)
        return selected

    def _key(self, candidate: dict) -> tuple[float, float]:
        return (round(float(candidate["x"]), 2), round(float(candidate["y"]), 2))

    def _is_safe(self, candidate: dict, obstacles: list[list[float]]) -> bool:
        if self._obstacle_risk(candidate["x"], candidate["y"], obstacles) >= 999.0:
            return False
        return True

    def _obstacle_risk(self, x: float, y: float, obstacles: list[list[float]]) -> float:
        if not obstacles:
            return 0.0
        nearest = min(math.hypot(float(p[0]) - x, float(p[1]) - y) for p in obstacles if len(p) >= 2)
        if nearest < self.obstacle_clearance:
            return 999.0
        return max(0.0, self.obstacle_clearance * 2.0 - nearest)

    def _edge_gain(self, x: float, y: float, ox: float, oy: float, width: float, height: float) -> float:
        dist_edge = min(x - ox, ox + width - x, y - oy, oy + height - y)
        return max(0.0, 2.0 - dist_edge)

    def _center_gain(self, x: float, y: float, ox: float, oy: float, width: float, height: float) -> float:
        cx = ox + width / 2.0
        cy = oy + height / 2.0
        max_dist = math.hypot(width / 2.0, height / 2.0)
        return max(0.0, 2.0 * (1.0 - math.hypot(x - cx, y - cy) / max(max_dist, 1e-6)))
```

- [ ] **Step 4: Run planner tests**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_active_search.py -v
```

Expected: PASS, 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add web/nx_active_search.py web/test_active_search.py
git commit -m "feat: add next-best-view room planner"
```

## Task 5: YOLO Snapshot And LaserScan Snapshot Integration

**Files:**
- Modify: `go2w_search_ws/web/nx_ai_node.py:208-1035`
- Modify: `go2w_search_ws/web/nx_web_server.py:300-343`
- Create: `go2w_search_ws/web/test_ai_snapshot_contract.py`
- Create: `go2w_search_ws/web/test_scan_snapshot_contract.py`

- [ ] **Step 1: Write failing snapshot contract tests**

Create `go2w_search_ws/web/test_ai_snapshot_contract.py`:

```python
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_ai_node import NxAiEngine


def test_person_detection_snapshot_filters_person_and_copies_frame():
    ai = NxAiEngine()
    frame = np.zeros((20, 30, 3), dtype=np.uint8)
    with ai._lock:
        ai._latest_frame = frame
        ai._latest_dets = [
            {"class": "person", "confidence": 0.9, "bbox": [1, 2, 10, 18]},
            {"class": "chair", "confidence": 0.8, "bbox": [4, 4, 8, 8]},
        ]
        ai._detect_frame_w = 30

    snap = ai.get_person_detection_snapshot()

    assert snap["frame_width"] == 30
    assert snap["frame_height"] == 20
    assert snap["frame"] is not frame
    assert [d["class"] for d in snap["detections"]] == ["person"]
    assert snap["detections"][0]["source"] == "yolo"
```

Create `go2w_search_ws/web/test_scan_snapshot_contract.py`:

```python
from pathlib import Path


SOURCE = Path(__file__).resolve().parent / "nx_web_server.py"


def test_nx_web_node_stores_scan_metadata_for_lidar_person_range():
    text = SOURCE.read_text(encoding="utf-8")

    assert "_scan_angle_min" in text
    assert "_scan_angle_increment" in text
    assert "_scan_range_min" in text
    assert "_scan_range_max" in text
    assert "def get_scan_snapshot" in text
```

- [ ] **Step 2: Run tests to verify failures**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_ai_snapshot_contract.py web/test_scan_snapshot_contract.py -v
```

Expected:

- `test_ai_snapshot_contract.py` fails with `AttributeError: 'NxAiEngine' object has no attribute 'get_person_detection_snapshot'`.
- `test_scan_snapshot_contract.py` fails because scan metadata fields are absent.

- [ ] **Step 3: Add YOLO-only snapshot method**

Modify `go2w_search_ws/web/nx_ai_node.py` below `get_frame_detection_count`:

```python
    def get_person_detection_snapshot(self):
        """Return a copy of the latest YOLO person detections and annotated frame.

        Product room search uses YOLO only. LocateAnything detections are not
        included in this snapshot.
        """
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            dets = [dict(d) for d in (self._latest_dets or []) if d.get("class") == "person"]
            frame_w = int(self._detect_frame_w if self._detect_frame_w > 0 else 1280)
        if frame is None:
            return {"frame": None, "frame_width": frame_w, "frame_height": 0, "detections": []}
        frame_h = int(frame.shape[0])
        for det in dets:
            det["frame_width"] = frame_w
            det["frame_height"] = frame_h
            det["source"] = "yolo"
        return {
            "frame": frame,
            "frame_width": frame_w,
            "frame_height": frame_h,
            "detections": dets,
        }
```

Do not modify `locate_target` in this task.

- [ ] **Step 4: Store scan metadata and expose snapshot**

Modify `go2w_search_ws/web/nx_web_server.py` in `NxWebNode.__init__`, next to `_scan_ranges`:

```python
        self._scan_angle_min = -math.pi
        self._scan_angle_increment = 0.0
        self._scan_range_min = 0.0
        self._scan_range_max = 0.0
```

Modify `_on_scan`:

```python
    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._scan_ranges = [round(float(r), 3) for r in msg.ranges]
            self._scan_angle_min = float(msg.angle_min)
            self._scan_angle_increment = float(msg.angle_increment)
            self._scan_range_min = float(msg.range_min)
            self._scan_range_max = float(msg.range_max)
            self._scan_count += 1
```

Add this method after `get_status_snapshot`:

```python
    def get_scan_snapshot(self):
        with self._lock:
            return {
                "angle_min": self._scan_angle_min,
                "angle_increment": self._scan_angle_increment,
                "range_min": self._scan_range_min,
                "range_max": self._scan_range_max,
                "ranges": list(self._scan_ranges),
                "count": self._scan_count,
            }
```

- [ ] **Step 5: Run snapshot tests**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_ai_snapshot_contract.py web/test_scan_snapshot_contract.py -v
```

Expected: PASS, 2 tests pass.

- [ ] **Step 6: Commit**

```bash
git add web/nx_ai_node.py web/nx_web_server.py web/test_ai_snapshot_contract.py web/test_scan_snapshot_contract.py
git commit -m "feat: expose yolo and scan snapshots for person search"
```

## Task 6: Product Search Branch In RoomSearchOrchestrator

**Files:**
- Modify: `go2w_search_ws/web/nx_room_orchestrator.py:513-986`
- Create: `go2w_search_ws/web/test_product_room_orchestrator.py`

- [ ] **Step 1: Write failing product orchestrator test**

Create `go2w_search_ws/web/test_product_room_orchestrator.py`:

```python
import math
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

import nx_room_orchestrator as orch


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
        }
        self.status = "pending"
        self.result = None


class FakeNav:
    def __init__(self):
        self.calls = []

    def wait_for_server(self, timeout=2.0):
        return True

    def send_goal_and_wait(self, x, y, yaw, frame_id="map"):
        self.calls.append((round(x, 2), round(y, 2), round(yaw, 2), frame_id))
        return {"ok": True, "status": 4}

    def cancel_current(self):
        return True


class FakeAi:
    def get_person_detection_snapshot(self):
        return {
            "frame": np.zeros((100, 100, 3), dtype=np.uint8),
            "frame_width": 100,
            "frame_height": 100,
            "detections": [{
                "class": "person",
                "confidence": 0.91,
                "bbox": [45, 10, 55, 90],
                "frame_width": 100,
                "frame_height": 100,
                "source": "yolo",
            }],
        }


class FakeNode:
    def get_scan_snapshot(self):
        ranges = [0.0] * 360
        ranges[180] = 2.0
        return {
            "angle_min": -math.pi,
            "angle_increment": math.pi / 180.0,
            "range_min": 0.15,
            "range_max": 10.0,
            "ranges": ranges,
        }


def test_product_search_creates_lidar_person_marker_and_report():
    events = []
    fake_nav = FakeNav()
    with tempfile.TemporaryDirectory() as td:
        o = orch.RoomSearchOrchestrator(node=FakeNode(), ai_engine=FakeAi(), ws_broadcast_fn=events.append)
        o._ensure_nav = lambda: fake_nav
        o._static_root = td
        task = FakeTask()

        o.run(task)

    assert task.status == "completed"
    report = [e["data"] for e in events if e.get("type") == "mission_report"][-1]
    assert report["targets_found"] == 1
    assert report["detections"][0]["position_quality"] == "range_lidar"
    assert report["detections"][0]["photo_url"].endswith("person_001_annotated.jpg")
    assert any(e.get("type") == "person_markers" for e in events)
```

- [ ] **Step 2: Run test to verify failure**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_product_room_orchestrator.py -v
```

Expected: FAIL because `RoomSearchOrchestrator.run()` still uses the old waypoint snapshot path and does not emit `person_markers`.

- [ ] **Step 3: Add imports and constructor state**

Modify `go2w_search_ws/web/nx_room_orchestrator.py` imports:

```python
from nx_active_search import ActiveSearchPlanner
from nx_person_localizer import DetectionFrame, LaserScanSnapshot, localize_person_detection
from nx_person_mission import PersonMissionStore
from nx_product_command import resolve_current_room
```

In `RoomSearchOrchestrator.__init__`, add:

```python
        self._person_markers: List[dict] = []
        self._static_root = os.path.join(_WEB_DIR, "static")
```

- [ ] **Step 4: Resolve `__current__` room**

In `run()`, after loading `room_map` and before `room = room_map.find(room_query)`, add:

```python
        if room_query == "__current__":
            robot_x, robot_y, _ = self._get_robot_pose()
            resolved = resolve_current_room(robot_x, robot_y, room_map.list_rooms_detail())
            if resolved is None:
                self._fail("no_room", room=room_query, msg="无法从当前位姿解析房间")
                task.status = "failed"
                task.result = {"reason": "no_room", "query": room_query}
                return
            room_query = resolved
            with self._lock:
                self._current_room_name = room_query
```

- [ ] **Step 5: Branch product mission before old waypoint loop**

In `run()`, after `target_classes = ...`, add:

```python
        if params.get("search_strategy") == "next_best_view":
            self._run_product_person_search(task, mission_id, start_time, room_map, room_query, task_target_classes)
            return
```

- [ ] **Step 6: Implement product search method**

Add this method inside `RoomSearchOrchestrator` before `_plan_room_waypoints`:

```python
    def _run_product_person_search(self, task, mission_id: str, start_time: float,
                                   room_map: RoomMap, room_query: str,
                                   task_target_classes: List[str]) -> None:
        room = room_map.find(room_query)
        if room is None:
            self._fail("no_room", room=room_query)
            task.status = "failed"
            task.result = {"reason": "no_room", "query": room_query}
            return
        with self._lock:
            self._current_room_name = room.name
            self._person_markers = []
        target_classes = room.target_classes if room.target_classes else task_target_classes
        if target_classes and "person" not in target_classes:
            target_classes = ["person"]

        self._phase("NAVIGATE", progress=0.0, room=room.name)
        nav = self._ensure_nav()
        if nav is None or not nav.wait_for_server(timeout=2.0):
            self._fail("no_nav", room=room.name)
            task.status = "failed"
            task.result = {"reason": "no_nav"}
            return
        entry = nav.send_goal_and_wait(room.nav_pose["x"], room.nav_pose["y"], room.nav_pose["yaw"], frame_id=room_map.frame_id)
        if not entry.get("ok"):
            reason = "no_nav" if entry.get("reason") == "no_server" else entry.get("reason", "nav_aborted")
            self._fail(reason, room=room.name, stage="navigate_to_room")
            task.status = "failed"
            task.result = {"reason": reason, "raw": entry}
            return

        self._phase("ACTIVE_SEARCH", progress=0.0, room=room.name)
        planner = ActiveSearchPlanner(spacing=float(room.search_area.get("spacing", 1.0)), obstacle_clearance=0.5)
        store = PersonMissionStore(mission_id, static_root=getattr(self, "_static_root", None))
        visited = 0
        max_views = int((task.params or {}).get("max_views", 12))

        for view_idx in range(max_views):
            if self._check_cancel("ACTIVE_SEARCH", room.name):
                task.status = "failed"
                task.result = {"reason": "cancelled"}
                return
            robot_x, robot_y, robot_yaw = self._get_robot_pose()
            obstacles = self._get_obstacle_points()
            candidates = planner.generate_candidates(room.search_area, (robot_x, robot_y, robot_yaw), obstacles)
            candidate = planner.select_next_best(candidates, (robot_x, robot_y, robot_yaw))
            if candidate is None:
                break
            self._phase("NEXT_BEST_VIEW", progress=view_idx / float(max_views), room=room.name,
                        current_wp=view_idx, total_wp=max_views, waypoint=(candidate["x"], candidate["y"]),
                        score=candidate.get("score"))
            result = nav.send_goal_and_wait(candidate["x"], candidate["y"], candidate.get("yaw", 0.0), frame_id=room_map.frame_id)
            if not result.get("ok"):
                planner.mark_blocked(candidate)
                continue
            visited += 1
            planner.mark_visited(candidate)
            self._observe_people_at_viewpoint(store, room.name, view_idx)
            self._person_markers = store.markers()
            self._safe_broadcast({"type": "person_markers", "data": {"mission_id": mission_id, "markers": self._person_markers}})

        self._phase("REPORT", progress=1.0, room=room.name, targets_found=len(store.markers()))
        self._finalize_report(task, mission_id, room, total_wp=max_views,
                              visited=visited, detections_log=store.markers(),
                              start_time=start_time)
```

- [ ] **Step 7: Add observation helper and obstacle snapshot helper**

Add these methods inside `RoomSearchOrchestrator`:

```python
    def _observe_people_at_viewpoint(self, store: PersonMissionStore, room_name: str, view_idx: int) -> None:
        if self._ai is None or not hasattr(self._ai, "get_person_detection_snapshot"):
            self._fail("no_yolo", room=room_name, stage="observe")
            return
        snapshot = self._ai.get_person_detection_snapshot()
        frame = snapshot.get("frame")
        detections = snapshot.get("detections") or []
        if not detections:
            return
        scan_data = self._node.get_scan_snapshot() if self._node is not None and hasattr(self._node, "get_scan_snapshot") else None
        if not scan_data:
            return
        scan = LaserScanSnapshot(
            angle_min=float(scan_data["angle_min"]),
            angle_increment=float(scan_data["angle_increment"]),
            ranges=list(scan_data["ranges"]),
            range_min=float(scan_data.get("range_min", 0.15)),
            range_max=float(scan_data.get("range_max", 10.0)),
        )
        robot_x, robot_y, robot_yaw = self._get_robot_pose()
        frame_info = DetectionFrame(
            width=int(snapshot.get("frame_width", 1280)),
            height=int(snapshot.get("frame_height", 720)),
            camera_hfov_rad=math.radians(float(os.environ.get("GO2W_CAMERA_HFOV", "70"))),
        )
        for det in detections:
            localized = localize_person_detection(det, frame_info, scan, robot_x, robot_y, robot_yaw)
            localized["robot_x"] = round(robot_x, 3)
            localized["robot_y"] = round(robot_y, 3)
            localized["robot_yaw"] = round(robot_yaw, 3)
            localized["wp_index"] = view_idx
            localized["timestamp"] = time.time()
            marker = store.add_observation(localized, frame=frame)
            logger.info(f"person marker {marker['id']} quality={marker.get('position_quality')} room={room_name}")

    def _get_obstacle_points(self) -> List[list]:
        try:
            if self._node is not None:
                with getattr(self._node, "_lock", threading.Lock()):
                    ranges = list(getattr(self._node, "_scan_ranges", []) or [])
                    yaw = float(getattr(self._node, "_imu_yaw", 0.0))
                    x = float(getattr(self._node, "_odom_x", 0.0))
                    y = float(getattr(self._node, "_odom_y", 0.0))
                points = []
                if ranges:
                    n = len(ranges)
                    cos_y = math.cos(yaw)
                    sin_y = math.sin(yaw)
                    for i, r in enumerate(ranges):
                        if 0.15 < float(r) < 9.9:
                            ang = -math.pi + i * 2.0 * math.pi / n
                            lx = float(r) * math.cos(ang)
                            ly = float(r) * math.sin(ang)
                            points.append([round(cos_y * lx - sin_y * ly + x, 2),
                                           round(sin_y * lx + cos_y * ly + y, 2)])
                    return points
        except Exception as e:
            logger.debug(f"obstacle snapshot failed: {e}")
        return []
```

- [ ] **Step 8: Run product orchestrator test**

Run:

```bash
cd go2w_search_ws
python -m pytest web/test_product_room_orchestrator.py -v
```

Expected: PASS, 1 test passes.

- [ ] **Step 9: Run existing stage-E tests**

Run:

```bash
cd go2w_search_ws
python tools/test_stage_e.py
```

Expected: all existing stage-E checks pass. The old `search_room` path still works when `search_strategy` is not `next_best_view`.

- [ ] **Step 10: Commit**

```bash
git add web/nx_room_orchestrator.py web/test_product_room_orchestrator.py
git commit -m "feat: add active person room search mission"
```

## Task 7: Map UI Person Markers And Mission Report Rendering

**Files:**
- Modify: `go2w_search_ws/web/static/map.js`
- Modify: `go2w_search_ws/web/static/panel.html`
- Modify: `go2w_search_ws/web/test_map_contract.js`

- [ ] **Step 1: Extend JS map contract tests**

Append this function to `go2w_search_ws/web/test_map_contract.js` before the final calls:

```javascript
function testPersonMarkersDrawLabelAndUseQualityColor() {
  const { map, ctx } = createMap();
  map.update({
    x: 0,
    y: 0,
    yaw: 0,
    trail: [[0, 0]],
    map: [],
    scan: [],
    detections: [],
    person_markers: [
      { id: 'person_001', x: 2, y: 1, class: 'person', confidence: 0.91, position_quality: 'range_lidar' },
      { id: 'person_002', x: -2, y: 1, class: 'person', confidence: 0.60, position_quality: 'bearing_only' },
    ],
    waypoints: [],
    currentWP: -1,
    slam_source: 'ros2_nx',
  });
  map._draw();

  const labels = ctx.ops.filter(op => op.type === 'fillText').map(op => String(op.text));
  assert(labels.some(t => t.includes('人1')), `expected 人1 label, got ${labels}`);
  assert(labels.some(t => t.includes('人2')), `expected 人2 label, got ${labels}`);
}
```

Then add the call:

```javascript
testPersonMarkersDrawLabelAndUseQualityColor();
```

- [ ] **Step 2: Run JS test to verify failure**

Run:

```bash
cd go2w_search_ws
node web/test_map_contract.js
```

Expected: FAIL because `person_markers` is ignored.

- [ ] **Step 3: Extend map state and update contract**

Modify `go2w_search_ws/web/static/map.js` constructor state:

```javascript
      detMarks: [], personMarkers: [], waypoints: [], currentWP: -1,
```

Modify `update(data)`:

```javascript
    if (data.person_markers) this.slam.personMarkers = data.person_markers;
```

In `_computeTransform()`, add:

```javascript
    for (const p of s.personMarkers || []) { allX.push(p.x); allY.push(p.y); }
```

- [ ] **Step 4: Draw person markers**

In `_draw()`, after the existing detection-target loop and before drawing the dog, add:

```javascript
    // 6b. Product person markers
    const people = s.personMarkers || [];
    for (let i = 0; i < people.length; i++) {
      const person = people[i];
      const px = toX(Number(person.x));
      const py = toY(Number(person.y));
      const confirmed = person.position_quality === 'range_lidar' || person.position_quality === 'multi_view';
      ctx.fillStyle = confirmed ? '#ff1744' : '#ffb300';
      ctx.beginPath();
      ctx.arc(px, py, confirmed ? 6 : 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = confirmed ? '#ffffff' : '#5d4037';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.font = '11px sans-serif';
      const label = `人${i + 1}`;
      ctx.fillText(label, px + 8, py + 3);
    }
```

- [ ] **Step 5: Handle person marker WebSocket event**

Modify `go2w_search_ws/web/static/panel.html` in `connectWS()`:

Add after the `slam` branch:

```javascript
      } else if (data.type === 'person_markers') {
        if (map && data.data && Array.isArray(data.data.markers)) {
          map.update({ person_markers: data.data.markers });
        }
        renderPersonMarkers(data.data && data.data.markers ? data.data.markers : []);
```

Add this function before `connectWS()`:

```javascript
function renderPersonMarkers(markers) {
  const el = document.getElementById('detList');
  if (!el) return;
  if (!markers || !markers.length) {
    el.innerHTML = '<div class="det-empty">未标注人员</div>';
    return;
  }
  el.innerHTML = markers.map((m, i) => {
    const conf = m.confidence ? `${Math.round(m.confidence * 100)}%` : '--';
    const quality = m.position_quality || 'unresolved';
    const link = m.photo_url ? `<a href="${m.photo_url}" target="_blank">照片</a>` : '<span>无照片</span>';
    return `<div class="det-item"><span>人${i + 1} ${quality}</span><span>${conf} ${link}</span></div>`;
  }).join('');
}
```

Add `search_room` to task icons and names:

```javascript
  const icons = { navigate: '→', search_area: '◎', search_room: '⌖', follow: '◎', move: '→', stop: '■', return_home: '↩', observe: '◎', wait: '◎' };
  const typeNames = { navigate: '导航', search_area: '区域搜索', search_room: '房间搜人', follow: '跟踪', move: '移动', stop: '停止', return_home: '返回', observe: '观察', wait: '等待' };
```

- [ ] **Step 6: Run map contract**

Run:

```bash
cd go2w_search_ws
node web/test_map_contract.js
```

Expected: PASS and prints `map contract tests passed`.

- [ ] **Step 7: Commit**

```bash
git add web/static/map.js web/static/panel.html web/test_map_contract.js
git commit -m "feat: render person search markers on map"
```

## Task 8: End-To-End Product Verification Script

**Files:**
- Create: `go2w_search_ws/web/verify_product_room_person_search.sh`
- Modify: `go2w_search_ws/docs/room_calibration.md`

- [ ] **Step 1: Create product verification script**

Create `go2w_search_ws/web/verify_product_room_person_search.sh`:

```bash
#!/bin/bash
set -u

cd "$(dirname "$0")/.."

PASS=0
FAIL=0

ok() { echo "  PASS: $1"; PASS=$((PASS + 1)); }
no() { echo "  FAIL: $1"; FAIL=$((FAIL + 1)); }

echo "===== Product room person search offline verification ====="

python -m pytest \
  web/test_product_command.py \
  web/test_person_localizer.py \
  web/test_person_mission.py \
  web/test_active_search.py \
  web/test_ai_snapshot_contract.py \
  web/test_scan_snapshot_contract.py \
  web/test_product_room_orchestrator.py \
  -v
if [ $? -eq 0 ]; then ok "python product tests"; else no "python product tests"; fi

node web/test_map_contract.js
if [ $? -eq 0 ]; then ok "map contract"; else no "map contract"; fi

python tools/test_stage_e.py
if [ $? -eq 0 ]; then ok "stage-e regression"; else no "stage-e regression"; fi

echo "===== Result: $PASS PASS, $FAIL FAIL ====="
[ "$FAIL" = "0" ]
```

- [ ] **Step 2: Run script to verify it passes**

Run:

```bash
cd go2w_search_ws
bash web/verify_product_room_person_search.sh
```

Expected: all three groups pass and the script exits with status 0.

- [ ] **Step 3: Update room calibration doc with product requirement**

Append this section to `go2w_search_ws/docs/room_calibration.md`:

```markdown

## Product Person Search Requirement

For product room person search, each room must have a calibrated `search_area`
large enough to cover the reachable room interior. `room="__current__"` uses
the current robot pose and these rectangles to resolve the current room.

Validation after editing `config/rooms.yaml`:

```bash
curl http://<NX_IP>:8000/api/reload_rooms
curl -X POST 'http://<NX_IP>:8000/api/command' \
  -H 'Content-Type: application/json' \
  -d '{"text":"去搜索这个房间，把所有人标注出来"}'
```

Expected behavior:

- The generated task is `search_room`.
- `params.search_strategy` is `next_best_view`.
- `params.target_classes` is `["person"]`.
- The mission uses LiDAR-backed person localization before publishing final person markers.
```

- [ ] **Step 4: Commit**

```bash
git add web/verify_product_room_person_search.sh docs/room_calibration.md
git commit -m "test: add product room person search verification"
```

## Final Verification

After all tasks are complete, run:

```bash
cd go2w_search_ws
python -m pytest web/test_product_command.py web/test_person_localizer.py web/test_person_mission.py web/test_active_search.py web/test_ai_snapshot_contract.py web/test_scan_snapshot_contract.py web/test_product_room_orchestrator.py -v
node web/test_map_contract.js
python tools/test_stage_e.py
bash web/verify_product_room_person_search.sh
```

Expected:

- All Python tests pass.
- `node web/test_map_contract.js` prints `map contract tests passed`.
- `tools/test_stage_e.py` prints `0 FAIL`.
- Product verification script prints `FAIL 0`.

## Hardware Acceptance Checklist

- [ ] `go2w-sensor.service` is running and `/scan`, `/odom`, `/imu` are live.
- [ ] `slam_toolbox` localization is running from a saved 2D map.
- [ ] `nav2_slim` exposes `/navigate_to_pose`.
- [ ] `config/rooms.yaml` has calibrated production room coordinates.
- [ ] YOLO model is loaded and `get_person_detection_snapshot()` returns person detections in a controlled test scene.
- [ ] LaserScan range at the person bearing is available for at least one viewpoint.
- [ ] The command `去搜索这个房间，把所有人标注出来` creates a `search_room` task with `search_strategy=next_best_view`.
- [ ] The dog navigates to at least one selected viewpoint through Nav2.
- [ ] The final `mission_report` contains person markers with `position_quality=range_lidar`, `photo_url`, and `crop_url`.
- [ ] The map displays person labels and photo links.
