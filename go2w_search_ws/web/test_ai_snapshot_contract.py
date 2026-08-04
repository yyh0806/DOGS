import sys
import time
import types
from pathlib import Path

import numpy as np
import pytest


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))


def test_person_detection_snapshot_copies_frame_and_only_yolo_people():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.arange(2 * 3 * 3, dtype=np.uint8).reshape((2, 3, 3))
    person_det = {"class": "person", "confidence": 0.91, "bbox": [1, 2, 3, 4]}
    dog_det = {"class": "dog", "confidence": 0.70, "bbox": [5, 6, 7, 8]}
    locate_det = {"class": "person", "confidence": 0.99, "bbox": [9, 10, 11, 12]}

    with ai._lock:
        ai._latest_frame = frame
        ai._latest_dets = [person_det, dog_det]
        ai._latest_locate_dets = [locate_det]
        ai._detect_frame_w = 3
        ai._detect_frame_h = 2

    snapshot = ai.get_person_detection_snapshot()

    assert snapshot["frame"] is not frame
    assert snapshot["frame"].tolist() == frame.tolist()
    assert snapshot["frame_width"] == 3
    assert snapshot["frame_height"] == 2
    assert len(snapshot["detections"]) == 1
    assert snapshot["detections"][0] is not person_det
    assert snapshot["detections"][0] == {
        "class": "person",
        "confidence": 0.91,
        "bbox": [1, 2, 3, 4],
        "frame_width": 3,
        "frame_height": 2,
        "source": "yolo",
    }


def test_detection_snapshot_filters_requested_table_class_and_copies_frame():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    table = {
        "class": "dining table", "confidence": 0.82,
        "bbox": [20, 10, 120, 90],
    }
    with ai._lock:
        ai._latest_frame = frame
        ai._latest_dets = [
            table,
            {"class": "person", "confidence": 0.91, "bbox": [130, 10, 190, 90]},
        ]
        ai._detect_frame_w = 200
        ai._detect_frame_h = 100

    snapshot = ai.get_detection_snapshot(["dining table"])

    assert snapshot["frame"] is not frame
    assert [det["class"] for det in snapshot["detections"]] == ["dining table"]
    assert snapshot["detections"][0]["bbox"] == [20, 10, 120, 90]
    assert snapshot["target_classes"] == ["dining table"]


def test_detection_snapshot_rejects_frame_from_previous_target_vocabulary():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    captured_at = time.time()
    with ai._lock:
        ai._latest_frame = np.zeros((100, 200, 3), dtype=np.uint8)
        ai._latest_dets = [{
            "class": "dining table",
            "confidence": 0.82,
            "bbox": [20, 10, 120, 90],
        }]
        ai._latest_detection_source = "c13_vis"
        ai._latest_source_frame_meta["c13_vis"] = {
            "frame_width": 200,
            "frame_height": 100,
            "ts": captured_at,
            "target_classes": ["person"],
        }

    stale = ai.get_detection_snapshot(["dining table"])

    assert stale["target_vocabulary_ready"] is False
    assert stale["inference_target_classes"] == ["person"]
    assert stale["timestamp"] == 0.0
    assert stale["detections"] == []

    with ai._lock:
        ai._latest_source_frame_meta["c13_vis"]["target_classes"] = [
            "dining table"]
    fresh = ai.get_detection_snapshot(["dining table"])

    assert fresh["target_vocabulary_ready"] is True
    assert fresh["timestamp"] == captured_at
    assert [det["class"] for det in fresh["detections"]] == [
        "dining table"]


def test_mission_detection_targets_are_forwarded_to_detector_and_restored():
    from nx_ai_node import NxAiEngine

    calls = []

    class Detector:
        def detect(self, frame, target_classes=None):
            calls.append(None if target_classes is None else list(target_classes))
            return []

    ai = NxAiEngine()
    ai._detector = Detector()
    frame = np.zeros((8, 8, 3), dtype=np.uint8)

    previous = ai.set_detection_targets(["dining table"])
    ai._run_detector(frame)
    restored = ai.set_detection_targets(previous)
    ai._run_detector(frame)

    assert previous is None
    assert restored == ["dining table"]
    assert calls == [["dining table"], None]


def test_person_detection_snapshot_rescales_detector_width_to_copied_frame():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    with ai._lock:
        ai._latest_frame = frame
        ai._latest_dets = [{
            "class": "person",
            "confidence": 0.88,
            "bbox": [860, 120, 1060, 360],
        }]
        ai._detect_frame_w = 1920
        ai._detect_frame_h = 720

    snapshot = ai.get_person_detection_snapshot()

    assert snapshot["frame_width"] == 1280
    assert snapshot["frame_height"] == 720
    bbox = snapshot["detections"][0]["bbox"]
    assert (bbox[0] + bbox[2]) / 2 == pytest.approx(640.0)
    assert bbox[0] == pytest.approx(573.333, abs=0.001)
    assert bbox[2] == pytest.approx(706.667, abs=0.001)
    assert bbox[1] == 120
    assert bbox[3] == 360


def test_person_detection_snapshot_includes_source_timestamp():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    captured_at = time.time()
    with ai._lock:
        ai._latest_frame = frame
        ai._latest_dets = [{
            "class": "person",
            "confidence": 0.88,
            "bbox": [80, 10, 120, 90],
        }]
        ai._detect_frame_w = 200
        ai._detect_frame_h = 100
        ai._latest_detection_source = "c13_vis"
        ai._latest_source_frame_meta["c13_vis"] = {
            "frame_width": 200,
            "frame_height": 100,
            "ts": captured_at,
        }

    snapshot = ai.get_person_detection_snapshot()

    assert snapshot["source"] == "c13_vis"
    assert snapshot["timestamp"] == captured_at


def test_person_detection_health_reports_fresh_detector_metadata_only():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    captured_at = time.time()
    with ai._lock:
        ai._running = True
        ai._detector_inited = True
        ai._detector = types.SimpleNamespace(
            is_world=True,
            _model_path="/home/nx/models/yolov8x-worldv2.pt",
        )
        ai._latest_frame = np.zeros((100, 200, 3), dtype=np.uint8)
        ai._latest_dets = [
            {"class": "person", "confidence": 0.88, "bbox": [80, 10, 120, 90]},
            {"class": "chair", "confidence": 0.71, "bbox": [10, 10, 30, 40]},
        ]
        ai._latest_detection_source = "c13_vis"
        ai._latest_source_frame_meta["c13_vis"] = {
            "frame_width": 200,
            "frame_height": 100,
            "ts": captured_at,
        }

    health = ai.get_person_detection_health(max_age_sec=2.0)

    assert health["healthy"] is True
    assert health["reason"] == "ok"
    assert health["detector_ready"] is True
    assert health["detector_open_vocabulary"] is True
    assert health["detector_model"].endswith("yolov8x-worldv2.pt")
    assert health["source"] == "c13_vis"
    assert health["frame_width"] == 200
    assert health["frame_height"] == 100
    assert health["detection_count"] == 2
    assert health["person_count"] == 1
    assert 0.0 <= health["age_sec"] < 0.5
    assert "frame" not in health
    assert "detections" not in health


def test_person_detection_health_rejects_stale_frame_and_missing_detector():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._running = True
        ai._detector_inited = True
        ai._detector = object()
        ai._latest_frame = np.zeros((100, 200, 3), dtype=np.uint8)
        ai._latest_detection_source = "c13_vis"
        ai._latest_source_frame_meta["c13_vis"] = {
            "frame_width": 200,
            "frame_height": 100,
            "ts": time.time() - 5.0,
        }

    stale = ai.get_person_detection_health(max_age_sec=2.0)
    assert stale["healthy"] is False
    assert stale["reason"] == "stale_detection_frame"

    with ai._lock:
        ai._detector = None
    unavailable = ai.get_person_detection_health(max_age_sec=2.0)
    assert unavailable["healthy"] is False
    assert unavailable["reason"] == "detector_not_ready"


def test_person_detection_health_rejects_mock_video_source():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._running = True
        ai._detector_inited = True
        ai._detector = object()
        ai._latest_frame = np.zeros((100, 200, 3), dtype=np.uint8)
        ai._latest_detection_source = "mock"
        ai._latest_source_frame_meta["mock"] = {
            "frame_width": 200,
            "frame_height": 100,
            "ts": time.time(),
        }

    health = ai.get_person_detection_health(max_age_sec=2.0)

    assert health["healthy"] is False
    assert health["reason"] == "mock_detection_source"


class _FakeJpeg:
    def tobytes(self):
        return b"jpeg"


def test_c13_detection_overlay_exposes_bbox_and_snapshots(monkeypatch):
    fake_cv2 = types.SimpleNamespace(
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *args, **kwargs: (True, _FakeJpeg()),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    det = {"class": "chair", "confidence": 0.83, "bbox": [40, 10, 90, 80]}

    ai._cache_detection_result(
        raw_frame=frame,
        display_frame=frame.copy(),
        dets=[det],
        source="c13_vis",
        detect_frame_w=200,
        detect_frame_h=100,
    )

    payload = ai.get_detection_overlay()

    assert payload["source"] == "c13_vis"
    assert payload["count"] == 1
    obs = payload["detections"][0]
    assert obs["class"] == "chair"
    assert obs["confidence"] == 0.83
    assert obs["bbox"] == [40.0, 10.0, 90.0, 80.0]
    assert obs["frame_width"] == 200
    assert obs["frame_height"] == 100
    assert obs["crop_url"].startswith("/api/detection_snapshot?id=")
    assert "&kind=crop" in obs["crop_url"]
    assert "&kind=frame" in obs["frame_url"]
    assert ai.get_detection_snapshot_jpeg(obs["snapshot_id"], "crop") == b"jpeg"


def test_detection_results_below_eighty_percent_are_discarded(monkeypatch):
    fake_cv2 = types.SimpleNamespace(
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *args, **kwargs: (True, _FakeJpeg()),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    ai._cache_detection_result(
        raw_frame=frame,
        display_frame=frame.copy(),
        dets=[
            {"class": "person", "confidence": 0.799,
             "bbox": [20, 10, 60, 80]},
            {"class": "person", "confidence": 0.8,
             "bbox": [100, 10, 150, 80]},
        ],
        source="c13_vis",
        detect_frame_w=200,
        detect_frame_h=100,
    )

    payload = ai.get_detection_overlay()

    assert payload["count"] == 1
    assert payload["detections"][0]["confidence"] == 0.8


def test_detection_overlays_keep_multiple_video_sources(monkeypatch):
    fake_cv2 = types.SimpleNamespace(
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *args, **kwargs: (True, _FakeJpeg()),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    ai._cache_detection_result(
        raw_frame=frame,
        display_frame=frame.copy(),
        dets=[{"class": "chair", "confidence": 0.83, "bbox": [40, 10, 90, 80]}],
        source="c13_vis",
        detect_frame_w=200,
        detect_frame_h=100,
    )
    ai._cache_detection_result(
        raw_frame=frame,
        display_frame=frame.copy(),
        dets=[{"class": "person", "confidence": 0.81, "bbox": [100, 20, 160, 90]}],
        source="c13_ir",
        detect_frame_w=200,
        detect_frame_h=100,
    )

    payload = ai.get_detection_overlays()

    assert payload["count"] == 2
    assert [stream["source"] for stream in payload["streams"]] == ["c13_vis", "c13_ir"]
    assert [det["source"] for det in payload["detections"]] == ["c13_vis", "c13_ir"]
    assert [det["class"] for det in payload["detections"]] == ["chair", "person"]
    assert {det["snapshot_id"] for det in payload["detections"]}
    assert len(ai.get_detection_list()) == 2


def test_detection_snapshot_cache_retains_clickable_history(monkeypatch):
    fake_cv2 = types.SimpleNamespace(
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *args, **kwargs: (True, _FakeJpeg()),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    first_snapshot_id = None

    for idx in range(48):
        ai._cache_detection_result(
            raw_frame=frame,
            display_frame=frame.copy(),
            dets=[{"class": "chair", "confidence": 0.83, "bbox": [40, 10, 90, 80]}],
            source="c13_vis",
            detect_frame_w=200,
            detect_frame_h=100,
        )
        if first_snapshot_id is None:
            first_snapshot_id = ai.get_detection_overlay()["detections"][0]["snapshot_id"]

    assert ai.get_detection_snapshot_jpeg(first_snapshot_id, "crop") == b"jpeg"


def test_latest_video_frame_is_available_by_source_even_without_detections(monkeypatch):
    fake_cv2 = types.SimpleNamespace(
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *args, **kwargs: (True, _FakeJpeg()),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    frame = np.zeros((100, 200, 3), dtype=np.uint8)

    ai._cache_detection_result(
        raw_frame=frame,
        display_frame=frame.copy(),
        dets=[],
        source="dog",
        detect_frame_w=200,
        detect_frame_h=100,
    )

    assert ai.get_video_frame_jpeg("dog") == b"jpeg"
    payload = ai.get_detection_overlays()
    assert payload["streams"][0]["source"] == "dog"
    assert payload["streams"][0]["frame_width"] == 200
    assert payload["streams"][0]["frame_height"] == 100


def test_detections_world_merges_multiple_video_sources():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._latest_detection_overlays = {
            "c13_vis": [{
                "id": "c13_vis-1-0",
                "class": "chair",
                "confidence": 0.8,
                "bbox": [90, 20, 110, 80],
                "frame_width": 200,
                "frame_height": 100,
                "source": "c13_vis",
                "ts": time.time(),
            }],
            "c13_ir": [{
                "id": "c13_ir-2-0",
                "class": "person",
                "confidence": 0.7,
                "bbox": [90, 20, 110, 80],
                "frame_width": 200,
                "frame_height": 100,
                "source": "c13_ir",
                "ts": time.time(),
            }],
        }
        ai._detection_source_order = ["c13_vis", "c13_ir"]

    dets = ai.get_detections_world(
        1.0,
        2.0,
        0.0,
        ranges=[],
        lidar_points=[[2.25, 0.02], [5.0, 2.0]],
    )

    assert [d["class"] for d in dets] == ["chair", "person"]
    assert [d["source"] for d in dets] == ["c13_vis", "c13_ir"]
    assert all(d["range_source"] == "livox" for d in dets)


def test_detections_world_ignores_mock_source_for_map_markers():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._latest_detection_overlays = {
            "mock": [{
                "id": "mock-1-0",
                "class": "person",
                "confidence": 0.99,
                "bbox": [90, 20, 110, 80],
                "frame_width": 200,
                "frame_height": 100,
                "source": "mock",
                "ts": time.time(),
            }],
            "dog": [{
                "id": "dog-2-0",
                "class": "chair",
                "confidence": 0.8,
                "bbox": [90, 20, 110, 80],
                "frame_width": 200,
                "frame_height": 100,
                "source": "dog",
                "ts": time.time(),
            }],
        }
        ai._detection_source_order = ["mock", "dog"]

    dets = ai.get_detections_world(
        1.0,
        2.0,
        0.0,
        ranges=[],
        lidar_points=[[2.25, 0.02]],
    )

    assert [d["source"] for d in dets] == ["dog"]
    assert [d["class"] for d in dets] == ["chair"]


def test_frame_detection_count_uses_all_video_sources():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._latest_frame = np.zeros((10, 10, 3), dtype=np.uint8)
        ai._latest_dets = []
        ai._latest_detection_overlays = {
            "c13_vis": [{"class": "chair"}, {"class": "cup"}],
            "c13_ir": [{"class": "person"}],
        }
        ai._detection_source_order = ["c13_vis", "c13_ir"]

    assert ai.get_frame_detection_count() == 3


def test_detections_world_uses_lidar_range_for_bbox_bearing():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._latest_detection_overlay = [{
            "id": "c13_vis-1-0",
            "class": "chair",
            "confidence": 0.8,
            "bbox": [90, 20, 110, 80],
            "frame_width": 200,
            "frame_height": 100,
            "source": "c13_vis",
            "ts": time.time(),
        }]

    ranges = [0.0] * 360
    ranges[180] = 2.0

    dets = ai.get_detections_world(1.0, 2.0, 0.0, ranges=ranges)

    assert len(dets) == 1
    assert dets[0]["class"] == "chair"
    assert dets[0]["x"] == pytest.approx(3.0, abs=0.01)
    assert dets[0]["y"] == pytest.approx(2.0, abs=0.01)
    assert dets[0]["range"] == pytest.approx(2.0)
    assert dets[0]["range_source"] == "lidar"


def test_detections_world_can_use_livox_local_points_when_scan_is_empty():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._latest_detection_overlay = [{
            "id": "c13_vis-2-0",
            "class": "chair",
            "confidence": 0.8,
            "bbox": [90, 20, 110, 80],
            "frame_width": 200,
            "frame_height": 100,
            "source": "c13_vis",
            "ts": time.time(),
        }]

    dets = ai.get_detections_world(
        1.0,
        2.0,
        0.0,
        ranges=[],
        lidar_points=[[2.25, 0.02], [5.0, 2.0]],
    )

    assert len(dets) == 1
    assert dets[0]["x"] == pytest.approx(3.25, abs=0.02)
    assert dets[0]["y"] == pytest.approx(2.0, abs=0.02)
    assert dets[0]["range"] == pytest.approx(2.25)
    assert dets[0]["range_source"] == "livox"


def test_detections_world_never_publishes_assumed_range_map_markers():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._latest_detection_overlay = [{
            "id": "c13_vis-3-0",
            "class": "person",
            "confidence": 0.92,
            "bbox": [90, 20, 110, 80],
            "frame_width": 200,
            "frame_height": 100,
            "source": "c13_vis",
            "ts": time.time(),
        }]

    dets = ai.get_detections_world(
        1.0, 2.0, 0.0, ranges=[], lidar_points=[])

    assert dets == []


def test_detections_world_rejects_stale_detection_even_with_current_lidar():
    from nx_ai_node import NxAiEngine

    ai = NxAiEngine()
    with ai._lock:
        ai._latest_detection_overlay = [{
            "id": "c13_vis-old-0",
            "class": "person",
            "confidence": 0.92,
            "bbox": [90, 20, 110, 80],
            "frame_width": 200,
            "frame_height": 100,
            "source": "c13_vis",
            "ts": time.time() - 5.0,
        }]

    ranges = [0.0] * 360
    ranges[180] = 2.0
    dets = ai.get_detections_world(
        1.0, 2.0, 0.0, ranges=ranges, lidar_points=[])

    assert dets == []
