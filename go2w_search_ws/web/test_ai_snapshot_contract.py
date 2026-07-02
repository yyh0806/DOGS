import sys
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
