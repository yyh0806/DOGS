import sys
from pathlib import Path

import numpy as np


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
