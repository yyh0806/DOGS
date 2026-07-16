import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from perception_preflight import evaluate_perception_status


def healthy_status():
    return {
        "release_id": "release-1",
        "motion_release_id": "release-1",
        "release_consistent": True,
        "perception": {
            "healthy": True,
            "reason": "ok",
            "running": True,
            "detector_initialized": True,
            "detector_ready": True,
            "detector_open_vocabulary": True,
            "frame_available": True,
            "source": "c13_vis",
            "age_sec": 0.1,
        },
    }


def test_perception_preflight_accepts_fresh_real_open_vocabulary_detector():
    report = evaluate_perception_status(healthy_status(), "release-1")

    assert report["ok"] is True
    assert report["read_only"] is True
    assert all(item["ok"] for item in report["checks"])


def test_perception_preflight_rejects_wrong_release_closed_vocab_mock_or_stale():
    mutations = (
        ("release_id", "old"),
        ("perception.detector_open_vocabulary", False),
        ("perception.source", "mock"),
        ("perception.age_sec", 9.0),
        ("perception.detector_ready", False),
    )
    for path, value in mutations:
        status = healthy_status()
        if path.startswith("perception."):
            status["perception"][path.split(".", 1)[1]] = value
        else:
            status[path] = value

        assert evaluate_perception_status(status, "release-1")["ok"] is False


def test_perception_preflight_has_no_publish_or_motion_client():
    source = (TOOLS_DIR / "perception_preflight.py").read_text(encoding="utf-8")

    assert "create_publisher" not in source
    assert "SportClient" not in source
    assert "NavigateToPose" not in source

