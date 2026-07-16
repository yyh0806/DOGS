import argparse
import json
import time
from collections import deque
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from nav_health_gate import load_fresh, satisfied
from nav_health_supervisor import atomic_write, stamp_age_snapshot, topic_snapshot


def test_topic_snapshot_reports_rate_and_age():
    report = topic_snapshot(deque([10.0, 10.1, 10.2]), 10.25)
    assert report["samples"] == 3
    assert 9.9 < report["rate_hz"] < 10.1
    assert report["age_sec"] == pytest.approx(0.05)


def test_stamp_age_snapshot_requires_history_and_reports_p95():
    # Nearest-rank P95 of 20 values is the 19th value, so two stalls must be
    # represented to prove the tail gate catches a 10% intermittent backlog.
    report = stamp_age_snapshot(deque([0.05] * 18 + [0.40, 0.40]))
    assert report["stamp_samples"] == 20
    assert report["stamp_median_sec"] == pytest.approx(0.05)
    assert report["stamp_p95_sec"] == pytest.approx(0.40)


def test_atomic_snapshot_and_all_gate_types(tmp_path):
    path = tmp_path / "health.json"
    data = {
        "updated_wall": time.time(),
        "topics": {"/odom": {"samples": 20, "rate_hz": 10.0, "age_sec": 0.1,
                              "stamp_samples": 20, "stamp_median_sec": 0.08,
                              "stamp_p95_sec": 0.10},
                   "/map_frontier": {"samples": 1, "rate_hz": None, "age_sec": 0.2}},
        "tf": {"map->base_link": True},
        "lifecycle": {"controller_server": {"id": 3, "label": "active"}},
        "actions": {"/navigate_to_pose": True},
        "parents_dynamic": {"base_link": ["odom"]},
        "parents_static": {},
    }
    atomic_write(path, data)
    loaded = load_fresh(path)
    common = {"stamp_age": None, "dog_ready": False}
    assert satisfied(loaded, argparse.Namespace(rate=["/odom", 5.0], message=None, tf=None, lifecycle=None, action=None, single_parent=None, **common))[0]
    assert satisfied(loaded, argparse.Namespace(rate=None, message="/map_frontier", tf=None, lifecycle=None, action=None, single_parent=None, **common))[0]
    assert satisfied(loaded, argparse.Namespace(rate=None, message=None, tf=["map", "base_link"], lifecycle=None, action=None, single_parent=None, **common))[0]
    assert satisfied(loaded, argparse.Namespace(rate=None, message=None, tf=None, lifecycle=["controller_server"], action=None, single_parent=None, **common))[0]
    assert satisfied(loaded, argparse.Namespace(rate=None, message=None, tf=None, lifecycle=None, action="/navigate_to_pose", single_parent=None, **common))[0]
    assert satisfied(loaded, argparse.Namespace(rate=None, message=None, tf=None, lifecycle=None, action=None, single_parent="base_link", **common))[0]
    stamp_common = {"dog_ready": False}
    assert satisfied(loaded, argparse.Namespace(rate=None, stamp_age=["/odom", 0.35], message=None, tf=None, lifecycle=None, action=None, single_parent=None, **stamp_common))[0]


def test_stamp_age_gate_rejects_single_fresh_sample_and_high_p95():
    base = {
        "updated_wall": time.time(),
        "topics": {"/Odometry": {"stamp_samples": 1, "stamp_median_sec": 0.05,
                                   "stamp_p95_sec": 0.05}},
    }
    args = argparse.Namespace(rate=None, stamp_age=["/Odometry", 0.35], message=None,
                              tf=None, lifecycle=None, action=None,
                              single_parent=None, dog_ready=False)
    assert not satisfied(base, args)[0]
    base["topics"]["/Odometry"] = {
        "stamp_samples": 20, "stamp_median_sec": 0.05, "stamp_p95_sec": 0.40,
    }
    assert not satisfied(base, args)[0]


def test_stamp_gate_requires_twenty_new_samples_after_gate_starts():
    data = {
        "topics": {"/Odometry": {
            "stamp_samples": 100, "stamp_total": 119,
            "stamp_median_sec": 0.05, "stamp_p95_sec": 0.06,
        }}
    }
    args = argparse.Namespace(
        rate=None, stamp_age=["/Odometry", 0.35], message=None, tf=None,
        lifecycle=None, action=None, single_parent=None, dog_ready=False,
        stamp_baseline=100,
    )
    assert satisfied(data, args)[0] is False
    data["topics"]["/Odometry"]["stamp_total"] = 120
    assert satisfied(data, args)[0] is True


def test_rate_and_message_gates_wait_cleanly_when_topic_has_no_samples():
    data = {
        "topics": {
            "/wheel_odom": {"samples": 0, "rate_hz": None, "age_sec": None},
            "/map_frontier": {"samples": 0, "rate_hz": None, "age_sec": None},
        }
    }
    common = dict(stamp_age=None, tf=None, lifecycle=None, action=None,
                  single_parent=None, dog_ready=False)
    rate = argparse.Namespace(rate=["/wheel_odom", 20.0], message=None, **common)
    message = argparse.Namespace(rate=None, message="/map_frontier", **common)
    assert satisfied(data, rate)[0] is False
    assert satisfied(data, message)[0] is False


def test_bringup_uses_single_supervisor_for_all_ros_health_gates():
    source = (ROOT / "docker" / "bringup_slam_nav2.sh").read_text(encoding="utf-8")
    marker = "# ---- single-participant health gates ----"
    assert marker in source
    main_path = source[source.index(marker):]
    assert "nav_health_supervisor.py" in main_path
    assert 'python3 "$NAV_HEALTH_GATE"' in main_path
    assert main_path.index("nav_health_supervisor.py") < main_path.index("start_transient wheel-odom")
    executable = "\n".join(
        line for line in main_path.splitlines()
        if not line.lstrip().startswith(("#", "echo "))
    )
    for obsolete_probe in (
        "topic_rate_gate.py", "fastlio_latency_gate.py", "wait_lifecycle_active.py",
        "ros2 topic echo", "tf2_echo", "ros2 action list",
    ):
        assert obsolete_probe not in executable
