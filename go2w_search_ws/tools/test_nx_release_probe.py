from pathlib import Path
import sys
import types

import pytest


def _version(release="release-1"):
    return {
        "release_id": release,
        "motion_release_id": release,
        "release_consistent": True,
    }


def _dog_state(release="release-1"):
    return {
        "schema_version": 4,
        "release_id": release,
        "sdk_ready": True,
        "telemetry_fresh": True,
        "motion_service": "ai-w",
        "session": "parked",
        "physical_mode": "joint_lock",
        "actual_motion": "stopped",
        "velocity_authorized": False,
        "fault": None,
    }


def test_release_probe_accepts_only_feedback_confirmed_safe_park():
    from nx_release_probe import validate_release_evidence

    report = validate_release_evidence(
        "release-1", _version(), _dog_state(), require_sdk_ready=True)

    assert report["ok"] is True
    assert report["release_id"] == "release-1"
    assert report["motion"]["session"] == "parked"


@pytest.mark.parametrize(
    "version, dog, reason",
    [
        (_version("old"), _dog_state(), "web release mismatch"),
        (_version(), _dog_state("old"), "dog release mismatch"),
        (_version(), {**_dog_state(), "sdk_ready": False}, "SDK is not ready"),
        (_version(), {**_dog_state(), "telemetry_fresh": False}, "telemetry is stale"),
        (_version(), {**_dog_state(), "session": "nav_active"}, "motion session is not parked"),
        (_version(), {**_dog_state(), "actual_motion": "moving"}, "wheels are not stopped"),
        (_version(), {**_dog_state(), "fault": "robot_error"}, "motion fault is latched"),
    ],
)
def test_release_probe_rejects_inconsistent_or_unsafe_state(version, dog, reason):
    from nx_release_probe import ReleaseProbeError, validate_release_evidence

    with pytest.raises(ReleaseProbeError, match=reason):
        validate_release_evidence(
            "release-1", version, dog, require_sdk_ready=True)


def test_release_probe_is_structurally_read_only():
    source = (Path(__file__).with_name("nx_release_probe.py")).read_text(
        encoding="utf-8")

    assert "create_subscription" in source
    assert "create_publisher" not in source
    assert "/cmd_vel" not in source
    assert "SportClient" not in source


def test_probe_refreshes_version_after_web_starts_before_motion_feedback(
        monkeypatch):
    import nx_release_probe as probe

    versions = [
        {"release_id": "release-1", "motion_release_id": None,
         "release_consistent": False},
        _version(),
    ]
    fetch_count = {"value": 0}

    def fetch(_url, _timeout):
        index = min(fetch_count["value"], len(versions) - 1)
        fetch_count["value"] += 1
        return versions[index]

    class Node:
        def create_subscription(self, _kind, _topic, callback, _depth):
            self.callback = callback

        def destroy_node(self):
            pass

    node = Node()
    fake_rclpy = types.SimpleNamespace(
        ok=lambda: True,
        create_node=lambda _name: node,
        spin_once=lambda current, timeout_sec: current.callback(
            types.SimpleNamespace(data=__import__("json").dumps(_dog_state()))),
    )
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = object
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    monkeypatch.setattr(probe, "_fetch_version", fetch)

    report = probe._wait_for_evidence(
        expected_release="release-1",
        version_url="http://local/api/version",
        timeout=0.05,
        require_sdk_ready=True,
    )

    assert report["ok"] is True
    assert fetch_count["value"] >= 2
