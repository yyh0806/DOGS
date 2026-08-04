import json
from pathlib import Path

from nx_motion_intent import build_motion_intent


def test_web_builds_versioned_canonical_motion_intent():
    encoded = build_motion_intent(
        "start_nav", source="navigation_arbiter", request_id="request-1")
    assert json.loads(encoded) == {
        "schema_version": 1,
        "request_id": "request-1",
        "intent": "start_nav",
        "source": "navigation_arbiter",
    }


def test_web_server_never_publishes_legacy_motion_session_strings():
    source = Path(__file__).with_name("nx_web_server.py").read_text(
        encoding="utf-8")
    assert "build_motion_intent(" in source
    assert 'publish_motion_session(f"{owner}_start")' not in source
    assert 'command = "nav_stop"' not in source


def test_navigation_readiness_consumes_status_v4_canonical_fields():
    source = Path(__file__).with_name("nx_web_server.py").read_text(
        encoding="utf-8")
    callback = source[source.index("    def _on_dog_state("):]
    callback = callback[:callback.index("    def _on_imu(")]
    for field in (
        "schema_version", "session", "physical_mode", "actual_motion",
        "velocity_authorized", "motion_service", "raw",
    ):
        assert f"d.get('{field}'" in callback or f'd.get("{field}"' in callback
