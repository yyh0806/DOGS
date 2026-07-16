from pathlib import Path

import pytest

from nx_control_auth import authorize_request, cors_origin_allowed


CONTROL_TOKEN = "test-token-0123456789-ABCDEFGHIJKLMN"


@pytest.mark.parametrize(
    "path",
    [
        "/api/move",
        "/api/navigate",
        "/api/search_room",
        "/api/stand",
        "/api/balance",
        "/api/e_stop",
    ],
)
def test_control_endpoint_requires_token(path):
    decision = authorize_request(
        method="POST",
        path=path,
        headers={},
        configured_token=CONTROL_TOKEN,
    )
    assert decision.allowed is False
    assert decision.status_code == 401


def test_valid_bearer_token_is_accepted_case_insensitively():
    decision = authorize_request(
        method="POST",
        path="/api/navigate",
        headers={"authorization": f"Bearer {CONTROL_TOKEN}"},
        configured_token=CONTROL_TOKEN,
    )
    assert decision.allowed is True
    assert decision.status_code == 200


@pytest.mark.parametrize("path", ["/api/status", "/api/version", "/static/missions/a.jpg"])
def test_read_only_routes_do_not_require_a_token(path):
    decision = authorize_request(
        method="GET", path=path, headers={}, configured_token=CONTROL_TOKEN)
    assert decision.allowed is True


def test_missing_server_token_fails_closed_for_state_changes():
    decision = authorize_request(
        method="POST", path="/api/stop", headers={}, configured_token="")
    assert decision.allowed is False
    assert decision.status_code == 503
    assert decision.reason == "control_auth_not_configured"


def test_weak_server_token_fails_closed_for_state_changes():
    decision = authorize_request(
        method="POST",
        path="/api/stop",
        headers={"Authorization": "Bearer short-token"},
        configured_token="short-token",
    )
    assert decision.allowed is False
    assert decision.status_code == 503
    assert decision.reason == "control_auth_weak_token"


def test_preflight_is_allowed_only_for_configured_panel_origin():
    assert cors_origin_allowed(
        "http://192.168.43.41:8000",
        ("http://192.168.43.41:8000", "http://127.0.0.1:8000"),
    )
    assert not cors_origin_allowed(
        "https://attacker.example",
        ("http://192.168.43.41:8000",),
    )


def test_http_adapter_authorizes_before_reading_request_body():
    source = (Path(__file__).with_name("nx_web_server.py")).read_text(
        encoding="utf-8")
    post = source[source.index("        def do_POST(self):"):]
    auth = post.index("authorize_request(")
    body_read = post.index("self.rfile.read")
    assert auth < body_read
    assert 'configured_token=control_token' in post
    assert 'self._json({"ok": False, "reason": decision.reason}' in post


def test_global_stop_does_not_infer_semantics_from_referer():
    source = (Path(__file__).with_name("nx_web_server.py")).read_text(
        encoding="utf-8")
    stop = source[source.index("elif p.path == '/api/stop':"):]
    stop = stop[:stop.index("elif p.path == '/api/e_stop':")]
    assert "Referer" not in stop
    assert "release_manual" not in stop
    assert "stop_all" in stop
