import http.server
import subprocess
import sys
import importlib.util
import json
import threading
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
VOICE_CONSOLE = ROOT / "tools" / "voice_console.py"
VOICE_REQUIREMENTS = ROOT / "requirements-voice.txt"


def load_voice_console():
    spec = importlib.util.spec_from_file_location(
        "voice_console_for_test", VOICE_CONSOLE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_voice_console_help_works_without_site_packages():
    result = subprocess.run(
        [sys.executable, "-S", str(VOICE_CONSOLE), "--help"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--nx" in result.stdout
    assert "--model" in result.stdout
    assert "--no-auto-send" in result.stdout
    assert "--text" in result.stdout
    assert "--dedupe-seconds" in result.stdout
    assert "--token-file" in result.stdout


def test_voice_requirements_cover_runtime_imports():
    requirements = {
        line.split("#", 1)[0].strip().lower()
        for line in VOICE_REQUIREMENTS.read_text(encoding="utf-8").splitlines()
        if line.split("#", 1)[0].strip()
    }

    assert {"requests", "sounddevice", "vosk", "pyttsx3", "websocket-client"}.issubset(
        requirements
    )


def test_default_voice_model_uses_ignored_project_models_directory(monkeypatch):
    monkeypatch.delenv("VOSK_MODEL_PATH", raising=False)
    module = load_voice_console()

    assert Path(module.default_model_path()) == (
        ROOT / "models" / "vosk-model-small-cn-0.22"
    )


def test_validate_search_command_accepts_spaced_stt_product_phrase():
    module = load_voice_console()

    result = module.validate_search_command(
        "去 搜索 这个 房间，把 所有 人 标注 出来")

    assert result["ok"] is True
    assert result["task"]["type"] == "search_room"
    assert result["task"]["params"]["room"] == "__current__"
    assert result["task"]["params"]["target_classes"] == ["person"]
    assert result["task"]["params"]["require_photos"] is True
    assert result["task"]["params"]["mark_on_map"] is True
    assert result["fingerprint"]


def test_validate_search_command_accepts_spaced_table_search_phrase():
    module = load_voice_console()

    result = module.validate_search_command(
        "去 搜索 这个 房间，把 所有 桌子 标记 出来")

    assert result["ok"] is True
    assert result["task"]["type"] == "search_room"
    assert result["task"]["params"]["target_classes"] == ["dining table"]
    assert result["task"]["params"]["require_photos"] is True
    assert result["task"]["params"]["mark_on_map"] is True


def test_validate_search_command_rejects_unrelated_and_negated_search():
    module = load_voice_console()
    # "前进两米" 现已支持 (move_relative, spec §1.1); 只测真正不支持的文本
    for transcript in ("别搜索这个房间", "你好"):
        result = module.validate_search_command(transcript)
        assert result == {
            "ok": False,
            "reason": "unsupported_voice_command",
            "text": transcript,
        }


def _serve_admission(payload, status=202):
    requests = []

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            request = {
                "path": self.path,
                "body": json.loads(self.rfile.read(length).decode("utf-8")),
            }
            if self.headers.get("Authorization"):
                request["authorization"] = self.headers["Authorization"]
            requests.append(request)
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread, requests


def test_send_command_returns_confirmed_nx_admission_payload():
    module = load_voice_console()
    payload = {
        "ok": True,
        "accepted": True,
        "parser": "product",
        "response": "搜索当前房间并标注所有人",
        "admission": {"ok": True, "owner": "tasks"},
    }
    server, thread, requests = _serve_admission(payload)
    try:
        url = f"http://127.0.0.1:{server.server_port}/api/command"
        result = module.send_command(url, "去搜索这个房间，把所有人标注出来")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert requests == [{
        "path": "/api/command",
        "body": {"text": "去搜索这个房间，把所有人标注出来"},
    }]
    assert result["ok"] is True
    assert result["accepted"] is True
    assert result["transport_ok"] is True
    assert result["status_code"] == 202
    assert result["admission"]["owner"] == "tasks"


def test_send_command_does_not_treat_unconfirmed_2xx_as_success():
    module = load_voice_console()
    server, thread, _ = _serve_admission({
        "ok": True, "accepted": None, "queued": True, "parser": "async"})
    try:
        url = f"http://127.0.0.1:{server.server_port}/api/command"
        result = module.send_command(url, "搜索这个房间的人")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert result["ok"] is False
    assert result["accepted"] is None
    assert result["transport_ok"] is True
    assert result["reason"] == "admission_unconfirmed"


def test_send_command_uses_explicit_bearer_token():
    module = load_voice_console()
    server, thread, requests = _serve_admission({
        "ok": True, "accepted": True})
    try:
        url = f"http://127.0.0.1:{server.server_port}/api/command"
        result = module.send_command(url, "搜索这个房间的人", token="abc_123-XYZ")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert result["ok"] is True
    assert requests[0]["authorization"] == "Bearer abc_123-XYZ"


def test_control_token_file_must_be_one_url_safe_line(tmp_path):
    module = load_voice_console()
    token_file = tmp_path / "control-token.txt"
    strong_token = "abc_123-XYZ~.0123456789-ABCDEFGHIJ"
    token_file.write_text(strong_token + "\n", encoding="ascii")

    assert module.load_control_token(token_file, environ={}) == strong_token

    token_file.write_text("first\nsecond\n", encoding="ascii")
    with pytest.raises(ValueError, match="one URL-safe line"):
        module.load_control_token(token_file, environ={})


def test_control_token_file_rejects_weak_token(tmp_path):
    module = load_voice_console()
    token_file = tmp_path / "control-token.txt"
    token_file.write_text("short-token\n", encoding="ascii")

    with pytest.raises(ValueError, match="at least 32"):
        module.load_control_token(token_file, environ={})


def test_dispatcher_suppresses_duplicate_only_after_confirmed_admission():
    module = load_voice_console()
    now = [100.0]
    sends = []

    def sender(url, text):
        sends.append((url, text))
        return {"ok": True, "accepted": True}

    dispatcher = module.SearchCommandDispatcher(
        sender=sender, dedupe_seconds=10.0, monotonic=lambda: now[0])
    text = "去搜索这个房间，把所有人标注出来"

    first = dispatcher.dispatch("http://nx/api/command", text)
    duplicate = dispatcher.dispatch("http://nx/api/command", text)
    now[0] += 10.1
    later = dispatcher.dispatch("http://nx/api/command", text)

    assert first["ok"] is True
    assert duplicate["ok"] is False
    assert duplicate["reason"] == "duplicate_voice_command"
    assert later["ok"] is True
    assert sends == [
        ("http://nx/api/command", text),
        ("http://nx/api/command", text),
    ]


def test_dispatcher_retries_after_failed_admission_and_never_sends_non_search():
    module = load_voice_console()
    replies = [
        {"ok": False, "accepted": False, "reason": "robot_unavailable"},
        {"ok": True, "accepted": True},
    ]
    sends = []

    def sender(url, text):
        sends.append(text)
        return replies.pop(0)

    dispatcher = module.SearchCommandDispatcher(sender=sender)
    text = "搜索当前房间标注所有人"

    assert dispatcher.dispatch("http://nx/api/command", text)["ok"] is False
    assert dispatcher.dispatch("http://nx/api/command", text)["ok"] is True
    rejected = dispatcher.dispatch("http://nx/api/command", "你好")

    assert rejected["reason"] == "unsupported_voice_command"
    assert sends == [text, text]


def test_text_validation_mode_needs_no_model_or_microphone(tmp_path):
    missing_model = tmp_path / "missing-vosk-model"
    result = subprocess.run(
        [
            sys.executable,
            "-S",
            str(VOICE_CONSOLE),
            "--model",
            str(missing_model),
            "--text",
            "去 搜索 这个 房间，把 所有 人 标注 出来",
            "--no-auto-send",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "search_room" in result.stdout
    assert "__current__" in result.stdout
    assert "仅验证" in result.stdout


# ---- move_relative 放行 (spec §3.1) ----
def test_validate_voice_command_accepts_forward_move():
    module = load_voice_console()
    result = module.validate_voice_command("前进两米")
    assert result["ok"] is True
    assert result["task"]["type"] == "move_relative"
    assert result["task"]["params"]["direction"] == "forward"
    assert result["task"]["params"]["distance_m"] == 2.0


def test_validate_voice_command_accepts_angular_turn():
    module = load_voice_console()
    result = module.validate_voice_command("左转45度")
    assert result["ok"] is True
    assert result["task"]["params"]["mode"] == "angular"


def test_validate_voice_command_rejects_unrelated_text():
    module = load_voice_console()
    result = module.validate_voice_command("今天天气真好")
    assert result["ok"] is False
    assert result["reason"] == "unsupported_voice_command"


def test_validate_voice_command_still_accepts_search():
    module = load_voice_console()
    result = module.validate_voice_command("搜索这个房间里所有人")
    assert result["ok"] is True
    assert result["task"]["type"] == "search_room"


def test_dedupe_distinguishes_move_distances():
    """前进两米 vs 前进一米 不同 fingerprint, 不互相压制."""
    module = load_voice_console()
    sent = []
    dispatcher = module.SearchCommandDispatcher(
        sender=lambda url, text: sent.append(text) or {"ok": True, "accepted": True})
    assert dispatcher.dispatch("http://x", "前进两米").get("ok")
    assert dispatcher.dispatch("http://x", "前进一米").get("ok")
