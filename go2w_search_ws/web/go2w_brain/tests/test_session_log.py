"""轨迹日志: jsonl 完整性 + 续写 + 消息重建。"""
from __future__ import annotations

from go2w_brain.session_log import SessionLog


def test_append_and_entries_roundtrip(tmp_path):
    log = SessionLog(tmp_path / "s.jsonl")
    log.append("task", content="报告状态")
    log.append("tool_call", name="get_gps", args={}, ok=True)
    entries = log.entries()
    assert [e["kind"] for e in entries] == ["task", "tool_call"]
    assert entries[1]["name"] == "get_gps"
    assert all(isinstance(e["ts"], float) for e in entries)


def test_resume_appends_same_file(tmp_path):
    path = tmp_path / "s.jsonl"
    SessionLog(path).append("task", content="t1")
    SessionLog.resume(path).append("reply", content="r1")
    kinds = [e["kind"] for e in SessionLog(path).entries()]
    assert kinds == ["task", "reply"]  # 续写不截断


def test_stats_counts(tmp_path):
    log = SessionLog(tmp_path / "s.jsonl")
    log.append("task", content="t")
    log.append("tool_call", name="a", ok=True)
    log.append("tool_result", name="a", result={})
    assert log.stats() == {"task": 1, "tool_call": 1, "tool_result": 1}


def test_corrupt_line_survives(tmp_path):
    path = tmp_path / "s.jsonl"
    log = SessionLog(path)
    log.append("task", content="t")
    with open(path, "a", encoding="utf-8") as fp:
        fp.write("{bad json\n")
    entries = log.entries()
    assert entries[-1]["kind"] == "corrupt_line"


def test_load_messages_reconstructs(tmp_path):
    log = SessionLog(tmp_path / "s.jsonl")
    log.append("task", content="报告")
    log.append("snapshot", data={"battery_soc": 50.0})
    log.append("tool_result", name="get_battery",
               result={"ok": True, "battery_soc": 50.0})
    log.append("reply", content="电量 50%")
    messages = log.load_messages()
    roles = [m["role"] for m in messages]
    assert roles == ["user", "user", "user", "assistant"]
    assert "50.0" in messages[2]["content"]
