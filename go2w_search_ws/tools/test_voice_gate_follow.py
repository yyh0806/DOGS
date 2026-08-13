"""voice gate 的 follow/go_landmark 接入回归测试 (review round8 补缺)。"""
import sys
from pathlib import Path

import pytest

_TOOLS = Path(__file__).resolve().parents[0]
_WEB = _TOOLS.parent / "web"
for _p in (_WEB, _TOOLS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import voice_console  # noqa: E402


def test_follow_passes_voice_gate():
    v = voice_console.validate_voice_command("跟踪穿黑衣服的人")
    assert v["ok"] is True
    assert v["task"]["type"] == "follow"
    assert v["task"]["params"]["target"] == "穿黑衣服的人"


def test_go_landmark_passes_voice_gate():
    v = voice_console.validate_voice_command("去大门")
    assert v["ok"] is True
    assert v["task"]["type"] == "go_landmark"
    assert v["task"]["params"]["landmark"] == "大门"


def test_follow_acknowledgement_text():
    assert voice_console.accepted_acknowledgement(
        {"task": {"type": "follow", "params": {"target": "x"}}}) == "跟踪任务已接收"
    assert voice_console.accepted_acknowledgement(
        {"task": {"type": "go_landmark", "params": {"landmark": "大门"}}}) == "地标导航任务已接收"
    assert voice_console.accepted_acknowledgement(
        {"task": {"type": "search_room", "params": {}}}) == "搜索任务已接收"
