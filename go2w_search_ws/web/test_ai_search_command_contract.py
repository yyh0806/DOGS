"""Safety contract for the NX VLM command adapter."""

import json
import sys
import threading
from pathlib import Path


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

import nx_ai_node as ai


def test_vlm_prompt_exposes_only_the_canonical_search_room_task():
    prompt = ai.NxAiEngine._SYS_PROMPT

    assert "search_room" in prompt
    assert "target_classes" in prompt
    assert "require_photos" in prompt
    assert "mark_on_map" in prompt
    for legacy_type in ("move", "follow", "search_area", "return_home"):
        assert legacy_type not in prompt


def test_vlm_result_is_canonicalized_through_the_mission_schema():
    result = ai.NxAiEngine._validate_vlm_search_result({
        "response": "开始搜索",
        "tasks": [{
            "type": "search_room",
            "priority": 8,
            "params": {
                "room": "__current__",
                "target_classes": ["chair", "chair"],
                "require_photos": True,
                "mark_on_map": True,
            },
        }],
    })

    task = result["tasks"][0]
    assert task["type"] == "search_room"
    assert task["params"]["room"] == "__current__"
    assert task["params"]["target_classes"] == ["chair"]
    assert task["params"]["mission_request"]["room"] == "current_room"
    assert task["params"]["search_strategy"] == "frontier_explore"
    assert task["params"]["require_photos"] is True
    assert task["params"]["mark_on_map"] is True


def test_vlm_result_rejects_legacy_or_ambiguous_tasks():
    invalid_results = (
        {"tasks": [{"type": "move", "params": {"vx": 0.2}}]},
        {"tasks": [{"type": "search_area", "params": {}}]},
        {"tasks": []},
        {"tasks": [
            {"type": "search_room", "params": {}},
            {"type": "search_room", "params": {}},
        ]},
    )

    for value in invalid_results:
        result = ai.NxAiEngine._validate_vlm_search_result(value)
        assert result["tasks"] == []
        assert result["parse_error"] == "invalid_vlm_mission"


def test_parse_failure_never_synthesizes_a_motion_task():
    for text in ("前进两米", "跟着前面的人", "搜索一个没有目标的房间"):
        result = ai.NxAiEngine._parse_failure("vlm_unavailable", text=text)
        assert result["tasks"] == []
        assert result["parse_error"] == "vlm_unavailable"
        assert result["understanding"] == text


def test_proxy_timeout_returns_an_empty_task_contract():
    class ImmediateTimeoutEvent(threading.Event):
        def wait(self, timeout=None):
            return False

    class FakeEngine:
        @staticmethod
        def submit_parse(text):
            return {
                "text": text,
                "response": None,
                "done": ImmediateTimeoutEvent(),
            }

        _parse_failure = staticmethod(ai.NxAiEngine._parse_failure)

    result = json.loads(ai.NxAiVlmProxy(FakeEngine()).chat([
        {"role": "user", "content": "前进两米"},
    ]))

    assert result["tasks"] == []
    assert result["parse_error"] == "vlm_timeout"

