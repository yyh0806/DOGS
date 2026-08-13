"""cloud_llm + llm_planner 契约测试 (mock LLM, 不真调 API)。"""
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]   # go2w_search_ws
for _p in (_ROOT, _ROOT / "web", _ROOT / "ai"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from ai.cloud_llm import CloudLLM  # noqa: E402
from nx_llm_planner import LLMPlanner, _validate_step  # noqa: E402


class _MockLLM:
    def __init__(self, answer, reasoning=None):
        self._answer = answer
        self._reasoning = reasoning
        self.configured = True

    def chat(self, system, text):
        return self._answer, self._reasoning


def test_llm_planner_accepts_valid_plan():
    llm = _MockLLM(
        '{"steps": [{"action": "go_landmark", '
        '"args": {"landmark": "大门"}}], "reasoning": "先去门口"}',
        reasoning="用户想去大门",
    )
    result = LLMPlanner(llm).plan("去大门")
    assert result is not None
    assert result["tasks"][0]["type"] == "go_landmark"
    assert result["reasoning"] == "用户想去大门"


def test_llm_planner_accepts_fetch_plan():
    llm = _MockLLM(
        '{"steps": [{"action": "fetch", "args": '
        '{"pickup": "大门口", "object": "咖啡", "deliver": null}}], '
        '"reasoning": "取咖啡"}')
    result = LLMPlanner(llm).plan("去大门口拿咖啡")
    assert result["tasks"][0]["type"] == "fetch"


def test_llm_planner_rejects_unknown_action():
    llm = _MockLLM('{"steps": [{"action": "fly", "args": {}}], '
                   '"reasoning": "飞"}')
    assert LLMPlanner(llm).plan("飞过去") is None


def test_llm_planner_rejects_bad_args():
    llm = _MockLLM('{"steps": [{"action": "move_relative", "args": '
                   '{"direction": "up", "distance_m": 1}}]}')
    assert LLMPlanner(llm).plan("向上走") is None


def test_llm_planner_rejects_malformed_json():
    llm = _MockLLM("不是 JSON")
    assert LLMPlanner(llm).plan("随便说") is None


def test_llm_planner_rejects_too_many_steps():
    steps = ",".join(
        '{"action": "go_landmark", "args": {"landmark": "L%d"}}' % i
        for i in range(5))
    llm = _MockLLM('{"steps": [%s]}' % steps)
    assert LLMPlanner(llm).plan("多步") is None


def test_llm_planner_handles_markdown_fence():
    llm = _MockLLM(
        '```json\n{"steps": [{"action": "follow", '
        '"args": {"target": "穿黑衣服的人"}}]}\n```')
    result = LLMPlanner(llm).plan("跟踪穿黑衣服的人")
    assert result is not None
    assert result["tasks"][0]["type"] == "follow"


def test_validate_step_plugin_action():
    ok, _ = _validate_step(
        {"action": "fetch", "args": {"pickup": "大门口", "object": "咖啡"}})
    assert ok


def test_cloud_llm_unconfigured_fails_closed():
    llm = CloudLLM(api_key="")
    assert not llm.configured
    assert llm.chat("s", "u") == (None, None)
