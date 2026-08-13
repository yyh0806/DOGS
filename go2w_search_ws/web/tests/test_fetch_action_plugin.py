"""fetch 插件契约测试 — 模板解析 + 状态机执行 (全部 mock ctx)。"""
import sys
from pathlib import Path

import pytest

_WEB = Path(__file__).resolve().parents[1]
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))

from nx_action_plugin import register_action, get_action, parse_plugin_intent  # noqa: E402
from nx_fetch_action import FetchActionPlugin, parse_fetch_command  # noqa: E402


@pytest.fixture(autouse=True)
def _register():
    register_action(FetchActionPlugin())
    yield


@pytest.mark.parametrize("text,expected_pickup,expected_obj", [
    ("去大门口拿咖啡", "大门口", "咖啡"),
    ("到门口帮我拿一杯咖啡", "门口", "咖啡"),
    ("去厨房取一瓶水过来", "厨房", "水"),
    ("拿咖啡", None, None),  # 无 pickup → 拒绝
    ("去大门口", None, None),  # 纯地标导航 → 不落 fetch
])
def test_parse_fetch(text, expected_pickup, expected_obj):
    r = parse_fetch_command(text)
    if expected_pickup is None:
        assert r is None
        return
    assert r is not None
    assert r["tasks"][0]["type"] == "fetch"
    assert r["tasks"][0]["params"]["pickup"] == expected_pickup
    assert r["tasks"][0]["params"]["object"] == expected_obj


def test_validate_params():
    plugin = get_action("fetch")
    ok, _ = plugin.validate({"pickup": "大门口", "object": "咖啡"})
    assert ok
    ok2, reason2 = plugin.validate({"pickup": "", "object": "咖啡"})
    assert not ok2 and "pickup" in reason2
    ok3, reason3 = plugin.validate({"pickup": "大门口", "object": ""})
    assert not ok3 and "object" in reason3


class _Task:
    def __init__(self, params):
        self.params = params
        self.status = None
        self.result = None


def _mk_ctx(ws_events, nav_results, locate_results, confirm_result=True):
    return {
        "ws": lambda msg: ws_events.append(msg),
        "landmarks_find": lambda name: type("LM", (), {
            "x": 2.0, "y": 1.0, "yaw": 0.0, "frame_id": "map"})(),
        "point_nav": lambda x, y, yaw, frame_id=None: nav_results.pop(0),
        "locate": lambda obj: locate_results.pop(0),
        "confirm_wait": lambda timeout=None: confirm_result,
        "robot_pos": lambda: (0.0, 0.0, 0.0),
    }


def test_execute_happy_path_s1_s5():
    events = []
    ctx = _mk_ctx(
        events,
        nav_results=[{"ok": True}, {"ok": True}],   # S1 去 + S5 回
        locate_results=[{"found": True}],
    )
    task = _Task({"pickup": "大门口", "object": "咖啡", "deliver": None})
    get_action("fetch").execute(task, ctx)
    assert task.status == "completed"
    phases = [e["data"]["phase"] for e in events]
    assert phases == ["NAVIGATING", "ARRIVED", "FOUND",
                      "AWAITING_LOAD", "LOADED", "RETURNING", "DONE"]


def test_execute_unknown_landmark_fails_closed():
    events = []
    ctx = _mk_ctx(events, nav_results=[], locate_results=[])
    ctx["landmarks_find"] = lambda name: None
    task = _Task({"pickup": "不存在", "object": "咖啡"})
    get_action("fetch").execute(task, ctx)
    assert task.status == "failed"
    assert "未知地标" in task.result


def test_execute_nav_fail_stops():
    events = []
    ctx = _mk_ctx(events, nav_results=[{"ok": False, "reason": "blocked"}],
                  locate_results=[])
    task = _Task({"pickup": "大门口", "object": "咖啡"})
    get_action("fetch").execute(task, ctx)
    assert task.status == "failed"
    assert "导航" in task.result


def test_execute_confirm_timeout_fails():
    events = []
    ctx = _mk_ctx(events, nav_results=[{"ok": True}],
                  locate_results=[{"found": True}], confirm_result=False)
    task = _Task({"pickup": "大门口", "object": "咖啡"})
    get_action("fetch").execute(task, ctx)
    assert task.status == "failed"
    assert "装载" in task.result


def test_plugin_intent_chain_parses_fetch():
    r = parse_plugin_intent("去大门口拿咖啡")
    assert r is not None and r["tasks"][0]["type"] == "fetch"


@pytest.mark.parametrize("text", [
    "先去门口，然后拿咖啡",
    "去大门口拿咖啡，然后送到客厅",
    "先巡逻一圈，然后去大门口拿咖啡",
])
def test_review_compound_falls_back_to_llm(text):
    # 复合连接词 → fetch 模板不匹配, 交 LLM 多步计划
    assert parse_fetch_command(text) is None
