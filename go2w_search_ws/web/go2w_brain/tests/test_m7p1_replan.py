"""M7.1 重规划测试: 计划步骤失败 → LLM 修订计划 (桩 LLM, 确定性)。"""
from __future__ import annotations

import json
import os
from pathlib import Path

_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

from go2w_brain.brain_loop import BrainSession  # noqa: E402
from go2w_brain.memory import MemoryStore  # noqa: E402
from go2w_brain.platform import MockAdapter  # noqa: E402
from go2w_brain.session_log import SessionLog  # noqa: E402
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402
from nx_drowning_detect import DrowningDetector  # noqa: E402
from nx_water_guard import WaterGuard  # noqa: E402
from synthetic_frame_source import SyntheticPatrolSource  # noqa: E402

BAD_DRAFT = json.dumps({"steps": [
    {"id": "p1", "verb": "plan_lake_loop", "args": {}, "preconditions": [],
     "depends": [], "memory_refs": []},
    {"id": "f1", "verb": "follow_route", "args": {"from_plan": True},
     "preconditions": ["mission_lock", "water_guard_armed"],
     "depends": ["p1"], "memory_refs": []},
    {"id": "a1", "verb": "arm_water_guard", "args": {"from_plan": True},
     "preconditions": [], "depends": ["p1"], "memory_refs": []},
]})  # follow 在 arm 之前 → 运行期守卫未布防 → 步骤失败

REVISED_DRAFT = json.dumps({"steps": [
    {"id": "r1", "verb": "arm_water_guard", "args": {"from_plan": True},
     "preconditions": [], "depends": [], "memory_refs": []},
    {"id": "r2", "verb": "follow_route", "args": {"from_plan": True},
     "preconditions": ["mission_lock", "water_guard_armed"],
     "depends": ["r1"], "memory_refs": []},
    {"id": "r3", "verb": "scan_water", "args": {"frames": 3},
     "preconditions": [], "depends": ["r2"], "memory_refs": []},
    {"id": "r4", "verb": "patrol_report", "args": {},
     "preconditions": [], "depends": ["r3"], "memory_refs": []},
]})


class StubLLM:
    """第一问返回坏计划 (order 错误), 第二问返回修订计划。"""

    def __init__(self):
        self.calls = []

    def available(self):
        return True

    def chat(self, messages, tools=None):
        self.calls.append(messages[-1]["content"][:80])
        # 第一条用户消息含 "draft_prompt" 特征: 任务计划编译
        if any("任务计划编译器" in m["content"] for m in messages):
            return _Resp(BAD_DRAFT)
        if any("修订剩余任务计划" in m["content"] for m in messages):
            return _Resp(REVISED_DRAFT)
        return _Resp("重规划后任务完成。")


class _Resp:
    def __init__(self, content):
        self.content = content
        self.reasoning = ""
        self.tool_calls = []


def test_replan_after_step_failure(offline_config, registry, gate, skills,
                                   tmp_path):
    llm = StubLLM()
    platform = MockAdapter()
    memory = MemoryStore(tmp_path / "mem.jsonl")
    log = SessionLog(tmp_path / "trace.jsonl")
    session = BrainSession(offline_config, platform, registry, gate, skills,
                           llm, log, guard=WaterGuard(approval_token="t"),
                           detector=DrowningDetector(),
                           frame_source=SyntheticPatrolSource(),
                           memory=memory)
    result = session.run("绕湖巡查，检查落水人员")
    entries = log.entries()
    events = [e for e in entries if e["kind"] == "event"]
    names = {e.get("event") for e in events}
    assert "plan_step_failed" in names, "坏计划的 follow_route 应失败"
    assert "plan_replanned" in names, "失败后应触发重规划"
    done = next(e for e in events if e.get("event") == "task_plan_done")
    assert done["state"] in ("replanned_done", "done")
    # 修订计划执行了 arm→follow→scan→report (平台收到航线受理)
    assert any(c[0] == "submit_gps_route" for c in platform.calls)
    assert "重规划" in result["answer"]
