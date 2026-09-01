"""M7 端到端: 指令×记忆→任务列表→执行 + 跨会话记忆复利 (规则模式, 零 LLM)。"""
from __future__ import annotations

import os
from pathlib import Path

_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

from go2w_brain.brain_loop import BrainSession  # noqa: E402
from go2w_brain.memory import MemoryStore  # noqa: E402
from go2w_brain.session_log import SessionLog  # noqa: E402
from test_m2_tools import _armed_guard  # noqa: E402


_SESSION_COUNTER = [0]


def _build(offline_config, registry, gate, skills, tmp_path, memory_path):
    from go2w_brain.llm import LLMClient
    from go2w_brain.platform import MockAdapter
    from nx_drowning_detect import DrowningDetector
    from synthetic_frame_source import SyntheticPatrolSource
    platform = MockAdapter()
    llm = LLMClient(offline_config)
    assert not llm.available()
    memory = MemoryStore(memory_path)
    # 每会话独立轨迹文件 (SessionLog 追加式, 共享路径会串会话)
    _SESSION_COUNTER[0] += 1
    log = SessionLog(tmp_path / (memory_path.stem
                                 + f".trace{_SESSION_COUNTER[0]}.jsonl"))
    session = BrainSession(offline_config, platform, registry, gate, skills,
                           llm, log, guard=_armed_guard(),
                           detector=DrowningDetector(),
                           frame_source=SyntheticPatrolSource(),
                           memory=memory)
    return session, log, memory, platform


def test_rule_plan_patrol_full_loop(offline_config, registry, gate, skills,
                                    tmp_path):
    session, log, memory, platform = _build(
        offline_config, registry, gate, skills, tmp_path,
        tmp_path / "mem.jsonl")
    result = session.run("绕湖巡查，检查落水人员")
    answer = result["answer"]
    assert "任务计划执行汇报" in answer
    assert "rule 计划" in answer
    entries = log.entries()
    assert any(e["kind"] == "event" and e.get("event") == "task_plan"
               for e in entries)
    plan_event = next(e for e in entries
                      if e["kind"] == "event" and e["event"] == "task_plan")
    assert plan_event["source"] == "rule"
    # 平台收到航线受理 (干跑前仍会走 submit? mock 无 dry flag → 提交)
    assert any(c[0] == "submit_gps_route" for c in platform.calls)


def test_status_plan_offline(offline_config, registry, gate, skills, tmp_path):
    session, log, memory, platform = _build(
        offline_config, registry, gate, skills, tmp_path,
        tmp_path / "mem.jsonl")
    result = session.run("报告当前状态")
    assert "31.488192" in result["answer"]
    assert "72.5" in result["answer"]
    assert "位姿" in result["answer"]


def test_second_mission_reads_first_missions_memory(
        offline_config, registry, gate, skills, tmp_path):
    """核心验收: 第一次任务的记忆, 第二次任务启动时被检索注入。"""
    memory_path = tmp_path / "mem.jsonl"
    # 第一次任务: 绕湖 (scan_water 检出 confirmed → 自动写 detection 记忆)
    session1, log1, memory1, _ = _build(offline_config, registry, gate,
                                        skills, tmp_path, memory_path)
    session1.run("绕湖巡查，检查落水人员")
    events1 = [e for e in log1.entries()
               if e["kind"] == "event" and e["event"] == "memory_recorded"]
    assert events1, "第一次任务应有记忆写入 (detection)"
    # 第二次任务 (新会话, 同一记忆库)
    session2, log2, memory2, _ = _build(offline_config, registry, gate,
                                        skills, tmp_path, memory_path)
    session2.run("绕园区巡查")
    events2 = [e for e in log2.entries()
               if e["kind"] == "event" and e["event"] == "memory_retrieved"]
    assert events2, "第二次任务应检索到记忆"
    assert events2[0]["count"] >= 1, "检索数量应 ≥1"
    # 计划仍是合法产物
    plan_events = [e for e in log2.entries()
                   if e["kind"] == "event" and e["event"] == "task_plan"]
    assert plan_events and plan_events[0]["source"] == "rule"


def test_unknown_task_falls_back_to_free_loop(
        offline_config, registry, gate, skills, tmp_path):
    session, log, memory, platform = _build(
        offline_config, registry, gate, skills, tmp_path,
        tmp_path / "mem.jsonl")
    result = session.run("把门打开")
    kinds = [e["kind"] for e in log.entries()]
    assert "task_plan" not in kinds          # 无计划 (非计划式任务)
    assert "无法理解" in result["answer"] or "只理解" in result["answer"]
