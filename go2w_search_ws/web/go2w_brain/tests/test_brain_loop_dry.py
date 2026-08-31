"""端到端干跑 (离线): M1 验收口径本身。

不依赖网络/ROS: mock 平台 + 无 LLM key 全链路可跑;
规则模式只懂状态报告, 不懂就诚实拒绝且绝不执行工具。
"""
from __future__ import annotations

from go2w_brain.brain_loop import BrainSession
from go2w_brain.session_log import SessionLog


def _build(offline_config, mock_platform, registry, gate, skills, tmp_path):
    from go2w_brain.llm import LLMClient
    llm = LLMClient(offline_config)
    assert not llm.available()  # 强制离线
    log = SessionLog(tmp_path / "s.jsonl")
    session = BrainSession(offline_config, mock_platform, registry, gate,
                           skills, llm, log)
    return session, log


def test_status_report_full_loop(offline_config, mock_platform, registry,
                                 gate, skills, tmp_path):
    session, log = _build(offline_config, mock_platform, registry, gate,
                          skills, tmp_path)
    result = session.run("报告当前状态")
    answer = result["answer"]
    assert "31.5163" in answer          # GPS 经纬度
    assert "72.5" in answer             # 电量
    assert "位姿" in answer
    stats = log.stats()
    assert stats["tool_call"] == 3      # get_gps/get_battery/get_pose
    assert stats["tool_result"] == 3
    assert stats["reply"] == 1
    assert result["trace"].endswith(".jsonl")


def test_unknown_task_refuses_honestly_and_executes_nothing(
        offline_config, mock_platform, registry, gate, skills, tmp_path):
    session, log = _build(offline_config, mock_platform, registry, gate,
                          skills, tmp_path)
    result = session.run("把门打开")
    assert "无法理解" in result["answer"] or "只理解" in result["answer"]
    assert "tool_call" not in log.stats()  # 一个工具都没执行


def test_trace_replayable(offline_config, mock_platform, registry, gate,
                          skills, tmp_path):
    session, log = _build(offline_config, mock_platform, registry, gate,
                          skills, tmp_path)
    session.run("报告状态")
    entries = log.entries()
    kinds = [e["kind"] for e in entries]
    assert kinds[0] == "session_start" and kinds[-1] == "session_end"
    messages = log.load_messages()
    assert messages[0]["role"] == "user"          # task
    assert messages[-1]["role"] == "assistant"    # reply
