"""M7.2 测试: 观测回写 / TTS 接入 / nx_ai 注入适配 / 告警推送。"""
from __future__ import annotations

import os
from pathlib import Path

_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

import pytest  # noqa: E402

from go2w_brain.memory import MemoryStore  # noqa: E402
from go2w_brain.platform import MockAdapter  # noqa: E402
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402
from go2w_brain.tts import ConsoleTtsBackend, build_backend  # noqa: E402
from nx_detector_bridge import (make_vlm_verifier,  # noqa: E402
                                make_yolo_person_detector)
from nx_drowning_detect import DrowningDetector  # noqa: E402
from nx_water_guard import WaterGuard  # noqa: E402

TOOLS = {t.name: t for t in BUILTIN_TOOLS}


class _NullLog:
    def append(self, kind, **fields):
        return None


def _ctx(platform=None, tts=None, memory=None, mission_lock="m72"):
    return {"platform": platform or MockAdapter(), "log": _NullLog(),
            "config": None, "mission_lock": mission_lock,
            "guard": None, "memory": memory, "tts": tts,
            "plan_store": {}, "approval_token": "tok"}


# ---------- 观测回写钩子 (事件 → 记忆) ----------------------------------------

def test_observation_event_writes_memory(offline_config, registry, gate,
                                         skills, tmp_path):
    from go2w_brain.brain_loop import BrainSession
    from go2w_brain.llm import LLMClient
    from go2w_brain.session_log import SessionLog
    memory = MemoryStore(tmp_path / "mem.jsonl")
    log = SessionLog(tmp_path / "t.jsonl")
    session = BrainSession(offline_config, MockAdapter(), registry, gate,
                           skills, LLMClient(offline_config), log,
                           guard=WaterGuard(approval_token="t"),
                           memory=memory)
    session.submit_event({
        "kind": "observation", "mem_kind": "blocked",
        "geo": {"lat": 31.4890, "lng": 120.3693},
        "confidence": 0.8,
        "data": {"reason": "construction"}})
    session.run("报告当前状态")
    hits = memory.query(31.4890, 120.3693, 100.0,
                        kinds=("blocked",), min_score=0.0)
    assert len(hits) == 1
    assert hits[0]["data"]["reason"] == "construction"
    events = [e for e in log.entries()
              if e["kind"] == "event"
              and e.get("event") == "observation_recorded"]
    assert events, "观测回写应留痕"


def test_bad_observation_ignored(offline_config, registry, gate, skills,
                                 tmp_path):
    from go2w_brain.brain_loop import BrainSession
    from go2w_brain.llm import LLMClient
    from go2w_brain.session_log import SessionLog
    memory = MemoryStore(tmp_path / "mem.jsonl")
    session = BrainSession(offline_config, MockAdapter(), registry, gate,
                           skills, LLMClient(offline_config),
                           SessionLog(tmp_path / "t.jsonl"),
                           guard=WaterGuard(approval_token="t"),
                           memory=memory)
    session.submit_event({"kind": "observation", "mem_kind": "teleport",
                          "geo": {"lat": 0}})
    session.run("报告当前状态")
    assert memory.summary()["total"] == 0  # 非法种类/geo 被忽略


# ---------- TTS ---------------------------------------------------------------

def test_console_tts_backend(capsys):
    backend = ConsoleTtsBackend()
    result = backend.speak("发现落水人员")
    assert result["ok"] and result["spoken"] is True
    assert "发现落水人员" in capsys.readouterr().out


def test_speak_tool_with_tts_backend():
    ctx = _ctx(tts=ConsoleTtsBackend())
    result = TOOLS["speak"].execute({"text": "任务完成"}, ctx)
    assert result["ok"] and result["spoken"] is True


def test_speak_tool_without_backend_honest():
    result = TOOLS["speak"].execute({"text": "hi"}, _ctx())
    assert result["spoken"] is False
    assert result["note"] == "no_tts_backend"


def test_build_backend_default():
    assert build_backend("").name == "console"
    assert build_backend("edge").name == "edge"
    assert build_backend("unknown").name == "console"


# ---------- nx_ai 注入适配 ----------------------------------------------------

def test_yolo_adapter_normalizes_shapes():
    from PIL import Image

    class FakeYolo:
        def _run_detector(self, frame):
            return [
                {"bbox": [10, 20, 60, 90], "conf": 0.8, "cls": "person"},
                {"bbox": [0, 0, 5, 5], "conf": 0.9, "cls": "car"},
                {"xyxy": [30, 40, 80, 90], "confidence": 0.7},
                [5, 10, 25, 30, 0.6, 0],  # 元组形态 (class id 0=person)
                [5, 10, 25, 30, 0.6, 2],  # class id 2 → 过滤
                {"bbox": [5, 10, 3, 30]},  # 非法框 (x1<x0) → 容错换序
            ]

    detector = make_yolo_person_detector(FakeYolo())
    frame = Image.new("RGB", (100, 100))
    dets = detector(frame)
    assert len(dets) == 4  # person/dict无类名/person(0)/容错框
    assert dets[0]["bbox"] == (10, 20, 60, 90)
    assert dets[0]["score"] == 0.8
    assert dets[2]["bbox"] == (5, 10, 25, 30)
    assert dets[3]["bbox"] == (3, 10, 5, 30)  # 换序后


def test_yolo_adapter_raises_without_detector():
    with pytest.raises(ValueError):
        make_yolo_person_detector(object())


def test_vlm_adapter_parses_verdict():
    class FakeVlm:
        def chat(self, messages, max_new_tokens=200):
            return '{"in_water": true, "struggling": true, "confidence": 0.9}'

    verifier = make_vlm_verifier(FakeVlm())
    verdict = verifier(None, "prompt")
    assert verdict["in_water"] is True and verdict["struggling"] is True
    assert verdict["engine"] == "vlm"


def test_vlm_adapter_conservative_on_garbage():
    class FakeVlm:
        def chat(self, messages, max_new_tokens=200):
            return "图片里没有看到人"

    verifier = make_vlm_verifier(FakeVlm())
    verdict = verifier(None, "prompt")
    assert verdict["in_water"] is False
    assert verdict["engine"] == "keyword"


def test_bridge_feeds_detector_pipeline():
    """适配后的 person_detector 能驱动 DrowningDetector (契约闭合)。"""
    from PIL import Image, ImageDraw

    class FakeYolo:
        def _run_detector(self, frame):
            return [{"bbox": [620, 340, 660, 385], "conf": 0.9,
                     "cls": "person"}]

    detector = DrowningDetector(person_detector=make_yolo_person_detector(
        FakeYolo()))
    # 合成水景帧 (水塘有人)
    img = Image.new("RGB", (1280, 720))
    d = ImageDraw.Draw(img)
    horizon = 320
    d.rectangle([0, 0, 1280, horizon], fill=(150, 200, 235))
    for i in range(horizon, 720, 4):
        base = 40 + 30 * (i - horizon) // 400
        d.rectangle([0, i, 1280, i + 4], fill=(base // 2, base, base + 45))
    d.ellipse([620, 300, 660, 385], fill=(235, 120, 30))
    robot = {"lat": 31.488192, "lng": 120.369486, "yaw_deg": 0.0}
    events = detector.process_frame(img, robot)
    assert any(e["tier"] in ("suspect", "confirmed") for e in events)


# ---------- 告警推送 ----------------------------------------------------------

def test_push_alert_tool_mock():
    platform = MockAdapter()
    ctx = _ctx(platform=platform)
    result = TOOLS["push_alert"].execute(
        {"lat": 31.49, "lng": 120.37, "tier": "confirmed",
         "note": "挥手呼救"}, ctx)
    assert result["ok"] and result["broadcast"] is True
    assert platform.calls[-1][0] == "post_alert"


def test_push_alert_rejects_bad_position():
    ctx = _ctx()
    result = TOOLS["push_alert"].execute(
        {"lat": "x", "lng": 120.37}, ctx)
    assert result["ok"] is False
    assert result["reason"] == "invalid_alert_position"
