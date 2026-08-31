"""M4 工具集测试: scan_water / get_detection_events (合成帧源)。"""
from __future__ import annotations

from test_m2_tools import _NullLog  # noqa: E402

from go2w_brain.platform import MockAdapter  # noqa: E402
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402
from nx_drowning_detect import DrowningDetector  # noqa: E402

TOOLS = {t.name: t for t in BUILTIN_TOOLS}

ROBOT = {"lat": 31.488192, "lng": 120.369486, "yaw_deg": 0.0}


def _drowning_frames(n=3, cx=640, head_h=18):
    """含同一落水者的连续帧 (合成, 同 test_drowning_detect 画法)。"""
    from PIL import Image, ImageDraw
    frames = []
    for k in range(n):
        img = Image.new("RGB", (1280, 720))
        d = ImageDraw.Draw(img)
        horizon = int(720 * 0.45)
        d.rectangle([0, 0, 1280, horizon], fill=(150, 200, 235))
        for i in range(horizon, 720, 4):
            t = (i - horizon) / max(1, 720 - horizon)
            base = int(40 + 30 * t)
            d.rectangle([0, i, 1280, i + 4],
                        fill=(base // 2, base, base + 45))
        x = cx + (k - 1) * 3
        cy = horizon + int((720 - horizon) * 0.45)
        d.ellipse([x - head_h // 2, cy - head_h, x + head_h // 2, cy],
                  fill=(235, 120, 30))
        d.line([x - head_h, cy - head_h // 2, x - head_h // 2,
                cy - head_h // 3], fill=(220, 140, 80),
               width=max(3, head_h // 6))
        d.line([x + head_h, cy - head_h // 2, x + head_h // 2,
                cy - head_h // 3], fill=(220, 140, 80),
               width=max(3, head_h // 6))
        frames.append(img)
    return frames


def _ctx(platform=None, detector=None, source=None, mission_lock="m4"):
    frames = _drowning_frames()
    calls = {"i": 0}

    def frame_source():
        frame = frames[min(calls["i"], len(frames) - 1)]
        calls["i"] += 1
        return frame, dict(ROBOT)

    return {"platform": platform or MockAdapter(), "log": _NullLog(),
            "config": None, "mission_lock": mission_lock,
            "detector": detector or DrowningDetector(),
            "frame_source": source or frame_source, "guard": None,
            "plan_store": {}}


def test_scan_water_detects_and_confirms():
    ctx = _ctx()
    result = TOOLS["scan_water"].execute({"frames": 3}, ctx)
    assert result["ok"]
    assert result["frames_scanned"] == 3
    assert result["confirmed_count"] == 1
    confirmed = [e for e in result["new_events"]
                 if e["tier"] == "confirmed"]
    assert confirmed, "三帧同一目标应升级 confirmed"
    event = confirmed[0]
    assert 31.487 < event["lat"] < 31.489      # 园区基准附近
    assert 120.368 < event["lng"] < 120.371


def test_scan_water_tier_progression():
    """首帧 suspect, 三帧后 confirmed —— 两级告警各出现一次。"""
    ctx = _ctx()
    tiers = []
    for _ in range(3):
        result = TOOLS["scan_water"].execute({"frames": 1}, ctx)
        tiers += [e["tier"] for e in result["new_events"]]
    assert "suspect" in tiers and "confirmed" in tiers


def test_scan_water_without_detector_fails_closed():
    ctx = _ctx()
    ctx["detector"] = None
    result = TOOLS["scan_water"].execute({}, ctx)
    assert result["ok"] is False
    assert result["reason"] == "detector_unavailable"


def test_get_detection_events_filter():
    ctx = _ctx()
    TOOLS["scan_water"].execute({"frames": 3}, ctx)
    all_events = TOOLS["get_detection_events"].execute({}, ctx)
    assert all_events["ok"] and all_events["count"] >= 2
    confirmed = TOOLS["get_detection_events"].execute(
        {"tier": "confirmed"}, ctx)
    assert confirmed["count"] == 1
    assert all(e["tier"] == "confirmed" for e in confirmed["events"])
