"""M7.3 语义锚定接入测试: plan_lake_loop 锚定事件 + VLM 纠正重规划 (封闭零网络)。

两个锁 (用户原话): 语义锚定必须留痕 (semantic_anchor 事件), VLM 认定
"湖"是另一候选时必须以该质心重规划 (prefer), 且歧义如实上报。
"""
from __future__ import annotations

import os
from pathlib import Path

import pytest

# lake_plan 夹具 (在导入 lake_plan 前设置): 与 test_m2_tools.py 同一缓存根
_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

from go2w_brain.platform import MockAdapter  # noqa: E402
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402
from nx_water_guard import WaterGuard  # noqa: E402

TOOLS = {t.name: t for t in BUILTIN_TOOLS}


class _Log:
    """与 SessionLog.append 同签名 (kind 关键字冲突在此即暴露)。"""

    def __init__(self):
        self.rows = []

    def append(self, kind, **fields):
        self.rows.append((kind, fields))

    def events(self, name):
        return [f for _, f in self.rows if f.get("event") == name]


def _ctx(log, vlm=None):
    guard = WaterGuard(approval_token="test-token")
    guard.arm([(31.4000, 120.5000), (31.4000, 120.5020),
               (31.4020, 120.5020), (31.4020, 120.5000)])
    return {"platform": MockAdapter(), "log": log,
            "config": None, "mission_lock": "m-test", "guard": guard,
            "plan_store": {}, "memory": None,
            "vlm": vlm, "task": "绕湖一周, 并巡查有没有落水人员"}


def test_plan_lake_loop_emits_rule_anchor_offline():
    """离线无卫星瓦片 → 规则锚定 + 事件留痕 (本体=GNSS, 湖=最近水体)。"""
    log = _Log()
    result = TOOLS["plan_lake_loop"].execute({}, _ctx(log))
    assert result["ok"], result.get("reason")
    anchor = result["anchor"]
    assert anchor["source"] == "rule"
    assert anchor["self"]["lat"] == 31.488192  # MockAdapter 默认 GNSS
    assert anchor["self"]["lng"] == 120.369486
    events = log.events("semantic_anchor")
    assert events, "缺少 semantic_anchor 事件"
    assert events[0]["source"] == "rule"
    assert events[0]["resolved_to_plan"] is True


def test_vlm_disagrees_triggers_replan_with_prefer(monkeypatch):
    """VLM 认定任务所指是另一候选 → 以其质心重规划一次 (至多一次)。"""
    import lake_plan
    from lake_plan import semantic_anchor as sa

    log = _Log()
    ctx = _ctx(log)
    prefer_seen = []
    real_plan = lake_plan.plan_route

    def fake_plan(lat, lng, offset_m=15.0, prefer=None, **kw):
        prefer_seen.append(prefer)
        return real_plan(lat, lng, offset_m=offset_m, prefer=prefer, **kw)

    monkeypatch.setattr(lake_plan, "plan_route", fake_plan)

    def fake_anchor(vlm, task, robot, candidates, center, **kw):
        return {"source": "vlm",
                "self": {"lat": robot["lat"], "lng": robot["lng"],
                         "confirmed": True},
                "target": {"idx": 1, "centroid": (31.50, 120.38),
                           "name": "南湖", "why": "红圈1更像任务湖"},
                "ambiguity": [0]}

    monkeypatch.setattr(sa, "anchor_semantics", fake_anchor)
    result = TOOLS["plan_lake_loop"].execute({}, ctx)
    assert result["ok"], result.get("reason")
    # 首规划 prefer=None → 锚定 disagree → 以 VLM 质心重规划一次
    assert prefer_seen == [None, (31.50, 120.38)]
    replans = log.events("anchor_replan")
    assert len(replans) == 1
    assert replans[0]["prefer"] == [31.50, 120.38]
    # 第二次锚定仍 disagree → 如实留痕 (绝不静默装作已解决)
    semantic = log.events("semantic_anchor")
    assert semantic[-1]["resolved_to_plan"] is False
    assert result["anchor"]["target"]["idx"] == 1
