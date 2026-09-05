"""test_m7p4_campus_lake.py — 园区湖语义链测试 (封闭: 合成卫星图 + 假 VLM)。

语义链 (2026-09-05 用户要求): "园区湖"任务必须先识别园区, 再在园区内
找湖 —— 而不是找最近水体。这里验证 A(园区识别)/B(园区内找湖+锚定)/
C(绕行规划) 三段与事件留痕。
"""
from __future__ import annotations

import os
from pathlib import Path

os.environ["GO2W_LAKE_OFFLINE"] = "1"  # 防意外联网

import numpy as np
import pytest
from PIL import Image

from go2w_brain.platform import MockAdapter
from go2w_brain.tools import BUILTIN_TOOLS
from go2w_brain.tools.plan_campus_lake import CAMPUSES, _match_campus

TOOLS = {t.name: t for t in BUILTIN_TOOLS}
HK = CAMPUSES[0]


class _Log:
    def __init__(self):
        self.rows = []

    def append(self, kind, **fields):
        self.rows.append((kind, fields))

    def events(self, name):
        return [f for _, f in self.rows if f.get("event") == name]


class _FakeVlm:
    def __init__(self, idx=0):
        self._idx = idx

    def available(self):
        return True

    def vision(self, image, prompt, max_tokens=1024):
        return ('{"self_near_water": true, "target_idx": %d, '
                '"ambiguity": [], "why": "synthetic"}' % self._idx)


def _ctx(log, task="绕着当前园区湖绕行一圈", vlm=_FakeVlm(0)):
    # 模拟本体在海康园区内 (园区识别按本体位置兜底时命中)
    return {"platform": MockAdapter(gps={"available": True,
                                         "lat": HK["lat"], "lng": HK["lng"]}),
            "log": log, "config": None,
            "mission_lock": "m", "plan_store": {}, "memory": None,
            "vlm": vlm,
            "task": task}


def _fake_stitch(monkeypatch, with_lake=True):
    """合成卫星图: 灰底 + (可选) 园区中心附近蓝色湖斑; 瓦片原点对齐园区。"""
    from lake_plan import tiles as tiles_mod
    from lake_plan.geo import latlon_to_tile

    def fake(provider, lat, lng, z, nx, ny, use_cache=True):
        tx, ty = latlon_to_tile(lat, lng, z)
        x0, y0 = tx - nx // 2, ty - ny // 2
        arr = np.zeros((ny * 256, nx * 256, 3), dtype=np.uint8)
        arr[:] = (110, 110, 108)
        if with_lake:
            h, w = arr.shape[:2]
            arr[h // 2 - 60:h // 2 + 20, w // 2 - 80:w // 2 + 80] = (30, 60, 130)
        return (Image.fromarray(arr),
                {"provider": provider, "z": z, "x0": x0, "y0": y0,
                 "nx": nx, "ny": ny, "w": nx * 256, "h": ny * 256,
                 "tiles_ok": nx * ny, "tiles_miss": [], "crs": "wgs84"})

    monkeypatch.setattr(tiles_mod, "stitch_centered", fake)
    return fake


# ---------- A. 园区识别 ----------

def test_match_campus_by_name():
    assert _match_campus("绕着中电海康园区里的湖绕行一圈", {}) == HK
    assert _match_campus("绕湖一圈", {"campus": "海康"}) == HK
    assert _match_campus("绕湖一圈", {}) is None


def test_unknown_campus_fails_honestly():
    log = _Log()
    # 本体远离一切已知园区 → 名字/位置都匹配不上
    ctx = _ctx(log, task="绕湖一圈")
    ctx["platform"] = MockAdapter(gps={"available": True,
                                       "lat": 31.30, "lng": 120.30})
    result = TOOLS["plan_campus_lake"].execute({}, ctx)
    assert result["ok"] is False
    assert result["reason"] == "unknown_campus"
    assert "中电海康" in result["hint"]


# ---------- B+C. 园区内找湖 + 绕行规划 ----------

def test_campus_lake_full_chain(monkeypatch):
    _fake_stitch(monkeypatch, with_lake=True)
    log = _Log()
    result = TOOLS["plan_campus_lake"].execute({}, _ctx(log))
    assert result["ok"], result.get("reason")
    assert result["campus"] == HK["name"]
    assert result["closed"] is True
    assert result["waypoint_count"] >= 8
    assert result["water_cross_ratio"] == 0.0
    assert result["anchor"]["source"] == "vlm"
    # 事件留痕: 园区识别 → 候选 → 锚定 → 规划
    assert log.events("campus_identified")[0]["campus"] == HK["name"]
    assert log.events("campus_water_candidates")[0]["count"] >= 1
    anchor_ev = log.events("semantic_anchor")[0]
    assert anchor_ev["source"] == "vlm"
    assert anchor_ev["resolved_to_plan"] is True
    plan_ev = log.events("plan_result")[0]
    assert plan_ev["plan_kind"] == "campus_lake"
    assert plan_ev["campus"] == HK["name"]
    # 目标水体在园区半径内 (湖在园区里, 不是园区外)
    tgt = result["target"]["centroid"]
    dlat = (tgt[0] - HK["lat"]) * 110540
    dlng = (tgt[1] - HK["lng"]) * 111320 * 0.85
    assert np.hypot(dlat, dlng) <= HK["radius_m"]


def test_campus_lake_no_water_in_campus(monkeypatch):
    _fake_stitch(monkeypatch, with_lake=False)
    log = _Log()
    result = TOOLS["plan_campus_lake"].execute({}, _ctx(log))
    assert result["ok"] is False
    assert result["reason"] == "no_water_in_campus"
    assert log.events("campus_identified")


def test_campus_lake_vlm_unavailable_rule_fallback(monkeypatch):
    _fake_stitch(monkeypatch, with_lake=True)
    log = _Log()
    result = TOOLS["plan_campus_lake"].execute({}, _ctx(log, vlm=None))
    assert result["ok"], result.get("reason")
    assert result["anchor"]["source"] == "rule"
    assert log.events("semantic_anchor")[0]["source"] == "rule"
