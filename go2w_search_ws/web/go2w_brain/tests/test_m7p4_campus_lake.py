"""test_m7p4_campus_lake.py — 园区湖语义链测试 (封闭: 合成卫星图 + 假 VLM)。

语义链 (2026-09-05): A 园区识别 → B VLM 圈园区 mask → C VLM 圈湖 mask
→ D 沿湖环线; VLM 失败走规则后备 (候选+湖形过滤+锚定)。
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
    """按提示词分流: 圈园区 → 园区多边形; 圈湖 → 湖多边形; 其他 → 锚定 JSON。"""

    def __init__(self, idx=0, with_masks=True):
        self._idx = idx
        self._masks = with_masks
        self.calls = []

    def available(self):
        return True

    def vision(self, image, prompt, max_tokens=1024):
        self.calls.append(prompt[:30])
        if self._masks and "工业园区" in prompt:
            return ('{"polygon": [[0.15,0.15],[0.85,0.15],[0.85,0.85],'
                    '[0.15,0.85]], "why": "synthetic campus"}')
        if self._masks and "岸线" in prompt:
            return ('{"polygon": [[0.40,0.40],[0.60,0.40],[0.60,0.60],'
                    '[0.40,0.60]], "why": "synthetic lake"}')
        return ('{"self_near_water": true, "target_idx": %d, '
                '"ambiguity": [], "why": "synthetic"}' % self._idx)


def _ctx(log, task="绕着当前园区湖绕行一圈", vlm=_FakeVlm(0)):
    return {"platform": MockAdapter(gps={"available": True,
                                         "lat": HK["lat"], "lng": HK["lng"]}),
            "log": log, "config": None,
            "mission_lock": "m", "plan_store": {}, "memory": None,
            "vlm": vlm,
            "task": task}


def _fake_stitch(monkeypatch, with_lake=True):
    """合成卫星图: 灰底 + (可选) 中心蓝色湖斑; 瓦片原点对齐中心点。"""
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

    def fake_area(provider, z, x0, y0, nx, ny, use_cache=True):
        arr = np.zeros((ny * 256, nx * 256, 3), dtype=np.uint8)
        arr[:] = (110, 110, 108)
        if with_lake:
            h, w = arr.shape[:2]
            arr[h // 2 - 60:h // 2 + 20, w // 2 - 80:w // 2 + 80] = (30, 60, 130)
        return (Image.fromarray(arr),
                {"provider": provider, "z": z, "x0": x0, "y0": y0,
                 "nx": nx, "ny": ny, "w": nx * 256, "h": ny * 256,
                 "tiles_ok": nx * ny, "tiles_miss": [], "crs": "wgs84"})

    monkeypatch.setattr(tiles_mod, "stitch_area", fake_area)
    return fake


# ---------- A. 园区识别 ----------

def test_match_campus_by_name():
    assert _match_campus("绕着中电海康园区里的湖绕行一圈", {}) == HK
    assert _match_campus("绕湖一圈", {"campus": "海康"}) == HK
    assert _match_campus("绕湖一圈", {}) is None


def test_unknown_campus_fails_honestly():
    log = _Log()
    ctx = _ctx(log, task="绕湖一圈")
    ctx["platform"] = MockAdapter(gps={"available": True,
                                       "lat": 31.30, "lng": 120.30})
    result = TOOLS["plan_campus_lake"].execute({}, ctx)
    assert result["ok"] is False
    assert result["reason"] == "unknown_campus"
    assert "中电海康" in result["hint"]


# ---------- B+C+D. VLM mask 主链路 ----------

def test_vlm_mask_chain(monkeypatch):
    """VLM 直接圈园区/圈湖 → 沿湖 mask 规划 (主链路)。"""
    _fake_stitch(monkeypatch, with_lake=True)
    log = _Log()
    result = TOOLS["plan_campus_lake"].execute({}, _ctx(log))
    assert result["ok"], result.get("reason")
    assert result["campus"] == HK["name"]
    assert result["closed"] is True
    assert result["waypoint_count"] >= 8
    assert result["anchor"]["source"] == "vlm"
    cm = log.events("campus_mask")
    assert cm and cm[0]["source"] == "vlm"
    lm = log.events("lake_mask")
    assert lm and lm[0]["source"] == "vlm"
    plan_ev = log.events("plan_result")[0]
    assert plan_ev["plan_kind"] == "campus_lake"
    # 湖在园区 mask 内 (synthetic: 都在中心附近)
    tgt = result["target"]["centroid"]
    dlat = (tgt[0] - HK["lat"]) * 110540
    dlng = (tgt[1] - HK["lng"]) * 111320 * 0.85
    assert np.hypot(dlat, dlng) <= HK["radius_m"]


def test_vlm_masks_unavailable_rule_fallback(monkeypatch):
    """VLM 不给 mask → 规则后备链路仍可规划。"""
    _fake_stitch(monkeypatch, with_lake=True)
    log = _Log()
    result = TOOLS["plan_campus_lake"].execute(
        {}, _ctx(log, vlm=_FakeVlm(0, with_masks=False)))
    assert result["ok"], result.get("reason")
    cm = log.events("campus_mask")
    assert cm and cm[0]["source"] == "rule"  # 半径圆后备
    assert log.events("campus_water_candidates")


def test_no_vlm_at_all_rule_path(monkeypatch):
    """无 VLM → 园区圆 mask + 候选锚定全规则。"""
    _fake_stitch(monkeypatch, with_lake=True)
    log = _Log()
    result = TOOLS["plan_campus_lake"].execute({}, _ctx(log, vlm=None))
    assert result["ok"], result.get("reason")
    assert result["anchor"]["source"] == "rule"
    assert log.events("campus_mask")[0]["source"] == "rule"


def test_no_water_in_campus(monkeypatch):
    _fake_stitch(monkeypatch, with_lake=False)
    log = _Log()
    result = TOOLS["plan_campus_lake"].execute(
        {}, _ctx(log, vlm=_FakeVlm(0, with_masks=False)))
    assert result["ok"] is False
    assert result["reason"] in ("no_water_in_campus",
                                "no_lake_like_water_in_campus")
    assert log.events("campus_identified")
