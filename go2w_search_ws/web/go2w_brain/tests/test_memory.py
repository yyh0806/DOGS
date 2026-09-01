"""M7 语义记忆库测试: 写入/检索/衰减/冲突消解/持久化。"""
from __future__ import annotations

import time

import pytest

from go2w_brain.memory import MemoryStore

_PT = {"lat": 31.488192, "lng": 120.369486}


class FakeClock:
    def __init__(self):
        self.now = 1_700_000_000.0

    def __call__(self):
        return self.now

    def advance(self, days):
        self.now += days * 86400.0


def _store(tmp_path, clock=None):
    return MemoryStore(tmp_path / "memory.jsonl", clock=clock or FakeClock())


def test_record_and_query_roundtrip(tmp_path):
    store = _store(tmp_path)
    entry = store.record("vantage", _PT, data={"dist_m": 12.0},
                         confidence=0.8)
    hits = store.query(31.4882, 120.3695, 500.0)
    assert len(hits) == 1
    assert hits[0]["id"] == entry["id"]
    assert hits[0]["kind"] == "vantage"
    assert hits[0]["score"] > 0.7
    assert hits[0]["dist_m"] < 50


def test_kind_filter_and_radius(tmp_path):
    store = _store(tmp_path)
    store.record("vantage", _PT)
    store.record("hazard", {"lat": 31.489, "lng": 120.369})
    assert len(store.query(31.488, 120.369, 300.0, kinds=("hazard",))) == 1
    assert len(store.query(31.488, 120.369, 300.0,
                           kinds=("vantage", "blocked"))) == 1
    # 半径外
    far = {"lat": 31.50, "lng": 120.39}  # ~2.3km
    store.record("blocked", far)
    assert len(store.query(31.488, 120.369, 1000.0)) == 2


def test_newer_observation_wins(tmp_path):
    clock = FakeClock()
    store = _store(tmp_path, clock)
    first = store.record("blocked", _PT, confidence=0.5)
    clock.advance(1.0)
    second = store.record("blocked", {"lat": _PT["lat"] + 0.0002,
                                      "lng": _PT["lng"]},
                          data={"cleared": True}, confidence=0.9)
    # 同源同类邻近 → 覆盖 (同 id)
    assert second["id"] == first["id"]
    hits = store.query(31.488, 120.369, 500.0, kinds=("blocked",))
    assert len(hits) == 1
    assert hits[0]["data"] == {"cleared": True}
    assert hits[0]["confidence"] == 0.9


def test_cross_source_no_overwrite(tmp_path):
    store = _store(tmp_path)
    store.record("hazard", _PT, source="osm", confidence=0.9)
    obs = store.record("hazard", _PT, source="observation", confidence=0.6)
    assert obs["id"] != store.entries()[0]["id"]  # 跨源不覆盖
    assert len(store.entries()) == 2


def test_confidence_decay(tmp_path):
    clock = FakeClock()
    store = _store(tmp_path, clock)
    store.record("fp_zone", _PT, confidence=0.9)  # 半衰期 3 天
    clock.advance(6.0)  # 2 个半衰期 → score ≈ 0.9*0.25 = 0.225
    hits = store.query(31.488, 120.369, 300.0, min_score=0.0)
    assert hits[0]["score"] < 0.3
    assert not store.query(31.488, 120.369, 300.0, min_score=0.25)


def test_persistence_across_instances(tmp_path):
    clock = FakeClock()
    store = _store(tmp_path, clock)
    entry = store.record("vantage", _PT, data={"q": "好视野"})
    reloaded = MemoryStore(tmp_path / "memory.jsonl", clock=clock)
    hits = reloaded.query(31.488, 120.369, 300.0)
    assert len(hits) == 1 and hits[0]["id"] == entry["id"]


def test_invalid_inputs(tmp_path):
    store = _store(tmp_path)
    with pytest.raises(ValueError):
        store.record("teleport", _PT)
    with pytest.raises(ValueError):
        store.record("hazard", {"lat": 95.0, "lng": 0.0})
    with pytest.raises(ValueError):
        store.record("hazard", {"points": [[0, 0]]})
