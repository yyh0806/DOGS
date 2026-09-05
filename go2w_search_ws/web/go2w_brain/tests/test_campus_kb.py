"""test_campus_kb.py — 园区标定知识库测试 (2026-09-05)。"""
from __future__ import annotations

import json

from go2w_brain import campus_kb


def test_upsert_get_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setenv("GO2W_CAMPUS_KB", str(tmp_path / "kb.json"))
    campus_kb.upsert("测试园区", "boundary",
                     [[31.48, 120.36], [31.49, 120.36],
                      [31.49, 120.37], [31.48, 120.37]])
    campus_kb.upsert("测试园区", "lake",
                     [[31.485, 120.365], [31.486, 120.366],
                      [31.485, 120.367]])
    entry = campus_kb.get("测试园区")
    assert entry is not None
    assert len(entry["boundary"]) == 4
    assert len(entry["lake"]) == 3
    assert entry["center"][0] == 31.485  # boundary 中心
    # 文件落盘
    data = json.loads((tmp_path / "kb.json").read_text(encoding="utf-8"))
    assert "测试园区" in data
    assert campus_kb.get("不存在园区") is None


def test_validate_rejects_bad_polygon():
    assert campus_kb.validate([[31.0, 120.0], [31.1, 120.1],
                               [31.2, 120.2]]) is True
    assert campus_kb.validate([[31.0, 120.0], [31.1, 120.1]]) is False
    assert campus_kb.validate([[95.0, 120.0], [31.0, 120.0],
                               [31.1, 120.1]]) is False  # 纬度越界
    assert campus_kb.validate("not a list") is False


def test_record_to_memory_persists(tmp_path):
    """标定 → 地图式记忆: 落盘 + 重启重载可查 (永久记录)。"""
    from go2w_brain.memory import MemoryStore
    path = tmp_path / "memory.jsonl"
    store = MemoryStore(path)
    mid = campus_kb.record_to_memory(
        store, "测试园区", "campus_boundary",
        [[31.48, 120.36], [31.49, 120.36], [31.49, 120.37],
         [31.48, 120.37]])
    assert mid
    # 多块湖: 一次标定一条记忆, data.parts 含全部环
    campus_kb.record_to_memory(
        store, "测试园区", "lake_shore",
        [[[31.485, 120.365], [31.486, 120.366], [31.485, 120.367]],
         [[31.488, 120.368], [31.489, 120.369], [31.488, 120.370]]])
    # 重启重载 (新 store 从文件重建索引)
    store2 = MemoryStore(path)
    entries = store2.query(31.488, 120.369, 2000.0,
                           kinds=("geometry",), min_score=0.01)
    by_kind = {}
    for e in entries:
        d = e["data"]
        by_kind[d.get("calibrated_kind")] = e
    assert "campus_boundary" in by_kind
    lake = by_kind["lake_shore"]
    assert len(lake["data"]["parts"]) == 2  # 两块湖面
    for e in entries:
        if e["data"].get("calibrated_kind"):
            assert e["source"] == "calibration"


def test_upsert_multipoly_and_lake_rings(tmp_path, monkeypatch):
    monkeypatch.setenv("GO2W_CAMPUS_KB", str(tmp_path / "kb.json"))
    campus_kb.upsert("园区", "lake",
                     [[[31.0, 120.0], [31.1, 120.0], [31.1, 120.1]],
                      [[31.2, 120.2], [31.3, 120.2], [31.2, 120.3]]],
                     multipoly=True)
    entry = campus_kb.get("园区")
    rings = campus_kb.lake_rings(entry)
    assert len(rings) == 2
    assert "lake" not in entry  # 多块覆盖旧单环
    # 单环兼容
    campus_kb.upsert("园区2", "lake",
                     [[31.0, 120.0], [31.1, 120.0], [31.0, 120.1]])
    assert len(campus_kb.lake_rings(campus_kb.get("园区2"))) == 1
