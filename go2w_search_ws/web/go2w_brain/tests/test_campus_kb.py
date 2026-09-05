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
