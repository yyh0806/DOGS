"""nx_landmarks 契约测试 — 地标注册表的核心行为。"""
import os
import sys
import tempfile
from pathlib import Path

import pytest

_WEB = Path(__file__).resolve().parents[1]
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))

from nx_landmarks import (  # noqa: E402
    Landmark,
    LandmarkMap,
    LandmarkValidationError,
)


def test_load_missing_file_returns_empty():
    with tempfile.TemporaryDirectory() as td:
        m = LandmarkMap.load(os.path.join(td, "none.yaml"))
        assert m.landmarks == []


def test_roundtrip_save_load():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "landmarks.yaml")
        m = LandmarkMap([Landmark("大门", 2.5, 1.8, yaw=0.5,
                                  aliases=["门口", "gate"],
                                  gps={"lat": 30.1, "lon": 120.1})])
        m.save(path)
        m2 = LandmarkMap.load(path)
        assert len(m2.landmarks) == 1
        lm = m2.landmarks[0]
        assert lm.name == "大门"
        assert lm.x == 2.5 and lm.y == 1.8 and lm.yaw == 0.5
        assert lm.aliases == ["门口", "gate"]
        assert lm.gps == {"lat": 30.1, "lon": 120.1}


def test_find_priority_name_exact_alias_then_substring():
    m = LandmarkMap([
        Landmark("大门", 1, 0, aliases=["门口"]),
        Landmark("大门后门", 2, 0),
    ])
    assert m.find("大门").x == 1.0           # name 完全相等优先
    assert m.find("门口").x == 1.0           # alias 完全相等
    assert m.find("大").x == 1.0             # 子串
    assert m.find("不存在") is None


def test_upsert_same_name_replaces():
    m = LandmarkMap([Landmark("大门", 1, 0)])
    m.upsert(Landmark("大门", 3, 4))
    assert len(m.landmarks) == 1
    assert m.landmarks[0].x == 3.0


def test_duplicate_name_load_rejected():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "dup.yaml")
        content = (
            "version: '1.0'\n"
            "landmarks:\n"
            "- name: 大门\n  x: 1\n  y: 0\n"
            "- name: 大门\n  x: 2\n  y: 0\n"
        )
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        with pytest.raises(LandmarkValidationError):
            LandmarkMap.load(path)


def test_invalid_entries_rejected():
    with pytest.raises(LandmarkValidationError):
        Landmark.from_dict({"name": "X"})                 # 缺 x/y
    with pytest.raises(LandmarkValidationError):
        Landmark.from_dict({"name": "X", "x": "a", "y": 0})  # x 非数值
    with pytest.raises(LandmarkValidationError):
        Landmark.from_dict({"name": "X", "x": 1, "y": 0,
                            "gps": {"lat": "a"}})          # gps 缺 lon


def test_review_str_alias_normalized():
    lm = Landmark("大门", 1, 0, aliases="门口")
    assert lm.aliases == ["门口"]


def test_review_atomic_save_roundtrip():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "lm.yaml")
        m = LandmarkMap([Landmark("大门", 2.5, 1.8)])
        m.save(path)
        m2 = LandmarkMap.load(path)
        assert m2.landmarks[0].name == "大门"
        leftovers = [f for f in os.listdir(td) if f.endswith(".tmp")]
        assert leftovers == []
