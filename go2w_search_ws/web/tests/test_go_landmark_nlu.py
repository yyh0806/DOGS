"""go_landmark NLU 模板契约测试。"""
import sys
from pathlib import Path

import pytest

_WEB = Path(__file__).resolve().parents[1]
_TOOLS = _WEB.parent / "tools"
for _p in (_WEB, _TOOLS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from nx_product_command import parse_go_landmark  # noqa: E402


@pytest.mark.parametrize("text,expected", [
    ("去大门", "大门"),
    ("去门口", "门口"),
    ("到大门去", "大门"),
    ("前往厨房", "厨房"),
    ("去门口那里", "门口"),
    ("去大门口", "大门口"),
])
def test_parse_go_landmark_matches(text, expected):
    r = parse_go_landmark(text)
    assert r is not None
    assert r["tasks"][0]["type"] == "go_landmark"
    assert r["tasks"][0]["params"]["landmark"] == expected


@pytest.mark.parametrize("text", [
    "前进两米",
    "后退一米",
    "搜索当前房间并标注人",
    "搜索客厅",
    "",
    "去",          # 无地标名
    "去那里",      # 只剩指代词
])
def test_parse_go_landmark_rejects(text):
    r = parse_go_landmark(text)
    assert r is None or r["tasks"][0]["type"] != "go_landmark"


@pytest.mark.parametrize("text", ["到达", "到", "跟踪一下吧"])
def test_review_regressions_rejected(text):
    r = parse_go_landmark(text)
    assert r is None or r["tasks"][0]["type"] != "go_landmark"


@pytest.mark.parametrize("text", ["去一下"])
def test_review_go_residue_rejected(text):
    r = parse_go_landmark(text)
    assert r is None
