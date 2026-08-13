"""follow 指令 NLU 模板契约测试。"""
import sys
from pathlib import Path

import pytest

_WEB = Path(__file__).resolve().parents[1]
if str(_WEB) not in sys.path:
    sys.path.insert(0, str(_WEB))

from nx_product_command import parse_follow_command  # noqa: E402


@pytest.mark.parametrize("text,expected_target", [
    ("跟踪穿黑衣服的人", "穿黑衣服的人"),
    ("跟着那个人", "人"),
    ("跟踪那个穿黑衣服的人", "穿黑衣服的人"),
    ("跟前面那个人", "人"),
    ("跟踪红色衣服的人", "红色衣服的人"),
])
def test_parse_follow_matches(text, expected_target):
    r = parse_follow_command(text)
    assert r is not None
    assert r["tasks"][0]["type"] == "follow"
    assert r["tasks"][0]["params"]["target"] == expected_target


@pytest.mark.parametrize("text", [
    "前进两米",
    "去大门",
    "搜索客厅",
    "跟踪",          # 无目标
    "跟",            # 无目标
    "",
])
def test_parse_follow_rejects(text):
    r = parse_follow_command(text)
    assert r is None or r["tasks"][0]["type"] != "follow"
