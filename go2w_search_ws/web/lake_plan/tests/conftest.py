"""lake_plan 测试公共设施。

关键: golden 回归必须封闭 (零网络) —— GO2W_LAKE_CACHE_DIR 指向入库的
fixtures/tiles, 所有瓦片走缓存命中。config.cache_dir() 每次调用读环境,
因此只需在调用 plan_route 前设好变量, 无需控制导入顺序。
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parents[2]  # .../go2w_search_ws/web
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

FIXTURES = Path(__file__).resolve().parent / "fixtures"
os.environ["GO2W_LAKE_CACHE_DIR"] = str(FIXTURES / "tiles")
os.environ["GO2W_LAKE_OFFLINE"] = "1"  # 强制封闭: 缓存未命中 = 测试失败

import pytest  # noqa: E402

from lake_plan import plan_route  # noqa: E402

LIHU_CENTER = (31.5163, 120.2673)


@pytest.fixture(scope="session")
def lihu_route() -> dict:
    result = plan_route(*LIHU_CENTER)
    assert result["ok"], result
    return result


@pytest.fixture(scope="session")
def golden() -> dict:
    import json
    path = FIXTURES / "lake_lihu_golden.json"
    return json.loads(path.read_text(encoding="utf-8"))["result"]
