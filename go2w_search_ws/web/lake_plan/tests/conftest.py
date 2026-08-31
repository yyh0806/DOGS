"""lake_plan 测试公共设施。

封闭性 (零网络): GO2W_LAKE_CACHE_DIR 指向入库的 fixtures/cache
(瓦片 + Overpass 响应), GO2W_LAKE_OFFLINE=1 未命中即错。

基准点: 无锡太科园园区 (31.488192, 120.369486) —— 园区内有湖,
两种目标 (绕湖/绕园区) 都必须在此点位可规划。
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parents[2]  # .../go2w_search_ws/web
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

FIXTURES = Path(__file__).resolve().parent / "fixtures"
os.environ["GO2W_LAKE_CACHE_DIR"] = str(FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"  # 强制封闭: 缓存未命中 = 测试失败

import pytest  # noqa: E402

from lake_plan import plan_route  # noqa: E402

CAMPUS_CENTER = (31.488192, 120.369486)  # 太科园园区 (M2.1 基准点)


@pytest.fixture(scope="session")
def water_route() -> dict:
    result = plan_route(*CAMPUS_CENTER, kind="water")
    assert result["ok"], result
    return result


@pytest.fixture(scope="session")
def campus_route() -> dict:
    result = plan_route(*CAMPUS_CENTER, kind="campus")
    assert result["ok"], result
    return result


def _load_golden(name: str) -> dict:
    import json
    return json.loads(
        (FIXTURES / f"{name}.json").read_text(encoding="utf-8"))["result"]


@pytest.fixture(scope="session")
def water_golden() -> dict:
    return _load_golden("water_campus_golden")


@pytest.fixture(scope="session")
def campus_golden() -> dict:
    return _load_golden("campus_golden")
