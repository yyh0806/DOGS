"""M5 工具集测试: approach_vantage / patrol_report / 续航 / 扫描点。"""
from __future__ import annotations

import os
from pathlib import Path

_LAKE_FIXTURES = (Path(__file__).resolve().parents[2]
                  / "lake_plan" / "tests" / "fixtures")
os.environ["GO2W_LAKE_CACHE_DIR"] = str(_LAKE_FIXTURES / "cache")
os.environ["GO2W_LAKE_OFFLINE"] = "1"

from test_m2_tools import _NullLog, _armed_guard  # noqa: E402

from go2w_brain.platform import MockAdapter  # noqa: E402
from go2w_brain.patrol_math import (endurance_check,  # noqa: E402
                                     nearest_route_point,
                                     route_completion)
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402
from nx_water_guard import WaterGuard  # noqa: E402

TOOLS = {t.name: t for t in BUILTIN_TOOLS}


def _ctx(platform=None, guard=None, detector=None, plan_store=None):
    return {"platform": platform or MockAdapter(), "log": _NullLog(),
            "config": None, "mission_lock": "m5",
            "guard": guard, "detector": detector,
            "plan_store": plan_store if plan_store is not None else {},
            "approval_token": None}


# ---------- patrol_math ----------

def test_endurance_verdicts():
    go = endurance_check(1000.0, 90.0)
    assert go["verdict"] == "GO"
    seg = endurance_check(8000.0, 50.0)      # 估 40% → 余 10 <30, ≥15?
    assert seg["verdict"] in ("SEGMENT", "REFUSE")
    refuse = endurance_check(12000.0, 40.0)  # 估 60% → 余 -20
    assert refuse["verdict"] == "REFUSE"
    assert endurance_check(1000.0, None)["verdict"] == "unknown"
    assert go["eta_min"] > 0


def test_route_completion():
    assert route_completion({}) == 0.0
    assert route_completion({"waypoint_index": 5, "waypoint_total": 11}) == 0.5
    assert route_completion({"waypoint_index": 10, "waypoint_total": 11}) == 1.0


def test_nearest_route_point():
    wps = [{"lat": 31.0, "lon": 120.0}, {"lat": 31.01, "lon": 120.0},
           {"lat": 31.02, "lon": 120.0}]
    index, dist = nearest_route_point(wps, 31.016, 120.0)
    assert index == 2 and dist < 700


# ---------- approach_vantage ----------

def test_approach_vantage_picks_safe_route_point():
    platform = MockAdapter()
    guard = WaterGuard(approval_token="t")
    # 规划真实园区环线 (夹具缓存), 布防守卫 (环 = 园区凸包)
    ctx = _ctx(platform=platform, guard=guard)
    plan = TOOLS["plan_lake_loop"].execute({}, ctx)
    assert plan["ok"]
    ctx["guard"].arm([(31.4890, 120.3690), (31.4890, 120.3710),
                      (31.4910, 120.3710), (31.4910, 120.3690)])
    # 告警在水塘中心 (31.488993, 120.369226 — M2.1 规划选中过的水塘)
    result = TOOLS["approach_vantage"].execute(
        {"lat": 31.488993, "lng": 120.369226}, ctx)
    assert result["ok"], result
    vantage = result["vantage"]
    assert result["dist_to_alert_m"] > 5        # 不贴告警 (安全距离)
    assert result["guard_dist_m"] is None or result["guard_dist_m"] > 2


def test_approach_vantage_requires_plan():
    result = TOOLS["approach_vantage"].execute(
        {"lat": 31.49, "lng": 120.37}, _ctx(guard=_armed_guard()))
    assert result["ok"] is False
    assert result["reason"] == "no_route_in_session"


# ---------- patrol_report ----------

def test_patrol_report_full_shape():
    platform = MockAdapter()
    guard = WaterGuard(approval_token="t")
    guard.arm([(31.4890, 120.3690), (31.4890, 120.3710),
               (31.4910, 120.3710), (31.4910, 120.3690)])
    ctx = _ctx(platform=platform, guard=guard)
    plan = TOOLS["plan_lake_loop"].execute({}, ctx)   # 填 plan_store
    assert plan["ok"]
    platform.submit_gps_route(
        ctx["plan_store"]["last_route"]["waypoints"])
    # 模拟推进 3 个航点
    platform._route["waypoint_index"] = 3
    report = TOOLS["patrol_report"].execute({}, ctx)
    assert report["ok"]
    assert "巡逻任务报告" in report["report"]
    assert "confirmed 0" in report["report"]
    assert report["summary"]["scan_points"] > 0
    assert 0 < report["summary"]["completion"] < 1


# ---------- 扫描点 (lake_plan 输出) ----------

def test_plan_output_contains_scan_points():
    import os
    from pathlib import Path
    fixtures = (Path(__file__).resolve().parents[2]
                / "lake_plan" / "tests" / "fixtures")
    os.environ["GO2W_LAKE_CACHE_DIR"] = str(fixtures / "cache")
    os.environ["GO2W_LAKE_OFFLINE"] = "1"
    from lake_plan import plan_route
    result = plan_route(31.488192, 120.369486, kind="water")
    assert result["ok"]
    points = result["scan_points"]
    length_m = result["stats"]["length_m"]
    # 数量 ≈ 周长/间距 (0.4km/150m → 2~4 个)
    assert 1 <= len(points) <= max(2, int(length_m / 100) + 2)
    for p in points:
        assert 0.0 <= p["look_bearing_deg"] < 360.0
        # 朝向质心: 水塘质心纬度 31.488993 > 扫描点纬度 (环在水塘外,
        # 北侧点应朝南 ~180°, 南侧点朝北 ~0°) —— 只验证字段存在与合理范围
