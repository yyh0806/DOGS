"""内置工具集合。

M1: get_gps / get_battery / get_pose / speak(桩) — 全部只读或无运动风险;
M2: plan_lake_loop / follow_route / cancel_route / calibrate_heading /
    get_route_state — 规划与航线 (运动类 = act 级, 受 mission_lock 前置);
M2.1: plan_campus_loop — 绕园区 (Overpass landuse 聚类 + 凸包)。
"""
from __future__ import annotations

from ..registry import ToolRegistration

from .calibrate_heading import TOOL as CALIBRATE_HEADING
from .cancel_route import TOOL as CANCEL_ROUTE
from .follow_route import TOOL as FOLLOW_ROUTE
from .get_battery import TOOL as GET_BATTERY
from .get_gps import TOOL as GET_GPS
from .get_pose import TOOL as GET_POSE
from .get_route_state import TOOL as GET_ROUTE_STATE
from .plan_campus_loop import TOOL as PLAN_CAMPUS_LOOP
from .plan_lake_loop import TOOL as PLAN_LAKE_LOOP
from .speak import TOOL as SPEAK

BUILTIN_TOOLS: list[ToolRegistration] = [
    GET_GPS, GET_BATTERY, GET_POSE, SPEAK,
    PLAN_LAKE_LOOP, PLAN_CAMPUS_LOOP, FOLLOW_ROUTE, CANCEL_ROUTE,
    CALIBRATE_HEADING, GET_ROUTE_STATE,
]
