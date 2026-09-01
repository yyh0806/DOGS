"""内置工具集合。

M1: get_gps / get_battery / get_pose / speak(桩) — 全部只读或无运动风险;
M2: plan_lake_loop / follow_route / cancel_route / calibrate_heading /
    get_route_state — 规划与航线 (运动类 = act 级, 受 mission_lock 前置);
M2.1: plan_campus_loop — 绕园区 (Overpass landuse 聚类 + 凸包);
M3: arm_water_guard / disarm_water_guard(approve) / get_guard_state —
    离水守卫; follow_route 前置升级为 mission_lock + water_guard_armed;
M4: scan_water / get_detection_events — 落水检测五级管线 (两级告警);
M5: approach_vantage / patrol_report — 安全接近点 + 任务报告 + 续航模型。
"""
from __future__ import annotations

from ..registry import ToolRegistration

from .approach_vantage import TOOL as APPROACH_VANTAGE
from .arm_water_guard import TOOL as ARM_WATER_GUARD
from .calibrate_heading import TOOL as CALIBRATE_HEADING
from .cancel_route import TOOL as CANCEL_ROUTE
from .disarm_water_guard import TOOL as DISARM_WATER_GUARD
from .follow_route import TOOL as FOLLOW_ROUTE
from .get_battery import TOOL as GET_BATTERY
from .get_detection_events import TOOL as GET_DETECTION_EVENTS
from .get_gps import TOOL as GET_GPS
from .get_guard_state import TOOL as GET_GUARD_STATE
from .get_memory import TOOL as GET_MEMORY
from .get_pose import TOOL as GET_POSE
from .get_route_state import TOOL as GET_ROUTE_STATE
from .load_skill import TOOL as LOAD_SKILL
from .patrol_report import TOOL as PATROL_REPORT
from .plan_campus_loop import TOOL as PLAN_CAMPUS_LOOP
from .plan_lake_loop import TOOL as PLAN_LAKE_LOOP
from .push_alert import TOOL as PUSH_ALERT
from .record_observation import TOOL as RECORD_OBSERVATION
from .scan_water import TOOL as SCAN_WATER
from .speak import TOOL as SPEAK

BUILTIN_TOOLS: list[ToolRegistration] = [
    GET_GPS, GET_BATTERY, GET_POSE, SPEAK,
    PLAN_LAKE_LOOP, PLAN_CAMPUS_LOOP, FOLLOW_ROUTE, CANCEL_ROUTE,
    CALIBRATE_HEADING, GET_ROUTE_STATE,
    ARM_WATER_GUARD, DISARM_WATER_GUARD, GET_GUARD_STATE,
    LOAD_SKILL, SCAN_WATER, GET_DETECTION_EVENTS,
    APPROACH_VANTAGE, PATROL_REPORT,
    GET_MEMORY, RECORD_OBSERVATION, PUSH_ALERT,
]
