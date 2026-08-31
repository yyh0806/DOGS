"""内置工具集合: M1 只读/无运动风险四件套。

get_gps / get_battery / get_pose 为 read 级 (纯查询);
speak 为 act 级 (有副作用但无运动, M1 桩实现, M5 接 TTS/狗喇叭)。
"""
from __future__ import annotations

from ..registry import ToolRegistration

from .get_battery import TOOL as GET_BATTERY
from .get_gps import TOOL as GET_GPS
from .get_pose import TOOL as GET_POSE
from .speak import TOOL as SPEAK

BUILTIN_TOOLS: list[ToolRegistration] = [GET_GPS, GET_BATTERY, GET_POSE, SPEAK]
