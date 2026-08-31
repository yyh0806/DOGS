"""派发安检 (DSH 沙箱/审批思想的运动版)。

执行任何工具前必须过闸:
1. 工具必须在注册表 —— 能力硬边界, 未知动词直接拒绝 (大脑幻觉不出动作);
2. 参数 schema 校验 —— 缺必填/类型错/enum 越界 → 拒绝;
3. 前置条件全部满足 —— 缺前置即 fail-closed (安全取向);
4. 风险分级: read 放行 / act 放行并留痕 / approve 必须带审批令牌。

M1 提供机制 + 通用前置; 具体业务前置 (arm_water_guard 先于
follow_route 等) 从 M3 起逐条登记。
"""
from __future__ import annotations

from typing import Any, Callable

from .registry import ToolRegistration, ToolRegistry

_PRECONDITION = Callable[[ToolRegistration, dict[str, Any]],
                         tuple[bool, str]]


class DispatchGate:
    def __init__(self, registry: ToolRegistry,
                 preconditions: dict[str, _PRECONDITION] | None = None):
        self._registry = registry
        self._preconditions: dict[str, _PRECONDITION] = dict(
            preconditions or {})

    def register_precondition(self, name: str, fn: _PRECONDITION) -> None:
        if not name or not name.replace("_", "a").isalnum():
            raise ValueError(f"非法前置名: {name!r}")
        self._preconditions[name] = fn

    def check(self, tool_name: str, args: dict[str, Any],
              ctx: dict[str, Any]) -> tuple[bool, str, ToolRegistration | None]:
        tool = self._registry.get(tool_name)
        if tool is None:
            return False, "unknown_tool", None
        ok, err = self._registry.validate_args(tool, args)
        if not ok:
            return False, "invalid_args:" + err, tool
        for name in tool.requires:
            fn = self._preconditions.get(name)
            if fn is None:
                # 声明了没人实现的前置 → 视为不安全, 拒绝 (fail-closed)
                return False, f"missing_precondition:{name}", tool
            ok, reason = fn(tool, ctx)
            if not ok:
                return False, reason, tool
        if tool.risk == "approve" and not ctx.get("approval_token"):
            return False, "approval_required", tool
        return True, "", tool


def require_mission_lock(tool: ToolRegistration,
                         ctx: dict[str, Any]) -> tuple[bool, str]:
    """通用前置示例: act 类工具必须持有任务锁 (互斥的机制表达)。

    M1 仅 get/speak 等 read 工具, 该前置先落机制; M2 follow_route
    挂上后, 同一时刻只允许一条主动航线。
    """
    if tool.risk == "read":
        return True, ""
    if ctx.get("mission_lock"):
        return True, ""
    return False, "mission_lock_required"


def require_water_guard_armed(tool: ToolRegistration,
                              ctx: dict[str, Any]) -> tuple[bool, str]:
    """M3 前置: 运动类任务必须先布防离水守卫 (安全先行)。

    follow_route 挂载此前置 —— 守卫不在岗, 航线一律不受理。
    """
    guard = ctx.get("guard")
    if guard is None:
        return False, "water_guard_unavailable"
    if guard.state().get("armed"):
        return True, ""
    return False, "water_guard_not_armed"
