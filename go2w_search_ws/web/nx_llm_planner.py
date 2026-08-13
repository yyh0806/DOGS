"""propose-verify LLM 规划器 — DeepSeek 提议计划, 模板校验器裁决。

架构 (与用户确认的演进方向):
  用户指令 → [确定性解析链全 miss] → LLM 提议 JSON 计划
          → 校验器逐条裁决 (动作必须已在注册表/白名单, 参数合法)
          → 通过: 执行计划; 拒绝: 回退拒绝 + 思考过程透出

安全边界:
  - LLM 输出不是自由动作: 每个 step.action 必须在 ALLOWED_ACTIONS
    (内置动作 + 插件注册表), 参数经插件 validate / 类型检查
  - LLM 失败/超时/格式错 → 返回 None (fail-closed, 走原有拒绝路径)
  - 思考过程 (reasoning) 只随响应透出展示, 不参与任何执行决策
"""
from __future__ import annotations

import json
import logging
import re
from typing import Callable, Optional, Tuple

logger = logging.getLogger("go2w.llm_planner")

# 内置确定性动作白名单 (与 TaskManager._worker 分发一致)
_BUILTIN_ACTIONS = {
    "search_room", "move_relative", "go_landmark", "follow", "fetch",
}

# 计划 schema 约束
_MAX_STEPS = 4

_SYSTEM_PROMPT = (
    "你是机器人任务规划器。把用户的自然语言指令转换成 JSON 计划。\n"
    "只允许输出如下格式的 JSON（不要 markdown、不要多余文字）：\n"
    '{"steps": [{"action": "<动作名>", "args": {<参数>}}, ...], '
    '"reasoning": "<简短中文说明>"}\n'
    "可用动作与参数：\n"
    '  go_landmark: {"landmark": "<地标名>"}\n'
    '  fetch: {"pickup": "<地标名>", "object": "<物品描述>", '
    '"deliver": "<地标名或null>"}\n'
    '  follow: {"target": "<人物描述>"}\n'
    '  search_room: {"room": "<房间名或__current__>", '
    '"target_classes": ["person"]}\n'
    '  move_relative: {"direction": "forward|backward|left|right", '
    '"distance_m": 数字}\n'
    "规则：\n"
    "1. action 只能用上面列出的名字\n"
    "2. 参数类型必须匹配示例\n"
    "3. 最多 4 步\n"
    "4. 指令无法表达时返回 {\"steps\": [], \"reasoning\": \"无法理解\"}\n"
)


def _plugin_action_names() -> set:
    try:
        from nx_action_plugin import registered_actions
        return set(registered_actions())
    except Exception:
        return set()


def allowed_actions() -> set:
    return _BUILTIN_ACTIONS | _plugin_action_names()


def _validate_step(step: dict) -> Tuple[bool, str]:
    if not isinstance(step, dict):
        return False, "step 必须是对象"
    action = step.get("action")
    if not isinstance(action, str) or action not in allowed_actions():
        return False, f"未知动作: {action}"
    args = step.get("args")
    if not isinstance(args, dict):
        return False, f"{action} 的 args 必须是对象"
    # 参数按动作类型做最小校验 (fail-closed 类型检查)
    checks = {
        "go_landmark": lambda a: isinstance(a.get("landmark"), str),
        "fetch": lambda a: isinstance(a.get("pickup"), str)
                           and isinstance(a.get("object"), str),
        "follow": lambda a: isinstance(a.get("target"), str),
        "move_relative": lambda a: a.get("direction") in
                                   ("forward", "backward", "left", "right")
                                   and isinstance(a.get("distance_m"),
                                                  (int, float)),
        "search_room": lambda a: isinstance(a.get("room"), str),
    }
    check = checks.get(action)
    if check is not None and not check(args):
        return False, f"{action} 参数不合法: {args}"
    # 插件动作走插件 validate
    if action in _plugin_action_names():
        try:
            from nx_action_plugin import get_action
            plugin = get_action(action)
            if plugin is not None:
                ok, reason = plugin.validate(args)
                if not ok:
                    return False, reason
        except Exception as e:
            return False, f"插件校验异常: {e}"
    return True, ""


class LLMPlanner:
    """propose-verify: LLM 提议 → 强校验 → 计划或 None。"""

    def __init__(self, llm):
        self._llm = llm

    def plan(self, text: str) -> Optional[dict]:
        """返回 {response, tasks, reasoning} 或 None (fail-closed 回退)。"""
        if not getattr(self._llm, "configured", False):
            return None
        answer, reasoning = self._llm.chat(_SYSTEM_PROMPT, text)
        if not answer:
            return None
        # 剥离可能的 markdown 代码块
        cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", answer.strip())
        try:
            data = json.loads(cleaned)
        except json.JSONDecodeError as e:
            logger.warning(f"[llm_planner] LLM 输出非 JSON: {e}")
            return None
        if not isinstance(data, dict):
            return None
        steps = data.get("steps", [])
        if not isinstance(steps, list) or not steps:
            return None
        if len(steps) > _MAX_STEPS:
            logger.warning(f"[llm_planner] 步数超限 {len(steps)}")
            return None
        tasks = []
        for step in steps:
            ok, reason = _validate_step(step)
            if not ok:
                logger.warning(f"[llm_planner] 步骤被拒: {reason}")
                return None  # 任一步非法 → 整体拒绝 (fail-closed)
            tasks.append({
                "type": step["action"],
                "priority": 8,
                "params": step["args"],
            })
        return {
            "response": str(data.get("reasoning") or "已生成计划"),
            "tasks": tasks,
            "reasoning": reasoning,
        }
