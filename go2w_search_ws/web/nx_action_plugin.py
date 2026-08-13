"""动作插件框架 (fetch-task-planner) — 让新动作"即插即用"。

设计哲学延续项目核心: 大模型只填模板槽位, 执行走确定性状态机。
每个动作插件是一个自包含对象, 注册进 ACTION_REGISTRY 后:
  1. NLU 层:  intent_parser(text) 尝试把自然语言解析成模板 (不匹配返回 None)
  2. 校验层:  validate(task_params) 参数合法性 (fail-closed)
  3. 执行层:  execute(task, ctx) 状态机执行 (ctx 提供 nav/ai/ws 能力)
  4. 语音层:  voice_phrases() 各类状态的人工播报文案

TaskManager._worker 的分发从 if/elif 链改为注册表查找, 新动作
(拿咖啡/取快递/巡逻) 只需新增一个插件文件并 register, 核心零改动。

示例 (fetch 插件):
    register_action(FetchActionPlugin())
    # 之后语音 "去大门口拿咖啡" → fetch 模板 → FetchActionPlugin.execute
"""
from __future__ import annotations

import logging
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger("go2w.actions")

# 动作上下文: 执行器访问系统能力的门面 (不直接 import 具体模块, 便于测试注入)
ActionContext = Dict[str, Any]


class ActionPlugin:
    """一个可执行动作的完整封装。"""

    #: 任务类型名 (task.type), 全局唯一
    name: str = ""

    def intent_parser(self, text: str) -> Optional[dict]:
        """把自然语言解析成任务模板 {type, priority, params}, 不匹配返回 None。

        模板即契约: LLM/语音只能填这些槽位, 之外的一切被拒绝。
        """
        return None

    def validate(self, params: dict) -> tuple[bool, str]:
        """参数校验 (fail-closed)。返回 (ok, reason)。"""
        return True, ""

    def execute(self, task, ctx: ActionContext) -> None:
        """状态机执行。必须设置 task.status/task.result, 不抛异常 (吞掉记日志)。"""
        task.status = "failed"
        task.result = f"插件 {self.name} 未实现 execute"

    def voice_phrases(self) -> Dict[str, str]:
        """人工播报文案: {状态: 文案}。"""
        return {}


_ACTION_REGISTRY: Dict[str, ActionPlugin] = {}


def register_action(plugin: ActionPlugin) -> None:
    """注册动作插件 (幂等, 后注册覆盖同名)。"""
    if not plugin.name:
        raise ValueError("ActionPlugin.name 必填")
    _ACTION_REGISTRY[plugin.name] = plugin
    logger.info(f"动作已注册: {plugin.name}")


def get_action(name: str) -> Optional[ActionPlugin]:
    return _ACTION_REGISTRY.get(name)


def registered_actions() -> list:
    return sorted(_ACTION_REGISTRY.keys())


def parse_plugin_intent(text: str) -> Optional[dict]:
    """按注册顺序尝试所有插件的 intent_parser。

    优先级: 内置确定性解析器 (move/search) 先跑, 插件在后;
    返回第一个匹配的模板, 全不匹配返回 None。
    """
    for plugin in _ACTION_REGISTRY.values():
        try:
            result = plugin.intent_parser(text)
        except Exception as e:
            logger.warning(f"插件 {plugin.name} intent_parser 异常: {e}")
            continue
        if result is not None:
            return result
    return None
