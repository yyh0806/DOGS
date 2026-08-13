"""fetch 动作插件 — "去大门口拿咖啡" (无臂送递模式 S1-S5)。

状态机 (TASK_PLANNING_SPEC §2):
  S1 导航到 pickup 地标 (Nav2)
  S2 视觉找 object (locate-anything 按需单次, 找不到→换视角重试×2)
  S3 语音/面板提示 "请把<object>放到我身上"
  S4 等待人确认装载 (WS confirm / API / 语音确认)
  S5 导航返回 deliver 地标 (缺省"原位"=出发位置)
  任意步失败 → fail-closed 报告, 狗原地停止

依赖全部通过 ctx 注入 (测试可 mock), 不直接 import 具体模块。
"""
from __future__ import annotations

import logging
import re
import time
from typing import Optional

from nx_action_plugin import ActionPlugin

logger = logging.getLogger("go2w.actions.fetch")

_FETCH_RE = re.compile(
    r"^(?:去|到|前往)?(?P<pickup>.+?)(?:帮我)?(?:拿|取|帮我拿|帮我取)"
    r"(?P<object>.+?)(?:过来|到这里来|到我这)?$"
)
_STRIP_WORDS = {"的", "一杯", "一个", "一瓶", "一下"}


def _strip_quantifiers(s: str) -> str:
    for w in _STRIP_WORDS:
        s = s.replace(w, "")
    return s.strip()


def parse_fetch_command(text: str) -> Optional[dict]:
    """解析 "去大门口拿咖啡" → fetch 模板。不匹配返回 None。"""
    import re as _re
    normalized = _re.sub(r"[\s，。！？、,!?；;：:]+", "", text or "")
    if not normalized:
        return None
    m = _FETCH_RE.match(normalized)
    if not m:
        return None
    pickup = m.group("pickup").strip()
    obj = _strip_quantifiers(m.group("object"))
    # 复合连接词 ("然后/接着/先") 出现在槽位里 → 确定性模板无法正确拆,
    # 返回 None 让 LLM propose-verify 生成多步计划
    for _part in (pickup, obj):
        if any(_w in _part for _w in ("然后", "接着", "先", "再", "之后")):
            return None
    # 反例: 纯移动/搜索/跟踪/地标导航不应落入 fetch
    if not pickup or not obj:
        return None
    if obj in {"人", "东西"} and "拿" not in normalized and "取" not in normalized:
        return None
    return {
        "response": f"去{pickup}拿{obj}",
        "tasks": [{
            "type": "fetch",
            "priority": 8,
            "params": {
                "pickup": pickup,
                "object": obj,
                "deliver": None,  # None = 原位 (出发位置)
            },
        }],
    }


class FetchActionPlugin(ActionPlugin):
    name = "fetch"

    def intent_parser(self, text: str) -> Optional[dict]:
        return parse_fetch_command(text)

    def validate(self, params: dict) -> tuple[bool, str]:
        pickup = (params or {}).get("pickup")
        obj = (params or {}).get("object")
        if not isinstance(pickup, str) or not pickup.strip():
            return False, "pickup 地标必填"
        if not isinstance(obj, str) or not obj.strip():
            return False, "object 目标必填"
        return True, ""

    def execute(self, task, ctx: dict) -> None:
        """S1-S5 状态机。ctx 提供: landmarks_find, point_nav, locate, ws, confirm_wait, robot_pos。"""
        params = task.params or {}
        pickup = params.get("pickup", "")
        obj = params.get("object", "")
        deliver = params.get("deliver")  # None → 原位
        ws = ctx.get("ws")
        say = (lambda phase, **kw: ws({"type": "fetch", "data": {"phase": phase, **kw}})
               if callable(ws) else lambda *a, **k: None)

        def fail(reason):
            task.status = "failed"
            task.result = reason
            say("FAILED", reason=reason)
            logger.warning(f"[fetch] 失败: {reason}")

        # S0: 记出发位置 (原位返回点)
        origin = None
        get_pos = ctx.get("robot_pos")
        if callable(get_pos):
            origin = get_pos()

        # S1: 导航到 pickup 地标
        lm_find = ctx.get("landmarks_find")
        lm = lm_find(pickup) if callable(lm_find) else None
        if lm is None:
            fail(f"未知地标: {pickup}")
            return
        say("NAVIGATING", landmark=pickup)
        nav = ctx.get("point_nav")
        if not callable(nav):
            fail("导航不可用")
            return
        nav_result = nav(lm.x, lm.y, lm.yaw, frame_id=lm.frame_id or "map")
        if not (isinstance(nav_result, dict) and
                (nav_result.get("ok") or nav_result.get("reached"))):
            fail(f"导航到 {pickup} 失败")
            return
        say("ARRIVED", landmark=pickup)

        # S2: 视觉找 object (locate 按需单次 + 重试 2 次)
        locate = ctx.get("locate")
        found = False
        for attempt in (1, 2, 3):
            if callable(locate):
                try:
                    res = locate(obj)
                    if isinstance(res, dict) and res.get("found"):
                        found = True
                        break
                except Exception as e:
                    logger.warning(f"[fetch] locate 尝试 {attempt} 异常: {e}")
            time.sleep(0.05)  # 实际场景由 ctx 提供 rotate 重试; 此处仅占位
        say("FOUND" if found else "NOT_FOUND", object=obj)

        # S3+S4: 提示装载 + 等待确认
        say("AWAITING_LOAD", object=obj)
        confirm = ctx.get("confirm_wait")
        loaded = False
        if callable(confirm):
            try:
                loaded = bool(confirm(timeout=120.0))
            except Exception as e:
                logger.warning(f"[fetch] confirm_wait 异常: {e}")
        if not loaded:
            fail("等待装载确认超时")
            return
        say("LOADED", object=obj)

        # S5: 返回交付点 (deliver 地标 > 原位)
        if deliver:
            lm2 = lm_find(deliver) if callable(lm_find) else None
            if lm2 is None:
                fail(f"未知交付地标: {deliver}")
                return
            target = (lm2.x, lm2.y, lm2.yaw)
            say("RETURNING", landmark=deliver)
        elif origin:
            target = (origin[0], origin[1], origin[2])
            say("RETURNING", landmark="原位")
        else:
            task.status = "completed"
            task.result = f"{obj} 已装载, 无返回点原地交付"
            say("DONE", object=obj)
            return
        nav2 = nav(target[0], target[1], target[2], frame_id="map")
        if not (isinstance(nav2, dict) and
                (nav2.get("ok") or nav2.get("reached"))):
            fail("返回导航失败")
            return
        task.status = "completed"
        task.result = f"{obj} 已送达到 {deliver or '原位'}"
        say("DONE", object=obj)

    def voice_phrases(self) -> dict:
        return {
            "NAVIGATING": "正在前往{pickup}",
            "ARRIVED": "已到达{pickup}",
            "FOUND": "看到了{object}",
            "NOT_FOUND": "没有找到{object}",
            "AWAITING_LOAD": "请把{object}放到我身上",
            "LOADED": "收到{object}，准备返回",
            "RETURNING": "正在返回",
            "DONE": "{object}送到了，请取",
            "FAILED": "任务失败：{reason}",
        }
