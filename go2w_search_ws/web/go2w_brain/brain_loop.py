"""大脑主循环 (DSH dsh-agent-loop 思想的 Python 化)。

事件驱动的自由循环:
- 有任务 → 装配提示词 (系统分节 + 工具 schema + 技能目录) →
  LLM 推理 (function-calling) → 每个工具调用过 DispatchGate →
  执行 → 结果追加进历史 → 继续, 直到模型给出最终答复或步数耗尽;
- 无 LLM → 规则退化 (rule_respond): 只懂"状态报告"类任务,
  不懂就老实报告 —— 工具照常可用, 能力边界诚实;
- 每轮遥测快照以追加消息注入 (不改写系统提示词, 前缀稳定);
- 全程 jsonl 轨迹。

慢回路纪律: run() 是任务级调度, 绝不进入运动控制频率;
外部事件经 submit_event() 唤醒 (M2 起 follow_route 进度等异步事件由此进)。
"""
from __future__ import annotations

import json
import queue
import threading
from typing import Any

from . import prompt_assembler
from .config import BrainConfig
from .dispatcher import DispatchGate, require_mission_lock
from .llm import LLMClient, LLMUnavailable, parse_tool_args
from .platform import PlatformAdapter
from .registry import SkillCatalog, ToolRegistry
from .session_log import SessionLog


class BrainSession:
    def __init__(self, config: BrainConfig, platform: PlatformAdapter,
                 registry: ToolRegistry, gate: DispatchGate,
                 skills: SkillCatalog, llm: LLMClient, log: SessionLog):
        self._config = config
        self._platform = platform
        self._registry = registry
        self._gate = gate
        self._skills = skills
        self._llm = llm
        self._log = log
        self._events: "queue.Queue[dict[str, Any]]" = queue.Queue()
        self._wake = threading.Event()
        self._mission_lock: Any = None

    # -- 事件 (M1 管道就绪, M2 起注入异步进度) ----------------------------

    def submit_event(self, event: dict[str, Any]) -> None:
        self._events.put(event)
        self._wake.set()

    def _drain_events(self) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                break
        self._wake.clear()
        return events

    def acquire_mission_lock(self, token: Any = "mission") -> None:
        self._mission_lock = token

    def release_mission_lock(self) -> None:
        self._mission_lock = None

    # -- 主循环 ------------------------------------------------------------

    def run(self, task: str) -> dict[str, Any]:
        self._log.append("session_start", version="0.1.0")
        self._log.append("task", content=task)

        system = prompt_assembler.assemble(
            self._registry.schemas(),
            self._skills.catalog_text(),
            variables={"model": self._config.llm_model},
        )["system"]
        history: list[dict[str, Any]] = [
            {"role": "system", "content": system},
            {"role": "user", "content": task},
        ]
        self._log.append("llm_call", kind_note="system_prompt",
                         system_prompt=system)

        answer = ""
        steps = 0
        max_steps = self._config.max_steps
        while steps < max_steps:
            events = self._drain_events()
            for event in events:
                self._log.append("event", **event)
                history.append({"role": "user", "content":
                                "[event] " + json.dumps(
                                    event, ensure_ascii=False, default=str)})
            snapshot = self._platform.snapshot()
            self._log.append("snapshot", data=snapshot)
            history.append({"role": "user", "content":
                            prompt_assembler.render_snapshot_message(snapshot)})

            if not self._llm.available():
                answer, calls = _rule_respond(task)
                results = [self._dispatch_and_run(call[0], call[1])
                           for call in calls]
                if calls:
                    answer = _compose_status_answer(results)
                break

            try:
                resp = self._llm.chat(history,
                                      tools=self._registry.schemas())
            except LLMUnavailable as exc:
                self._log.append("event", event="llm_unavailable",
                                 reason=str(exc))
                answer, calls = _rule_respond(task)
                results = [self._dispatch_and_run(call[0], call[1])
                           for call in calls]
                if calls:
                    answer = _compose_status_answer(results)
                break
            self._log.append("llm_call", content=resp.content or None,
                             tool_calls=[tc["function"]["name"]
                                         for tc in resp.tool_calls],
                             reasoning=resp.reasoning[:200] or None)
            if not resp.tool_calls:
                answer = resp.content or "(空回复)"
                break
            for tc in resp.tool_calls:
                name = (tc.get("function") or {}).get("name", "")
                args = parse_tool_args((tc.get("function") or {})
                                       .get("arguments"))
                result = self._dispatch_and_run(name, args)
                history.append({"role": "user", "content":
                                f"[tool_result {name}] " + json.dumps(
                                    result, ensure_ascii=False, default=str)})
            steps += 1

        if not answer:
            answer = "(步数耗尽, 未得到最终答复)"
        self._log.append("reply", content=answer)
        self._log.append("session_end", steps=steps,
                         llm_used=self._llm.available())
        return {"answer": answer, "trace": str(self._log.path),
                "steps": steps, "llm_used": self._llm.available()}

    # -- 内部 --------------------------------------------------------------

    def _dispatch_and_run(self, name: str, args: dict[str, Any]) -> dict[str, Any]:
        ctx = {"platform": self._platform, "log": self._log,
               "mission_lock": self._mission_lock}
        ok, reason, tool = self._gate.check(name, args, ctx)
        self._log.append("tool_call", name=name, args=args, ok=ok,
                         reason=reason)
        if not ok:
            return {"ok": False, "dispatch_denied": reason}
        try:
            result = tool.execute(args, ctx)
        except Exception as exc:  # noqa: BLE001 — 工具异常必须兜底
            result = {"ok": False,
                      "error": f"{type(exc).__name__}: {exc}"}
        self._log.append("tool_result", name=name, result=result)
        return result


def _rule_respond(task: str) -> tuple[str, list[tuple[str, dict[str, Any]]]]:
    """规则退化 (无 LLM): M1 只懂"状态报告"。不懂就诚实拒绝。"""
    text = task or ""
    keywords = ("状态", "报告", "电量", "gps", "在哪", "位置", "坐标",
                "定位", "电量多少")
    if any(keyword in text for keyword in keywords):
        return "", [("get_gps", {}), ("get_battery", {}), ("get_pose", {})]
    return (f"离线规则模式目前只理解'状态报告'类任务; 任务"
            f"「{text}」需要 LLM 或后续里程碑扩展规则。", [])


def _compose_status_answer(results: list[dict[str, Any]]) -> str:
    gps, battery, pose = (results + [{}, {}, {}])[:3]
    lines = ["状态报告 (离线规则模式):"]
    if gps.get("ok"):
        lines.append(f"- 定位: {gps['lat']}, {gps['lng']}"
                     f" (HDOP {gps.get('hdop')}, 卫星 {gps.get('sats')},"
                     f" 新鲜度 {gps.get('fix_age_s')}s)")
    else:
        lines.append(f"- 定位: 不可用 ({gps.get('reason') or
                     gps.get('dispatch_denied') or '?'})")
    if battery.get("ok"):
        lines.append(f"- 电量: {battery['battery_soc']}%")
    else:
        lines.append(f"- 电量: 不可用 ({battery.get('reason') or
                     battery.get('dispatch_denied') or '?'})")
    if pose.get("ok"):
        lines.append(f"- 位姿: x={pose['x']}, y={pose['y']},"
                     f" yaw={pose.get('yaw_deg')}°")
    else:
        lines.append(f"- 位姿: 不可用 ({pose.get('reason') or
                     pose.get('dispatch_denied') or '?'})")
    return "\n".join(lines)
