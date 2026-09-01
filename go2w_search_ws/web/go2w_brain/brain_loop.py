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
import time
from typing import Any

from . import prompt_assembler, task_plan as task_plan_mod
from .config import BrainConfig
from .dispatcher import (DispatchGate, require_mission_lock,
                         require_water_guard_armed)
from .llm import LLMClient, LLMUnavailable, parse_tool_args
from .platform import PlatformAdapter
from .registry import SkillCatalog, ToolRegistry
from .session_log import SessionLog


class BrainSession:
    def __init__(self, config: BrainConfig, platform: PlatformAdapter,
                 registry: ToolRegistry, gate: DispatchGate,
                 skills: SkillCatalog, llm: LLMClient, log: SessionLog,
                 guard: Any = None, detector: Any = None,
                 frame_source: Any = None, memory: Any = None,
                 tts: Any = None):
        self._config = config
        self._platform = platform
        self._registry = registry
        self._gate = gate
        self._skills = skills
        self._llm = llm
        self._log = log
        self._guard = guard  # M3: 离水守卫 (nx_water_guard.WaterGuard)
        self._detector = detector  # M4: 落水检测引擎 (nx_drowning_detect)
        self._frame_source = frame_source  # M4: 帧源 callable → (frame, robot)
        self._memory = memory  # M7: 语义记忆库 (MemoryStore)
        self._tts = tts  # M7.2: 语音播报后端 (go2w_brain.tts)
        self._events: "queue.Queue[dict[str, Any]]" = queue.Queue()
        self._wake = threading.Event()
        self._mission_lock: Any = None
        self._plan_store: dict[str, Any] = {}  # M2: 规划结果引用传递

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

    def plan_snapshot(self) -> dict[str, Any]:
        """最近一次规划的完整结果 (实时控制台/外部观察者取几何用)。"""
        return dict(self._plan_store.get("last_route") or {})

    def acquire_mission_lock(self, token: Any = "mission") -> None:
        self._mission_lock = token

    def release_mission_lock(self) -> None:
        self._mission_lock = None

    # -- 主循环 ------------------------------------------------------------

    def run(self, task: str) -> dict[str, Any]:
        self._log.append("session_start", version="0.2.0")
        self._log.append("task", content=task)
        # M2: 任务上下文即任务锁 —— act 级工具 (follow_route 等) 的前置
        self._mission_lock = f"mission:{time.time():.0f}"
        self._log.append("event", event="mission_lock_acquired",
                         token=self._mission_lock)
        end_meta: dict[str, Any] = {}
        try:
            result = self._run_locked(task)
            end_meta = result.pop("_session_end", {})
            return result
        finally:
            self.release_mission_lock()
            self._log.append("event", event="mission_lock_released")
            # session_end 恒为轨迹最后一行 (任何退出路径都闭合)
            self._log.append("session_end", **end_meta)

    def _run_locked(self, task: str) -> dict[str, Any]:

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

        # M7: 计划式任务 (指令 × 记忆 → 任务列表, 动词+参数级强语义)
        plan_kind = task_plan_mod.detect_plan_kind(task)
        if plan_kind:
            plan_result = self._run_plan(task, plan_kind, history)
            if plan_result is not None:
                return plan_result

        answer = ""
        steps = 0
        max_steps = self._config.max_steps
        while steps < max_steps:
            events = self._drain_events()
            for event in events:
                self._log_event(event)
                history.append({"role": "user", "content":
                                "[event] " + json.dumps(
                                    event, ensure_ascii=False, default=str)})
            self._apply_observation_events(events)  # M7.2: 观测→记忆自动回写
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
        return {"answer": answer, "trace": str(self._log.path),
                "steps": steps, "llm_used": self._llm.available(),
                "_session_end": {"steps": steps,
                                 "llm_used": self._llm.available()}}

    # -- M7: 计划式任务 (指令 × 记忆 → 任务列表) ---------------------------

    def _run_plan(self, task: str, plan_kind: str,
                  history: list[dict[str, Any]]) -> Optional[dict[str, Any]]:
        """生成并执行任务列表。返回 None = 计划不可行, 回退自由循环。"""
        # 1. 记忆检索 (决策 3-B: 确定性检索 → 摘要注入, LLM 只读摘要)
        memory_summary = "（无历史记忆）"
        memory_entries = []
        if self._memory is not None:
            gps = self._platform.snapshot().get("gps") or {}
            if gps.get("available"):
                memory_entries = self._memory.query(
                    gps["lat"], gps["lng"], 2000.0, min_score=0.15)
                lines = []
                for entry in memory_entries:
                    lines.append(
                        f"- id={entry['id']} kind={entry['kind']} "
                        f"dist={entry['dist_m']}m score={entry['score']} "
                        f"data={json.dumps(entry.get('data'), ensure_ascii=False)}")
                if lines:
                    memory_summary = "\n".join(lines)
        history.append({"role": "user", "content":
                        "[memory] 本区域历史经验:\n" + memory_summary})
        self._log.append("event", event="memory_retrieved",
                         count=len(memory_entries))

        # 2. 计划生成: LLM 起草 (结构化 JSON) → 强校验 → 规则组合器兜底
        plan = None
        if self._llm.available():
            for attempt in range(2):
                try:
                    resp = self._llm.chat(
                        history + [{"role": "user",
                                    "content": task_plan_mod.draft_prompt(
                                        self._registry.schemas(),
                                        memory_summary, task)}],
                        tools=None)
                except LLMUnavailable:
                    break
                self._log.append("llm_call",
                                 content="draft_task_plan",
                                 draft_raw=(resp.content or "")[:600],
                                 reasoning=resp.reasoning[:200] or None)
                plan = task_plan_mod.parse_draft(resp.content or "")
                if plan is None:
                    self._log.append("event", event="plan_unparseable")
                    continue
                ok, errors = plan.validate(self._registry, self._gate,
                                           self._memory)
                if ok:
                    break
                self._log.append("event", event="plan_invalid",
                                 errors=errors)
                plan = None
        if plan is None:
            plan = task_plan_mod.rule_compose(plan_kind)
        if plan is None:
            return None
        ok, errors = plan.validate(self._registry, self._gate, self._memory)
        if not ok:
            self._log.append("event", event="plan_rejected",
                             errors=errors)
            return None
        self._log.append("event", event="task_plan",
                         source=plan.source,
                         steps=[s.to_dict() for s in plan.steps])
        history.append({"role": "user", "content": json.dumps(
            plan.to_dict(), ensure_ascii=False)})

        # 3. 执行: 逐条过既有 DispatchGate (计划与自由循环共用一条安检)
        results: dict[str, Any] = {}
        for step in plan.ordered_steps():
            step.status = "running"
            result = self._dispatch_and_run(step.verb, step.args)
            history.append({"role": "user", "content":
                            f"[tool_result {step.verb}] " + json.dumps(
                                result, ensure_ascii=False, default=str)})
            if result.get("dispatch_denied") or result.get("ok") is False:
                step.status = "failed"
                self._log.append("event", event="plan_step_failed",
                                 step=step.id, verb=step.verb,
                                 reason=result.get("dispatch_denied")
                                 or result.get("reason")
                                 or result.get("error"))
                # M7.1 重规划: LLM 有且仅一次修订机会 (修订计划仍过强校验)
                revised = self._replan(history, step, result)
                if revised is not None:
                    self._log.append("event", event="plan_replanned",
                                     source="llm",
                                     steps=[s.id for s in revised.steps])
                    for rstep in revised.ordered_steps():
                        rstep.status = "running"
                        rresult = self._dispatch_and_run(rstep.verb,
                                                         rstep.args)
                        history.append({"role": "user", "content":
                                        f"[tool_result {rstep.verb}] "
                                        + json.dumps(rresult,
                                                     ensure_ascii=False,
                                                     default=str)})
                        if (rresult.get("dispatch_denied")
                                or rresult.get("ok") is False):
                            rstep.status = "failed"
                            self._log.append("event",
                                             event="plan_step_failed",
                                             step=rstep.id,
                                             verb=rstep.verb,
                                             reason=rresult.get(
                                                 "dispatch_denied")
                                             or rresult.get("reason")
                                             or rresult.get("error"))
                            break
                        rstep.status = "ok"
                        results[rstep.id] = rresult
                    else:
                        plan.state = "replanned_done"
                        for s in plan.steps:
                            if s.status == "failed":
                                s.status = "replanned"
                    if any(s.status == "failed"
                           for s in revised.steps):
                        plan.state = "failed"
                break
            step.status = "ok"
            results[step.id] = result
            # M7.2: 步骤间隙处理执行期观测事件 (可走/堵等 → 记忆回写)
            step_events = self._drain_events()
            for event in step_events:
                self._log_event(event)
            self._apply_observation_events(step_events)
        else:
            plan.state = "done"
        if any(s.status == "failed" for s in plan.steps):
            plan.state = "failed"
            for step in plan.steps:
                if step.status == "pending":
                    step.status = "skipped"
        self._log.append("event", event="task_plan_done",
                         state=plan.state,
                         steps={s.id: s.status for s in plan.steps})

        # 4. 最终答复: LLM 汇总 (可用时) 或确定性组合
        if self._llm.available():
            try:
                resp = self._llm.chat(
                    history + [{"role": "user", "content":
                                "请用简洁中文汇报本次任务执行结果 (含关键数值)。"}])
                answer = resp.content or "(空回复)"
            except LLMUnavailable:
                answer = _compose_plan_answer(plan, results)
        else:
            answer = _compose_plan_answer(plan, results)
        self._log.append("reply", content=answer)
        return {"answer": answer, "trace": str(self._log.path),
                "steps": len(plan.steps), "llm_used": self._llm.available(),
                "_session_end": {"steps": len(plan.steps),
                                 "llm_used": self._llm.available()}}

    def _log_event(self, event: dict[str, Any]) -> None:
        """事件落轨迹。观测事件自带 kind 字段, 折叠为 event_kind 防冲突。"""
        payload = dict(event)
        if "kind" in payload:
            payload["event_kind"] = payload.pop("kind")
        self._log.append("event", **payload)

    def _apply_observation_events(self, events: list[dict[str, Any]]) -> None:
        """M7.2: 执行期观测事件 → 记忆自动回写 (确定性, 不经 LLM)。

        事件契约: {"kind": "observation", "mem_kind": "blocked"|"passable"|...,
                   "geo": {"lat","lng"} | {"points":[...]},
                   "confidence": 0.0-1.0, "data": {...}}
        来源 (真机/未来): 运动链堵点回调、导航恢复事件、操作员标注。
        """
        if self._memory is None:
            return
        for event in events:
            if event.get("kind") != "observation":
                continue
            mem_kind = event.get("mem_kind")
            geo = event.get("geo")
            if not mem_kind or not isinstance(geo, dict):
                continue
            try:
                entry = self._memory.record(
                    mem_kind, geo, data=event.get("data") or {},
                    confidence=float(event.get("confidence", 0.7)),
                    source="observation")
                self._log.append("event", event="observation_recorded",
                                 memory_id=entry["id"],
                                 mem_kind=mem_kind)
            except ValueError:
                pass

    def _replan(self, history, failed_step, failure_result):
        """步骤失败 → LLM 修订计划 (一次机会)。任何环节失败返回 None。"""
        if not self._llm.available():
            return None
        names = sorted(self._registry.get(n).name
                       for n in self._registry.names())
        reason = (failure_result.get("dispatch_denied")
                  or failure_result.get("reason")
                  or failure_result.get("error"))
        prompt = (
            "任务执行中步骤失败, 请修订剩余任务计划。\n"
            f"失败步骤: {failed_step.verb}, 原因: {reason}\n"
            "可用工具: " + json.dumps(names, ensure_ascii=False) + "\n"
            "输出与之前相同的任务列表 JSON 格式 (只包含尚未完成的步骤, "
            "id 用 r1/r2/... 重新编号; 前置只允许 mission_lock/"
            "water_guard_armed; from_plan 是布尔 true)。只输出 JSON。")
        try:
            resp = self._llm.chat(history + [{"role": "user",
                                              "content": prompt}],
                                  tools=None)
        except LLMUnavailable:
            return None
        self._log.append("llm_call", content="replan_draft",
                         draft_raw=(resp.content or "")[:400])
        plan = task_plan_mod.parse_draft(resp.content or "")
        if plan is None:
            self._log.append("event", event="replan_unparseable")
            return None
        ok, errors = plan.validate(self._registry, self._gate, self._memory)
        if not ok:
            self._log.append("event", event="replan_invalid", errors=errors)
            return None
        return plan

    def _dispatch_and_run(self, name: str, args: dict[str, Any]) -> dict[str, Any]:
        ctx = {"platform": self._platform, "log": self._log,
               "mission_lock": self._mission_lock,
               "config": self._config,
               "plan_store": self._plan_store,
               "guard": self._guard,
               "detector": self._detector,
               "frame_source": self._frame_source,
               "memory": self._memory,
               "tts": self._tts,
               "skills": self._skills,
               "approval_token": self._config.approval_token}
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


def _compose_plan_answer(plan, results: dict[str, Any]) -> str:
    """无 LLM 时的计划执行汇报 (确定性组合)。"""
    zh = {"get_battery": "电量", "get_gps": "定位", "get_pose": "位姿",
          "plan_lake_loop": "规划绕湖环线", "plan_campus_loop": "规划绕园区环线",
          "arm_water_guard": "布防离水守卫", "follow_route": "受理航线",
          "calibrate_heading": "北向标定", "scan_water": "湖面扫描",
          "patrol_report": "任务报告", "approach_vantage": "安全接近点",
          "cancel_route": "取消航线", "record_observation": "记忆写入",
          "get_memory": "记忆检索"}
    lines = [f"任务计划执行汇报 ({plan.source} 计划, 状态 {plan.state}):"]
    for step in plan.steps:
        mark = {"ok": "✅", "failed": "❌", "skipped": "⏭",
                "pending": "·"}.get(step.status, "·")
        result = results.get(step.id, {})
        detail = ""
        if step.status == "ok":
            if step.verb == "get_battery":
                detail = f"{result.get('battery_soc', '?')}%"
            elif step.verb == "get_gps":
                detail = f"{result.get('lat')}, {result.get('lng')}"
            elif step.verb == "get_pose":
                detail = f"x={result.get('x')}, y={result.get('y')}"
            elif step.verb in ("plan_lake_loop", "plan_campus_loop"):
                detail = (f"{result.get('waypoint_count', '?')} 航点 / "
                          f"{result.get('length_km', '?')}km")
            elif step.verb == "scan_water":
                detail = f"confirmed {result.get('confirmed_count', 0)}"
            elif step.verb == "arm_water_guard":
                detail = f"{result.get('vertices', '?')} 顶点禁区"
        if step.status == "failed":
            detail = (result.get("dispatch_denied") or result.get("reason")
                      or result.get("error") or "失败")
        lines.append(f"  {mark} [{step.id}] {zh.get(step.verb, step.verb)}"
                     + (f" — {detail}" if detail else ""))
    failed = next((s for s in plan.steps if s.status == "failed"), None)
    if failed:
        lines.append(f"计划因步骤 {failed.id} 失败而中止 (诚实停手, 未执行余下步骤)。")
    return "\n".join(lines)


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
