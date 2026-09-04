"""task_plan — 任务列表 (M7, 强语义): 动词+参数级结构化计划。

用户的架构决策: 指令 × 记忆 → 任务列表。本模块实现任务列表的对象形态:

  PlanStep = {id, verb, args, preconditions[], depends[], memory_refs[], status}
  TaskPlan.validate: 动词必须在注册表 / 参数过 schema / 前置已注册 /
                    依赖可解析 / 无环 / memory_refs 存在于记忆库 ——
                    全部确定性强校验, 不通过就不执行 (计划也要过安检);
  rule_compose: 无 LLM 时的规则组合器 (退化路径, 同时是 LLM 起草的参照);
  draft_prompt: LLM 起草提示词 (结构化 JSON 输出 + 记忆摘要)。

执行由 brain_loop 逐条经既有 DispatchGate 派发 —— 计划执行与自由循环
共用同一条安检链路, 不新开旁路。
"""
from __future__ import annotations

from typing import Any, Optional


class PlanStep:
    def __init__(self, step_id: str, verb: str, args: dict[str, Any] | None,
                 preconditions=None, depends=None, memory_refs=None):
        self.id = step_id
        self.verb = verb
        self.args = dict(args or {})
        self.preconditions = list(preconditions or [])
        self.depends = list(depends or [])
        self.memory_refs = list(memory_refs or [])
        self.status = "pending"          # pending → running → ok / failed / skipped

    def to_dict(self) -> dict[str, Any]:
        return {"id": self.id, "verb": self.verb, "args": self.args,
                "preconditions": self.preconditions,
                "depends": self.depends,
                "memory_refs": self.memory_refs,
                "status": self.status}

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> Optional["PlanStep"]:
        if not isinstance(raw, dict):
            return None
        step_id = str(raw.get("id", "")).strip()
        verb = str(raw.get("verb", "")).strip()
        if not step_id or not verb:
            return None
        args = raw.get("args")
        if args is not None and not isinstance(args, dict):
            return None
        return cls(step_id, verb, args,
                   preconditions=raw.get("preconditions"),
                   depends=raw.get("depends"),
                   memory_refs=raw.get("memory_refs"))


class TaskPlan:
    def __init__(self, steps: list[PlanStep], source: str = "unknown"):
        self.steps = steps
        self.source = source  # llm | rule
        self.state = "draft"  # draft → valid → running → done / failed

    def to_dict(self) -> dict[str, Any]:
        return {"steps": [s.to_dict() for s in self.steps],
                "source": self.source, "state": self.state}

    def step(self, step_id: str) -> Optional[PlanStep]:
        return next((s for s in self.steps if s.id == step_id), None)

    def ordered_steps(self) -> list[PlanStep]:
        """拓扑序 (依赖在前)。有环时返回部分序 (校验器据此判环)。"""
        remaining = list(self.steps)
        ordered: list[PlanStep] = []
        while remaining:
            ready = [s for s in remaining
                     if all(d not in {r.id for r in remaining}
                            or any(o.id == d for o in ordered)
                            for d in s.depends)]
            if not ready:  # 环: 停止, 返回部分序
                return ordered
            for step in ready:
                ordered.append(step)
                remaining.remove(step)
        return ordered

    def validate(self, registry, gate, memory) -> tuple[bool, list[str]]:
        """确定性强校验。失败原因逐条列出, 不通过即不执行。"""
        errors: list[str] = []
        ids = {s.id for s in self.steps}
        if not self.steps:
            return False, ["empty_plan"]
        if len(ids) != len(self.steps):
            errors.append("duplicate_step_id")
        registered_preconditions = gate.registered_preconditions()
        for step in self.steps:
            tool = registry.get(step.verb)
            if tool is None:
                errors.append(f"{step.id}: unknown_verb:{step.verb}")
                continue
            ok, err = registry.validate_args(tool, step.args)
            if not ok:
                errors.append(f"{step.id}: invalid_args:{err}")
            for pre in step.preconditions:
                if pre not in registered_preconditions:
                    errors.append(f"{step.id}: unknown_precondition:{pre}")
            for dep in step.depends:
                if dep not in ids:
                    errors.append(f"{step.id}: unknown_dep:{dep}")
            for ref in step.memory_refs:
                if memory is None or memory.get(ref) is None:
                    errors.append(f"{step.id}: unresolved_memory_ref:{ref}")
        # 环检测 (拓扑)
        ordered = self.ordered_steps()
        if len(ordered) != len(self.steps):
            errors.append("cycle_detected")
        if errors:
            return False, errors
        self.state = "valid"
        return True, []


# ---------- 规则组合器 (退化路径) ----------------------------------------------

_RULE_TEMPLATES = {
    "lake": [
        ("p1", "plan_lake_loop", {}, [], []),
        ("g1", "arm_water_guard", {"from_plan": True}, [], ["p1"]),
        ("f1", "follow_route", {"from_plan": True},
         ["mission_lock", "water_guard_armed"], ["g1"]),
        ("s1", "scan_water", {"frames": 3}, [], ["f1"]),
        ("r1", "patrol_report", {}, [], ["s1"]),
    ],
    "campus": [
        ("p1", "plan_campus_loop", {}, [], []),
        ("g1", "arm_water_guard", {"from_plan": True}, [], ["p1"]),
        ("f1", "follow_route", {"from_plan": True},
         ["mission_lock", "water_guard_armed"], ["g1"]),
        ("s1", "scan_water", {"frames": 3}, [], ["f1"]),
        ("r1", "patrol_report", {}, [], ["s1"]),
    ],
    "status": [
        ("b1", "get_battery", {}, [], []),
        ("g1", "get_gps", {}, [], []),
        ("p1", "get_pose", {}, [], []),
    ],
}


def detect_plan_kind(task: str) -> Optional[str]:
    text = task or ""
    if any(k in text for k in ("绕湖", "环湖")):
        return "lake"
    # M7.3 自然说法: "绕着当前园区湖绕行一圈" 等 (湖 + 绕/环/巡)
    if "湖" in text and any(k in text for k in ("绕", "环", "巡")):
        return "lake"
    if any(k in text for k in ("绕园区", "园区巡查", "绕厂区")):
        return "campus"
    if "园区" in text and any(k in text for k in ("绕", "环", "巡")):
        return "campus"
    if any(k in text for k in ("状态", "报告", "电量", "在哪", "位置", "坐标")):
        return "status"
    return None


def rule_compose(kind: str) -> Optional[TaskPlan]:
    template = _RULE_TEMPLATES.get(kind)
    if not template:
        return None
    steps = [PlanStep(sid, verb, args, pre, deps)
             for (sid, verb, args, pre, deps) in template]
    return TaskPlan(steps, source="rule")


# ---------- LLM 起草 -----------------------------------------------------------

def draft_prompt(tool_schemas, memory_summary: str, task: str) -> str:
    import json
    names = sorted(s.get("function", {}).get("name", "?")
                   for s in tool_schemas)
    return (
        "你是任务计划编译器。把用户任务编译成任务列表 (DAG), 只输出 JSON:\n"
        '{"steps": [{"id": "p1", "verb": "<工具名>", "args": {...}, '
        '"preconditions": [...], "depends": [...], "memory_refs": [...]}]}\n\n"'
        "规则:\n"
        "1. verb 必须来自这个工具清单: " + json.dumps(names, ensure_ascii=False) + "\n"
        "2. 运动类任务必须包含: 规划 → arm_water_guard(from_plan) → "
        "follow_route(from_plan) → scan_water → patrol_report 的固定顺序; "
        "depends 表达先后; arm_water_guard/follow_route 的 from_plan 是布尔 "
        "值 true (引用本会话最近规划), 不是步骤 id; 不要添加模板外的步骤;\n"
        "2b. preconditions 只允许填这两个已注册前置: mission_lock, "
        "water_guard_armed —— 不要发明其他前置名; 除 follow_route 需要 "
        "两个都填外, 其他步骤留空;\n"
        "2c. 可在计划开头加一步 load_skill, 它的参数名是 name (技能名, "
        "如 \"lake-patrol\"), 不要写成 skill;\n"
        "3. memory_refs 引用下方记忆摘要里的条目 id (没有就省略该字段);\n"
        "4. 不要发明清单外的动词; 参数只填工具 schema 允许的字段;\n"
        "5. 任务: " + task + "\n\n"
        "记忆摘要 (本任务相关经验):\n" + memory_summary + "\n"
        "只输出 JSON, 不要解释。"
    )


def parse_draft(raw: str) -> Optional[TaskPlan]:
    import json
    import re
    text = str(raw or "").strip()
    # 容忍模型包裹 ```json ... ``` 或前缀解释
    match = re.search(r"```(?:json)?\s*(\{.*\})\s*```", text, re.S)
    if match:
        text = match.group(1)
    else:
        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            text = text[start:end + 1]
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return None
    raw_steps = payload.get("steps") if isinstance(payload, dict) else None
    if not isinstance(raw_steps, list) or not raw_steps:
        return None
    steps = []
    for i, raw_step in enumerate(raw_steps):
        step = PlanStep.from_dict(raw_step)
        if step is None:
            return None
        if not step.id:
            step.id = f"s{i}"
        steps.append(step)
    return TaskPlan(steps, source="llm")
