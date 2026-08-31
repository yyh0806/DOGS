"""提示词组装 (DSH dsh-system-prompt 思想的 Python 化)。

- 有序分节: 身份 → 人设 → 安全铁律 → 工具指引 → 技能目录;
- 严格变量替换: {{name}} 未注册即报错, 绝不静默交付格式错误的提示词;
- 前缀稳定纪律: 组装结果确定 (分节有序、工具按名排序),
  运行时遥测以「追加消息」注入历史, 不改写系统提示词 (KV cache 友好)。
"""
from __future__ import annotations

from typing import Any

SECTIONS: list[tuple[str, int, str]] = [
    ("identity", -100,
     "你是机器狗 Go2W 的任务大脑 (go2w_brain)。你负责理解任务、选择工具、"
     "监督执行, 不做实时控制。"),
    ("persona", 0,
     "人设: 谨慎、诚实、可审计。每次工具调用与决策都有 jsonl 轨迹。"),
    ("safety", 100,
     "安全铁律: 你只做慢回路决策; 运动安全/离水看门狗/急停在快回路, "
     "与你无关且你无权绕过。凡是拿不准的动作, 宁可拒绝也要说清原因。"),
    ("tool_guidance", 200,
     "工具使用规则:\n"
     "1. 工具是动词, 是能力的硬边界 —— 只能调用下方列表里的工具, "
     "绝不虚构、组合或假装调用不存在的工具;\n"
     "2. 技能是方法论 —— 目录在下方, 命中任务时先加载全文再行动;\n"
     "3. 需要的能力没有对应动词时, 如实报告缺什么, 不要硬编。"),
    ("skills_catalog", 300, "{{skill_catalog}}"),
]

TOOL_SCHEMA_PREFIX = "可用工具 (JSON schema, 按名排序):"


def render(text: str, variables: dict[str, str]) -> str:
    """{{name}} 严格替换: 未注册引用、格式错误 → 抛错。"""
    out: list[str] = []
    pos = 0
    while True:
        start = text.find("{{", pos)
        if start < 0:
            out.append(text[pos:])
            break
        end = text.find("}}", start + 2)
        if end < 0:
            raise ValueError("提示词变量格式错误: 存在未闭合的 '{{'")
        name = text[start + 2:end].strip()
        if name not in variables:
            raise ValueError(f"提示词引用了未注册变量: {name}")
        out.append(text[pos:start])
        out.append(variables[name])
        pos = end + 2
    return "".join(out)


def assemble(tool_schemas: list[dict[str, Any]], skill_catalog_text: str,
             variables: dict[str, Any] | None = None) -> dict[str, Any]:
    """返回 {system: str, tools: [...], sections: [...]}。"""
    variables = dict(variables or {})
    variables.setdefault("skill_catalog", skill_catalog_text)
    ordered = sorted(SECTIONS, key=lambda s: s[1])
    parts = [render(text, variables) for (_, _, text) in ordered]
    system = "\n\n".join(parts) + "\n\n" + TOOL_SCHEMA_PREFIX
    return {"system": system, "tools": list(tool_schemas),
            "sections": [name for (name, _, _) in ordered]}


def render_snapshot_message(snapshot: dict[str, Any]) -> str:
    """遥测快照 → 追加式用户消息 (DSH runtime-context 对应物)。"""
    slim = {
        "gps": snapshot.get("gps"),
        "battery_soc": snapshot.get("battery_soc"),
        "pose": snapshot.get("pose"),
        "gps_route": snapshot.get("gps_route"),
    }
    import json
    return "[runtime-context] " + json.dumps(slim, ensure_ascii=False,
                                              default=str)
