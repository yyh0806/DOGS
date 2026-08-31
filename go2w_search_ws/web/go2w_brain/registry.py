"""工具注册表 + Skill 目录 (DSH 思想的 Python 化)。

- ToolRegistry: scope 分层 (global / mission), 同名时 mission 层遮蔽
  global 层 (近者胜), 同层重名直接拒绝 —— 能力注册是显式的, 不存在静默覆盖;
- SkillCatalog: Markdown 懒加载, 目录(名字+摘要)常驻上下文,
  全文按需 load(); 前端元数据手动解析 (零第三方依赖)。
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Callable

RISK_LEVELS = ("read", "act", "approve")

_EXECUTOR = Callable[[dict[str, Any], dict[str, Any]], dict[str, Any]]


class ToolRegistration:
    """一个工具 = 名字 + 描述 + JSON schema + 确定性 execute。"""

    def __init__(self, name: str, description: str, parameters: dict[str, Any],
                 execute: _EXECUTOR, risk: str = "read",
                 requires: tuple[str, ...] = ()):
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
            raise ValueError(f"非法工具名: {name!r}")
        if risk not in RISK_LEVELS:
            raise ValueError(f"非法风险级: {risk!r} (可选 {RISK_LEVELS})")
        if parameters.get("type") != "object":
            raise ValueError("parameters 必须是 object 型 schema")
        self.name = name
        self.description = description
        self.parameters = parameters
        self.execute = execute
        self.risk = risk
        self.requires = tuple(requires)

    def schema(self) -> dict[str, Any]:
        """OpenAI function-calling 形态 (喂给 LLM 的正是这个)。"""
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.parameters,
            },
        }


class ToolRegistry:
    def __init__(self) -> None:
        self._layers: dict[str, dict[str, ToolRegistration]] = {
            "global": {}, "mission": {}}

    def register(self, tool: ToolRegistration, scope: str = "global") -> None:
        if scope not in self._layers:
            raise ValueError(f"未知 scope: {scope!r}")
        layer = self._layers[scope]
        if tool.name in layer:
            raise ValueError(f"同层重名工具已存在: {tool.name}")
        layer[tool.name] = tool

    def get(self, name: str) -> ToolRegistration | None:
        return self._layers["mission"].get(name) or self._layers["global"].get(name)

    def names(self) -> list[str]:
        return sorted(set(self._layers["global"]) | set(self._layers["mission"]))

    def schemas(self) -> list[dict[str, Any]]:
        return [self.get(n).schema() for n in self.names()]  # type: ignore[union-attr]

    def validate_args(self, tool: ToolRegistration,
                      args: dict[str, Any]) -> tuple[bool, str]:
        """schema 子集校验: type/required/enum (M1 范围, 未知字段容忍)。"""
        if not isinstance(args, dict):
            return False, "args_not_object"
        props = tool.parameters.get("properties") or {}
        required = set(tool.parameters.get("required") or [])
        missing = required - set(args)
        if missing:
            return False, "missing_required:" + ",".join(sorted(missing))
        for key, value in args.items():
            prop = props.get(key)
            if prop is None:
                continue  # 未知字段: 容忍 (对齐主流 LLM 行为)
            ptype = prop.get("type")
            if ptype == "string" and not isinstance(value, str):
                return False, f"type:{key}!=string"
            if ptype == "number" and not isinstance(value, (int, float)):
                return False, f"type:{key}!=number"
            if ptype == "boolean" and not isinstance(value, bool):
                return False, f"type:{key}!=boolean"
            if ptype == "array" and not isinstance(value, list):
                return False, f"type:{key}!=array"
            enum = prop.get("enum")
            if enum is not None and value not in enum:
                return False, f"enum:{key}"
        return True, ""


class Skill:
    def __init__(self, name: str, description: str, when_to_use: str,
                 body: str, path: Path):
        self.name = name
        self.description = description
        self.when_to_use = when_to_use
        self.body = body
        self.path = path

    def summary(self) -> dict[str, str]:
        return {"name": self.name, "description": self.description,
                "when_to_use": self.when_to_use}


class SkillCatalog:
    """skill 目录扫描 + 懒加载。发现即读正文 (本地文件, 成本可忽略)。"""

    def __init__(self, skill_dir: Path):
        self._dir = Path(skill_dir)
        self._skills: dict[str, Skill] = {}
        for path in sorted(self._dir.glob("*.md")):
            skill = _parse_skill_file(path)
            if skill is not None and skill.name not in self._skills:
                self._skills[skill.name] = skill

    def summaries(self) -> list[dict[str, str]]:
        return [s.summary() for s in self._skills.values()]

    def load(self, name: str) -> Skill | None:
        return self._skills.get(name)

    def catalog_text(self) -> str:
        """目录常驻上下文 (喂进系统提示词, 只占几百 token)。"""
        lines = ["可用技能 (按需加载全文, 不要凭空使用不存在的技能):"]
        for s in self._skills.values():
            lines.append(f"- {s.name}: {s.description}")
            if s.when_to_use:
                lines.append(f"  适用: {s.when_to_use}")
        return "\n".join(lines)

    @staticmethod
    def render_skill_content(skill: Skill) -> str:
        return (f'<skill_content name="{skill.name}">\n'
                f"{skill.body.strip()}\n</skill_content>")


_FRONT_RE = re.compile(r"\A---\s*\n(.*?)\n---\s*\n(.*)\Z", re.S)


def _parse_skill_file(path: Path) -> Skill | None:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    match = _FRONT_RE.match(text)
    if not match:
        return None
    meta: dict[str, str] = {}
    for line in match.group(1).splitlines():
        if ":" in line:
            key, _, value = line.partition(":")
            meta[key.strip()] = value.strip()
    name = meta.get("name") or path.stem
    return Skill(name=name, description=meta.get("description", ""),
                 when_to_use=meta.get("when_to_use", ""),
                 body=match.group(2), path=path)
