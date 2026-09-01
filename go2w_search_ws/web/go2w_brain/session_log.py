"""任务轨迹 (DSH 会话持久化思想的 Python 化): jsonl 追加, 可恢复可回放。

每行一个 JSON 条目, kind 标注来源: session_start / task / snapshot /
llm_call / tool_call / tool_result / event / reply / session_end。
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


class SessionLog:
    def __init__(self, path: Path, on_append=None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # 可选广播钩子 (实时控制台用): 每写入一条轨迹即回调 entry
        self._on_append = on_append

    def append(self, kind: str, **fields: Any) -> dict[str, Any]:
        entry: dict[str, Any] = {"ts": round(time.time(), 3), "kind": kind}
        entry.update(fields)
        line = json.dumps(entry, ensure_ascii=False, default=str)
        with open(self.path, "a", encoding="utf-8") as fp:
            fp.write(line + "\n")
        if self._on_append is not None:
            try:
                self._on_append(entry)
            except Exception:  # noqa: BLE001 — 广播失败不得影响轨迹
                pass
        return entry

    def entries(self) -> list[dict[str, Any]]:
        if not self.path.exists():
            return []
        out: list[dict[str, Any]] = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                out.append({"ts": None, "kind": "corrupt_line",
                            "raw": line[:200]})
        return out

    @classmethod
    def resume(cls, path: Path) -> "SessionLog":
        """指向同一文件的续写句柄 (不重建、不截断)。"""
        return cls(path)

    def stats(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for entry in self.entries():
            counts[entry["kind"]] = counts.get(entry["kind"], 0) + 1
        return counts

    def load_messages(self) -> list[dict[str, str]]:
        """从轨迹重建最小对话历史 (供恢复/回放, M1 范围)。"""
        messages: list[dict[str, str]] = []
        for entry in self.entries():
            kind = entry.get("kind")
            if kind == "task":
                messages.append({"role": "user", "content": entry["content"]})
            elif kind == "snapshot":
                from .prompt_assembler import render_snapshot_message
                messages.append({"role": "user",
                                 "content": render_snapshot_message(
                                     entry.get("data") or {})})
            elif kind == "tool_result":
                messages.append({"role": "user",
                                 "content": json.dumps(entry.get("result"),
                                                       ensure_ascii=False)})
            elif kind == "reply":
                messages.append({"role": "assistant",
                                 "content": entry["content"]})
        return messages
