"""大脑配置: 环境变量优先, 缺省值保证离线可跑。"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

_HERE = Path(__file__).resolve().parent


@dataclass
class BrainConfig:
    # 平台接入
    platform: str = "mock"                 # mock | nx
    nx_url: str = "http://localhost:8000"  # NX 侧 nx_web_server 基址
    tool_timeout_s: float = 5.0
    # LLM (DeepSeek function-calling; key 为空 = 离线规则模式)
    llm_base_url: str = "https://api.deepseek.com"
    llm_model: str = "deepseek-chat"
    llm_api_key: str = ""
    llm_temperature: float = 0.2
    llm_max_tokens: int = 1500
    # 循环与轨迹
    max_steps: int = 20
    skill_dir: Path = _HERE / "skills"
    log_dir: Path = Path("runs/brain")

    @classmethod
    def from_env(cls) -> "BrainConfig":
        return cls(
            platform=os.environ.get("GO2W_BRAIN_PLATFORM", "mock").strip(),
            nx_url=os.environ.get(
                "GO2W_NX_URL", "http://localhost:8000").strip(),
            llm_base_url=os.environ.get(
                "GO2W_LLM_BASE_URL", "https://api.deepseek.com").strip(),
            llm_model=os.environ.get("GO2W_LLM_MODEL", "deepseek-chat").strip(),
            llm_api_key=(os.environ.get("DEEPSEEK_API_KEY", "").strip()
                         or _read_legacy_credential()),
            log_dir=Path(os.environ.get("GO2W_BRAIN_LOG_DIR", "runs/brain")),
        )


def _read_legacy_credential() -> str:
    """兼容 lake_loop 的凭据位置: ~/.dsh/.credentials.yaml 中的 deepseek key。"""
    p = Path.home() / ".dsh" / ".credentials.yaml"
    if not p.exists():
        return ""
    try:
        lines = p.read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return ""
    for line in lines:
        line = line.strip()
        if ":" not in line:
            continue
        key, _, value = line.partition(":")
        if "deepseek" in key.lower():
            value = value.strip().strip("\"'")
            if value:
                return value
    return ""
