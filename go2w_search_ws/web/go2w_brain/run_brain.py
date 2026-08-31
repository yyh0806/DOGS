"""CLI 入口: python -m go2w_brain.run_brain --task "报告当前状态"

M1 干跑口径 (验收): --platform mock --no-llm 在无网络/无 ROS 环境全链路可跑。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .config import BrainConfig
from .dispatcher import DispatchGate, require_mission_lock
from .llm import LLMClient
from .platform import MockAdapter, NxHttpAdapter
from .registry import SkillCatalog, ToolRegistry
from .session_log import SessionLog
from .tools import BUILTIN_TOOLS


def build_session(config: BrainConfig, log: SessionLog):
    platform = (NxHttpAdapter(config.nx_url, timeout=config.tool_timeout_s,
                              control_token=config.control_token)
                if config.platform == "nx" else MockAdapter())
    registry = ToolRegistry()
    for tool in BUILTIN_TOOLS:
        registry.register(tool)
    gate = DispatchGate(registry)
    gate.register_precondition("mission_lock", require_mission_lock)
    skills = SkillCatalog(config.skill_dir)
    llm = LLMClient(config)
    from .brain_loop import BrainSession
    return BrainSession(config, platform, registry, gate, skills, llm, log)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="go2w_brain 任务大脑 (M1)")
    parser.add_argument("--task", required=True, help="任务文本")
    parser.add_argument("--platform", choices=("mock", "nx"),
                        default=None, help="默认取 GO2W_BRAIN_PLATFORM/mock")
    parser.add_argument("--nx-url", default=None, help="NX 基址")
    parser.add_argument("--no-llm", action="store_true",
                        help="强制离线规则模式 (忽略 DEEPSEEK_API_KEY)")
    parser.add_argument("--dry", action="store_true",
                        help="干跑: 运动类工具只记录不下发 (默认 GO2W_BRAIN_DRY)")
    parser.add_argument("--log-dir", default=None, help="轨迹目录")
    args = parser.parse_args(argv)

    config = BrainConfig.from_env()
    if args.platform:
        config.platform = args.platform
    if args.nx_url:
        config.nx_url = args.nx_url
    if args.no_llm:
        config.llm_api_key = ""
    if args.dry:
        config.dry_run = True
    if args.log_dir:
        config.log_dir = Path(args.log_dir)

    from datetime import datetime
    log_path = config.log_dir / (datetime.now().strftime(
        "%Y%m%d_%H%M%S_%f") + "_brain.jsonl")
    log = SessionLog(log_path)
    session = build_session(config, log)
    started = time.time()
    result = session.run(args.task)
    elapsed = time.time() - started

    print()
    print("=" * 60)
    print(result["answer"])
    print("=" * 60)
    print(f"llm_used={result['llm_used']} steps={result['steps']} "
          f"elapsed={elapsed:.1f}s")
    print(f"轨迹: {result['trace']}")
    print(f"轨迹统计: {log.stats()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
