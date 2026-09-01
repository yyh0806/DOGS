"""pytest 公共设施: 把 web/ 加入 sys.path, 提供标准夹具。"""
from __future__ import annotations

import sys
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parents[2]  # .../go2w_search_ws/web
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

import pytest  # noqa: E402

from go2w_brain.config import BrainConfig  # noqa: E402
from go2w_brain.dispatcher import DispatchGate  # noqa: E402
from go2w_brain.llm import LLMClient  # noqa: E402
from go2w_brain.platform import MockAdapter  # noqa: E402
from go2w_brain.registry import SkillCatalog, ToolRegistry  # noqa: E402
from go2w_brain.tools import BUILTIN_TOOLS  # noqa: E402


@pytest.fixture
def mock_platform():
    return MockAdapter()


@pytest.fixture
def registry():
    reg = ToolRegistry()
    for tool in BUILTIN_TOOLS:
        reg.register(tool)
    return reg


@pytest.fixture
def gate(registry):
    g = DispatchGate(registry)
    from go2w_brain.dispatcher import (require_mission_lock,
                                       require_water_guard_armed)
    g.register_precondition("mission_lock", require_mission_lock)
    g.register_precondition("water_guard_armed",
                            require_water_guard_armed)
    return g


@pytest.fixture
def skills():
    return SkillCatalog(Path(__file__).resolve().parents[1] / "skills")


@pytest.fixture
def offline_config(tmp_path):
    config = BrainConfig.from_env()
    config.llm_api_key = ""  # 强制离线, 测试不依赖网络
    config.log_dir = tmp_path / "runs"
    return config
