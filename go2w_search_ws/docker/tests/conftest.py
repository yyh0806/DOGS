"""pytest 共享配置: 恢复 docker/ 与 go2w_search_ws/ 的 import 路径。

移动背景: 2026-08 目录重组 docker/test_*.py → docker/tests/。业务脚本在 docker/ 下,
部分部署契约测试还直接 import tools/* (go2w_search_ws/tools)。此 conftest 恢复路径,
业务代码零改动。
"""
import sys
from pathlib import Path

_DOCKER_ROOT = Path(__file__).resolve().parent.parent  # go2w_search_ws/docker
_WS_ROOT = _DOCKER_ROOT.parent                        # go2w_search_ws
for _p in (_DOCKER_ROOT, _WS_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
