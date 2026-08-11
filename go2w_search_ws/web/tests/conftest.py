"""pytest 共享配置: 恢复 web/ 与 go2w_search_ws/ 的 import 路径。

移动背景: 2026-08 目录重组 web/test_*.py → web/tests/。pytest 收集测试时,
测试文件所在目录(web/tests/)被加入 sys.path, 但业务模块在 web/ 下,
部分测试还直接 import tools/* (go2w_search_ws/tools)。此 conftest 恢复两条路径,
业务代码零改动。
"""
import sys
from pathlib import Path

_WEB_ROOT = Path(__file__).resolve().parent.parent  # go2w_search_ws/web
_WS_ROOT = _WEB_ROOT.parent                        # go2w_search_ws
for _p in (_WEB_ROOT, _WS_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
