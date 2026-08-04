"""pytest 配置: 跳过 tools/ 下的脚本式测试。

test_round2_fixes.py 和 test_video_nodecode.py 是"脚本式"测试 (模块级 print
+ sys.exit), 设计为 `python tools/test_xxx.py` 直接跑。pytest 收集时会在
import 阶段执行 sys.exit → 整个 pytest 会话 no tests ran 崩溃。

collect_ignore 让 pytest 忽略这两个文件 (它们仍能手动跑), 其他 test_*.py
正常被 pytest 收集。
"""

collect_ignore = [
    "test_round2_fixes.py",
    "test_video_nodecode.py",
    "test_stage_e.py",  # 同样脚本式 (verify_product_room_person_search.sh 用 python 直接跑)
]
