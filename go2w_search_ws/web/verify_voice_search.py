"""PC 端语音搜索人员 NLU 验证 (不需 NX)。

跑法:
    python web/verify_voice_search.py

验证各类自然说法 → parse_product_command → search_room task 的解析正确性。
这是"语音→任务"链路的核心, 让你接入 NX 前先在 PC 确认 NLU 健康;
任何 STT 误识别或 NLU 回归会在这里第一时间暴露。
"""
import sys
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_product_command import parse_product_command


# (描述, 指令, 期望 room 或 None, 期望 type)
CASES = [
    ("用户核心场景",   "去搜索这个房间，把所有人标注出来", "__current__", "search_room"),
    ("简化说法",       "搜索这个房间把所有人标注出来",     "__current__", "search_room"),
    ("当前房间",       "搜索当前房间标注所有人",           "__current__", "search_room"),
    ("里+的双 sep",    "找一下这个房间里的人",             "__current__", "search_room"),
    ("标记人员",       "标记这个房间的人员",               "__current__", "search_room"),
    ("本房间",         "搜寻本房间所有人",                 "__current__", "search_room"),
    ("把字句",         "把当前房间的人员标出来",           "__current__", "search_room"),
    ("命名-客厅",      "去客厅搜索所有人",                 "客厅",        "search_room"),
    ("命名-实验室",    "搜索实验室里的人",                 "实验室",      "search_room"),
    ("命名-办公室",    "去办公室找所有人标注出来",         "办公室",      "search_room"),
    ("否定-别",        "别搜索这个房间",                   None,          None),
    ("否定-不要",      "不要找这个房间的人",               None,          None),
    ("否定-不用",      "不用标记所有人",                   None,          None),
    ("非搜索-前进",    "前进两米",                         None,          None),
    ("非搜索-返回",    "返回起点",                         None,          None),
    ("follow 指令",    "跟着前面的人",                     None,          None),
]


def _evaluate(text, expect_room, expect_type):
    """返回 (ok, detail) — 期望 None 时校验拒绝; 否则校验 type+room。"""
    result = parse_product_command(text)
    if expect_room is None:
        if result is None:
            return True, "OK (正确拒绝)"
        t = result.get("tasks", [{}])[0]
        return False, f"FAIL (应拒绝但命中 type={t.get('type')})"
    if result is None:
        return False, "FAIL (应命中但被拒绝)"
    t = result["tasks"][0]
    if t["type"] != expect_type:
        return False, f"FAIL (type={t['type']!r}, 期望 {expect_type!r})"
    if t["params"].get("room") != expect_room:
        return False, f"FAIL (room={t['params'].get('room')!r}, 期望 {expect_room!r})"
    return True, f"OK (room={expect_room}, target={t['params'].get('target_classes')})"


def main():
    print("=" * 64)
    print("语音搜索人员 NLU 验证 (PC 端, 不需 NX)")
    print(f"测试用例: {len(CASES)}")
    print("=" * 64)
    pass_count = 0
    fail_count = 0
    for desc, text, expect_room, expect_type in CASES:
        ok, detail = _evaluate(text, expect_room, expect_type)
        status = "[ OK ]" if ok else "[FAIL]"
        print(f"{status} [{desc}] {text!r}")
        print(f"    {detail}")
        if ok:
            pass_count += 1
        else:
            fail_count += 1
    print("=" * 64)
    print(f"Summary: {pass_count} PASS, {fail_count} FAIL, {pass_count + fail_count} total")
    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
