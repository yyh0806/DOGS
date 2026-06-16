#!/usr/bin/env python3
"""
VLM 命令解析验证脚本
=====================
验证：
1. VLM 能否正确解析 panel 上的简单命令
2. VLM 能否将复杂命令分解为简单任务队列
3. fallback 关键词匹配作为对照组

用法:
    python test_vlm_commands.py              # 完整测试 (加载VLM)
    python test_vlm_commands.py --no-vlm     # 仅测试 fallback
    python test_vlm_commands.py --quick      # 快速测试 (少用token)
"""

import json, re, sys, time, argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from ai.vlm import VLMEngine


# ============================================================================
# 从 panel.py 复制的解析逻辑
# ============================================================================
def vlm_parse_command(vlm, text):
    """模拟 TaskManager._vlm_parse_command()"""
    sys_prompt = """你是一个机器狗助手。将用户指令分解为任务序列。
可用任务类型: move/follow/search_area/stop/return_home
move: 运动控制, 参数 {"vx": 前进速度(m/s), "vy": 侧移速度, "vyaw": 旋转速度(rad/s, 正=左转, 负=右转), "duration": 持续秒数}
follow: 跟踪目标, 参数 {"target": "目标描述"}
search_area: 搜索区域, 参数 {"pattern": "路径模式", "width": 宽度米, "height": 高度米}
stop: 停止, 参数 {}
return_home: 返回起点, 参数 {}
重要: 旋转/转弯用 move 任务设置 vyaw 参数，不要用 follow!
输出纯JSON(不要markdown代码块): {"understanding":"...","tasks":[{"type":"...","priority":1-10,"params":{...}}],"response":"..."}"""
    response = vlm.chat([
        {"role": "system", "content": sys_prompt},
        {"role": "user", "content": text}
    ], max_new_tokens=512)
    print(f"  VLM 原始输出: {response[:300]}")
    try:
        # 用括号匹配提取最外层 JSON
        m = re.search(r'\{', response)
        if m:
            start = m.start()
            depth = 0
            end = start
            for i, ch in enumerate(response[start:], start):
                if ch == '{': depth += 1
                elif ch == '}':
                    depth -= 1
                    if depth == 0:
                        end = i + 1
                        break
            json_str = response[start:end]
            return json.loads(json_str)
    except Exception as e:
        print(f"  JSON 解析失败: {e}")
    return fallback_parse(text)


def fallback_parse(text):
    """模拟 TaskManager._fallback_parse() - 关键词匹配"""
    r = {"understanding": text, "tasks": [], "response": ""}
    if "跟着" in text or "跟随" in text:
        for kw in ["跟着", "跟随"]:
            if kw in text: target = text[text.index(kw)+len(kw):].strip().rstrip("。，！？")
        r["tasks"] = [{"type": "follow", "priority": 8, "params": {"target": target}}]
        r["response"] = f"跟踪{target}"
    elif "搜索" in text or "找" in text:
        r["tasks"] = [{"type": "search_area", "priority": 5, "params": {"pattern": "lawnmower", "width": 10, "height": 10}}]
        r["response"] = "开始搜索"
    elif "停" in text:
        r["tasks"] = [{"type": "stop", "priority": 10, "params": {}}]; r["response"] = "已停止"
    elif "回来" in text or "返回" in text:
        r["tasks"] = [{"type": "return_home", "priority": 7, "params": {}}]; r["response"] = "返回"
    elif "前进" in text or "向前" in text:
        r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": 0.5, "duration": 2.0}}]; r["response"] = "前进"
    elif "后退" in text or "向后" in text:
        r["tasks"] = [{"type": "move", "priority": 6, "params": {"vx": -0.5, "duration": 2.0}}]; r["response"] = "后退"
    elif "左转" in text:
        r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": 0.5, "duration": 2.0}}]; r["response"] = "左转"
    elif "右转" in text:
        r["tasks"] = [{"type": "move", "priority": 6, "params": {"vyaw": -0.5, "duration": 2.0}}]; r["response"] = "右转"
    else:
        r["response"] = f"收到: {text}"
    return r


# ============================================================================
# 测试用例
# ============================================================================
TEST_CASES = [
    # === 简单命令 (panel 快速按钮对应) ===
    ("前进",           "简单: 前进 (panel按钮)"),
    ("后退",           "简单: 后退 (panel按钮)"),
    ("左转",           "简单: 左转 (panel按钮)"),
    ("右转",           "简单: 右转 (panel按钮)"),
    ("停",             "简单: 停止 (panel按钮)"),
    ("停止",           "简单: 停止(变体)"),
    ("返回",           "简单: 返回起点"),
    ("回来",           "简单: 回来"),

    # === 复合命令 (含距离/时长) ===
    ("前进3米",        "复合: 前进+距离"),
    ("后退1米",        "复合: 后退+距离"),
    ("左转90度",       "复合: 左转+角度"),
    ("右转180度",      "复合: 右转+角度"),
    ("前进5秒",        "复合: 前进+时长"),

    # === 复杂命令队列 ===
    ("前进两米然后左转",                       "复杂: 前进+左转"),
    ("往前走三米，然后左转90度，再往前走两米",   "复杂: 前进→左转→前进"),
    ("后退一米然后右转再前进两米",              "复杂: 后退→右转→前进"),
    ("前进两米，停一下，然后后退一米",           "复杂: 前进→停→后退"),

    # === 搜索/跟踪 ===
    ("搜索一下这个区域",     "搜索"),
    ("找一找有没有人",       "搜索变体"),
    ("跟着前面的人",         "跟踪"),
    ("跟随那个红色的球",     "跟踪变体"),

    # === 边界/异常 ===
    ("",                "边界: 空输入"),
    ("你好",             "边界: 无关输入"),
    ("帮我查一下天气",   "边界: 不支持的功能"),
]


# ============================================================================
# 分析和展示
# ============================================================================
PASS = "✅"
FAIL = "❌"
WARN = "⚠️"

def validate_result(text, result, test_type):
    """验证解析结果是否合理"""
    tasks = result.get("tasks", [])
    issues = []

    # 必须字段
    if "understanding" not in result:
        issues.append("缺少 understanding 字段")
    if "response" not in result:
        issues.append("缺少 response 字段")

    # 任务检查
    for i, t in enumerate(tasks):
        if "type" not in t:
            issues.append(f"任务{i}缺少 type 字段")
        if t.get("type") not in ("move", "follow", "search_area", "stop", "return_home", "navigate", "wait"):
            issues.append(f"任务{i}有未知类型: {t.get('type')}")
        if "params" not in t:
            issues.append(f"任务{i}缺少 params")

    # 简单命令应该有 ≤2 个任务
    if test_type == "简单" and len(tasks) > 2:
        issues.append(f"简单命令产生了{len(tasks)}个任务(预期≤2)")

    # 复杂命令应该有 >1 个任务
    if test_type == "复杂" and len(tasks) <= 1:
        issues.append(f"复杂命令只产生了{len(tasks)}个任务(预期>1)")

    # 空输入不应产生任务
    if not text.strip() and len(tasks) > 0:
        issues.append("空输入不应产生任务")

    return issues


def format_task(t):
    """格式化单个任务为一行"""
    ttype = t.get("type", "?")
    p = t.get("params", {})
    pri = t.get("priority", "?")

    if ttype == "move":
        vx = p.get("vx", 0)
        vy = p.get("vy", 0)
        vyaw = p.get("vyaw", 0)
        dur = p.get("duration", "?")
        return f"  ├─ move(vx={vx}, vy={vy}, vyaw={vyaw}, dur={dur}s) [优先级:{pri}]"
    elif ttype == "stop":
        return f"  ├─ stop() [优先级:{pri}]"
    elif ttype == "follow":
        return f"  ├─ follow(target={p.get('target', '?')}) [优先级:{pri}]"
    elif ttype == "search_area":
        return f"  ├─ search_area({p}) [优先级:{pri}]"
    elif ttype == "return_home":
        return f"  ├─ return_home() [优先级:{pri}]"
    elif ttype == "wait":
        return f"  ├─ wait(dur={p.get('duration', '?')}s) [优先级:{pri}]"
    else:
        return f"  ├─ {ttype}({p}) [优先级:{pri}]"


# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="VLM 命令解析验证")
    parser.add_argument("--no-vlm", action="store_true", help="仅测试 fallback")
    parser.add_argument("--quick", action="store_true", help="快速测试 (少量用例)")
    parser.add_argument("--single", type=str, help="仅测试单条命令")
    args = parser.parse_args()

    # ---- 选择测试用例 ----
    if args.single:
        cases = [(args.single, "自定义")]
    elif args.quick:
        # 快速测试: 取几个代表性用例
        indices = [0, 1, 2, 3, 4, 9, 10, 14, 15, 16]
        cases = [TEST_CASES[i] for i in indices if i < len(TEST_CASES)]
    else:
        cases = list(TEST_CASES)

    # ---- 确定测试类型 ----
    def classify(text):
        if "然后" in text or "再" in text or "再往前" in text or "停一下" in text:
            return "复杂"
        if text in ("前进", "后退", "左转", "右转", "停", "停止", "返回", "回来"):
            return "简单"
        if "米" in text or "度" in text or "秒" in text:
            return "复合"
        if "搜索" in text or "找" in text or "跟着" in text or "跟随" in text:
            return "搜索/跟踪"
        return "边界"

    # ---- Fallback 对照测试 ----
    print("=" * 72)
    print("  📋 Fallback 关键词匹配 — 对照组")
    print("=" * 72)
    for text, desc in cases:
        r = fallback_parse(text)
        tasks = r.get("tasks", [])
        print(f"  {desc}: '{text}'")
        print(f"    → {r['response']} | 任务数={len(tasks)}")
        if tasks:
            for t in tasks:
                print(format_task(t))
        print()

    # ---- VLM 测试 ----
    if args.no_vlm:
        print("\n  (跳过 VLM 测试, --no-vlm 已设置)")
        return

    print("=" * 72)
    print("  🤖 加载 VLM 模型...")
    print("=" * 72)
    t0 = time.time()
    vlm = VLMEngine()
    if not vlm.load():
        print("  ❌ VLM 加载失败，终止测试")
        return
    print(f"  ✅ VLM 加载完成 ({time.time()-t0:.1f}s)")

    print("\n" + "=" * 72)
    print("  🔬 VLM 命令解析测试")
    print("=" * 72)

    results_summary = []
    for text, desc in cases:
        ttype = classify(text)
        print(f"\n{'─'*68}")
        print(f"  [{ttype}] {desc}")
        print(f"  输入: '{text}'")
        print(f"{'─'*68}")

        t_start = time.time()
        result = vlm_parse_command(vlm, text)
        elapsed = time.time() - t_start

        tasks = result.get("tasks", [])
        understanding = result.get("understanding", "?")
        response = result.get("response", "?")

        print(f"  ⏱ 耗时: {elapsed:.1f}s")
        print(f"  💡 理解: {understanding}")
        print(f"  📢 回复: {response}")
        print(f"  📦 任务数: {len(tasks)}")

        for i, t in enumerate(tasks):
            print(f"  #{i+1} {format_task(t)}")

        # 验证
        issues = validate_result(text, result, ttype)
        status = PASS if not issues else WARN if len(issues) <= 1 else FAIL
        if issues:
            for issue in issues:
                print(f"  {status} {issue}")
        else:
            print(f"  {status} 验证通过")

        results_summary.append({
            "text": text, "desc": desc, "type": ttype,
            "tasks": len(tasks), "time": elapsed,
            "issues": issues, "status": status,
            "understanding": understanding, "response": response
        })

    # ---- 汇总 ----
    print("\n" + "=" * 72)
    print("  📊 测试汇总")
    print("=" * 72)

    total = len(results_summary)
    passed = sum(1 for r in results_summary if r["status"] == PASS)
    warned = sum(1 for r in results_summary if r["status"] == WARN)
    failed = sum(1 for r in results_summary if r["status"] == FAIL)
    total_time = sum(r["time"] for r in results_summary)
    total_tasks = sum(r["tasks"] for r in results_summary)

    print(f"  总用例: {total}  |  {PASS}通过: {passed}  |  {WARN}警告: {warned}  |  {FAIL}失败: {failed}")
    print(f"  总耗时: {total_time:.1f}s  |  平均: {total_time/max(total,1):.1f}s/条")
    print(f"  总生成任务: {total_tasks}  |  平均: {total_tasks/max(total,1):.1f}任务/条")

    # 按类型统计
    from collections import Counter
    type_counter = Counter(r["type"] for r in results_summary)
    print(f"\n  按类型:")
    for t in ("简单", "复合", "复杂", "搜索/跟踪", "边界"):
        count = type_counter.get(t, 0)
        type_tasks = sum(r["tasks"] for r in results_summary if r["type"] == t)
        print(f"    {t}: {count}条 → {type_tasks}个任务")

    # 列出失败/警告项
    if warned + failed > 0:
        print(f"\n  需要关注:")
        for r in results_summary:
            if r["status"] in (WARN, FAIL):
                print(f"    {r['status']} [{r['type']}] '{r['text']}': {r['issues']}")

    # 核心结论
    print(f"\n  🎯 核心验证:")
    complex_cases = [r for r in results_summary if r["type"] == "复杂"]
    if complex_cases:
        all_multi = all(r["tasks"] > 1 for r in complex_cases)
        print(f"     复杂命令→多任务队列: {'✅ 正确' if all_multi else '❌ 部分失败'} "
              f"({sum(1 for r in complex_cases if r['tasks']>1)}/{len(complex_cases)})")
    simple_cases = [r for r in results_summary if r["type"] == "简单"]
    if simple_cases:
        print(f"     简单命令→单任务: {'✅ 正确' if all(r['tasks']<=2 for r in simple_cases) else '⚠️ 部分异常'}")

    print(f"\n  ✅ 验证完成!")


if __name__ == "__main__":
    main()