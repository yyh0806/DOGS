#!/usr/bin/env python3
"""端到端: VLM(few-shot prompt) → 任务解析 → 搜索路径生成"""
import sys, json, re, time
sys.path.insert(0, '.')
from ai.vlm import VLMEngine
from web.panel import plan_lawnmower, _wp_to_moves

vlm = VLMEngine(); vlm.load()
print("VLM 就绪\n")

SYS_PROMPT = """你是机器狗指令解析器。把用户中文指令转成JSON任务列表。

任务类型和参数:
- move: {"vx":前进速度m/s, "vy":侧移, "vyaw":旋转(正=左转), "duration":秒}
- follow: {"target":"目标"}
- search_area: {"pattern":"lawnmower", "width":米, "height":米}
- stop: {}
- return_home: {}

示例:
输入"前进两米然后左转"
输出: {"tasks":[{"type":"move","priority":8,"params":{"vx":0.5,"duration":4.0}},{"type":"move","priority":7,"params":{"vyaw":0.5,"duration":3.0}}]}

输入"搜索这个房间"
输出: {"tasks":[{"type":"search_area","priority":5,"params":{"pattern":"lawnmower","width":8,"height":8}}]}

输入"跟着前面的人"
输出: {"tasks":[{"type":"follow","priority":8,"params":{"target":"前面的人"}}]}

只输出JSON, 不要解释, 不要markdown代码块。"""

def parse_json(resp):
    clean = re.sub(r"```(?:json)?\s*", "", resp)
    clean = re.sub(r"```\s*$", "", clean)
    clean = re.sub(r"//[^\n]*", "", clean)
    m = re.search(r"\{", clean)
    if not m:
        return None, "未找到JSON"
    start = m.start(); depth = 0; end = start
    for i, ch in enumerate(clean[start:], start):
        if ch == "{": depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0: end = i + 1; break
    try:
        data = json.loads(clean[start:end])
        if "tasks" in data:
            return data, None
        return None, "无tasks字段"
    except Exception as e:
        return None, str(e)

tests = [
    ("前进", "move", 1),
    ("停", "stop", 1),
    ("左转90度", "move", 1),
    ("搜索这片区域", "search_area", 1),
    ("跟着前面的人", "follow", 1),
    ("回来", "return_home", 1),
    ("前进两米然后左转", "move", 2),  # 期望2个move
]

results = []
for text, expected_type, min_tasks in tests:
    t0 = time.time()
    resp = vlm.chat([
        {"role": "system", "content": SYS_PROMPT},
        {"role": "user", "content": f'输入"{text}"'}
    ], max_new_tokens=512)
    elapsed = time.time() - t0

    data, err = parse_json(resp)
    if data:
        tasks = data["tasks"]
        types = [t["type"] for t in tasks]
        match = expected_type in types and len(tasks) >= min_tasks
        status = "✅" if match else "⚠️"
        print(f'{status} [{elapsed:.1f}s] "{text}" -> {types}')
        for t in tasks:
            print(f'    {t["type"]}: {json.dumps(t.get("params", {}), ensure_ascii=False)}')
        results.append(match)
    else:
        print(f'❌ [{elapsed:.1f}s] "{text}" -> {err}\n  raw: {resp[:120]}')
        results.append(False)

print(f'\n{"="*50}')
print(f'VLM解析: {sum(results)}/{len(results)} 通过')

# 搜索路径生成
print(f'\n--- 搜索路径生成 ---')
for w, h, s in [(5, 5, 2.0), (10, 10, 2.5), (3, 3, 1.5)]:
    wp = plan_lawnmower(w, h, spacing=s)
    moves = _wp_to_moves(wp, speed=0.3)
    print(f'  {w}x{h}m: {len(wp)}航点 → {len(moves)}个move')
print("✅ 路径生成正常")