"""SSE 端到端验证: 触发真实任务并消费实时事件流 (30s 观察窗)。"""
import json
import time
import urllib.request

TASK = "干跑预演：绕湖巡查。查电量，规划绕湖环线，布防守卫，扫描湖面，生成任务报告。"

req = urllib.request.Request(
    "http://127.0.0.1:8088/api/run",
    data=json.dumps({"task": TASK}).encode(),
    headers={"Content-Type": "application/json"}, method="POST")
print("POST /api/run:", json.loads(urllib.request.urlopen(req, timeout=5).read()))

events = []
with urllib.request.urlopen("http://127.0.0.1:8088/events", timeout=60) as resp:
    buffer = ""
    start = time.time()
    while time.time() - start < 30 and not any(
            e.get("event") == "done" for e in events):
        chunk = resp.read(512).decode("utf-8", errors="ignore")
        if not chunk:
            break
        buffer += chunk
        while "\n\n" in buffer:
            block, buffer = buffer.split("\n\n", 1)
            lines = [l for l in block.split("\n") if l]
            event = {"event": lines[0].split(": ", 1)[1]
                     if lines[0].startswith("event: ") else "message"}
            for line in lines:
                if line.startswith("data: "):
                    event["data"] = json.loads(line[6:])
            events.append(event)

kinds = {}
plans = alerts = done = 0
for event in events:
    if event["event"] == "log":
        kinds[event["data"].get("kind")] = kinds.get(
            event["data"].get("kind"), 0) + 1
    elif event["event"] == "plan":
        plans += 1
        print("  [plan 事件] 环线 %d 航点 / 扫描点 %d"
              % (len(event["data"]["waypoints"]),
                 len(event["data"]["scan_points"])))
    elif event["event"] == "done":
        done += 1
print("实时事件流:", kinds)
print("plan 事件:", plans, "| done 事件:", done)
print("LLM 答复开头:", next((e["data"]["answer"][:90] for e in events
                              if e["event"] == "done"), "(未在窗口内完成)"))
assert plans >= 1, "未收到规划几何事件"
assert done >= 1, "任务未在 30s 内完成"
assert kinds.get("tool_call", 0) >= 5, "工具调用事件不足"
print("SSE 端到端验证: PASS")
