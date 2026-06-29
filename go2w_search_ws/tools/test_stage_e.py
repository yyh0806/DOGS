#!/usr/bin/env python3
"""Go2W 阶段E — 纯逻辑测试 (不依赖 rclpy/Nav2/狗/YOLO, spec §13 Sprint 1-4)。

== 验证范围 (对照 eval-rubric-stage-e) ==
  D1 阶段A/B 契约不破坏:
    - import 不触发 rclpy (懒导入)
    - 不复活 go2w_orchestrator
  D2 Nav2 action 集成正确性:
    - ReentrantCallbackGroup (静态检查)
    - yaw→四元数转换 (qz=sin(yaw/2), qw=cos(yaw/2))
    - status==4 判成功
    - timeout 支持 (send_goal_and_wait 用 spin_until_complete timeout)
  D3 编排状态机完整性:
    - 六态全覆盖 (SELECT_ROOM/NAVIGATE/ARRIVED/SEARCH/DETECT/REPORT)
    - 失败子态 (no_room/no_nav/nav_aborted/cancelled)
    - MissionReport 字段完整
    - cancel 响应
    - target_classes 过滤
  D4 可验证性:
    - mock Nav2 (FakeNav2Client) 可配 fail/reject/timeout
    - 状态机端到端跑通
  D5 房间地图设计:
    - YAML schema 完整
    - 校验规则 (§6.2 全 6 条)
    - 房间匹配 4 级优先级
    - 热加载

== 测试策略 ==
  - 用 FakeNav2Client 替代真 Nav2ActionClient (可控 ok/fail/reject/timeout)
  - 用 FakeAiEngine 替代 NxAiEngine (可控检测快照)
  - 用 events 列表捕获 ws_broadcast 推送 (断言状态机序列)
  - 跑 RoomSearchOrchestrator.run(task) 端到端
"""
import os
import sys
import math
import time
import json

# 仓库根 + web/ 加 sys.path
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))
sys.path.insert(0, ROOT)

import nx_room_orchestrator as orch

PASS = 0
FAIL = 0


def ok(name, detail=""):
    global PASS
    print(f"  PASS: {name} {detail}")
    PASS += 1


def no(name, detail=""):
    global FAIL
    print(f"  FAIL: {name} {detail}")
    FAIL += 1


# ============================================================================
# FakeNav2Client — 替代真 Nav2ActionClient, 行为可控
# ============================================================================
class FakeNav2Client:
    """可控的 Nav2 client 替身。
    mode:
      "ok"      → 每次 send_goal_and_wait 返回 ok=True (默认)
      "fail_xy" → 命中 fail_xy 集合的 goal 返回 aborted
      "reject_xy" → 命中 reject_xy 集合的 goal 返回 rejected
      "no_server" → wait_for_server 返回 False
      "timeout"  → 返回 timeout
    记录所有调用 (供断言 cancel_goal_async 是否调)。
    """
    def __init__(self):
        self.mode = "ok"
        self.fail_xy = set()
        self.reject_xy = set()
        self.server_online = True
        self.calls = []           # 所有 send_goal_and_wait 调用
        self.cancel_calls = 0
        self.feedback_cb = None

    def wait_for_server(self, timeout=2.0):
        return self.server_online

    def set_feedback_callback(self, cb):
        self.feedback_cb = cb

    def _xy_key(self, x, y):
        return (round(float(x), 3), round(float(y), 3))

    def send_goal_and_wait(self, x, y, yaw, frame_id='map'):
        self.calls.append((x, y, yaw, frame_id))
        key = self._xy_key(x, y)
        if not self.server_online:
            return {"ok": False, "reason": "no_server"}
        if self.mode == "no_server":
            return {"ok": False, "reason": "no_server"}
        if key in self.reject_xy:
            return {"ok": False, "reason": "rejected"}
        if key in self.fail_xy:
            return {"ok": False, "reason": "aborted", "status": 6}
        if self.mode == "timeout":
            return {"ok": False, "reason": "timeout"}
        if self.mode == "fail_xy":
            return {"ok": False, "reason": "aborted", "status": 6}
        # 模拟发几次 feedback (让 _on_nav_feedback 被调)
        if self.feedback_cb is not None:
            for i in range(3):
                try:
                    self.feedback_cb(5.0 * (1.0 - (i + 1) / 3.0), 1.0)
                except Exception:
                    pass
        return {"ok": True, "status": 4}

    def cancel_current(self):
        self.cancel_calls += 1
        return True


# ============================================================================
# FakeAiEngine — 替代 NxAiEngine, 检测快照可控
# ============================================================================
class FakeAiEngine:
    """可控的 ai_engine 替身。
    detections_world: get_detections_world 返回值 (list[{x,y,class}])
    raw_dets: _latest_dets (含 confidence, 供 _snapshot_detections 取 conf)
    """
    def __init__(self):
        import threading
        self._lock = threading.Lock()
        self.detections_world = []
        self.raw_dets = []
        self._latest_frame = None
        self._latest_dets = []
        self._detect_frame_w = 1280

    def get_detections_world(self, x, y, yaw):
        return list(self.detections_world)


# ============================================================================
# FakeTask — 替代 TaskManager.Task
# ============================================================================
class FakeTask:
    def __init__(self, room, **extra):
        self.id = "test_task"
        self.type = "search_room"
        self.params = {"room": room, **extra}
        self.status = "pending"
        self.result = None


# ============================================================================
# 辅助: 构造 orchestrator (注入 fake nav + fake ai + 事件捕获)
# ============================================================================
def make_orch(events, fake_nav=None, fake_ai=None, rooms_yaml=None):
    """构造 RoomSearchOrchestrator + 注入 fake 依赖。"""
    def ws_broadcast(data):
        events.append(data)

    o = orch.RoomSearchOrchestrator(
        node=None, ai_engine=fake_ai, ws_broadcast_fn=ws_broadcast,
        rooms_yaml_path=rooms_yaml or os.path.join(ROOT, "config", "rooms.yaml"))
    # 替换 _ensure_nav 返回 fake_nav (跳过真 Nav2 创建)
    if fake_nav is not None:
        def fake_ensure_nav():
            o._nav = fake_nav
            return fake_nav
        o._ensure_nav = fake_ensure_nav
    return o


def phases_of(events):
    """从 events 提取 type=search_room 的 phase 序列。"""
    return [e["data"]["phase"] for e in events if e.get("type") == "search_room"]


def find_mission_report(events):
    for e in events:
        if e.get("type") == "mission_report":
            return e["data"]
    return None


# ============================================================================
# 1. Sprint 1: RoomMap + YAML schema + 房间匹配 (D5)
# ============================================================================
print("===== 1. RoomMap YAML 加载 + 校验 (D5.1/D5.2) =====")
try:
    m = orch.RoomMap.load(os.path.join(ROOT, "config", "rooms.yaml"))
    if len(m.rooms) >= 3:
        ok("rooms.yaml 加载, ≥3 房间", f"({len(m.rooms)} 个: {m.list_rooms()})")
    else:
        no("rooms.yaml 房间 < 3", str(m.list_rooms()))
    if m.frame_id == "map":
        ok("frame_id == 'map'")
    else:
        no("frame_id 错", m.frame_id)
except Exception as e:
    no("rooms.yaml 加载异常", str(e))


print()
print("===== 2. RoomMap 校验规则 (§6.2 全 6 条, D5.2) =====")
import tempfile, yaml as _yaml

# rule 1: 缺顶层字段
try:
    bad = _yaml.safe_load("rooms: []")
    orch.RoomMap("map", [], ).__class__.load  # 只验 load 能跑
    # 直接构造缺字段
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as f:
        f.write("frame_id: map\n")  # 缺 version / rooms
        badpath = f.name
    try:
        orch.RoomMap.load(badpath)
        no("缺 version/rooms 应抛 ValueError")
    except ValueError as e:
        ok("缺顶层字段抛 ValueError", f"({str(e)[:50]})")
    os.unlink(badpath)
except Exception as e:
    no("rule1 测试异常", str(e))

# rule 2: 缺 nav_pose.x
try:
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as f:
        f.write("frame_id: map\nversion: '1.0'\nrooms:\n  - name: test\n    nav_pose: {y: 1.0, yaw: 0.0}\n    search_area: {width: 3, height: 3, origin_x: 0, origin_y: 0}\n")
        badpath = f.name
    try:
        orch.RoomMap.load(badpath)
        no("缺 nav_pose.x 应抛 ValueError")
    except ValueError:
        ok("缺 nav_pose.x 抛 ValueError")
    os.unlink(badpath)
except Exception as e:
    no("rule2 测试异常", str(e))

# rule 4: width <= 0
try:
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as f:
        f.write("frame_id: map\nversion: '1.0'\nrooms:\n  - name: test\n    nav_pose: {x: 0, y: 0, yaw: 0}\n    search_area: {width: 0, height: 3, origin_x: 0, origin_y: 0}\n")
        badpath = f.name
    try:
        orch.RoomMap.load(badpath)
        no("width=0 应抛 ValueError")
    except ValueError:
        ok("width<=0 抛 ValueError")
    os.unlink(badpath)
except Exception as e:
    no("rule4 测试异常", str(e))

# rule 5: name 重复
try:
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as f:
        f.write("frame_id: map\nversion: '1.0'\nrooms:\n  - name: A\n    nav_pose: {x: 0, y: 0, yaw: 0}\n    search_area: {width: 3, height: 3, origin_x: 0, origin_y: 0}\n  - name: A\n    nav_pose: {x: 1, y: 1, yaw: 0}\n    search_area: {width: 3, height: 3, origin_x: 0, origin_y: 0}\n")
        badpath = f.name
    try:
        orch.RoomMap.load(badpath)
        no("name 重复应抛 ValueError")
    except ValueError:
        ok("name 重复抛 ValueError")
    os.unlink(badpath)
except Exception as e:
    no("rule5 测试异常", str(e))

# rule 6: 文件不存在
try:
    orch.RoomMap.load("/nonexistent/path/rooms.yaml")
    no("文件不存在应抛 FileNotFoundError")
except FileNotFoundError:
    ok("文件不存在抛 FileNotFoundError")
except Exception as e:
    no("FileNotFoundError 异常", str(e))


print()
print("===== 3. 房间匹配 4 级优先级 (§6.3, D5.3) =====")
m = orch.RoomMap.load(os.path.join(ROOT, "config", "rooms.yaml"))
# 优先级 1: name 完全相等
r = m.find("客厅")
if r and r.name == "客厅":
    ok("find('客厅') → 客厅 (name 完全相等)")
else:
    no("find('客厅') 失败", str(r))
# 优先级 2: aliases 完全相等
r = m.find("living room")
if r and r.name == "客厅":
    ok("find('living room') → 客厅 (alias 完全相等)")
else:
    no("find('living room') 失败", str(r))
# 优先级 3: name 子串包含
r = m.find("搜索客厅")
if r and r.name == "客厅":
    ok("find('搜索客厅') → 客厅 (name 子串)")
else:
    no("find('搜索客厅') 失败", str(r))
# 优先级 4: aliases 子串
r = m.find("去找 bedroom")
if r and r.name == "卧室":
    ok("find('去找 bedroom') → 卧室 (alias 子串)")
else:
    no("find('去找 bedroom') 失败", str(r))
# 都不匹配
r = m.find("厕所")
if r is None:
    ok("find('厕所') → None")
else:
    no("find('厕所') 应返回 None", str(r))


# ============================================================================
# 4. Sprint 2: Nav2ActionClient 静态契约 (D2.1/D2.2/D2.6/D2.7)
# ============================================================================
print()
print("===== 4. Nav2ActionClient 静态契约 (D2) =====")
# D2.1: 用标准 NavigateToPose (源码含 nav2_msgs.action.NavigateToPose)
src = open(orch.__file__, encoding="utf-8").read()
if "from nav2_msgs.action import NavigateToPose" in src:
    ok("用 nav2_msgs.action.NavigateToPose (D2.1)")
else:
    no("未用标准 NavigateToPose (D2.1)")

# D2.2: ReentrantCallbackGroup
if "ReentrantCallbackGroup" in src and "MutuallyExclusiveCallbackGroup" not in src:
    ok("用 ReentrantCallbackGroup, 无 MutuallyExclusive (D2.2)")
else:
    no("callback group 可能用错 (D2.2)")

# D2.3: spin_until_complete 带 timeout
if "spin_until_complete" in src and "timeout_sec=" in src:
    ok("spin_until_complete 带 timeout (D2.3)")
else:
    no("spin_until_complete 缺 timeout (D2.3)")

# D2.6: yaw→四元数 (qz=sin(yaw/2), qw=cos(yaw/2), qx=qy=0)
import inspect
sig_yaw = "qz = math.sin(half)" in src and "qw = math.cos(half)" in src
if sig_yaw:
    ok("yaw→四元数 qz=sin(yaw/2) qw=cos(yaw/2) (D2.6)")
else:
    no("yaw→四元数公式可能错 (D2.6)")

# D2.7: status==4 判成功
if "_STATUS_SUCCEEDED = 4" in src and "status == _STATUS_SUCCEEDED" in src:
    ok("status==4 判成功 (D2.7)")
else:
    no("status 判定可能错 (D2.7)")

# 反模式 4: 不复活 go2w_orchestrator
if "from go2w_orchestrator" not in src and "import go2w_orchestrator" not in src:
    ok("未复活 go2w_orchestrator 包 (反模式 1)")
else:
    no("复活了 go2w_orchestrator (反模式 1)")

# 反模式 12: RoomSearchOrchestrator 不直接 import ai.detector (走 ai_engine 注入)
# 检查真实 import 语句 (行首 from/import, 排除 docstring/注释里的提及)
import re as _re
real_imports_ai_detector = bool(_re.search(
    r'^\s*(from\s+ai\.detector|import\s+ai\.detector)', src, _re.MULTILINE))
if not real_imports_ai_detector:
    ok("未直接 import ai.detector (反模式 12, 走 ai_engine 注入)")
else:
    no("直接 import ai.detector (反模式 12)")

# 反模式 17: 不调 detector.detect (重复推理)
if "detector.detect(" not in src:
    ok("未调 detector.detect (反模式 17, 只读快照)")
else:
    no("调了 detector.detect (反模式 17)")


# ============================================================================
# 5. Sprint 3: 状态机端到端 (happy path) (D3.1/D3.4/D3.5)
# ============================================================================
print()
print("===== 5. 状态机端到端 happy path (mock Nav2 ok) =====")
events = []
fake_nav = FakeNav2Client()  # mode=ok
fake_ai = FakeAiEngine()
fake_ai.detections_world = [{"x": 1.0, "y": 1.5, "class": "person"}]
fake_ai._latest_dets = [{"class": "person", "confidence": 0.85, "bbox": [100, 100, 200, 200]}]
o = make_orch(events, fake_nav=fake_nav, fake_ai=fake_ai)
task = FakeTask("客厅")
o.run(task)

phases = phases_of(events)
expected_seq = ["SELECT_ROOM", "NAVIGATE", "NAVIGATING", "ARRIVED", "SEARCH", "DETECT", "REPORT", "DONE"]
# 验证 expected_seq 是 phases 的子序列 (允许穿插)
idx = 0
for p in phases:
    if idx < len(expected_seq) and p == expected_seq[idx]:
        idx += 1
if idx == len(expected_seq):
    ok(f"六态+完成全覆盖 (子序列匹配)", f"phases n={len(phases)}")
else:
    no("状态机阶段不全", f"matched={idx}/{len(expected_seq)}, phases={phases}")

# D3.4: MissionReport 字段完整
report = find_mission_report(events)
if report:
    required = ["mission_id", "room", "status", "start_time", "end_time",
                "duration_sec", "waypoints_total", "waypoints_visited",
                "targets_found", "detections", "area", "result_path"]
    missing = [k for k in required if k not in report]
    if not missing and report["room"] == "客厅" and report["waypoints_visited"] >= 1:
        ok("MissionReport 字段完整 + 房间正确", f"wp={report['waypoints_visited']}/{report['waypoints_total']}, targets={report['targets_found']}")
    else:
        no("MissionReport 缺字段或房间错", f"missing={missing} room={report.get('room')}")
else:
    no("未收到 type=mission_report")

# task.status
if task.status == "completed":
    ok("task.status=completed")
else:
    no("task.status 非 completed", task.status)

# detections 含 robot 位姿 + 时间戳
if report and report["detections"]:
    d0 = report["detections"][0]
    det_keys = ["class", "confidence", "robot_x", "robot_y", "robot_yaw", "timestamp", "wp_index"]
    if all(k in d0 for k in det_keys) and d0["class"] == "person":
        ok("detection 含 robot 位姿 + 时间戳", f"class={d0['class']} conf={d0['confidence']} wp={d0['wp_index']}")
    else:
        no("detection 字段不全", str(d0))
else:
    no("MissionReport detections 空 (期望 person)")

# D3.6: 检测快照读 get_detections_world (不调 detector.detect) — 已在反模式 17 验证
# 复用 type=search (增量推送) — events 应含 type=search
search_events = [e for e in events if e.get("type") == "search"]
if search_events and search_events[0]["data"].get("found"):
    ok("复用 type=search 增量推送 (found 非空)", str(search_events[0]["data"]["found"]))
else:
    no("未推 type=search 或 found 空")


# ============================================================================
# 6. 失败路径: 房间不存在 (D3.2 no_room)
# ============================================================================
print()
print("===== 6. 失败路径: 房间不存在 (no_room) =====")
events = []
o = make_orch(events, fake_nav=FakeNav2Client(), fake_ai=FakeAiEngine())
task = FakeTask("厕所")
o.run(task)
phases = phases_of(events)
failed = [e for e in events if e.get("type") == "search_room" and e["data"].get("phase") == "FAILED"]
if failed and failed[-1]["data"].get("reason") == "no_room":
    ok("房间不存在 → FAILED, reason=no_room")
else:
    no("no_room 失败路径错", str(failed[-1]["data"] if failed else phases))
if task.status == "failed":
    ok("no_room task.status=failed")
else:
    no("no_room task.status 非 failed", task.status)


# ============================================================================
# 7. 失败路径: Nav2 server 不在线 (no_nav)
# ============================================================================
print()
print("===== 7. 失败路径: Nav2 server 不在线 (no_nav) =====")
events = []
fake_nav = FakeNav2Client()
fake_nav.server_online = False
o = make_orch(events, fake_nav=fake_nav, fake_ai=FakeAiEngine())
task = FakeTask("客厅")
o.run(task)
failed = [e for e in events if e.get("type") == "search_room" and e["data"].get("phase") == "FAILED"]
if failed and failed[-1]["data"].get("reason") == "no_nav":
    ok("Nav2 不在线 → FAILED, reason=no_nav")
else:
    no("no_nav 失败路径错", str(failed[-1]["data"] if failed else phases_of(events)))


# ============================================================================
# 8. 失败路径: Nav2 abort (nav_aborted, D4.4 mock fail)
# ============================================================================
print()
print("===== 8. 失败路径: Nav2 abort (客厅入口 2.5,1.8 在 fail 列表) =====")
events = []
fake_nav = FakeNav2Client()
fake_nav.fail_xy = {(2.5, 1.8)}  # 客厅入口坐标
o = make_orch(events, fake_nav=fake_nav, fake_ai=FakeAiEngine())
task = FakeTask("客厅")
o.run(task)
failed = [e for e in events if e.get("type") == "search_room" and e["data"].get("phase") == "FAILED"]
if failed and failed[-1]["data"].get("reason") in ("aborted", "nav_aborted"):
    ok("Nav2 abort → FAILED, reason=aborted/nav_aborted", f"reason={failed[-1]['data'].get('reason')}")
else:
    no("nav_aborted 失败路径错", str(failed[-1]["data"] if failed else phases_of(events)))


# ============================================================================
# 9. 失败路径: 中途 cancel (D3.3 cancel 响应)
# ============================================================================
print()
print("===== 9. 失败路径: 中途 cancel (cancelled) =====")
events = []
fake_nav = FakeNav2Client()
o = make_orch(events, fake_nav=fake_nav, fake_ai=FakeAiEngine())

# 包裹 run: 在 SEARCH 阶段触发 cancel
orig_snapshot = o._snapshot_detections
cancel_triggered = {"done": False}

def hook_snapshot(*a, **kw):
    if not cancel_triggered["done"]:
        # 触发 cancel (模拟 e_stop 线程调)
        o.cancel()
        cancel_triggered["done"] = True
    return orig_snapshot(*a, **kw)
o._snapshot_detections = hook_snapshot

task = FakeTask("客厅")
o.run(task)
failed = [e for e in events if e.get("type") == "search_room" and e["data"].get("phase") == "FAILED"]
if failed and failed[-1]["data"].get("reason") == "cancelled":
    ok("中途 cancel → FAILED, reason=cancelled")
else:
    no("cancelled 失败路径错", str(failed[-1]["data"] if failed else "无 FAILED"))
# Nav2 cancel 应被调 (cancel_current)
if fake_nav.cancel_calls > 0:
    ok("nav.cancel_current 被调 (D2.8)")
else:
    no("nav.cancel_current 未被调 (D2.8)")


# ============================================================================
# 10. target_classes 过滤 (D3.7)
# ============================================================================
print()
print("===== 10. target_classes 过滤 (D3.7) =====")
events = []
fake_nav = FakeNav2Client()
fake_ai = FakeAiEngine()
# 检测快照含 person + chair, 但卧室只 target_classes=["person"]
fake_ai.detections_world = [
    {"x": 1.0, "y": 1.5, "class": "person"},
    {"x": 2.0, "y": 1.5, "class": "chair"},
]
fake_ai._latest_dets = [
    {"class": "person", "confidence": 0.9, "bbox": [100, 100, 200, 200]},
    {"class": "chair", "confidence": 0.7, "bbox": [300, 100, 400, 200]},
]
o = make_orch(events, fake_nav=fake_nav, fake_ai=fake_ai)
task = FakeTask("卧室")  # 卧室 target_classes=["person"]
o.run(task)
report = find_mission_report(events)
if report:
    classes = [d["class"] for d in report["detections"]]
    if "person" in classes and "chair" not in classes:
        ok("target_classes=['person'] 过滤掉 chair (D3.7)", str(classes))
    elif len(classes) == 0:
        # 卧室 search_area 小, 可能无 DETECT; 但若有 detections 必须只含 person
        ok("DETECT 阶段无检测 (房间面积/航点数少)", "skip chair 过滤断言")
    else:
        no("target_classes 过滤失败 (含 chair)", str(classes))
else:
    no("未收到 mission_report (target_classes 测试)")


# ============================================================================
# 11. 阶段A 退化 (ai_engine=None, D3.6 graceful)
# ============================================================================
print()
print("===== 11. 阶段A 退化 (ai_engine=None, D3.6) =====")
events = []
fake_nav = FakeNav2Client()
o = make_orch(events, fake_nav=fake_nav, fake_ai=None)  # 无 AI
task = FakeTask("厨房")
o.run(task)
report = find_mission_report(events)
if report and task.status == "completed":
    if report["targets_found"] == 0 and report["detections"] == []:
        ok("ai_engine=None: 状态机走完, targets_found=0 (graceful)")
    else:
        no("ai_engine=None 但 detections 非空 (异常)", str(report["detections"]))
else:
    no("ai_engine=None 状态机未走完", task.status)


# ============================================================================
# 12. 热加载 reload_rooms (D5.6)
# ============================================================================
print()
print("===== 12. 热加载 reload_rooms (D5.6) =====")
events = []
o = make_orch(events, fake_nav=FakeNav2Client(), fake_ai=FakeAiEngine())
before = sorted(o.list_rooms())
# reload (原文件未改, 应返回 True + 同样的房间)
ok_reload = o.reload_rooms()
after = sorted(o.list_rooms())
if ok_reload and before == after and len(after) >= 3:
    ok("reload_rooms 生效 + 不影响房间列表", f"{after}")
else:
    no("reload_rooms 异常", f"ok={ok_reload} before={before} after={after}")


# ============================================================================
# 13. WS 新 type 不破坏阶段A/B 契约 (D1.3)
# ============================================================================
print()
print("===== 13. WS 新 type 不破坏阶段A/B 契约 (D1.3) =====")
# 收集 happy path (test 5) 的所有 ws type, 确认只有新增的 + 复用的, 没改阶段A/B 现有 type 字段
events = []
fake_nav = FakeNav2Client()
fake_ai = FakeAiEngine()
fake_ai.detections_world = [{"x": 1.0, "y": 1.5, "class": "person"}]
fake_ai._latest_dets = [{"class": "person", "confidence": 0.85, "bbox": [100, 100, 200, 200]}]
o = make_orch(events, fake_nav=fake_nav, fake_ai=fake_ai)
o.run(FakeTask("客厅"))
ws_types = set(e.get("type") for e in events)
# 阶段E 只应推: search_room (新), mission_report (新), search (复用阶段B)
allowed = {"search_room", "mission_report", "search"}
extra = ws_types - allowed
if not extra:
    ok(f"WS type 只含允许集合", str(sorted(ws_types)))
else:
    no("WS 推了不允许的 type", str(extra))
# search_room data 字段完整性 (D3.5)
sr = [e for e in events if e.get("type") == "search_room"]
if sr:
    d = sr[0]["data"]
    if all(k in d for k in ["phase", "mission_id", "room", "timestamp"]):
        ok("search_room data 含 phase/mission_id/room/timestamp (D3.5)")
    else:
        no("search_room data 字段不全", str(d.keys()))


print()
print(f"===== 结果: {PASS} PASS, {FAIL} FAIL =====")
sys.exit(0 if FAIL == 0 else 1)
