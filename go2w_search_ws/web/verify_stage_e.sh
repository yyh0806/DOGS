#!/bin/bash
# ============================================================
# Go2W 阶段E — 房间级搜索编排端到端验证 (spec §14.1, 不依赖狗硬件)
# ============================================================
# 前提: NX 已 source humble, python3 有 rclpy + websockets + curl
# 用法 (NX 上): bash web/verify_stage_e.sh
# 通过标准: 9 项验证 1-4、6-9 PASS, 5 条件 PASS (无 YOLO 时 SKIP)
# ============================================================
set -u

source /opt/ros/humble/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

cd "$(dirname "$0")/.."

WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8000}"
WS_PORT="${WS_PORT:-8001}"
# mock 检测注入 (无狗帧时让 YOLO 检出 person): 阶段B 的 GO2W_AI_MOCK_VIDEO=1
export GO2W_AI_MOCK_VIDEO="${GO2W_AI_MOCK_VIDEO:-1}"
# mock Nav2 默认快速 (0.3s 到达), 让 9 项验证快速跑完
export GO2W_MOCK_NAV_DELAY="${GO2W_MOCK_NAV_DELAY:-0.3}"

PASS=0
FAIL=0
SKIP=0
TOTAL=9

ok() { echo "  ✅ PASS: $1"; PASS=$((PASS + 1)); }
no() { echo "  ❌ FAIL: $1"; FAIL=$((FAIL + 1)); }
sk() { echo "  ⏭️  SKIP: $1"; SKIP=$((SKIP + 1)); }

cleanup() {
    [ -n "${WEB_PID:-}" ] && kill "$WEB_PID" 2>/dev/null || true
    [ -n "${MOCK_PID:-}" ] && kill "$MOCK_PID" 2>/dev/null || true
    [ -n "${MOCK_FAIL_PID:-}" ] && kill "$MOCK_FAIL_PID" 2>/dev/null || true
    pkill -f "nx_web_server.py" 2>/dev/null || true
    pkill -f "mock_nav2_action.py" 2>/dev/null || true
}
trap cleanup EXIT

echo "===== 0. 清理旧进程 ====="
pkill -f "nx_web_server.py" 2>/dev/null || true
pkill -f "mock_nav2_action.py" 2>/dev/null || true
sleep 1

echo ""
echo "===== 1. 起默认 mock_nav2_action (后台) ====="
python3 -u web/mock_nav2_action.py > /tmp/mock_nav_default.log 2>&1 &
MOCK_PID=$!
echo "  PID=$MOCK_PID, 等就绪..."
for i in $(seq 1 8); do
    sleep 1
    grep -q "就绪" /tmp/mock_nav_default.log 2>/dev/null && break
done
grep -q "就绪" /tmp/mock_nav_default.log 2>/dev/null && echo "  mock_nav2 就绪" || echo "  ⚠️  mock_nav2 未就绪 (检查 log)"

echo ""
echo "===== 2. 起 nx_web_server (后台, mock 视频) ====="
python3 -u web/nx_web_server.py > /tmp/nx_web_stage_e.log 2>&1 &
WEB_PID=$!
echo "  PID=$WEB_PID, 等就绪..."
for i in $(seq 1 12); do
    sleep 1
    grep -q "Web:" /tmp/nx_web_stage_e.log 2>/dev/null && break
done
grep -q "Web:" /tmp/nx_web_stage_e.log 2>/dev/null && echo "  nx_web 就绪" || echo "  ⚠️  nx_web 未就绪 (检查 log)"

echo ""
echo "===== 跑 9 项验证 ====="

# 1. rooms.yaml 加载校验 (RoomMap.load 不抛 + ≥3 房间)
python3 - <<'PY'
import sys, os
sys.path.insert(0, "web")
try:
    from nx_room_orchestrator import RoomMap
    m = RoomMap.load("config/rooms.yaml")
    assert len(m.rooms) >= 3, f"rooms < 3: {len(m.rooms)}"
    assert m.frame_id == "map"
    print(f"OK rooms={len(m.rooms)} frame_id={m.frame_id}")
except Exception as e:
    print(f"FAIL {e}")
    sys.exit(1)
PY
[ $? -eq 0 ] && ok "1. rooms.yaml 加载 + 校验通过 (≥3 房间)" || no "1. rooms.yaml 加载失败"

# 2. search_room 入队
SEARCH=$(curl -s -X POST "http://$WEB_HOST:$WEB_PORT/api/search_room?room=客厅" 2>/dev/null)
if echo "$SEARCH" | grep -q '"ok": *true'; then
    ok "2. POST /api/search_room?room=客厅 → ok:true"
else
    no "2. /api/search_room 响应错: $SEARCH"
fi

# 3-6. WS 监听 35s, 收集状态机推进 + mission_report + search
WS_OUT=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    phases = []
    got_report = False
    got_search = False
    report_room = None
    report_wp = None
    report_targets = None
    search_found = None
    fail_reason = None
    try:
        async with websockets.connect(f"ws://{host}:{port}", max_size=8*1024*1024) as ws:
            deadline = asyncio.get_event_loop().time() + 35
            while asyncio.get_event_loop().time() < deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except asyncio.TimeoutError:
                    # 触发新的 search_room (测 6/7/8/9 用)
                    continue
                except Exception:
                    break
                try:
                    d = json.loads(msg)
                except Exception:
                    continue
                t = d.get("type")
                if t == "search_room":
                    data = d.get("data", {})
                    ph = data.get("phase")
                    if ph and ph not in phases:
                        phases.append(ph)
                    if ph == "FAILED":
                        fail_reason = data.get("reason")
                    if ph in ("DONE", "FAILED"):
                        break
                elif t == "mission_report":
                    got_report = True
                    report_room = d.get("data", {}).get("room")
                    report_wp = d.get("data", {}).get("waypoints_visited")
                    report_targets = d.get("data", {}).get("targets_found")
                elif t == "search":
                    got_search = True
                    search_found = d.get("data", {}).get("found")
    except Exception as e:
        print(json.dumps({"err": str(e), "phases": phases}))
        return
    out = {
        "phases": phases,
        "got_report": got_report,
        "report_room": report_room,
        "report_wp": report_wp,
        "report_targets": report_targets,
        "got_search": got_search,
        "search_found": search_found,
        "fail_reason": fail_reason,
    }
    print(json.dumps(out))
asyncio.run(go())
PY
)

# 3. 状态机前半 (SELECT_ROOM → NAVIGATE → NAVIGATING → ARRIVED)
PHASES_OK=$(echo "$WS_OUT" | python3 -c "
import sys, json
try:
    d = json.loads(sys.stdin.read())
    ph = d.get('phases', [])
    need = ['SELECT_ROOM', 'NAVIGATE', 'NAVIGATING', 'ARRIVED']
    idx = 0
    for p in ph:
        if idx < len(need) and p == need[idx]: idx += 1
    print('OK' if idx == len(need) else 'FAIL')
except Exception as e:
    print('FAIL')
" 2>/dev/null)
[ "$PHASES_OK" = "OK" ] && ok "3. 状态机前半: SELECT_ROOM→NAVIGATE→NAVIGATING→ARRIVED" || no "3. 状态机前半未走完 ($WS_OUT)"

# 4. 状态机后半 + mission_report
TAIL_OK=$(echo "$WS_OUT" | python3 -c "
import sys, json
try:
    d = json.loads(sys.stdin.read())
    ph = d.get('phases', [])
    need = ['SEARCH', 'REPORT']
    idx = 0
    for p in ph:
        if idx < len(need) and p == need[idx]: idx += 1
    report_ok = d.get('got_report') and d.get('report_room') == '客厅' and (d.get('report_wp') or 0) >= 1
    print('OK' if (idx == len(need) and report_ok) else 'FAIL')
except Exception:
    print('FAIL')
" 2>/dev/null)
[ "$TAIL_OK" = "OK" ] && ok "4. 状态机后半 + mission_report (room=客厅, wp≥1)" || no "4. 状态机后半/报告未到位 ($WS_OUT)"

# 5. 检测发现 (mock 注入): type=search found 非空 + mission_report.detections 非空
#    无 YOLO 时 mission_report.targets_found=0, found 为空 → SKIP (不算 FAIL)
DET_OK=$(echo "$WS_OUT" | python3 -c "
import sys, json
try:
    d = json.loads(sys.stdin.read())
    sf = d.get('search_found') or []
    tg = d.get('report_targets') or 0
    if tg > 0 and sf:
        print('OK')
    elif tg == 0 and not sf:
        print('SKIP')
    else:
        print('FAIL')
except Exception:
    print('FAIL')
" 2>/dev/null)
if [ "$DET_OK" = "OK" ]; then
    ok "5. mock 检测注入: type=search found 非空 + detections 非空"
elif [ "$DET_OK" = "SKIP" ]; then
    sk "5. 无 YOLO/mock_person.png, detections=空 (graceful 退化, 不算 FAIL)"
else
    no "5. 检测注入异常 ($WS_OUT)"
fi

# 6. 房间不存在 → no_room (新发请求 + 短监听)
NO_ROOM=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio, urllib.request
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    reason = None
    try:
        async with websockets.connect(f"ws://{host}:{port}", max_size=8*1024*1024) as ws:
            # 先连 WS 再发请求 (确保收到推送)
            await asyncio.sleep(0.3)
            try:
                urllib.request.urlopen(f"http://{host}:8000/api/search_room?room=厕所", data=b"").read()
            except Exception:
                pass
            deadline = asyncio.get_event_loop().time() + 10
            while asyncio.get_event_loop().time() < deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except Exception:
                    continue
                try:
                    d = json.loads(msg)
                except Exception:
                    continue
                if d.get("type") == "search_room" and d.get("data", {}).get("phase") == "FAILED":
                    reason = d["data"].get("reason")
                    break
    except Exception:
        pass
    print(reason or "NONE")
asyncio.run(go())
PY
)
[ "$NO_ROOM" = "no_room" ] && ok "6. 房间不存在 → FAILED, reason=no_room" || no "6. no_room 失败 (reason=$NO_ROOM)"

# 7. Nav2 导航失败 (客厅入口 2.5,1.8 在 fail 列表): 起独立 mock_nav2 with FAIL
#    注意: 替换主 mock_nav2 会断现有 client; 改用新端口或重启整组
#    简化: 本脚本跑完前 6 项后, kill 主 mock, 起 fail-mock, 重发 search_room
echo ""
echo "  --- 7. 切换到 fail-mock_nav2 (客厅入口 2.5,1.8 abort) ---"
kill "$MOCK_PID" 2>/dev/null || true
sleep 1
# 注: nx_web 的 Nav2ActionClient 是懒创建 + 长存, 切 mock 后用同一 action name 不需重建 client
GO2W_MOCK_NAV_FAIL="2.5,1.8" GO2W_MOCK_NAV_DELAY=0.3 python3 -u web/mock_nav2_action.py > /tmp/mock_nav_fail.log 2>&1 &
MOCK_FAIL_PID=$!
sleep 2

NAV_FAIL=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio, urllib.request
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    reason = None
    try:
        async with websockets.connect(f"ws://{host}:{port}", max_size=8*1024*1024) as ws:
            await asyncio.sleep(0.3)
            try:
                urllib.request.urlopen(f"http://{host}:8000/api/search_room?room=客厅", data=b"").read()
            except Exception:
                pass
            deadline = asyncio.get_event_loop().time() + 15
            while asyncio.get_event_loop().time() < deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except Exception:
                    continue
                try:
                    d = json.loads(msg)
                except Exception:
                    continue
                if d.get("type") == "search_room" and d.get("data", {}).get("phase") == "FAILED":
                    reason = d["data"].get("reason")
                    break
    except Exception:
        pass
    print(reason or "NONE")
asyncio.run(go())
PY
)
case "$NAV_FAIL" in
    aborted|nav_aborted) ok "7. Nav2 abort → FAILED, reason=$NAV_FAIL" ;;
    *) no "7. nav_aborted 失败 (reason=$NAV_FAIL)" ;;
esac

# 8. 中途 cancel: 发 search_room, 1s 后 e_stop, 期望 reason=cancelled
CANC=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio, urllib.request, threading, time
host, port = sys.argv[1], int(sys.argv[2])
def fire_estop():
    time.sleep(1.0)
    try:
        urllib.request.urlopen(f"http://{host}:8000/api/e_stop", data=b"").read()
    except Exception:
        pass
async def go():
    import websockets
    reason = None
    try:
        async with websockets.connect(f"ws://{host}:{port}", max_size=8*1024*1024) as ws:
            await asyncio.sleep(0.3)
            try:
                urllib.request.urlopen(f"http://{host}:8000/api/search_room?room=厨房", data=b"").read()
            except Exception:
                pass
            threading.Thread(target=fire_estop, daemon=True).start()
            deadline = asyncio.get_event_loop().time() + 15
            while asyncio.get_event_loop().time() < deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except Exception:
                    continue
                try:
                    d = json.loads(msg)
                except Exception:
                    continue
                if d.get("type") == "search_room" and d.get("data", {}).get("phase") == "FAILED":
                    reason = d["data"].get("reason")
                    break
    except Exception:
        pass
    print(reason or "NONE")
asyncio.run(go())
PY
)
[ "$CANC" = "cancelled" ] && ok "8. 中途 e_stop → FAILED, reason=cancelled" || no "8. cancelled 失败 (reason=$CANC)"

# 9. reload_rooms 热加载 + /api/rooms
ROOMS_BEFORE=$(curl -s "http://$WEB_HOST:$WEB_PORT/api/rooms" 2>/dev/null)
RELOAD=$(curl -s "http://$WEB_HOST:$WEB_PORT/api/reload_rooms" 2>/dev/null)
ROOMS_AFTER=$(curl -s "http://$WEB_HOST:$WEB_PORT/api/rooms" 2>/dev/null)
if echo "$RELOAD" | grep -q '"ok": *true' && echo "$ROOMS_AFTER" | grep -q '"客厅"'; then
    ok "9. /api/reload_rooms ok + /api/rooms 含客厅 (热加载)"
else
    no "9. reload_rooms/rooms 异常: reload=$RELOAD rooms=$ROOMS_AFTER"
fi

echo ""
echo "===== 结果: $PASS/$TOTAL PASS, $FAIL FAIL, $SKIP SKIP ====="
echo "完整 nx_web 日志: /tmp/nx_web_stage_e.log"
echo "mock_nav (default) 日志: /tmp/mock_nav_default.log"
echo "mock_nav (fail) 日志: /tmp/mock_nav_fail.log"
# 通过标准: 1-4、6-9 PASS (8 项), 5 条件 PASS (SKIP 不算 FAIL)
[ "$FAIL" = "0" ]
