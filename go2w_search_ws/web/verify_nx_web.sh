#!/bin/bash
# ============================================================
# Go2W NX Web 端到端验证 (8 项 PASS/FAIL, 不依赖狗硬件)
# ============================================================
# 前提: NX 已 source humble, python3 有 rclpy + websockets + curl
# 用法 (NX 上): bash web/verify_nx_web.sh
# 通过标准: 8/8 PASS
# ============================================================
set -u

source /opt/ros/humble/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

cd "$(dirname "$0")/.."

WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8000}"
WS_PORT="${WS_PORT:-8001}"
PASS=0
FAIL=0
TOTAL=8

ok() { echo "  ✅ PASS: $1"; PASS=$((PASS + 1)); }
no() { echo "  ❌ FAIL: $1"; FAIL=$((FAIL + 1)); }

echo "===== 启动 nx_web_server (后台) ====="
pkill -f "nx_web_server.py" 2>/dev/null || true
sleep 1
python3 -u web/nx_web_server.py > /tmp/nx_web_verify.log 2>&1 &
WEB_PID=$!
echo "  PID=$WEB_PID, 等待就绪..."
for i in $(seq 1 10); do
    sleep 1
    grep -q "Web:" /tmp/nx_web_verify.log 2>/dev/null && break
done

echo ""
echo "===== 启动 mock_dog_state_publisher (后台, 给订阅链路喂数据) ====="
pkill -f "mock_dog_state_publisher.py" 2>/dev/null || true
sleep 1
python3 -u web/mock_dog_state_publisher.py > /tmp/mock_verify.log 2>&1 &
MOCK_PID=$!
sleep 3

cleanup() {
    kill "$WEB_PID" 2>/dev/null || true
    kill "$MOCK_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo ""
echo "===== 跑 8 项验证 ====="

# 1. GET / → panel.html
TITLE=$(curl -s "http://$WEB_HOST:$WEB_PORT/" | grep -o "<title>[^<]*</title>" | head -1)
if [ -n "$TITLE" ]; then ok "1. GET / 含 <title>: $TITLE"; else no "1. GET / 无 <title>"; fi

# 2. GET /map.js → Go2WMap class
MAPJS=$(curl -s "http://$WEB_HOST:$WEB_PORT/map.js" | grep -c "class Go2WMap")
if [ "$MAPJS" -ge 1 ] 2>/dev/null; then ok "2. GET /map.js 含 class Go2WMap"; else no "2. GET /map.js 缺 class Go2WMap"; fi

# 3. GET /api/status → JSON 含 connected
STATUS=$(curl -s "http://$WEB_HOST:$WEB_PORT/api/status")
if echo "$STATUS" | grep -q '"connected"'; then ok "3. /api/status 含 connected 字段"; else no "3. /api/status 缺 connected 字段"; fi

# 4. POST /api/move?vx=0.3 → {"ok":true} 且 /cmd_vel linear.x≈0.3
MOVE=$(curl -s -X POST "http://$WEB_HOST:$WEB_PORT/api/move?vx=0.3&vy=0&vyaw=0")
if echo "$MOVE" | grep -q '"ok": *true'; then
    CMDVEL=$(timeout 3 ros2 topic echo /cmd_vel --once 2>/dev/null | grep -A1 "linear:" | grep "x:" | head -1 | grep -oE "[-0-9.]+")
    if echo "$CMDVEL" | grep -qE "^0\.3"; then
        ok "4. /api/move → /cmd_vel linear.x=$CMDVEL (≈0.3)"
    else
        no "4. /api/move ok 但 /cmd_vel linear.x=$CMDVEL (期望 0.3)"
    fi
else
    no "4. /api/move 响应非 ok: $MOVE"
fi

# 5. POST /api/stand → {"ok":true} 且 /cmd_pose data=stand
STAND=$(curl -s -X POST "http://$WEB_HOST:$WEB_PORT/api/stand")
if echo "$STAND" | grep -q '"ok": *true'; then
    CMDPOSE=$(timeout 3 ros2 topic echo /cmd_pose --once 2>/dev/null | grep "data:" | head -1 | tr -d '"' | grep -oE "stand|sit|estop")
    if [ "$CMDPOSE" = "stand" ]; then
        ok "5. /api/stand → /cmd_pose data=stand"
    else
        no "5. /api/stand ok 但 /cmd_pose data='$CMDPOSE' (期望 stand)"
    fi
else
    no "5. /api/stand 响应非 ok: $STAND"
fi

# 6. WS 收到 type=slam 且 slam_source=ros2_nx (5s 内)
SLAM=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    try:
        async with websockets.connect(f"ws://{host}:{port}") as ws:
            deadline = asyncio.get_event_loop().time() + 5
            while asyncio.get_event_loop().time() < deadline:
                msg = await asyncio.wait_for(ws.recv(), timeout=5)
                d = json.loads(msg)
                if d.get("type") == "slam":
                    src = d.get("data", {}).get("slam_source", "")
                    print(src)
                    return
    except Exception as e:
        print(f"ERR:{e}", file=sys.stderr)
asyncio.run(go())
PY
)
if [ "$SLAM" = "ros2_nx" ]; then ok "6. WS 收到 type=slam, slam_source=ros2_nx"; else no "6. WS slam 未收到或 slam_source='$SLAM' (期望 ros2_nx)"; fi

# 7. WS 收到 type=status 含 dog_state 字段 (5s 内)
DOGSTATE=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    try:
        async with websockets.connect(f"ws://{host}:{port}") as ws:
            deadline = asyncio.get_event_loop().time() + 5
            while asyncio.get_event_loop().time() < deadline:
                msg = await asyncio.wait_for(ws.recv(), timeout=5)
                d = json.loads(msg)
                if d.get("type") == "status" and "dog_state" in d:
                    print(d.get("dog_state", ""))
                    return
    except Exception as e:
        print(f"ERR:{e}", file=sys.stderr)
asyncio.run(go())
PY
)
if [ -n "$DOGSTATE" ]; then ok "7. WS 收到 type=status 含 dog_state='$DOGSTATE'"; else no "7. WS type=status 缺 dog_state"; fi

# 8. mock 起后 dog_state 从 UNKNOWN → 非 UNKNOWN (证明订阅链路通)
#    (mock 已在第 7 步喂了 3+ 秒, status 里 dog_state 应已是 MOVING)
if [ -n "$DOGSTATE" ] && [ "$DOGSTATE" != "UNKNOWN" ]; then
    ok "8. dog_state 从 UNKNOWN → $DOGSTATE (订阅链路通)"
else
    no "8. dog_state 仍 UNKNOWN (订阅 /dog_state 未生效)"
fi

echo ""
echo "===== 结果: $PASS/$TOTAL PASS, $FAIL FAIL ====="
echo "完整 web 日志: /tmp/nx_web_verify.log"
echo "完整 mock 日志: /tmp/mock_verify.log"
[ "$FAIL" = "0" ]
