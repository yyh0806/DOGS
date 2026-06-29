#!/bin/bash
# ============================================================
# Go2W 阶段B NX AI 端到端验证 (6 项, 不依赖狗硬件, spec-stage-b §7.1)
# ============================================================
# 前提:
#   - 阶段A verify_nx_web.sh 已 8/8 PASS (本脚本不重复阶段A 验证)
#   - NX 已 source humble, python3 有 rclpy + websockets + numpy
#   - 跑前由调用方用 GO2W_AI_MOCK_VIDEO=1 启动 nx_web_server (mock 视频源)
#     或 NX 真连狗时 VideoClient 失败会自动切 mock (graceful)
#
# SKIP 规则 (spec-stage-b eval-rubric 维度4 验证环境说明):
#   - 无 GPU / 无 torch+ultralytics → YOLO 跑 PT 降级 (慢) 或检测项标 SKIP
#   - 无 GPU 跑 VLM → VLM 解析项 (4) 标 SKIP (走 fallback 仍可验链路)
#   - 无狗硬件 → 自动 mock 视频源 (不影响 1-3, 6)
#
# 用法 (NX 上): GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py & ; sleep 8 ; bash web/verify_nx_ai.sh
# 通过标准: 1-3 + 6 PASS, 4 条件 PASS (VLM), 5 条件 PASS (有 GPU 才验)
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
SKIP=0
TOTAL=6

ok()   { echo "  PASS: $1"; PASS=$((PASS + 1)); }
no()   { echo "  FAIL: $1"; FAIL=$((FAIL + 1)); }
skip() { echo "  SKIP: $1"; SKIP=$((SKIP + 1)); }

# 探测环境能力
HAS_PYTORCH=$(python3 -c "import torch; print('1' if torch.cuda.is_available() else '0')" 2>/dev/null || echo "0")
HAS_ULTRALYTICS=$(python3 -c "import ultralytics; print('1')" 2>/dev/null || echo "0")
HAS_GPU=$(python3 -c "import torch; print('1' if torch.cuda.is_available() else '0')" 2>/dev/null || echo "0")

echo "===== 环境探测: torch.cuda=$HAS_PYTORCH ultralytics=$HAS_ULTRALYTICS GPU=$HAS_GPU ====="
echo ""

# 前置: 确认 web 服务活着
echo "===== 检查 web 服务 (HTTP:$WEB_PORT / WS:$WS_PORT) ====="
if ! curl -s -o /dev/null -w "%{http_code}" "http://$WEB_HOST:$WEB_PORT/" | grep -q 200; then
    echo "  FAIL: web 服务未启动 (先跑: GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py &)"
    echo "  结果: 0/$TOTAL PASS (web 未就绪)"
    exit 1
fi
echo "  web 服务在线"
echo ""

echo "===== 跑 6 项验证 ====="

# 1. WS 收到 type=frame, base64 解码合法 JPEG (spec §7.1 验证项 1, C4.2.1)
FRAME_RESULT=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio, base64
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    try:
        async with websockets.connect(f"ws://{host}:{port}") as ws:
            deadline = asyncio.get_event_loop().time() + 15
            while asyncio.get_event_loop().time() < deadline:
                msg = await asyncio.wait_for(ws.recv(), timeout=15)
                d = json.loads(msg)
                if d.get("type") == "frame":
                    data = d.get("data", "")
                    det = d.get("detections", None)
                    if not data:
                        print("NO_DATA"); return
                    try:
                        raw = base64.b64decode(data)
                        import cv2, numpy as np
                        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                        if img is None:
                            print("BAD_JPEG"); return
                        # detections 必须是整数 (C1.4), 不是 list
                        det_ok = isinstance(det, int)
                        print(f"OK {img.shape[1]}x{img.shape[0]} detections={det} det_is_int={det_ok}")
                        return
                    except Exception as e:
                        print(f"DECODE_ERR:{e}"); return
            print("TIMEOUT_NO_FRAME")
    except Exception as e:
        print(f"ERR:{e}")
asyncio.run(go())
PY
)
if echo "$FRAME_RESULT" | grep -q "^OK"; then
    DET_INT=$(echo "$FRAME_RESULT" | grep -oE "det_is_int=(True|False)" | head -1)
    if echo "$DET_INT" | grep -q "True"; then
        ok "1. WS type=frame, base64→JPEG 合法, $FRAME_RESULT"
    else
        no "1. WS type=frame 但 detections 不是整数 (C1.4 违反): $FRAME_RESULT"
    fi
else
    no "1. WS 未收到合法 type=frame: $FRAME_RESULT"
fi

# 2. WS type=slam 的 data.detections 字段存在且是数组 (C1.5, C4.2.2)
SLAM_RESULT=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    try:
        async with websockets.connect(f"ws://{host}:{port}") as ws:
            deadline = asyncio.get_event_loop().time() + 15
            while asyncio.get_event_loop().time() < deadline:
                msg = await asyncio.wait_for(ws.recv(), timeout=15)
                d = json.loads(msg)
                if d.get("type") == "slam":
                    det = d.get("data", {}).get("detections", "__MISSING__")
                    if det == "__MISSING__":
                        print("NO_FIELD"); return
                    is_list = isinstance(det, list)
                    print(f"OK is_list={is_list} len={len(det)}")
                    return
            print("TIMEOUT_NO_SLAM")
    except Exception as e:
        print(f"ERR:{e}")
asyncio.run(go())
PY
)
if echo "$SLAM_RESULT" | grep -q "^OK is_list=True"; then
    ok "2. WS type=slam.data.detections 是数组 ($SLAM_RESULT)"
else
    no "2. slam.data.detections 缺失或非数组: $SLAM_RESULT"
fi

# 3. POST /api/command → {"ok":true} (C4.2 前半, 链路通; VLM 真实响应在项4 验)
CMD=$(curl -s -X POST "http://$WEB_HOST:$WEB_PORT/api/command?text=%E5%89%8D%E8%BF%9B%E4%B8%A4%E7%B1%B3")
if echo "$CMD" | grep -q '"ok": *true'; then
    ok "3. /api/command?text=前进两米 → ok:true ($CMD)"
else
    no "3. /api/command 响应非 ok: $CMD"
fi

# 4. 30s 内收到 type=vlm 消息, response 非空 (C4.2.3, H4.1)
#    无 GPU 跑不动 VLM → 走 fallback, response 仍非空 (链路可验, VLM 真解析 SKIP)
if [ "$HAS_GPU" = "1" ] && [ "$HAS_ULTRALYTICS" = "1" ]; then
    VLM_TAG="VLM 真解析"
else
    VLM_TAG="VLM 链路 (无 GPU 走 fallback)"
fi
VLM_RESULT=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    try:
        async with websockets.connect(f"ws://{host}:{port}") as ws:
            deadline = asyncio.get_event_loop().time() + 35
            while asyncio.get_event_loop().time() < deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=35)
                except asyncio.TimeoutError:
                    print("TIMEOUT_NO_VLM"); return
                d = json.loads(msg)
                if d.get("type") == "vlm":
                    resp = d.get("data", {}).get("response", "")
                    has_tasks = "tasks" in d.get("data", {})
                    print(f"resp_len={len(resp)} has_tasks={has_tasks} resp={resp[:50]}")
                    return
            print("TIMEOUT_NO_VLM")
    except Exception as e:
        print(f"ERR:{e}")
asyncio.run(go())
PY
)
if echo "$VLM_RESULT" | grep -q "^resp_len=[1-9]"; then
    ok "4. WS type=vlm 收到, response 非空 [$VLM_TAG]: $VLM_RESULT"
else
    no "4. 35s 内未收到有效 type=vlm: $VLM_RESULT"
fi

# 5. VLM 空闲 unload 后显存释放 (C3.2/C3.4, 仅 GPU 环境验, spec §7.1 验证项 5)
if [ "$HAS_GPU" != "1" ]; then
    skip "5. VLM 显存释放 (无 GPU, 跳过 — 部署到 NX 再验)"
else
    # 取两次显存: 加载时 vs 空闲 70s 后 (GO2W_VLM_IDLE=60 + 10s 余量)
    MEM_BEFORE=$(python3 -c "import torch; print(torch.cuda.memory_reserved(0)//(1024*1024))" 2>/dev/null || echo "?")
    echo "  (VLM 空闲卸载等待中, 当前 reserved=${MEM_BEFORE}MB...)"
    sleep 70
    MEM_AFTER=$(python3 -c "import torch; print(torch.cuda.memory_reserved(0)//(1024*1024))" 2>/dev/null || echo "?")
    if [ "$MEM_BEFORE" != "?" ] && [ "$MEM_AFTER" != "?" ] && [ "$MEM_AFTER" -lt "$MEM_BEFORE" ]; then
        ok "5. VLM unload 显存释放: ${MEM_BEFORE}MB → ${MEM_AFTER}MB"
    else
        no "5. VLM 显存未释放 (before=${MEM_BEFORE}MB after=${MEM_AFTER}MB)"
    fi
fi

# 6. POST /api/e_stop → {"ok":true}, 且视频线程仍存活 (后续仍收到 type=frame, C4.2.4)
ESTOP=$(curl -s -X POST "http://$WEB_HOST:$WEB_PORT/api/e_stop")
ESTOP_OK=$(echo "$ESTOP" | grep -c '"ok": *true' 2>/dev/null || echo 0)
# e_stop 后再连 WS, 看 15s 内是否仍有 type=frame (e_stop 不应中断视频线程)
FRAME_AFTER=$(python3 - "$WEB_HOST" "$WS_PORT" <<'PY' 2>/dev/null
import sys, json, asyncio
host, port = sys.argv[1], int(sys.argv[2])
async def go():
    import websockets
    try:
        async with websockets.connect(f"ws://{host}:{port}") as ws:
            deadline = asyncio.get_event_loop().time() + 15
            while asyncio.get_event_loop().time() < deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=15)
                except asyncio.TimeoutError:
                    print("NO_FRAME_AFTER_ESTOP"); return
                d = json.loads(msg)
                if d.get("type") == "frame":
                    print("FRAME_OK"); return
            print("NO_FRAME_AFTER_ESTOP")
    except Exception as e:
        print(f"ERR:{e}")
asyncio.run(go())
PY
)
if [ "$ESTOP_OK" = "1" ] && echo "$FRAME_AFTER" | grep -q "FRAME_OK"; then
    ok "6. /api/e_stop ok 且视频流未中断 (视频线程存活)"
else
    no "6. e_stop=$ESTOP 视频=$FRAME_AFTER (e_stop 不应中断视频)"
fi

echo ""
echo "===== 结果: $PASS PASS, $FAIL FAIL, $SKIP SKIP (共 $TOTAL 项) ====="
echo "  注: SKIP 项不算 FAIL (无 GPU / 无狗环境)"
echo "  web 日志: journalctl -u go2w-web -f (或前台启动的终端)"
[ "$FAIL" = "0" ]
