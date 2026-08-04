#!/bin/bash
# ============================================================
# Go2W 阶段B NX AI 部署脚本 — 把 nx_ai_node + verify + mock 资源部署到载荷 NX
# ============================================================
# 前提 (spec-stage-b §7.1):
#   - 阶段A deploy_nx_web.sh 已部署 (go2w-web.service 在跑, panel.html/map.js 不动)
#   - NX 已装 ultralytics + transformers (YOLO/VLM 推理; 暂未装可走 mock)
#   - TensorRT engine 由 NX 本机生成 (yolo export ... format=engine half=True), 不入库
#   - Qwen2.5-VL 模型已下载到 NX 的 models/Qwen/ (GB 级, 不入库)
#   - NX 已连手机热点, SSH 可达
#
# 本脚本做的事 (spec §7.1 deploy_nx_ai.sh 职责):
#   1. 拷贝 nx_ai_node.py + verify_nx_ai.sh + mock_person.png + ai/ 到 NX
#   2. 重启 go2w-web 服务 (加载新的 ai 注入)
#   3. 打印"在 NX 跑 verify_nx_ai.sh 验证"
#
# 用法 (PC 上跑, 从仓库根目录):
#   NX_HOST=192.168.43.41 NX_USER=nx bash docker/deploy_nx_ai.sh
# ============================================================
set -e

NX_HOST="${NX_HOST:-192.168.43.41}"
NX_USER="${NX_USER:-nx}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REMOTE_DIR="~/go2w_ws"

echo "================================================"
echo "  Go2W 阶段B NX AI 部署 → $NX_USER@$NX_HOST"
echo "================================================"

# ---- 1. 连通性检查 ----
echo ""
echo "[1/3] 检查 NX 连通性..."
# 用 ssh 而非 ping 验真 (跨平台 + 代理坑: Windows ping 不认 -c, Mihomo/Tailscale 让 ping 假超时)
if ! ssh -o ConnectTimeout=3 -o StrictHostKeyChecking=no "$NX_USER@$NX_HOST" 'true' >/dev/null 2>&1; then
  echo "FAIL: NX ($NX_HOST) SSH 不通 (ping 假阴性是已知坑, 以 TCP/ssh 为准)"
  exit 1
fi
echo "OK: NX 在线"

# ---- 2. 拷贝阶段B 文件 ----
echo ""
echo "[2/3] 拷贝阶段B AI 代码到 NX:$REMOTE_DIR/ ..."
# ai/ 模块 (nx_ai_node.py import ai.detector/vlm/config; 不改原 ai/, 只增量传)
ssh "$NX_USER@$NX_HOST" "mkdir -p $REMOTE_DIR/web/static $REMOTE_DIR/ai $REMOTE_DIR/docker"
scp -q "$WS_DIR/web/nx_ai_node.py"          "$NX_USER@$NX_HOST:$REMOTE_DIR/web/"
scp -q "$WS_DIR/web/nx_camera_calibration.py" "$NX_USER@$NX_HOST:$REMOTE_DIR/web/"
scp -q "$WS_DIR/web/verify_nx_ai.sh"        "$NX_USER@$NX_HOST:$REMOTE_DIR/web/"
scp -q "$WS_DIR/web/static/mock_person.png" "$NX_USER@$NX_HOST:$REMOTE_DIR/web/static/"
scp -q "$WS_DIR/ai/config.py"               "$NX_USER@$NX_HOST:$REMOTE_DIR/ai/"
scp -q "$WS_DIR/ai/detector.py"             "$NX_USER@$NX_HOST:$REMOTE_DIR/ai/"
scp -q "$WS_DIR/ai/vlm.py"                  "$NX_USER@$NX_HOST:$REMOTE_DIR/ai/"
scp -q "$WS_DIR/ai/locate_anything.py"      "$NX_USER@$NX_HOST:$REMOTE_DIR/ai/"
scp -q "$WS_DIR/ai/tracker.py"              "$NX_USER@$NX_HOST:$REMOTE_DIR/ai/" 2>/dev/null || true
scp -q "$WS_DIR/ai/__init__.py"             "$NX_USER@$NX_HOST:$REMOTE_DIR/ai/" 2>/dev/null || true
echo "OK: nx_ai_node.py + verify_nx_ai.sh + mock_person.png + ai/ 已拷贝"

# ---- 3. 重启 go2w-web 服务 ----
echo ""
echo "[3/3] 重启 go2w-web 服务 (加载 AI 注入)..."
ssh "$NX_USER@$NX_HOST" "echo '$NX_USER' | sudo -S systemctl restart go2w-web.service 2>&1 | tail -1" || true
sleep 4
ACTIVE=$(ssh "$NX_USER@$NX_HOST" 'systemctl is-active go2w-web.service 2>/dev/null' || echo "unknown")
echo "go2w-web.service: $ACTIVE"

# ---- 部署完提示 ----
echo ""
echo "================================================"
echo "  部署完成! ($ACTIVE)"
echo "  PC 浏览器: http://$NX_HOST:8000 (第一视角应见 YOLO 框 + 地图 detections)"
echo ""
echo "  验证 (NX 上):"
echo "    ssh $NX_USER@$NX_HOST"
echo "    cd ~/go2w_ws && GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py &"
echo "    sleep 8 && bash web/verify_nx_ai.sh"
echo ""
echo "  查日志: ssh $NX_USER@$NX_HOST 'journalctl -u go2w-web -f'"
echo ""
echo "  环境变量 (在 go2w-web.service 的 Environment= 里调):"
echo "    GO2W_AI_MOCK_VIDEO=1    强制 mock 视频 (不连狗)"
echo "    GO2W_VLM_IDLE=60        VLM 空闲卸载超时 (秒)"
echo "    DOG_INTERFACE=enxc8a362616c4c  狗网卡"
echo "    GO2W_YOLO_ENGINE=models/yolov8n.engine  TensorRT engine 路径"
echo "    GO2W_VLM_MODEL_NX=/home/nx/models/Qwen/Qwen2___5-VL-3B-Instruct  VLM 路径"
echo "    GO2W_LOCATE_BIN=/home/nx/locate-anything.cpp/build/locate-anything-cli  LocateAnything CLI"
echo "    GO2W_LOCATE_MODEL=/home/nx/models/locate-anything-q8_0.gguf  LocateAnything GGUF"
echo "    GO2W_LOCATE_TIMEOUT=120  LocateAnything inference timeout"
echo "================================================"
