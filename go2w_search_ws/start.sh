#!/bin/bash
# Go2W 搜索系统启动脚本
# 用法: ./start.sh [选项]
#
# 普通启动（连接狗）:
#   ./start.sh
#
# 不连接狗（只启动 Web 服务）:
#   ./start.sh --no-connect
#
# 指定网卡和模型:
#   ./start.sh --interface enp65s0 --model yolov8n.pt

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 默认参数
INTERFACE="${GO2W_INTERFACE:-enp65s0}"
MODEL="${GO2W_MODEL:-}"
HOST="${GO2W_HOST:-0.0.0.0}"
PORT="${GO2W_PORT:-8000}"
EXTRA_ARGS=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --interface) INTERFACE="$2"; shift 2 ;;
        --model) MODEL="$2"; shift 2 ;;
        --host) HOST="$2"; shift 2 ;;
        --port) PORT="$2"; shift 2 ;;
        --no-connect) EXTRA_ARGS="$EXTRA_ARGS --no-connect"; shift ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 查找 YOLO 模型
if [ -z "$MODEL" ]; then
    for candidate in "$SCRIPT_DIR/yolov8n.pt" "$HOME/yolov8n.pt"; do
        if [ -f "$candidate" ]; then
            MODEL="$candidate"
            break
        fi
    done
fi

echo "=========================================="
echo "  Go2W 搜索系统"
echo "=========================================="
echo "  网卡:  $INTERFACE"
echo "  模型:  ${MODEL:-未指定}"
echo "  地址:  http://$HOST:$PORT"
echo "  GPU:   $(python3 -c 'import torch; print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")' 2>/dev/null || echo '未检测')"
echo "=========================================="

MODEL_ARG=""
if [ -n "$MODEL" ]; then
    MODEL_ARG="--model $MODEL"
fi

exec python3 web/server.py \
    --interface "$INTERFACE" \
    --host "$HOST" \
    --port "$PORT" \
    $MODEL_ARG \
    $EXTRA_ARGS
