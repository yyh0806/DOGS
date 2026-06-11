#!/bin/bash
# Go2W 一键搜索脚本
# 用法: ./go.sh [宽度] [高度] [目标类别]
# 示例: ./go.sh 4 4 person
#
# 脚本会自动:
#   1. 配置有线网卡连接 Go2W
#   2. 执行搜索+检测
#   3. 打印结果
#   4. 清理网络配置

WIDTH=${1:-4}
HEIGHT=${2:-4}
TARGET=${3:-""}
IFACE="enp65s0"
GO2W_IP="192.168.123.161"
LOCAL_IP="192.168.123.100"

echo "============================================================"
echo "  Go2W 搜索任务"
echo "  区域: ${WIDTH}m x ${HEIGHT}m"
echo "  目标: ${TARGET:-所有}"
echo "============================================================"
echo ""

# 1. 配网
echo "[1/3] 配置网络..."
sudo ip addr flush dev $IFACE 2>/dev/null
sudo ip addr add ${LOCAL_IP}/24 dev $IFACE 2>/dev/null
sudo ip link set $IFACE up 2>/dev/null
sleep 1

# 验证连通
if ! ping -c 1 -W 2 $GO2W_IP &>/dev/null; then
    echo "错误: 连不上 Go2W ($GO2W_IP)，请检查网线"
    exit 1
fi
echo "  Go2W 连通!"

# 2. 跑搜索
echo ""
echo "[2/3] 执行搜索..."
TARGET_ARG=""
if [ -n "$TARGET" ]; then
    TARGET_ARG="--target $TARGET"
fi

cd "$(dirname "$0")"
python3 test_standalone.py --width $WIDTH --height $HEIGHT --spacing 1.5 $TARGET_ARG --skip-camera 2>&1

# 3. 清理
echo ""
echo "[3/3] 清理网络..."
sudo ip addr flush dev $IFACE 2>/dev/null
sudo ip link set $IFACE down 2>/dev/null
echo "  有线网卡已清理，可以重新连外网了"
echo ""
echo "============================================================"
echo "  完成! 结果在 output/ 目录"
echo "============================================================"
