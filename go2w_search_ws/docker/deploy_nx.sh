#!/bin/bash
# ============================================================
# Go2W NX 部署脚本 — 把我们的程序部署到载荷 NX
# ============================================================
# 前提: NX 出厂已装好 ROS2 Humble + unitree_sdk2py + CycloneDDS
#       (~/CycloneDDS/lib, /opt/ros/humble/setup.bash)
#       NX 已连手机热点, SSH 可达
#
# 本脚本做的事:
#   1. 拷贝节点代码到 NX (~/go2w_ws/)
#   2. 自动探测连狗的 USB 网卡名 (不再硬编码 enxc8a362616c4c)
#   3. 安装 go2w-motion systemd 服务 (崩溃自动重启夺 lease)
#
# 用法 (在 PC 上跑, 从仓库根目录):
#   NX_HOST=192.168.43.41 NX_USER=nx bash docker/deploy_nx.sh
#
# 测试完想停止服务: bash docker/deploy_nx.sh stop
# ============================================================
set -e

NX_HOST="${NX_HOST:-192.168.43.41}"
NX_USER="${NX_USER:-nx}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# --- 子命令: stop ---
if [ "$1" = "stop" ]; then
  echo "停止 NX 上的 go2w-motion 服务..."
  ssh "$NX_USER@$NX_HOST" "echo '$NX_USER' | sudo -S systemctl stop go2w-motion.service 2>&1 | tail -1"
  echo "已停止 (服务仍 enabled, 重启NX会自启)"
  exit 0
fi

echo "================================================"
echo "  Go2W NX 部署 → $NX_USER@$NX_HOST"
echo "================================================"

# ---- 1. 连通性检查 ----
echo ""
echo "[1/4] 检查 NX 连通性..."
if ! ping -c1 -W2 "$NX_HOST" >/dev/null 2>&1; then
  echo "❌ NX ($NX_HOST) 不可达, 请确认 NX 开机且连热点"
  exit 1
fi
echo "✅ NX 在线"

# ---- 2. 自动探测连狗的 USB 网卡 ----
echo ""
echo "[2/4] 探测连狗的 USB 网卡 (狗主控 192.168.123.161)..."
DOG_IFACE=$(ssh "$NX_USER@$NX_HOST" 'bash -s' <<'REMOTE'
# 找能 ping 通狗主控 192.168.123.161 的网卡
# 优先找 enx* (USB转网口的典型命名)
for iface in $(ls /sys/class/net/ | grep -E "^enx|^enP|^enp"); do
  ip=$(ip -br addr show "$iface" 2>/dev/null | awk '{print $3}' | grep -oE "192.168.123.[0-9]+")
  if [ -n "$ip" ]; then
    if ping -c1 -W1 192.168.123.161 >/dev/null 2>&1; then
      echo "$iface"
      exit 0
    fi
  fi
done
# 兜底: 找名字含 123 网段的任何网卡
for iface in $(ls /sys/class/net/); do
  ip=$(ip -br addr show "$iface" 2>/dev/null | awk '{print $3}' | grep -oE "192.168.123.[0-9]+")
  if [ -n "$ip" ]; then echo "$iface"; exit 0; fi
done
echo ""
REMOTE
)

if [ -z "$DOG_IFACE" ]; then
  echo "❌ 未找到连狗主控(192.168.123.161)的网卡!"
  echo "   请检查: USB网线是否插好? 网卡是否配了 192.168.123.100/24 ?"
  echo "   参考 docs/TROUBLESHOOTING.md 问题1 配置网卡"
  exit 1
fi
echo "✅ 连狗网卡: $DOG_IFACE"

# 验证狗主控可达
if ! ssh "$NX_USER@$NX_HOST" "ping -c1 -W1 192.168.123.161 >/dev/null 2>&1"; then
  echo "⚠️  网卡 $DOG_IFACE 找到了, 但 ping 不通狗主控 192.168.123.161"
  echo "   狗主控可能没开机, 或网线物理连接有问题。继续部署代码, 但启动服务前需解决。"
fi

# ---- 3. 拷贝节点代码 ----
echo ""
echo "[3/4] 拷贝节点代码到 NX:~/$NX_USER/go2w_ws/ ..."
ssh "$NX_USER@$NX_HOST" "mkdir -p ~/go2w_ws"
scp -q "$WS_DIR/src/go2w_bridge/go2w_bridge/nx_motion_node.py" "$NX_USER@$NX_HOST:~/go2w_ws/"
scp -q "$WS_DIR/src/go2w_bridge/go2w_bridge/nx_sensor_node.py" "$NX_USER@$NX_HOST:~/go2w_ws/"
echo "✅ 节点代码已拷贝 (nx_motion_node.py, nx_sensor_node.py)"

# ---- 4. 安装 systemd 服务 ----
echo ""
echo "[4/4] 安装 go2w-motion systemd 服务 (用探测到的网卡 $DOG_IFACE)..."
# 生成适配当前网卡名的 service 文件
TMP_SERVICE=$(mktemp)
sed "s|enxc8a362616c4c|$DOG_IFACE|g" "$WS_DIR/docker/go2w-motion.service" > "$TMP_SERVICE"
scp -q "$TMP_SERVICE" "$NX_USER@$NX_HOST:/tmp/go2w-motion.service"
rm -f "$TMP_SERVICE"

ssh "$NX_USER@$NX_HOST" "echo '$NX_USER' | sudo -S bash -c '
  cp /tmp/go2w-motion.service /etc/systemd/system/go2w-motion.service &&
  systemctl daemon-reload &&
  systemctl enable go2w-motion.service &&
  systemctl restart go2w-motion.service
' 2>&1 | tail -1"
echo "✅ 服务已安装并启动 (enabled + active)"

# ---- 验证 ----
echo ""
echo "等待 6 秒验证 lease..."
sleep 6
ssh "$NX_USER@$NX_HOST" 'journalctl -u go2w-motion.service --no-pager -n 4 2>/dev/null | grep -iE "lease 已激活|error" | grep -vE "ddsi_udp" | tail -2'

echo ""
echo "================================================"
echo "  ✅ 部署完成!"
echo "  狗应已站立 (StandUp), lease 持有中"
echo ""
echo "  网卡: $DOG_IFACE (自动探测, 非硬编码)"
echo "  代码: ~/$NX_USER/go2w_ws/"
echo "  服务: go2w-motion.service (崩溃自动重启)"
echo ""
echo "  查看日志: ssh $NX_USER@$NX_HOST 'journalctl -u go2w-motion -f'"
echo "  停止服务: bash docker/deploy_nx.sh stop"
echo "================================================"
