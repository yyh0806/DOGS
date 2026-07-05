#!/bin/bash
# ============================================================
# Go2W NX Web 部署脚本 — 把 nx_web_server + mock + static + service 部署到载荷 NX
# ============================================================
# 前提: NX 已装好 ROS2 Humble (websockets 由 panel.py 已验证在 NX 可用)
#       go2w-motion.service 已部署 (deploy_nx.sh), 否则 /cmd_vel 无人消费
#       NX 已连手机热点, SSH 可达
#
# 本脚本做的事:
#   1. 拷贝 nx_web_server.py + mock + static 资源到 NX:~/go2w_ws/
#   2. 安装 go2w-web systemd 服务 (After=go2w-motion.service)
#   3. 打印"浏览器访问 http://NX_IP:8000"
#
# 用法 (在 PC 上跑, 从仓库根目录):
#   NX_HOST=192.168.43.41 NX_USER=nx bash docker/deploy_nx_web.sh
#
# 停止服务: bash docker/deploy_nx_web.sh stop
# ============================================================
set -e

NX_HOST="${NX_HOST:-192.168.43.41}"
NX_USER="${NX_USER:-nx}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# --- 子命令: stop ---
if [ "$1" = "stop" ]; then
  echo "停止 NX 上的 go2w-web 服务..."
  ssh "$NX_USER@$NX_HOST" "echo '$NX_USER' | sudo -S systemctl stop go2w-web.service 2>&1 | tail -1"
  echo "已停止 (服务仍 enabled, 重启NX会自启)"
  exit 0
fi

echo "================================================"
echo "  Go2W NX Web 部署 → $NX_USER@$NX_HOST"
echo "================================================"

# ---- 1. 连通性检查 ----
echo ""
echo "[1/3] 检查 NX 连通性..."
if ! ping -c1 -W2 "$NX_HOST" >/dev/null 2>&1; then
  echo "❌ NX ($NX_HOST) 不可达, 请确认 NX 开机且连热点"
  exit 1
fi
echo "✅ NX 在线"

# ---- 2. 拷贝 web 代码 + static 资源 ----
echo ""
echo "[2/3] 拷贝 nx_web 代码到 NX:~/$NX_USER/go2w_ws/ ..."
ssh "$NX_USER@$NX_HOST" "mkdir -p ~/go2w_ws/web ~/go2w_ws/web/static"
scp -q "$WS_DIR/web/nx_web_server.py"            "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/nx_gimbal_node.py"           "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/nx_lidar_node.py"            "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/nx_slam_map.py"              "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/mock_dog_state_publisher.py" "$NX_USER@$NX_HOST:~/go2w_ws/web/"
# 阶段E: 房间级搜索编排 (nx_web_server.py 运行时动态 import 这些; 缺失 → product/person search
# 被 try-import 优雅降级跳过, 不报错但功能静默失效。务必随 web 层一起部署)
scp -q "$WS_DIR/web/nx_room_orchestrator.py"     "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/nx_product_command.py"       "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/nx_active_search.py"         "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/nx_person_mission.py"        "$NX_USER@$NX_HOST:~/go2w_ws/web/"
scp -q "$WS_DIR/web/nx_person_localizer.py"      "$NX_USER@$NX_HOST:~/go2w_ws/web/"
# 部署后冒烟测试 (NX 上跑产品/人员搜索 + 契约套件, 不用回 PC 找脚本)
scp -q "$WS_DIR/web/verify_product_room_person_search.sh" "$NX_USER@$NX_HOST:~/go2w_ws/web/" 2>/dev/null || true
scp -q "$WS_DIR/web/verify_nx_web.sh"            "$NX_USER@$NX_HOST:~/go2w_ws/web/" 2>/dev/null || true
scp -q "$WS_DIR/web/static/panel.html"           "$NX_USER@$NX_HOST:~/go2w_ws/web/static/"
scp -q "$WS_DIR/web/static/map.js"               "$NX_USER@$NX_HOST:~/go2w_ws/web/static/"
echo "✅ web 代码 + stage-E 编排 + verify 脚本 + static 已拷贝"

# ---- 3. 安装 go2w-web systemd 服务 ----
echo ""
echo "[3/3] 安装 go2w-web systemd 服务..."

# 可选: MID360 驱动服务。网络 service 只配 192.168.1.200/32 + 路由, driver service 才真正发布 /livox/lidar。
if ssh "$NX_USER@$NX_HOST" "test -f ~/ws_livox/install/setup.bash"; then
  echo "检测到 ~/ws_livox/install/setup.bash, 安装 Livox MID360 systemd 服务..."
  scp -q "$WS_DIR/docker/livox-mid360-net.service"    "$NX_USER@$NX_HOST:/tmp/livox-mid360-net.service"
  scp -q "$WS_DIR/docker/livox-mid360-driver.service" "$NX_USER@$NX_HOST:/tmp/livox-mid360-driver.service"
  ssh "$NX_USER@$NX_HOST" "echo '$NX_USER' | sudo -S bash -c '
    cp /tmp/livox-mid360-net.service /etc/systemd/system/livox-mid360-net.service &&
    cp /tmp/livox-mid360-driver.service /etc/systemd/system/livox-mid360-driver.service &&
    systemctl daemon-reload &&
    systemctl enable livox-mid360-net.service livox-mid360-driver.service &&
    systemctl restart livox-mid360-net.service &&
    systemctl restart livox-mid360-driver.service
  ' 2>&1 | tail -1"
  echo "✅ Livox MID360 网络 + 驱动服务已安装并启动"
else
  echo "⚠️  未检测到 ~/ws_livox/install/setup.bash, 跳过 Livox driver service 安装"
fi

# 3a. 探测连狗的 USB 网卡 (与 deploy_nx.sh 一致; 让 ExecStartPre + DOG_INTERFACE
#     适配实测网卡名, 否则换 NX 时 web service 的 ExecStartPre 永远等不到网卡 → 卡死).
#     注意: 这里不 ping 狗主控 (web 部署时狗可能没开机), 只要有 192.168.123.x 的网卡即可.
DOG_IFACE=$(ssh "$NX_USER@$NX_HOST" 'bash -s' <<'REMOTE'
for iface in $(ls /sys/class/net/ | grep -E "^enx|^enP|^enp"); do
  ip=$(ip -br addr show "$iface" 2>/dev/null | awk '{print $3}' | grep -oE "192.168.123.[0-9]+")
  if [ -n "$ip" ]; then echo "$iface"; exit 0; fi
done
for iface in $(ls /sys/class/net/); do
  ip=$(ip -br addr show "$iface" 2>/dev/null | awk '{print $3}' | grep -oE "192.168.123.[0-9]+")
  if [ -n "$ip" ]; then echo "$iface"; exit 0; fi
done
echo ""
REMOTE
)
if [ -z "$DOG_IFACE" ]; then
  echo "⚠️  未找到 192.168.123.x 网卡, 用模板默认 enxc8a362616c4c (ExecStartPre 会持续等网卡)"
  DOG_IFACE="enxc8a362616c4c"
else
  echo "✅ 连狗网卡: $DOG_IFACE"
fi

# 3b. 用实测网卡名生成 service (替换 ExecStartPre + DOG_INTERFACE 里的 enxc8a362616c4c)
TMP_SERVICE=$(mktemp)
sed "s|enxc8a362616c4c|$DOG_IFACE|g" "$WS_DIR/docker/go2w-web.service" > "$TMP_SERVICE"
scp -q "$TMP_SERVICE" "$NX_USER@$NX_HOST:/tmp/go2w-web.service"
rm -f "$TMP_SERVICE"
ssh "$NX_USER@$NX_HOST" "echo '$NX_USER' | sudo -S bash -c '
  cp /tmp/go2w-web.service /etc/systemd/system/go2w-web.service &&
  systemctl daemon-reload &&
  systemctl enable go2w-web.service &&
  systemctl restart go2w-web.service
' 2>&1 | tail -1"
echo "✅ 服务已安装并启动 (enabled + active, 网卡=$DOG_IFACE)"

# ---- 验证 ----
echo ""
echo "等待 4 秒验证 web 服务..."
sleep 4
ssh "$NX_USER@$NX_HOST" 'systemctl is-active go2w-web.service 2>/dev/null' || true

echo ""
echo "================================================"
echo "  ✅ 部署完成!"
echo "  PC 浏览器打开: http://$NX_HOST:8000"
echo "  WebSocket    : ws://$NX_HOST:8001  (前端自动连)"
echo ""
echo "  代码: ~/$NX_USER/go2w_ws/web/"
echo "  服务: go2w-web.service (崩溃自动重启, After=go2w-motion.service)"
echo ""
echo "  查看日志: ssh $NX_USER@$NX_HOST 'journalctl -u go2w-web -f'"
echo "  跑验证  : ssh $NX_USER@$NX_HOST 'bash ~/go2w_ws/web/verify_nx_web.sh'"
echo "  停止服务: bash docker/deploy_nx_web.sh stop"
echo "================================================"
