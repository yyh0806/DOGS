#!/bin/bash
# 阶段A: PC 端不再起任何服务 (旧 PC ROS2 容器链路已退役)。
# web 服务在载荷 NX 上 (go2w-web.service), PC 只需开浏览器。
#
# 用法 (PC 上): bash web/start_pc_browser.sh
# 若打不开, 在 NX 上检查: ssh nx@$NX_HOST 'systemctl status go2w-web'
set -e

NX_HOST="${NX_HOST:-192.168.43.41}"

echo "================================================"
echo "  阶段A: web 服务在载荷 NX 上, PC 只开浏览器"
echo "================================================"
echo ""
echo "PC 端无需启动任何服务 (旧 PC ROS2 容器 + panel.py 链路已退役)。"
echo ""
echo "浏览器打开: http://${NX_HOST}:8000"
echo "WebSocket : ws://${NX_HOST}:8001  (前端 panel.html 自动拼)"
echo ""
echo "如果打不开:"
echo "  1. 确认 NX 在线: ping -c1 ${NX_HOST}"
echo "  2. 确认 web 服务: ssh nx@${NX_HOST} 'systemctl status go2w-web'"
echo "  3. 看 NX 日志   : ssh nx@${NX_HOST} 'journalctl -u go2w-web -n 20'"
echo "================================================"
