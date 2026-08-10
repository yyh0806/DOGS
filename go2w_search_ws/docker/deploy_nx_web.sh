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

# Compatibility entrypoint only. Never copy files into the live workspace;
# build a complete immutable payload and restart the web subsystem only.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "deploy_nx_web.sh is retired; forwarding to the atomic web release flow" >&2
artifact="$("$SCRIPT_DIR/build_release.sh" web)"
exec "$SCRIPT_DIR/deploy_release.sh" "$artifact" "$@"
