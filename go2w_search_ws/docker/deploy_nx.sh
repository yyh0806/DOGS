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

# Compatibility entrypoint only. Production deployment is content-addressed
# and atomic. Motion restart authorization must still be supplied explicitly
# as --allow-motion-restart by the operator.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "deploy_nx.sh is retired; forwarding to the atomic motion release flow" >&2
artifact="$("$SCRIPT_DIR/build_release.sh" motion)"
exec "$SCRIPT_DIR/deploy_release.sh" "$artifact" "$@"
