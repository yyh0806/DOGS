#!/bin/bash
# 在载荷 NX 本机启动 web 服务 (阶段A: web 通信层上移 NX)。
# 不是 PC! PC 已退役 docker 容器, 只开浏览器 (见 web/start_pc_browser.sh)。
#
# 用法 (NX 上): bash /home/nx/go2w_ws/web/start_nx_web.sh
# 停止: pkill -f nx_web_server.py
# 日志: /tmp/nx_web.log
set -e

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
# LD_LIBRARY_PATH 保留 CycloneDDS (与 go2w-motion.service 一致; nx_web 不直连狗SDK, 但保持环境一致)
export LD_LIBRARY_PATH="$HOME/CycloneDDS/lib:${LD_LIBRARY_PATH:-}"

# 进入代码目录 (脚本相对路径: web/start_nx_web.sh → 仓库根)
cd "$(dirname "$0")/.."

# 杀旧进程 (避免端口占用)
pkill -f "nx_web_server.py" 2>/dev/null || true
sleep 1

# setsid 脱离会话 (参考 run_panel.sh:14), 完全脱离调用 shell, 常驻
setsid bash -c 'exec python3 -u web/nx_web_server.py' \
    > /tmp/nx_web.log 2>&1 < /dev/null &
WEB_PID=$!
disown 2>/dev/null || true

echo "nx_web 启动 PID=$WEB_PID (日志: /tmp/nx_web.log)"
echo "等待初始化..."
READY=0
for i in $(seq 1 10); do
    sleep 1
    if grep -q "Web:" /tmp/nx_web.log 2>/dev/null; then
        echo "就绪 (用时 ${i}s)"
        READY=1
        break
    fi
done
if [ "$READY" = "0" ]; then
    echo "⚠️  未在 10s 内看到就绪日志, 可能启动失败, 看完整日志:"
fi
tail -8 /tmp/nx_web.log 2>/dev/null
