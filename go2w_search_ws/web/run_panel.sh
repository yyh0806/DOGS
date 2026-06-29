#!/usr/bin/env bash
# 启动 panel.py (ROS2 模式), 完全脱离当前 shell 会话常驻。
# 用法: bash web/run_panel.sh   (启动后立即返回, panel 在后台跑)
# 停止: pkill -f panel.py
set -e
cd "$(dirname "$0")/.."
mkdir -p /tmp/go2w_logs

pkill -f "panel.py" 2>/dev/null || true
sleep 1

# setsid 创建新会话, 完全脱离调用者的进程组 (harness的Bash每次是独立短命shell,
# 不脱离的话 panel 会随 shell 退出被回收)
setsid bash -c 'GO2W_USE_ROS2=1 exec python3 -u web/panel.py' \
    > /tmp/go2w_logs/panel.log 2>&1 < /dev/null &
PANEL_PID=$!
disown 2>/dev/null || true

echo "panel 已启动 PID=$PANEL_PID (日志: /tmp/go2w_logs/panel.log)"
echo "等待初始化..."
# 保持脚本进程存活几秒, 让 setsid 的子进程有时间初始化 + 输出日志
for i in $(seq 1 8); do
    sleep 1
    if grep -q "server listening" /tmp/go2w_logs/panel.log 2>/dev/null; then
        echo "✅ panel 就绪 (用时 ${i}s)"; break
    fi
done
tail -6 /tmp/go2w_logs/panel.log 2>/dev/null
