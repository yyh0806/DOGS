#!/bin/bash
# diagnose_motion.sh — 在 NX 上跑, 精确定位"控不了狗"的断点
# ============================================================
# 跑法 (NX 上):
#   bash ~/go2w_ws/web/diagnose_motion.sh
#
# 链路: 前端 → /api/move → nx_web_server 发 /cmd_vel → nx_motion_node 订阅 → Go2W SDK → 狗
# 任一环断 = 控不了。本脚本逐环检查, 输出 [OK]/[FAIL]/[WARN] 让你一眼定位。
# ============================================================
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/ws_livox/install/setup.bash ]; then source ~/ws_livox/install/setup.bash 2>/dev/null; fi

PASS=0; FAIL=0; WARN=0
ok()   { echo "[ OK ] $1"; PASS=$((PASS+1)); }
fail() { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "[WARN] $1"; WARN=$((WARN+1)); }

echo "========== 控狗链路诊断 =========="

# 1. motion service 状态
if systemctl is-active --quiet go2w-motion; then ok "go2w-motion service active"
else fail "go2w-motion service 不 active (试: sudo systemctl restart go2w-motion)"; fi

# 2. motion 进程在跑?
if pgrep -f "nx_motion_node.py" >/dev/null; then ok "nx_motion_node 进程在跑"
else fail "nx_motion_node 进程不存在"; fi

# 3. 关键: NX 上的 motion_node 是新代码吗? (含 rclpy GC 修复标志 _cmd_vel_sub)
GC_FIX=$(grep -c "_cmd_vel_sub" ~/go2w_ws/nx_motion_node.py 2>/dev/null)
if [ "$GC_FIX" -ge 1 ] 2>/dev/null; then
  ok "motion_node 含 GC 修复 (_cmd_vel_sub 持引用, $GC_FIX 处)"
else
  fail "motion_node 是旧代码! 缺 _cmd_vel_sub (rclpy GC 静默吞 /cmd_vel 订阅). 修复: 在 PC 跑 'bash docker/deploy_nx.sh' 重部署"
fi

# 4. web service 状态 (发 /cmd_vel 的一端)
if systemctl is-active --quiet go2w-web; then ok "go2w-web service active"
else fail "go2w-web service 不 active (前端 /api/move 没人处理)"; fi

# 5. /dog_state 里 cmd_vel_n (按键盘时应递增; 卡 0 = 订阅又失效)
echo "--- 5. 请按几下方向键 (5s 内), 看 cmd_vel_n 是否递增 ---"
DOG_STATE=$(timeout 5 ros2 topic echo /dog_state --once 2>/dev/null | grep -i cmd_vel_n | head -1)
if [ -n "$DOG_STATE" ]; then
  echo "     当前 $DOG_STATE"
  warn "cmd_vel_n 应随键盘递增; 卡 0 = motion 订阅又失效 (GC 坑); 若递增但狗不动 = SDK/lease 端"
else
  warn "/dog_state 5s 内没收到 (motion 没在发布? 或 ros2 topic 没就绪)"
fi

# 6. /cmd_vel 有人发布? (web 那侧)
echo "--- 6. 按方向键, 5s 内 /cmd_vel 应有 Twist ---"
CV=$(timeout 5 ros2 topic echo /cmd_vel --once 2>/dev/null | head -3)
if [ -n "$CV" ]; then ok "/cmd_vel 有数据 (web → motion 通)"; echo "     $CV" | head -2
else warn "/cmd_vel 5s 内没数据 (web 没发? 或前端没连 web)"; fi

# 7. motion 最近日志 (lease / error)
echo "--- 7. motion 最近 12 行日志 ---"
journalctl -u go2w-motion --no-pager -n 12 2>/dev/null | tail -12

# 8. 狗主控可达?
if ping -c1 -W1 192.168.123.161 >/dev/null 2>&1; then ok "狗主控 192.168.123.161 可达"
else fail "狗主控 ping 不通 (USB 网线/狗没开机/网卡没配 192.168.123.100/24)"; fi

# 9. service 重启计数 (Restart=always 频繁触发 = 反复崩)
NR=$(systemctl show go2w-motion -p NRestarts --value 2>/dev/null)
if [ "$NR" -lt 5 ] 2>/dev/null; then ok "motion 重启次数 $NR (正常)"
else warn "motion 重启次数 $NR (反复崩? journalctl -u go2w-motion -f 看 traceback)"; fi

# 10. 其他程序抢 lease?
LEAK=$(pgrep -af "unitree|robot_service|lcm|go2" 2>/dev/null | grep -v "nx_motion\|diagnose" | head -3)
if [ -z "$LEAK" ]; then ok "无其他控狗进程抢 lease"
else warn "检测到其他控狗进程 (可能抢 lease): $LEAK"; fi

echo "========== 诊断完成: $PASS OK / $FAIL FAIL / $WARN WARN =========="
echo ""
echo "最常见根因 (按概率):"
echo "  1. NX 上 motion_node 是旧代码 (步骤 3 FAIL) → PC 跑 'bash docker/deploy_nx.sh'"
echo "  2. service 没重启 (步骤 1/4 FAIL) → 'sudo systemctl restart go2w-motion go2w-web'"
echo "  3. Mihomo/Tailscale 干扰路由 → 临时关了再试"
echo "  4. 狗 lease 被抢 (步骤 10 WARN) → sudo systemctl stop go2w-motion 等几秒再 start"
