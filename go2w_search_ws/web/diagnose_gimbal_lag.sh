#!/bin/bash
# ============================================================
# C13 云台双流卡顿诊断 — 在 NX 上跑, 收集根因证据
# ============================================================
# 用法 (PC 上 ssh 进 NX 一行跑):
#   ssh nx@192.168.1.104 'bash -s' < go2w_search_ws/web/diagnose_gimbal_lag.sh
# 或 scp 到 NX 后:
#   scp go2w_search_ws/web/diagnose_gimbal_lag.sh nx@192.168.1.104:~/go2w_ws/web/
#   ssh nx@192.168.1.104 'bash ~/go2w_ws/web/diagnose_gimbal_lag.sh'
#
# 跑完把全部输出贴回给 Claude 分析。
# ============================================================
set +e   # 诊断脚本: 任一项失败都要继续往下跑完

echo "============================================================"
echo "  C13 双流卡顿诊断  ($(date '+%F %T'))"
echo "============================================================"

echo ""
echo "=== [1] go2w-web 服务状态 + PID ==="
systemctl is-active go2w-web.service
PID=$(pgrep -f nx_web_server.py | head -1)
echo "nx_web_server.py PID = ${PID:-<未运行>}"

echo ""
echo "=== [2] 进程内实际生效的视频相关环境变量 ==="
echo "(确认 C13_* 默认值有没有被 service/部署脚本覆盖)"
if [ -n "$PID" ]; then
  sudo cat /proc/$PID/environ 2>/dev/null | tr '\0' '\n' \
    | grep -E "C13_|OPENCV_FFMPEG|LD_LIBRARY_PATH|GO2W_AI_VIDEO|GO2W_YOLO" \
    || echo "  (读不到 /proc/$PID/environ, 试试 sudo)"
fi

echo ""
echo "=== [3] 启动日志 (关键: 看 backend 到底是 ffmpeg 还是 gst) ==="
echo "    期望看到: '[gimbal] vis 已连接 rtsp://...'  无 '(backend=gst)' = 软解"
sudo journalctl -u go2w-web.service --no-pager 2>/dev/null \
  | grep -iE "\[gimbal\]|C13|backend|双流|connected|已连接" | tail -20

echo ""
echo "=== [4] NX CPU 占用 (定位: 解码吃 CPU 还是编码吃 CPU) ==="
if [ -n "$PID" ]; then
  echo "--- 线程级 top (前 20 个最吃 CPU 线程) ---"
  top -bn1 -H -p $PID 2>/dev/null | head -22
fi
echo "--- 系统整体负载 (1/5/15 min) ---"
uptime
echo "--- 整机 top 前 10 ---"
top -bn1 | head -16

echo ""
echo "=== [5] C13 RTSP 源端参数 (源头分辨率/帧率/编码) ==="
if which ffprobe >/dev/null 2>&1; then
  echo "--- vis 流 (554) ---"
  timeout 6 ffprobe -v error -rtsp_transport tcp -select_streams v:0 \
    -show_entries stream=width,height,r_frame_rate,codec_name,bit_rate \
    -of default=noprint_wrappers=1 rtsp://192.168.144.108:554/stream=1 2>&1
  echo "--- ir 流 (555) ---"
  timeout 6 ffprobe -v error -rtsp_transport tcp -select_streams v:0 \
    -show_entries stream=width,height,r_frame_rate,codec_name,bit_rate \
    -of default=noprint_wrappers=1 rtsp://192.168.144.108:555/stream=2 2>&1
else
  echo "  (NX 没装 ffprobe; 可 sudo apt install ffmpeg)"
fi

echo ""
echo "=== [6] 实际 WS 推送频率 (后端侧 10s 采样) ==="
echo "    前端 fpsBadge 是 vis+frame 合计, 这里独立看后端推送节奏"
echo "    开始采样, 请同时盯前端 fpsBadge 10 秒..."
sleep 10
echo "  (注: 只 warning 级日志会落盘, 实际推送频率主要看前端 fpsBadge 数值)"
echo "  前端 fpsBadge 显示 = ___ FPS  (请手填)"

echo ""
echo "=== [7] NX 网络接口 + 出向带宽 (NX→PC 是否挤) ==="
ls -1 /sys/class/net/ 2>/dev/null
echo "--- 各接口 1s × 2 采样 (RX/TX KB/s) ---"
if which sar >/dev/null 2>&1; then
  sar -n DEV 1 2 2>/dev/null | grep -vE "IFACE|^$|lo|Average" | head -20
  echo "(看连 PC 那张网卡 TX 是否接近上限)"
else
  echo "  (没装 sar; 可改用: ssh 后台跑 iftop, 或 cat /proc/net/dev 两次相减)"
fi

echo ""
echo "=== [8] OpenCV 构建 (FFMPEG / GStreamer 是否编译进 cv2) ==="
python3 -c "
import cv2
bi = cv2.getBuildInformation()
print('opencv version:', cv2.__version__)
for line in bi.split('\n'):
    if 'FFMPEG' in line or 'GStreamer' in line:
        print('  ', line.strip())
" 2>&1

echo ""
echo "=== [9] GST 是否真被 ws_livox 污染 (试拉一条 gst pipeline) ==="
if which gst-launch-1.0 >/dev/null 2>&1; then
  echo "    试 3s 拉流, 不污染环境直接跑:"
  timeout 3 gst-launch-1.0 rtspsrc location=rtsp://192.168.144.108:554/stream=1 \
    protocols=tcp latency=0 ! fakesink 2>&1 | tail -8 \
    || echo "  (gst 拉流失败 — 可能就是 LD_LIBRARY_PATH 污染)"
else
  echo "  (无 gst-launch-1.0)"
fi

echo ""
echo "============================================================"
echo "  诊断完成。请把以上输出完整贴回给 Claude。"
echo "  并手填:"
echo "    - 前端 fpsBadge 刚启动时的 FPS = ___"
echo "    - 前端 fpsBadge 跑 5 分钟后的 FPS = ___"
echo "    - 卡顿是『一直卡』还是『越跑越卡』?"
echo "============================================================"
