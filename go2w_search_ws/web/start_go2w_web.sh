#!/bin/bash
# go2w-web.service 启动 wrapper — 让 nx_web_server 在"ROS2 lib 完整 + ws_livox LD 污染剥离"环境运行
# 目的: 解锁 nx_gimbal_node.py 的 GStreamer NVDEC 硬解 (nvv4l2decoder), 治 C13 双流延迟.
#
# 背景坑:
#   go2w-web.service 要 source ~/ws_livox/install/setup.bash (nx_web_server 需 import
#   livox_ros_driver2.msg, 该 Python 包的 PYTHONPATH 由 setup.bash 注入).
#   但 setup.bash 同时把 ws_livox/install/{fast_lio,livox_ros_driver2}/lib 前置到
#   LD_LIBRARY_PATH, 污染 GStreamer 插件扫描 → nvv4l2decoder 加载失败 → 双流 fallback
#   FFmpeg 软解 → 30fps 消费不动 → FIFO 单向堆积 → 延迟越跑越大 (nx_gimbal_node.py:35-44).
#
# 解法: source 完整环境 (拿 PYTHONPATH + ROS2 lib), 再剥离 LD 里的 ws_livox 段.
#   livox msg 纯 Python 不靠 LD; rclpy 靠 /opt/ros/humble/lib 保留.
#   已验证: gst nvv4l2decoder hevc 硬解 pipeline 跑通 (NvMMLiteOpen BlockType=279),
#   且 rclpy / livox_ros_driver2.msg / cv2 / gi.Gst 四个 import 全部正常.
set -e

source /opt/ros/humble/setup.bash
if [ -f /home/nx/ws_livox/install/setup.bash ]; then
  source /home/nx/ws_livox/install/setup.bash
fi

# 剥离 ws_livox native lib (保留 ROS2 humble + CycloneDDS), 解锁 NVDEC 硬解
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v 'ws_livox' | paste -sd:)

exec python3 -u /home/nx/go2w_ws/web/nx_web_server.py
