#!/bin/bash
# Go2W 搜索系统一键启动脚本
# 用法: ./web_start.sh
cd "$(dirname "$0")"
echo "====================================="
echo "  Go2W Search System"
echo "  浏览器打开: http://localhost:8000"
echo "====================================="
python3 web/server.py --interface enp65s0 --model yolov8n.pt "$@"
