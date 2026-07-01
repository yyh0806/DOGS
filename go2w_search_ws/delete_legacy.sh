#!/bin/bash
# A 类旧架构删除脚本 — 用户用 '! bash delete_legacy.sh' 执行
# 回滚: cd /c/Users/ROG/yangyuhui/DOGS/go2w_search_ws && git restore web/ src/ docker/ test_standalone.py audio/
set -e
cd /c/Users/ROG/yangyuhui/DOGS/go2w_search_ws

echo ">>> 删除 web/ 旧 PC 闭环..."
rm -f web/panel.py
rm -f web/server.py
rm -f web/cmd_publisher.py
rm -f web/ros_to_json.py
rm -f web/static/index.html
rm -f web/run_panel.sh
rm -f web/start_ros2.sh.legacy
rm -f web/test_e2e.py
rm -f web/test_vlm_commands.py
rm -f web/test_vlm_pipeline.py

echo ">>> 删除根目录 legacy..."
rm -f test_standalone.py

echo ">>> 删除 go2w_bridge legacy 节点..."
rm -f src/go2w_bridge/go2w_bridge/bridge_node.py
rm -f src/go2w_bridge/go2w_bridge/sport_client.py
rm -f src/go2w_bridge/go2w_bridge/odom_publisher.py
rm -f src/go2w_bridge/go2w_bridge/lidar_publisher.py
rm -f src/go2w_bridge/go2w_bridge/nx_panel_bridge.py

echo ">>> 删除 docker legacy..."
rm -f docker/ros_humble.sh

echo ">>> 删除整包 legacy..."
rm -rf src/go2w_orchestrator
rm -rf src/go2w_detector
rm -rf src/go2w_web
rm -rf src/go2w_bringup
rm -rf src/go2w_interfaces
rm -rf audio

echo ""
echo "=========== 删除完成, 验证 ==========="
echo "[web/] (应无 panel/server/cmd_publisher/ros_to_json/run_panel/start_ros2.legacy/test_*):"
ls web/ | grep -v __pycache__
echo ""
echo "[web/static/] (应无 index.html):"
ls web/static/
echo ""
echo "[src/] (应无 orchestrator/detector/web/bringup/interfaces):"
ls src/
echo ""
echo "[src/go2w_bridge/go2w_bridge/] (应无 bridge_node/sport_client/odom/lidar/nx_panel_bridge):"
ls src/go2w_bridge/go2w_bridge/ | grep -v __pycache__
echo ""
echo "[docker/] (应无 ros_humble.sh):"
ls docker/
