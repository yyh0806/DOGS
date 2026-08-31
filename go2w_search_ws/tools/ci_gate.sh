#!/usr/bin/env bash
# ci_gate.sh — 本地验收门禁 (第 1 层)。退出码 0 = 允许 merge。
# M1: go2w_brain 全量单测 + 干跑演示;
# M2: + lake_plan 全量单测 (封闭瓦片夹具) + GPS 航线回归 (纯逻辑, 无 ROS)。
# 依赖: python3 + pytest + numpy + pillow (lake_plan 需要)。
set -euo pipefail
cd "$(dirname "$0")/../web"

echo "[ci_gate] go2w_brain 单测 (离线, 不依赖网络/ROS)"
python3 -m pytest go2w_brain/tests -q

echo "[ci_gate] lake_plan 单测 (封闭瓦片夹具, 零网络)"
python3 -m pytest lake_plan/tests -q

echo "[ci_gate] GPS 航线控制器回归 (纯逻辑)"
python3 -m pytest tests/test_gps_nav.py -q

echo "[ci_gate] M3 离水守卫纯逻辑回归"
python3 -m pytest tests/test_water_guard.py -q

echo "[ci_gate] M4 落水检测管线回归 (合成数据集)"
python3 -m pytest tests/test_drowning_detect.py -q

echo "[ci_gate] M3 运动链守卫客户端回归 (bridge 纯逻辑, 无 ROS)"
(cd ../src/go2w_bridge/go2w_bridge && \
 python3 -m pytest ../../test/test_water_guard_client.py \
                    ../../test/test_motion_safety.py \
                    ../../test/test_motion_controller.py -q)

echo "[ci_gate] 干跑演示: 状态报告 (mock, 规则模式)"
python3 -m go2w_brain.run_brain --task "报告当前状态" \
    --platform mock --no-llm > /dev/null

echo "[ci_gate] 干跑演示: 未知任务诚实拒绝"
python3 -m go2w_brain.run_brain --task "把门打开" \
    --platform mock --no-llm > /dev/null

echo "[ci_gate] OK"
