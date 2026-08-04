#!/bin/bash
# ============================================================
# Go2W: Fast-LIVO2 (LiDAR-Inertial-Visual) 部署脚本
# ============================================================
# 在 FastLIO (deploy_fastlio.sh) 已就绪的 NX 上, 叠加视觉融合:
#   1. 编译 Fast-LIVO2 到 ~/ws_livox (复用 Livox-SDK2 + livox_ros_driver2)
#   2. 部署 C13 RTSP→ROS Image 桥 (web/nx_c13_image_node.py) 到 ~/go2w_ws/web/
#   3. 部署 LIVO 配置 (fastlivo2_mid360_c13.yaml + c13_intrinsic.yaml)
#
# 前提 (deploy_fastlio.sh 已做, 本脚本只检查):
#   - Livox-SDK2 已装 (~/Livox-SDK2)
#   - livox_ros_driver2 已编译 (~/ws_livox/install/livox_ros_driver2)
#   - MID360_config.json 的 broadcast_code + host_ip 已配
#
# ⚠️ 仓库选型 (决策 3, 关键):
#   Fast-LIVO2 官方 HKU-MARS/FAST_LIVO2 是 **ROS1**。本脚本默认指向官方仓库,
#   但在 ROS2 Humble 上官方 ROS1 源码编译不过 — 必须用 ROS2 移植。
#   用法: export FASTLIVO2_REPO=<ROS2 移植仓库 URL> 后再跑本脚本。
#   已知移植选项见 docs/fastlivo2_runbook.md §1 (连通后实测选定, 离线不锁死)。
#
# 用法 (在 NX 上跑):
#   # 用 ROS2 移植:
#   export FASTLIVO2_REPO=https://github.com/<ros2-port>/FAST_LIVO2.git
#   bash docker/deploy_fastlivo2.sh
#   # 或留默认 (官方 ROS1, 仅看脚本提示):
#   bash docker/deploy_fastlivo2.sh
# ============================================================
set -e

echo "================================================"
echo "  Fast-LIVO2 (LiDAR-Inertial-Visual) 部署"
echo "================================================"

# ---- 配置 ----
WS_LIVOX="${WS_LIVOX:-$HOME/ws_livox}"
GO2W_WS="${GO2W_WS:-$HOME/go2w_ws}"
# 默认官方 ROS1 仓库 (会在下面校验: ROS2 环境必须用 ROS2 移植, 否则报错退出)
# ⚠️ 官方仓库名是 FAST-LIVO2 (连字符), 不是 FAST_LIVO2 (下划线, 404)
FASTLIVO2_REPO="${FASTLIVO2_REPO:-https://github.com/hku-mars/FAST-LIVO2.git}"
FASTLIVO2_DIRNAME="${FASTLIVO2_DIRNAME:-FAST_LIVO2}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---- 0. 前置检查: deploy_fastlio.sh 产物 ----
echo ""
echo "[0/4] 检查 FastLIO 前置 (deploy_fastlio.sh 已跑过)?..."
if [ ! -d "$HOME/Livox-SDK2" ]; then
    echo "❌ ~/Livox-SDK2 不存在 — 先跑 docker/deploy_fastlio.sh (装 SDK2 + livox_ros_driver2 + fast_lio)"
    exit 1
fi
if [ ! -f "$WS_LIVOX/install/setup.bash" ]; then
    echo "❌ $WS_LIVOX/install/setup.bash 不存在 — 先跑 docker/deploy_fastlio.sh"
    exit 1
fi
echo "✅ FastLIO 前置就绪 (Livox-SDK2 + ws_livox)"

source /opt/ros/humble/setup.bash
source "$WS_LIVOX/install/setup.bash"

# ---- 0b. 仓库选型校验 (决策 3) ----
if [[ "$FASTLIVO2_REPO" == *"hku-mars/FAST-LIVO2"* ]]; then
    echo ""
    echo "⚠️⚠️⚠️  FASTLIVO2_REPO 是官方 HKU-MARS/FAST_LIVO2 (ROS1) ⚠️⚠️⚠️"
    echo "  ROS2 Humble 编译不过官方 ROS1 源码。必须用 ROS2 移植:"
    echo "    export FASTLIVO2_REPO=https://github.com/<ros2-port>/FAST_LIVO2.git"
    echo "  已知移植选项见 docs/fastlivo2_runbook.md §1。"
    echo "  连通 NX 后选定一个能编译过的, 再跑本脚本。"
    echo "  (本脚本继续克隆, 仅用于查看源码结构; colcon build 会失败属预期)"
    echo ""
fi

# ---- swap 防 OOM (PCL+Sophus+视觉前端编译吃内存, 复用 fastlio 的 swap) ----
if ! swapon --show | grep -q fastlio_swap; then
    if [ ! -f /fastlio_swap ]; then
        echo "  创建 8GB swap 防编译 OOM..."
        sudo fallocate -l 8G /fastlio_swap && sudo chmod 600 /fastlio_swap
        sudo mkswap /fastlio_swap && sudo swapon /fastlio_swap || echo "  swap 创建失败, 继续"
    else
        sudo swapon /fastlio_swap 2>/dev/null || true
    fi
fi

# ---- 1. 编译 Fast-LIVO2 ----
echo ""
echo "[1/4] Fast-LIVO2 源码 ($FASTLIVO2_REPO)..."
if [ ! -d "$WS_LIVOX/src/$FASTLIVO2_DIRNAME" ]; then
    cd "$WS_LIVOX/src"
    git clone --recurse-submodules "$FASTLIVO2_REPO" "$FASTLIVO2_DIRNAME"
    cd "$FASTLIVO2_DIRNAME"
    # Humble 编译坑 (跟 deploy_fastlio.sh 同款修): pcl_ros → tf2_ros
    sed -i 's/pcl_ros/tf2_ros/g' CMakeLists.txt 2>/dev/null || true
else
    echo "✅ $WS_LIVOX/src/$FASTLIVO2_DIRNAME 已存在, 跳过克隆"
fi

echo "  colcon build fast_livo (Release, -Wno-deprecated-copy)..."
cd "$WS_LIVOX"
colcon build --symlink-install --packages-select fast_livo \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DCMAKE_CXX_FLAGS="-Wno-deprecated-copy" \
    || {
        echo ""
        echo "❌ fast_livo 编译失败。"
        if [[ "$FASTLIVO2_REPO" == *"hku-mars/FAST-LIVO2"* ]]; then
            echo "   原因可能: 用了官方 ROS1 仓库 (决策 3)。换 ROS2 移植:"
            echo "     export FASTLIVO2_REPO=https://github.com/<ros2-port>/FAST_LIVO2.git"
            echo "   再跑本脚本。见 docs/fastlivo2_runbook.md §1。"
        fi
        echo "   (连不上 NX 时这一步预期失败 — 离线阶段跳过编译, 连通后定 repo 再跑)"
        exit 1
    }
echo "✅ fast_livo 已编译"

# ---- 2. 部署 C13 Image 桥节点 ----
echo ""
echo "[2/4] 部署 nx_c13_image_node.py → $GO2W_WS/web/..."
mkdir -p "$GO2W_WS/web"
if [ -f "$REPO_DIR/web/nx_c13_image_node.py" ]; then
    cp "$REPO_DIR/web/nx_c13_image_node.py" "$GO2W_WS/web/"
    echo "✅ nx_c13_image_node.py 已部署"
else
    echo "⚠️  仓库内无 web/nx_c13_image_node.py — 从 PC scp 上来后再跑, 或手写"
fi

# ---- 3. 部署 LIVO 配置 ----
echo ""
echo "[3/4] 部署 LIVO 配置..."
LIVO_CFG_DIR="$WS_LIVOX/src/$FASTLIVO2_DIRNAME/config"
mkdir -p "$LIVO_CFG_DIR"
if [ -f "$REPO_DIR/src/go2w_nav/config/fastlivo2_mid360_c13.yaml" ]; then
    cp "$REPO_DIR/src/go2w_nav/config/fastlivo2_mid360_c13.yaml" "$LIVO_CFG_DIR/"
    cp "$REPO_DIR/src/go2w_nav/config/c13_intrinsic.yaml" "$LIVO_CFG_DIR/"
    echo "✅ fastlivo2_mid360_c13.yaml + c13_intrinsic.yaml → $LIVO_CFG_DIR"
else
    echo "⚠️  仓库内无 LIVO yaml — 从 PC scp 上来后再跑"
fi

# ---- 4. 指纹校验: 关键 topic/外参自检 ----
echo ""
echo "[4/4] 配置自检..."
CFG="$LIVO_CFG_DIR/fastlivo2_mid360_c13.yaml"
if [ -f "$CFG" ]; then
    grep -q 'img_topic: "/c13/image_raw"' "$CFG" && echo "  ✅ img_topic 对齐 nx_c13_image_node" || echo "  ❌ img_topic 不匹配"
    grep -q 'lid_topic: "/livox/lidar"' "$CFG" && echo "  ✅ lid_topic 对齐 livox driver" || echo "  ❌ lid_topic 不匹配"
    grep -q 'extrinsic_R: \[ 1, 0, 0' "$CFG" && echo "  ✅ LiDAR→IMU 单位阵 (模组出厂, 倾斜不改)" || echo "  ❌ extrinsic_R 非单位阵 (检查!)"
fi

echo ""
echo "================================================"
echo "  ✅ 部署完成! 启动 LIVO 栈:"
echo "  bash docker/bringup_livo.sh"
echo ""
echo "  ⚠️ 首次连通 NX 必做:"
echo "    1. 相机内参标定 (覆盖 c13_intrinsic.yaml, 见 runbook §3)"
echo "    2. LiDAR-Camera 外参 T_lc 实测 (覆盖 extrinsic_R/T_LiDAR2CAM, 见 runbook §3)"
echo "    3. 确认 20° 倾斜轴向 (默认假设 pitch 绕 Y, 见 yaml 贡献位)"
echo "================================================"
