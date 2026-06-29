#!/bin/bash
# ============================================================
# Go2W 阶段2: MID360 + FAST_LIO 3D建图 部署脚本
# ============================================================
# 前提: NX已装 ROS2 Humble + apt依赖(slam_toolbox/pcl_conversions等, 已由主部署装好)
#       MID360雷达已物理连接到NX, 配192.168.1.x网段
#
# 本脚本在NX上:
#   1. 编译安装 Livox-SDK2
#   2. 编译 livox_ros_driver2 (MID360驱动)
#   3. 编译 FAST_LIO_ROS2 (Ericsii版, Humble移植)
#
# 用法(在NX上跑):
#   bash deploy_fastlio.sh
# ============================================================
set -e

echo "================================================"
echo "  MID360 + FAST_LIO 部署"
echo "================================================"

# 系统依赖
echo "[0/3] 安装编译依赖..."
sudo apt-get install -y libeigen3-dev libpcl-dev pcl-tools \
    libgoogle-glog-dev libgflags-dev libtbb-dev libyaml-cpp-dev \
    ros-humble-tf2-ros ros-humble-pcl-conversions 2>&1 | tail -1

# 加swap防OOM (PCL+Sophus编译吃内存)
if ! swapon --show | grep -q fastlio_swap; then
    echo "  创建8GB swap防编译OOM..."
    sudo fallocate -l 8G /fastlio_swap && sudo chmod 600 /fastlio_swap
    sudo mkswap /fastlio_swap && sudo swapon /fastlio_swap || echo "swap已存在或创建失败,继续"
fi

source /opt/ros/humble/setup.bash

# ---- 1. Livox-SDK2 ----
echo ""
echo "[1/3] Livox-SDK2..."
if [ ! -d ~/Livox-SDK2 ]; then
    cd ~
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2 && mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
    sudo make install
    echo "✅ Livox-SDK2 已安装"
else
    echo "✅ Livox-SDK2 已存在, 跳过"
fi

# ---- 2. livox_ros_driver2 ----
echo ""
echo "[2/3] livox_ros_driver2..."
if [ ! -d ~/ws_livox/install/livox_ros_driver2 ]; then
    mkdir -p ~/ws_livox/src && cd ~/ws_livox/src
    if [ ! -d livox_ros_driver2 ]; then
        git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    fi
    cd livox_ros_driver2
    # ⚠️ livox_ros_driver2 用官方build.sh (ROS1/ROS2共用代码, 不能直接colcon build)
    # package_ROS2.xml 是ROS2的package.xml, build.sh会处理
    source /opt/ros/humble/setup.bash
    ./build.sh humble
    echo "✅ livox_ros_driver2 已编译"
    echo "  ⚠️ 请确认 MID360_config.json 里的 broadcast_code (印在雷达机身, 15位)"
    echo "  ⚠️ 请确认 host_ip 是NX连雷达网卡的IP"
else
    echo "✅ livox_ros_driver2 已编译, 跳过"
fi

# ---- 3. FAST_LIO_ROS2 ----
echo ""
echo "[3/3] FAST_LIO_ROS2 (Ericsii版)..."
if [ ! -d ~/ws_livox/src/FAST_LIO_ROS2 ]; then
    cd ~/ws_livox/src
    git clone --recurse-submodules https://github.com/Ericsii/FAST_LIO_ROS2.git
    cd FAST_LIO_ROS2
    # Humble编译坑: CMakeLists里 pcl_ros → tf2_ros
    sed -i 's/pcl_ros/tf2_ros/g' CMakeLists.txt 2>/dev/null || true
    cd ~/ws_livox
    colcon build --symlink-install --packages-select fast_lio \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
                     -DCMAKE_CXX_FLAGS="-Wno-deprecated-copy"
    echo "✅ FAST_LIO 已编译"
else
    echo "✅ FAST_LIO_ROS2 已存在, 跳过"
fi

echo ""
echo "================================================"
echo "  ✅ 部署完成! 启动建图:"
echo "  # 终端1: 雷达驱动"
echo "  source ~/ws_livox/install/setup.bash"
echo "  ros2 launch livox_ros_driver2 msg_MID360.launch.py"
echo ""
echo "  # 终端2: FAST_LIO建图"
echo "  source ~/ws_livox/install/setup.bash"
echo "  ros2 launch fast_lio mapping.launch.py config_path:=src/FAST_LIO_ROS2/config/mid360.yaml"
echo ""
echo "  验证: ros2 topic echo /Odometry  (应有~100Hz位姿)"
echo "  验证: ros2 run tf2_tools view_frames (TF: camera_init→body)"
echo "================================================"
