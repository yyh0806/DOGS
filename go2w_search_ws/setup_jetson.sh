#!/bin/bash
# ============================================================
# Go2W Search & Discover - Jetson NX 一键部署脚本
# ============================================================
# 适用环境: NVIDIA Jetson Xavier NX (JetPack 4.6.x, Ubuntu 20.04, aarch64)
# 用法: chmod +x setup_jetson.sh && ./setup_jetson.sh
# ============================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()  { echo -e "\n${BLUE}==== $1 ====${NC}"; }

# 工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

# ============================================================
# Step 0: 系统检查
# ============================================================
log_step "Step 0: 系统环境检查"

if [[ "$(uname -m)" != "aarch64" ]]; then
    log_warn "当前不是 aarch64 架构，可能不在 Jetson 上运行"
    log_warn "脚本将继续执行，但某些步骤可能需要调整"
fi

if ! command -v lsb_release &> /dev/null || [[ "$(lsb_release -rs)" != "20.04" ]]; then
    log_warn "建议使用 Ubuntu 20.04 (当前: $(lsb_release -rs 2>/dev/null || '未知'))"
fi

log_info "工作空间: $WS_DIR"

# ============================================================
# Step 1: 系统基础依赖
# ============================================================
log_step "Step 1: 安装系统依赖"

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    python3-pip \
    python3-dev \
    python3-venv \
    libssl-dev \
    pkg-config \
    libclang-dev \
    libopencv-dev \
    v4l-utils \
    net-tools \
    usbutils

log_info "系统依赖安装完成"

# ============================================================
# Step 2: ROS2 Galactic 安装
# ============================================================
log_step "Step 2: 安装 ROS2 Galactic"

if [[ -f "/opt/ros/galactic/setup.bash" ]]; then
    log_info "ROS2 Galactic 已安装，跳过"
else
    log_info "添加 ROS2 apt 源..."
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe

    # ROS2 apt key
    sudo apt-get install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update

    log_info "安装 ROS2 Galactic (桌面版可能耗时较长，使用基础版以节省空间)..."
    sudo apt-get install -y \
        ros-galactic-ros-base \
        ros-galactic-geometry-msgs \
        ros-galactic-sensor-msgs \
        ros-galactic-nav-msgs \
        ros-galactic-std-msgs \
        ros-galactic-visualization-msgs \
        ros-galactic-image-transport \
        ros-galactic-cv-bridge \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-argcomplete

    # rosdep 初始化
    if [[ ! -d "/etc/ros/rosdep/sources.list.d" ]]; then
        sudo rosdep init || true
    fi
    rosdep update || true

    log_info "ROS2 Galactic 安装完成"
fi

source /opt/ros/galactic/setup.bash

# ============================================================
# Step 3: CycloneDDS (Go2W 兼容 DDS)
# ============================================================
log_step "Step 3: 安装 CycloneDDS"

sudo apt-get install -y ros-galactic-rmw-cyclonedds-cpp

# CycloneDDS 配置文件
mkdir -p ~/.ros
cat > ~/.ros/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF

log_info "CycloneDDS 配置完成 (~/.ros/cyclonedds.xml)"

# ============================================================
# Step 4: SLAM Toolbox + Nav2
# ============================================================
log_step "Step 4: 安装 SLAM Toolbox 和 Nav2"

sudo apt-get install -y \
    ros-galactic-slam-toolbox \
    ros-galactic-nav2-bringup \
    ros-galactic-nav2-msgs \
    ros-galactic-tf2-ros \
    || log_warn "部分 Nav2 包安装失败"

log_info "SLAM Toolbox + Nav2 安装完成"

# ============================================================
# Step 5: whisper.cpp (语音转文本)
# ============================================================
log_step "Step 5: 编译 whisper.cpp"

WHISPER_DIR="$WS_DIR/deps/whisper.cpp"
mkdir -p "$WS_DIR/deps"

if [[ -f "$WS_DIR/models/ggml-base.bin" ]]; then
    log_info "whisper 模型已存在"
else
    log_info "下载 whisper base 模型..."
    mkdir -p "$WS_DIR/models"
    wget -q https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.bin \
        -O "$WS_DIR/models/ggml-base.bin" || {
        log_warn "whisper 模型下载失败，将使用 Python whisper 降级"
    }
fi

if [[ -f "$WS_DIR/deps/whisper.cpp/main" ]]; then
    log_info "whisper.cpp 已编译"
else
    log_info "编译 whisper.cpp..."
    git clone https://github.com/ggerganov/whisper.cpp.git "$WHISPER_DIR" || true
    cd "$WHISPER_DIR" && make -j$(nproc) 2>/dev/null || log_warn "whisper.cpp 编译失败"
    cd "$WS_DIR"
fi

# 创建符号链接
ln -sf "$WS_DIR/deps/whisper.cpp/main" /usr/local/bin/whisper-cpp 2>/dev/null || true

# ============================================================
# Step 6: Unitree Go2W SDK
# ============================================================
log_step "Step 6: 安装 Unitree SDK2 Python"

pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git || {
    log_warn "Unitree SDK 安装失败 (可能需要先安装 cyclonedds pip 包)"
    pip3 install cyclonedds
    pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git || {
        log_warn "Unitree SDK 仍然安装失败，将以模拟模式运行"
    }
}

# ============================================================
# Step 7: Python 依赖
# ============================================================
log_step "Step 7: 安装 Python 依赖"

pip3 install --upgrade pip

# OpenCV (Jetson 上使用 JetPack 自带版本)
python3 -c "import cv2" 2>/dev/null || {
    log_info "安装 opencv-python..."
    pip3 install opencv-python-headless
}

# YOLO
pip3 install ultralytics || {
    log_warn "ultralytics 安装失败，尝试 pip install --no-deps"
    pip3 install --no-deps ultralytics
}

# 其他依赖
pip3 install pyyaml numpy

log_info "Python 依赖安装完成"

# ============================================================
# Step 8: YOLO 模型准备
# ============================================================
log_step "Step 8: 准备 YOLO 模型"

MODEL_DIR="$WS_DIR/models"
mkdir -p "$MODEL_DIR"

if [[ -f "$MODEL_DIR/yolov8n.pt" ]]; then
    log_info "YOLOv8n 模型已存在"
else
    log_info "下载 YOLOv8n 模型 (首次运行会自动下载，此处预下载)..."
    python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
print('模型下载完成')
" || log_warn "模型下载失败，将在首次运行时自动下载"
fi

# 导出 TensorRT FP16 引擎 (Jetson 加速)
if [[ -f "$MODEL_DIR/yolov8n.engine" ]]; then
    log_info "TensorRT 引擎已存在"
else
    log_info "导出 TensorRT FP16 引擎 (可能需要几分钟)..."
    python3 -c "
from ultralytics import YOLO
import os
model = YOLO('yolov8n.pt')
engine = model.export(format='engine', half=True, imgsz=640)
print(f'TensorRT 引擎导出完成: {engine}')
# 复制到 models 目录
import shutil
shutil.copy(str(engine), '$MODEL_DIR/')
" || log_warn "TensorRT 导出失败，将使用 PyTorch 模型"
fi

# ============================================================
# Step 9: 环境变量配置
# ============================================================
log_step "Step 9: 配置环境变量"

BASHRC="$HOME/.bashrc"
GO2W_ENV_MARKER="# >>> go2w_search_env >>>"

if grep -q "go2w_search_env" "$BASHRC" 2>/dev/null; then
    log_info "环境变量已配置，跳过"
else
    cat >> "$BASHRC" << 'ENVEOF'

# >>> go2w_search_env >>>
# ROS2 Galactic
source /opt/ros/galactic/setup.bash 2>/dev/null || true

# Go2W 工作空间
source $HOME/ZCodeProject/go2w_search_ws/install/setup.bash 2>/dev/null || true

# CycloneDDS (Go2W 兼容)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
# <<< go2w_search_env <<<
ENVEOF

    log_info "环境变量已写入 ~/.bashrc"
fi

# ============================================================
# Step 10: 编译工作空间
# ============================================================
log_step "Step 10: 编译工作空间"

cd "$WS_DIR"

# source 环境
source /opt/ros/galactic/setup.bash

log_info "开始 colcon 构建 (首次构建可能需要较时间)..."

# 先构建 interfaces (其他包依赖)
colcon build --packages-select go2w_interfaces \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    || log_error "go2w_interfaces 构建失败"

# source 生成的消息
source "$WS_DIR/install/setup.bash"

# 构建 Python 包
colcon build --packages-select go2w_bridge go2w_detector go2w_orchestrator go2w_web \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    || log_warn "Python 包构建有警告"

# 构建 CMake 配置包
colcon build --packages-select go2w_nav go2w_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    || log_warn "配置包构建有警告"

log_info "工作空间构建完成"

# ============================================================
# 完成
# ============================================================
log_step "部署完成!"

echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  Go2W Search & Discover 系统部署完成!${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo "下一步:"
echo ""
echo "  1. 连接 Jetson NX 到 Go2W (USB-Ethernet)"
echo "     确保 NX 能 ping 通 192.168.123.161"
echo ""
echo "  2. 一键启动全系统:"
echo "     source install/setup.bash"
echo "     ros2 launch go2w_bringup search.launch.py"
echo ""
echo "  3. Web 控制面板:"
echo "     http://<jetson-ip>:8000"
echo ""
echo "  4. 通过话题发送文本指令:"
echo "     ros2 topic pub --once /go2w/voice_text std_msgs/String '{data: 搜索左边房间}'"
echo ""
echo "  5. 查看任务队列:"
echo "     ros2 topic echo /go2w/task_queue"
echo ""
echo -e "${YELLOW}  硬件接线详见: hardware/SETUP_GUIDE.md${NC}"
echo -e "${GREEN}============================================================${NC}"
