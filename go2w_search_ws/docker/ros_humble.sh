#!/usr/bin/env bash
# go2w_search_ws 的 ROS2 Humble 容器启动脚本。
#
# 为什么用容器:
#   PC 是 Ubuntu 20.04, 只能装 Galactic(EOL), 和 NX 的 Humble 跨机 DDS 不兼容
#   (Galactic↔Humble 会出现"能发现但收不到数据"的单向故障)。
#   于是在 PC 上用 Docker 跑 Humble, 和 NX 对齐, 对现有 Galactic/noetic 项目零侵入。
#
# 关键设计:
#   --net=host      容器共享主机网络栈, DDS 多播直接走物理网卡, 能发现 NX
#   --privileged    方便后续访问串口/USB (PX4, 相机); 可按需收紧为 --device
#   -e ROS_DOMAIN_ID / RMW  与 NX 对齐 (默认 0 / rmw_fastrtps_cpp)
#   挂载项目目录     代码在宿主机改, 容器内 colcon build / ros2 launch
#
# 用法:
#   ./docker/ros_humble.sh            # 交互式进入容器
#   ./docker/ros_humble.sh build      # 进容器并直接 colcon build
#   ./docker/ros_humble.sh shell       # 只进 bash (默认)
#
# 首次会自动 pull 镜像 (~2.5GB)。

set -euo pipefail

# ---------- 配置 ----------
IMAGE="osrf/ros:humble-desktop-full"
CONTAINER_NAME="go2w_humble"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"   # go2w_search_ws 根目录
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ---------- 镜像 ----------
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "镜像 $IMAGE 不存在, 开始拉取 (~2.5GB, 请耐心)..."
  docker pull "$IMAGE"
fi

# ---------- 启动 / 复用 ----------
# 如果容器已在运行, 直接 exec 进去; 否则新建。
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "容器 $CONTAINER_NAME 已在运行, 进入..."
  docker exec -it "$CONTAINER_NAME" bash
  exit 0
fi

# 已存在但已停止 → 启动它
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "容器 $CONTAINER_NAME 已停止, 重新启动..."
  docker start "$CONTAINER_NAME"
  docker exec -it "$CONTAINER_NAME" bash
  exit 0
fi

# ---------- 新建容器 ----------
echo "创建新容器 $CONTAINER_NAME (挂载 $PROJECT_DIR)..."
docker run -dit \
  --name "$CONTAINER_NAME" \
  --net=host \
  --privileged \
  --gpus all \
  -e DISPLAY="${DISPLAY:-}" \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PROJECT_DIR:/workspace:rw" \
  -v "$HOME/.ros:/root/.ros:rw" \
  --workdir /workspace \
  "$IMAGE"

echo ""
echo "容器已启动。进入:"
echo "  $0            # 交互式 bash"
echo "  docker exec -it $CONTAINER_NAME bash"

# 进容器并按子命令执行
case "${1:-shell}" in
  build)
    echo "执行 colcon build..."
    docker exec -it "$CONTAINER_NAME" bash -c \
      "source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install"
    ;;
  shell|"")
    docker exec -it "$CONTAINER_NAME" bash
    ;;
esac
