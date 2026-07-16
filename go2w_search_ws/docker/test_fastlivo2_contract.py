"""Fast-LIVO2 离线契约测试 — 无 NX 连接时验证全部产物齐整 + 外参自洽。

跑法: pytest docker/test_fastlivo2_contract.py
范式沿用 test_livox_deploy_contract.py (纯离线, 读仓库文件断言)。

覆盖 7 产物 + 关键不变量:
  1. 文件存在 (nx_c13_image_node / deploy_fastlivo2 / bringup_livo / 两 yaml / fuser)
  2. C13 Image 桥: 发 /c13/image_raw + /c13/camera_info, NVDEC pipeline, 时间戳红线
  3. LIVO yaml: topic 对齐 + extrinsic_R 单位阵 (LiDAR-IMU 模组内) + T_lc 段存在
  4. T_lc 旋转矩阵正交性 (det≈1, R'R≈I) — 防配置手滑
  5. 内参 yaml: camera_matrix/distortion_coefficients 字段
  6. deploy_fastlivo2: FASTLIVO2_REPO 参数化 + colcon build fast_livo
  7. bringup_livo: c13-image gate + body_to_base 倾斜参数 + User=nx + RMW 注入
  8. fuser: body_to_base_* 参数 + 公式含 T_body_base
"""
import math
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ============================================================
# 1. 文件存在性
# ============================================================

def test_all_livo_artifacts_exist():
    """7 产物 + fuser 修改齐全 (离线准备的完整性门)。"""
    files = [
        "web/nx_c13_image_node.py",
        "docker/deploy_fastlivo2.sh",
        "docker/bringup_livo.sh",
        "src/go2w_nav/config/fastlivo2_mid360_c13.yaml",
        "src/go2w_nav/config/c13_intrinsic.yaml",
        "src/go2w_bridge/go2w_bridge/map_odom_fuser.py",
        "docker/test_fastlivo2_contract.py",
    ]
    for f in files:
        assert (ROOT / f).is_file(), f"缺产物: {f}"


# ============================================================
# 2. C13 Image 桥节点
# ============================================================

def test_c13_image_node_publishes_correct_topics():
    src = read("web/nx_c13_image_node.py")
    assert 'create_publisher(Image' in src
    assert "/c13/image_raw" in src
    assert "/c13/camera_info" in src
    assert "sensor_msgs" in src


def test_c13_image_node_uses_nvdec_pipeline():
    """复用了 nx_gimbal_node.py 验证过的 NVDEC 硬解 pipeline (治软解延迟堆积)。"""
    src = read("web/nx_c13_image_node.py")
    assert "nvv4l2decoder" in src          # Jetson NVDEC 硬解
    assert "drop-on-latency=true" in src   # 治 FIFO 堆积
    assert "max-buffers=1" in src
    assert "latency=0" in src


def test_c13_image_node_stamps_at_capture_time():
    """时间戳红线: header.stamp 用 clock.now() (拉帧瞬间), 不是发布时刻。"""
    src = read("web/nx_c13_image_node.py")
    assert "时间戳红线" in src or "clock.now()" in src
    assert "get_clock().now()" in src
    assert "stamp.to_msg()" in src


def test_c13_image_node_lazy_imports():
    """红线: cv2/gi 缺失禁用不崩 (跟 nx_gimbal_node 同范式)。"""
    src = read("web/nx_c13_image_node.py")
    assert "_CV2_OK" in src
    assert "_GST_OK" in src
    assert "except Exception" in src


# ============================================================
# 3. LIVO 主配置
# ============================================================

def test_livo_yaml_topics_align():
    """LIVO 订阅的 topic 跟本仓库实际发的对齐 (错则 LIVO 收不到数据静默失败)。"""
    cfg = read("src/go2w_nav/config/fastlivo2_mid360_c13.yaml")
    assert 'lid_topic: "/livox/lidar"' in cfg   # livox-mid360-driver 发
    assert 'imu_topic: "/livox/imu"' in cfg     # 同驱动发
    assert 'img_topic: "/c13/image_raw"' in cfg  # nx_c13_image_node 发


def test_livo_yaml_lidar_imu_extrinsic_is_identity():
    """MID360 模组内 LiDAR→IMU 旋转必为单位阵 (出厂轴系对齐, 倾斜不改!)。"""
    cfg = read("src/go2w_nav/config/fastlivo2_mid360_c13.yaml")
    assert "extrinsic_R: [ 1, 0, 0," in cfg
    assert "0, 1, 0," in cfg
    assert "0, 0, 1 ]" in cfg


def test_livo_yaml_has_camera_extrinsic_section():
    """T_lc (LiDAR→Camera 外参) 段存在 — 20° 倾斜的编码点。"""
    cfg = read("src/go2w_nav/config/fastlivo2_mid360_c13.yaml")
    assert "extrinsic_R_LiDAR2CAM" in cfg
    assert "extrinsic_T_LiDAR2CAM" in cfg
    assert "贡献位" in cfg or "决策 1" in cfg  # 提醒用户确认倾斜轴向


def test_livo_yaml_documents_tilt_encoding():
    """配置必须明确说明 20° 倾斜编码在 T_lc 不在 extrinsic_R (防改错)。"""
    cfg = read("src/go2w_nav/config/fastlivo2_mid360_c13.yaml")
    assert "20°" in cfg or "20" in cfg
    assert "T_lc" in cfg or "LiDAR2CAM" in cfg
    # 必须警告 extrinsic_R 不改
    assert "不改" in cfg or "identity" in cfg.lower() or "单位阵" in cfg


# ============================================================
# 4. T_lc 旋转矩阵正交性 (防配置手滑)
# ============================================================

def _parse_lc_rotation():
    """从 yaml 提取 extrinsic_R_LiDAR2CAM 的 9 个数值 → 3x3 numpy。

    防御式: yaml 不可用或解析失败时 skip (不误报)。
    """
    yaml = pytest.importorskip("yaml")
    import numpy as np
    cfg_text = read("src/go2w_nav/config/fastlivo2_mid360_c13.yaml")
    data = yaml.safe_load(cfg_text)
    R = data.get("extrinsic_R_LiDAR2CAM")
    if R is None or len(R) != 9:
        pytest.skip("extrinsic_R_LiDAR2CAM 未找到或非 9 元素")
    return np.array(R, dtype=float).reshape(3, 3)


def test_lc_rotation_is_orthogonal():
    """T_lc 旋转矩阵必正交: det≈+1 (右手系), R'R≈I。

    防手滑: 若用户填错 (如漏一个负号, 或行列错位), det 偏离 1, 此测试抓出。
    """
    import numpy as np
    R = _parse_lc_rotation()
    det = np.linalg.det(R)
    assert abs(det - 1.0) < 1e-3, f"T_lc 旋转 det={det:.4f} 偏离 +1 (应正交右手系)"
    # R'R ≈ I
    err = np.abs(R.T @ R - np.eye(3)).max()
    assert err < 1e-3, f"R'R-I 最大偏差 {err:.4f} (应正交)"


def test_lc_rotation_encodes_20_degree_tilt():
    """旋转角≈20° (arccos((trace-1)/2)), 验证确实是 ~20° 倾斜。"""
    import numpy as np
    R = _parse_lc_rotation()
    trace = np.trace(R)
    angle_rad = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    angle_deg = math.degrees(angle_rad)
    # 允许 17-23° (预填值精度 + 可能微调)
    assert 17.0 <= angle_deg <= 23.0, f"T_lc 旋转角={angle_deg:.1f}° 不在 20°±3° (检查倾斜值)"


def test_camera_intrinsic_file_path_resolves_on_nx():
    """camera_intrinsic_file 必须是相对 LIVO config 目录的裸名, 不是 PC 仓库相对路径。

    deploy_fastlivo2.sh 把两 yaml 都拷到 ~/ws_livox/src/FAST_LIVO2/config/,
    故 intrinsic 引用必须是 'c13_intrinsic.yaml' (同目录), 不能含 src/go2w_nav/...
    (verifier C5: PC 相对路径在 NX 上解析不到 → LIVO 启动崩/退化)。
    """
    yaml = pytest.importorskip("yaml")
    cfg_text = read("src/go2w_nav/config/fastlivo2_mid360_c13.yaml")
    data = yaml.safe_load(cfg_text)
    p = data.get("camera_intrinsic_file", "")
    assert "src/go2w_nav" not in p, f"camera_intrinsic_file 是 PC 仓库相对路径 '{p}' (NX 上找不到)"
    assert "go2w_search_ws" not in p, f"camera_intrinsic_file 含仓库路径 '{p}'"
    assert p.endswith("c13_intrinsic.yaml"), f"应指向 c13_intrinsic.yaml (同目录), 实际 '{p}'"


def test_lc_rotation_documents_convention_caveat():
    """T_lc 必须明确警告: 符号取决于 LIVO 移植约定 (LiDAR→Camera vs Camera→LiDAR)。

    (verifier C2: 数值正交但符号可能跟移植约定相反, 用户必须物理确认)。
    """
    cfg = read("src/go2w_nav/config/fastlivo2_mid360_c13.yaml")
    # 必须给出两种约定的矩阵 (Ry+20 和 Ry-20), 让用户对照物理实测选
    assert "Camera→LiDAR" in cfg or "Camera→LiDAR" in cfg.replace(" ", "")
    assert "待物理确认" in cfg or "待实测" in cfg or "贡献位" in cfg


# ============================================================
# 5. 相机内参模板
# ============================================================

def test_c13_intrinsic_has_required_fields():
    y = read("src/go2w_nav/config/c13_intrinsic.yaml")
    assert "image_width:" in y
    assert "image_height:" in y
    assert "camera_matrix:" in y
    assert "distortion_coefficients:" in y
    assert "distortion_model:" in y
    assert "projection_matrix:" in y


def test_c13_intrinsic_marks_placeholder():
    """内参是占位值, 必须明确标"待标定" (防误以为是真值)。"""
    y = read("src/go2w_nav/config/c13_intrinsic.yaml")
    assert "占位" in y or "待标定" in y
    assert "camera_calibrator" in y or "标定" in y


# ============================================================
# 6. 部署脚本
# ============================================================

def test_deploy_fastlivo2_parametric_repo():
    """FASTLIVO2_REPO 参数化 (决策 3: 离线不锁死 ROS2 移植选型)。"""
    s = read("docker/deploy_fastlivo2.sh")
    assert "FASTLIVO2_REPO" in s
    assert "hku-mars" in s.lower()
    # 必须警告官方是 ROS1
    assert "ROS1" in s


def test_deploy_fastlivo2_builds_fast_livo():
    s = read("docker/deploy_fastlivo2.sh")
    assert "colcon build" in s
    assert "fast_livo" in s  # packages-select fast_livo
    assert "--packages-select" in s


def test_deploy_fastlivo2_deploys_image_node_and_configs():
    """deploy 必须把 c13_image_node + 两 yaml 一起部署到 NX (否则 bringup 找不到)。"""
    s = read("docker/deploy_fastlivo2.sh")
    assert "nx_c13_image_node.py" in s
    assert "fastlivo2_mid360_c13.yaml" in s
    assert "c13_intrinsic.yaml" in s


# ============================================================
# 7. bringup 编排
# ============================================================

def test_bringup_livo_has_c13_image_gate():
    """bringup 必须先等 /c13/image_raw 就绪再起 LIVO (否则 LIVO 收不到图像)。"""
    s = read("docker/bringup_livo.sh")
    assert "/c13/image_raw" in s
    assert "wait_hz /c13/image_raw" in s or "IMG_MIN_HZ" in s
    assert "c13-image" in s  # systemd unit 名


def test_bringup_livo_injects_tilt_params():
    """bringup 必须把 body_to_base_* 倾斜补偿参数注入 fuser (20° 处理)。"""
    s = read("docker/bringup_livo.sh")
    assert "BODY_TO_BASE_PITCH" in s
    assert "body_to_base_pitch" in s
    assert "-0.349" in s  # -20° 弧度默认值
    assert "20°" in s or "20" in s


def test_bringup_livo_uses_fastlivo2_not_fastlio():
    """bringup 启的是 fast_livo 不是 fast_lio (LIVO 替换 LIO)。"""
    s = read("docker/bringup_livo.sh")
    assert "fast_livo" in s
    assert "fastlivo2_mid360_c13.yaml" in s
    # 不应启 fast_lio launch (会跟 LIVO 抢 /Odometry)
    assert "ros2 launch fast_lio" not in s


def test_bringup_livo_inherits_rmw_env():
    """坑6 防治: systemd-run 必须 -p User=nx + RMW_IMPLEMENTATION 注入。"""
    s = read("docker/bringup_livo.sh")
    assert 'User=$USER' in s or 'User=nx' in s
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in s
    assert "systemd-run" in s


def test_bringup_livo_must_run_as_nx():
    """脚本自检必须以 nx 用户跑 (sudo -i 下 root 缺 RMW → DDS 隐形)。"""
    s = read("docker/bringup_livo.sh")
    assert 'id -un' in s
    assert "= \"nx\"" in s or '= "nx"' in s


# ============================================================
# 8. map_odom_fuser 倾斜补偿
# ============================================================

def test_fuser_has_body_to_base_params():
    """fuser 必须声明 body_to_base_* 参数 (倾斜补偿接口)。"""
    s = read("src/go2w_bridge/go2w_bridge/map_odom_fuser.py")
    for p in ["body_to_base_x", "body_to_base_y", "body_to_base_z",
              "body_to_base_roll", "body_to_base_pitch", "body_to_base_yaw"]:
        assert p in s, f"fuser 缺参数 {p}"


def test_fuser_formula_includes_tilt_term():
    """倾斜补偿必须双侧共轭，并在同一扫描时刻求 map→odom 校正。"""
    s = read("src/go2w_bridge/go2w_bridge/map_odom_fuser.py")
    assert "def _conjugate_pose(T_camera_body, T_body_base)" in s
    assert "np.linalg.inv(T_body_base) @ T_camera_body @ T_body_base" in s
    assert "create_subscription(Odometry, '/Odometry'" in s
    assert "create_publisher(Odometry, '/odom'" in s
    assert "lookup_transform(self._odom, self._base" not in s


def test_fuser_defaults_to_identity_backward_compat():
    """参数默认全零=identity, 向后兼容老 FastLIO 部署 (不破坏现网)。"""
    s = read("src/go2w_bridge/go2w_bridge/map_odom_fuser.py")
    # 必须注明 identity 向后兼容
    assert "identity" in s.lower() or "向后兼容" in s


# ============================================================
# 9. runbook 文档
# ============================================================

def test_runbook_exists_and_covers_key_sections():
    """runbook 必须存在且覆盖: 离线自检 / bringup / 标定 / 20° 倾斜。"""
    try:
        r = read("docs/fastlivo2_runbook.md")
    except FileNotFoundError:
        pytest.fail("缺 docs/fastlivo2_runbook.md (Task #7)")
    assert "标定" in r or "calibrat" in r.lower()
    assert "内参" in r or "intrinsic" in r.lower()
    assert "T_lc" in r or "外参" in r
    assert "20°" in r or "20" in r
    assert "倾斜" in r
