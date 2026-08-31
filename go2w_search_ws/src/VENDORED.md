# src/ 下 vendored 第三方包清单

四个第三方 ROS2 包直接 vendor 进本仓库（不再依赖 clone 脚本），各自的来源与本地改动如下。
目的：换机器 clone 即可 colcon build，不再出现"躺在树里但不受版本控制"的状态。

| 包 | 上游 | 本地改动 | 入库时排除 |
|---|---|---|---|
| `FAST_LIO_ROS2` | https://github.com/hku-mars/FAST_LIO （ros2 分支系） | 实机 IMU 单位 patch（`src/laserMapping.cpp`，见 QUICKSTART.md 路径 B 说明） | `doc/`（127MB 上游论文 PDF + 实验 gif，非构建必需） |
| `livox_ros_driver2` | https://github.com/Livox-SDK/livox_ros_driver2 | 无 | 无（0.9MB 全量） |
| `sophus` | https://github.com/stonier/sophus （ROS2 打包） | 无 | 无（0.1MB 全量） |
| `ros2_livox_simulation` | https://github.com/stm32f303ret6/livox_laser_simulation_RO2 | 无 | 内嵌 `.git` 历史（2026-08 Phase 0 剥离）；`scan_mode/*.csv`（164MB 扫描模式表，见下） |

## scan_mode 扫描模式表获取

仿真插件的扫描模式表体积大，git 已排除。本项目 LiDAR 为 MID360，默认只需 `mid360.csv`（16MB）：

```bash
bash tools/fetch_livox_scan_modes.sh          # 只取 mid360.csv
bash tools/fetch_livox_scan_modes.sh --all    # 全部 7 个模式 (164MB, 一般用不到)
```

脚本从上游 master 分支 raw 地址下载，目标目录 `src/ros2_livox_simulation/scan_mode/`。

## 升级策略

需要跟进上游修复时：在上游 fork 一份 → cherry-pick → 用 `diff -r` 对照本目录 →
把差异作为普通 commit 提交，并在本表"本地改动"列追加说明。**不要**在树内重新
`git clone` 覆盖（会重新引入内嵌 `.git`）。
