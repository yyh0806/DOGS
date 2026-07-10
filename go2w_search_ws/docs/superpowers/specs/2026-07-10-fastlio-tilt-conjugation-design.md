# FastLIO 倾斜安装共轭补偿设计

## 诊断结论

MID360 点内时间跨度约 100 ms，与 10 Hz 扫描一致；`timestamp_unit=3`
在 FAST_LIO_ROS2 中对应纳秒。录包中轮速、MID360 陀螺积分与 FastLIO
yaw 一致，因此时间戳、IMU 轴向和角速度单位正常。

MID360 物理倾斜约 18–20 度。以 0.1 rad/s 旋转约 59 度时，FastLIO
平移变化约 0.21 米，定位稳定；以 0.2 rad/s 及以上旋转会出现点云配准
发散。因此导航、平滑器和恢复行为的最大角速度统一限制为 0.1 rad/s。

FastLIO 的 roll/pitch 随 yaw 变化，是倾斜传感器坐标中的旋转共轭，不代表
底盘真实倾斜。当前 fuser 声明了 `body_to_base`，但公式没有使用外参；旧版
仅加入单侧 `_R_level`，同时启动脚本没有传 `body_to_base_pitch`，导致静止时
出现约 +20 度倾斜。

## 坐标补偿

令 `T_body_base` 表示 FastLIO body 到水平底盘 base_link 的安装外参，令：

```text
T_level = inverse(T_body_base)
T_map_odom = T_level * T_camera_body * T_body_base * inverse(T_odom_base)
```

静止时 `T_camera_body` 为单位阵，`T_level` 与 `T_body_base` 严格相消，因而
不会重现旧版静态 +20 度问题。运动时双侧共轭把倾斜传感器坐标中的旋转转换
为水平底盘 yaw。该形式同时允许以后加入经过标定的传感器平移外参。

录包网格标定得到约 17.4 度 pitch 时，整段旋转的等效底盘 roll/pitch RMS
约 0.53 度。首轮实机参数使用：

```text
body_to_base_pitch = -0.3037 rad  # -17.4 degrees
```

暂不加入 roll 与平移项，避免用单条轨迹过拟合次要误差。

## 启动与持久化

`bringup_slam_nav2.sh` 定义 `BODY_TO_BASE_PITCH`，默认 `-0.3037`，并在启动
`map_odom_fuser.py` 时通过 ROS 参数传入。NX 根目录脚本和 fuser 文件同步部署。

FastLIO 保持 `extrinsic_est_en: true`。DWB、velocity_smoother 和
behavior_server 的最大角速度统一为 0.1 rad/s，最小恢复角速度使用
0.05 rad/s，角加速度限制为 0.2 rad/s²。

## 验证与停止条件

验证顺序：

1. 静止重启后，`map -> base_link` roll/pitch 均小于 1 度。
2. 以 0.1 rad/s 旋转约 55–60 度，`map -> base_link` roll/pitch 小于 2 度，
   FastLIO 平移变化小于 1 米，map TF 连续存在。
3. 重置后执行 1 米 Nav2 目标。
4. 重置后执行 5 米 Nav2 目标。

最终成功仍要求 NavigateToPose 返回 `SUCCEEDED`、`linear.x > 0.01`、轮速
`odom -> base_link` 实际位移约 5 米、FastLIO 不发生米级跳变、fuser CPU
低于 20%，且核心服务保持 active。

任何旋转测试若 FastLIO 平移跳变超过 1 米、补偿后 roll/pitch 超过 5 度或
map TF 消失，立即发送零速度，停止导航并回滚 fuser 公式与启动参数。
