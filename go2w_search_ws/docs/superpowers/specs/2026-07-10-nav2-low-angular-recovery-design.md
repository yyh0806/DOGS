# Nav2 低角速度恢复与 5 米导航验证设计

## 背景与根因

实机测试已确认 Nav2 的速度链曾在 `controller_server -> /cmd_vel_nav`
处中断；激活 `velocity_smoother` 后，DWB 可以通过 `/cmd_vel` 驱动
`nx_motion_node` 正向行驶。剩余失败发生在恢复阶段：当前进度检查要求
10 秒移动 0.5 米，失败后恢复行为以最高 1.0 rad/s 旋转。实测轮速里程计
累计旋转约 173 度，同时 FastLIO 产生数米至数百米的位姿跳变并丢失
`map` TF。

纯低速直行实验中，恢复 FastLIO 的在线外参估计后，FastLIO 与轮速里程计
均保持亚米级一致。因此本设计保留在线外参估计，并限制导航与恢复阶段的
角速度，避免恢复动作把定位推入发散状态。

## 参数设计

- DWB `max_vel_theta`: 1.0 -> 0.3 rad/s。
- `velocity_smoother` 角速度范围: [-0.3, 0.3] rad/s。
- `velocity_smoother` 角加速度/减速度: +/-0.5 rad/s^2。
- `behavior_server.max_rotational_vel`: 0.3 rad/s。
- `behavior_server.min_rotational_vel`: 0.1 rad/s。
- `behavior_server.rotational_acc_lim`: 0.5 rad/s^2。
- `progress_checker.required_movement_radius`: 0.5 -> 0.2 m。
- `progress_checker.movement_time_allowance`: 10 -> 20 s。
- FastLIO `extrinsic_est_en` 保持 `true`。

规划、障碍检测、旋转恢复和后退恢复继续启用，不绕过 Nav2 安全链路。

## 数据流与安全停止

控制链保持：

`controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> nx_motion_node`

每次实验前重启 FastLIO、清空局部/全局 costmap，并确认 `map -> base_link`
接近原点且 roll/pitch 合理。实验期间并行采集 action 结果、`/cmd_vel`、
`odom -> base_link` 与 `camera_init -> body`。若 FastLIO 平移跳变超过 2 米、
roll/pitch 超过 15 度或 `map` TF 消失，立即取消 goal 并发布零速度。

## 验证顺序

1. 低速原地旋转，确认 FastLIO 不发生米级跳变。
2. 执行 1 米 Nav2 目标，要求出现正向速度且定位稳定。
3. 重置定位后执行 5 米 Nav2 目标。

最终 5 米验收必须同时满足：

- NavigateToPose 返回 `SUCCEEDED`。
- `/cmd_vel` 的 `linear.x > 0.01 m/s`。
- `odom -> base_link` 起终点直线位移接近 5 米，容许导航目标误差与轮速漂移。
- FastLIO 无米级跳变，`map` TF 全程存在。
- fuser CPU 保持低于 20%。

## 回滚

若低速旋转仍导致 FastLIO 发散，不继续扩大导航容差或地图尺寸；恢复此次
角速度参数，保留已经验证的速度链修复，转入 MID360/FastLIO 标定与时间同步
专项排查。
