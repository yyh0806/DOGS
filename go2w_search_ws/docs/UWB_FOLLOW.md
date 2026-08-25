# UWB 钥匙扣跟随 (功能 B-2) — 控制器与 Web 集成

> 分支 `feature/uwb-follow` · 模块 `web/nx_uwb_follow.py` (ROS-free 纯逻辑)
> 姊妹件: B-1 UWB 桥 (`feature/uwb-bridge` → `nx_uwb_bridge.py`)、B-3 集成 (`feature/outdoor-follow-integration`)。

## 1. 设计原则

1. **ROS-free**: 控制器不 import rclpy; 测距源、避障探测、速度发布、运动
   所有权全部依赖注入。无 ROS 开发机/CI 可完整单测 (参照 nx_navigation_arbiter.py)。
2. **避障闸门第一优先**: 每 tick 先评闸门再算跟随; 探测缺失/异常 fail-closed
   (宁停不撞); 挡住 → 平移清零仅留原地转向; 迟滞 (stop 距离挡, stop+margin 放)
   防边界抖动; 近障碍按净空线性降速。
3. **只前进不后退**: 距离小于死区站住, 绝不倒车。
4. **所有权经 arbiter manual 通道**: 跟随是非 Nav2 的持续运动生产者, 借用
   NavigationArbiter 的 manual 所有权 (零速 handoff / idle 租约语义由 arbiter
   保证); 零速闲置 3s 自动释放 (狗回 joint_lock)。

## 2. 状态机

```
idle ──start()──► searching ──有效测距+闸门开──► following
searching ◄──测距失效── following / obstacle_hold
following ⇄ obstacle_hold      (闸门挡/恢复, 迟滞防抖)
searching ──超时──► idle  (target_not_acquired / target_lost)
任意 ──stop()──► idle
任意活跃态 ──发布连续失败/所有权重试耗尽──► fault (锁存, stop() 复位)
```

tick 顺序: **闸门 → 测距 → 超时 → 控制律 → 所有权 → 发布**。

## 3. UWB fix 字段合同 (B-1 对齐)

源对象暴露 `get_fix() -> dict|None`, 或任意线程直调
`controller.ingest_fix(fix)` 推模式 (两路并存取最新鲜者):

| 字段 | 必填 | 说明 |
|------|------|------|
| `ok` | 建议 | `False` 表示本拍无有效测距 (可带 `reason`) |
| `range_m` | ✔ | 钥匙扣↔机器人距离 (米), 有限数; 超出 [0.1, 30] 视为无效 |
| `azimuth_rad` | ✘ | 机器人系方位角, +左 (REP-103); 一维测距可缺省 (只跟不转) |
| `age_sec` | ✘ | 源侧测距年龄; 缺省按本拍新鲜 (丢失看门狗仍兜底) |
| `anchor_id` / `quality` | ✘ | 透传到状态快照 |

**B-1 挂载点**: `nx_uwb_bridge.get_follow_fix_source()` 返回上述源对象;
web 启动时 `_load_uwb_follow_source()` 自动发现 ( ImportError → 源为 None,
start 拒绝 `uwb_source_unavailable`, 查询/调参仍可用)。
环境变量 `GO2W_UWB_FOLLOW_DISABLE=1` 可强制关闭。

## 4. Web API

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/uwb_follow/start` | POST | body 可选 JSON 参数 (`{"follow_distance_m":1.5}` 或 `{"params":{...}}`); 202/400/409 |
| `/api/uwb_follow/stop` | POST | 停止 (零速 + 释放所有权) |
| `/api/uwb_follow/status` | GET | 状态快照 (无源也返回 idle 只读态) |
| `/api/uwb_follow/params` | POST | 运行期调参 (跟随中可用, 下一 tick 生效) |

联动: `/api/stop` 与 `/api/e_stop` 先停跟随再 arbiter 停车 (否则 10Hz tick
下一拍复活 manual 所有权); `/api/status` 汇聚 `uwb_follow` 键;
WS 推送 `{"type":"uwb_follow","data":<快照>}` (状态变化 + 1s 心跳);
前端 panel.html「🎯 跟随」按钮 (障碍停/搜索/故障分色显示)。
三个 POST 端点已加入 audited_paths 审计。

## 5. 控制律 (可调参数, 见 FollowParams)

- `vx = clamp(range_gain × (距离误差 − 死区), 0, max_vx)`, 再被
  `clearance_taper_gain × (净空 − 挡距)` 压制, 再乘 `max(0, cos(方位角))`;
- `wz = clamp(yaw_gain × 方位角, ±max_wz)` (带方位死区);
- 野值剔除: 单帧跳变 > `jump_threshold_m` 丢弃, 连续 2 帧才接受。

## 6. 测试

```bash
cd go2w_search_ws
python -m pytest web/tests/test_uwb_follow.py web/tests/test_uwb_follow_web.py -q
# 35 (状态机) + 14 (web 集成契约) = 49 全绿
```

## 7. B-3 集成注意

- 若引入独立 motion owner: 只替换 `build_uwb_follow_controller` 里的
  `_acquire/_release` 闭包, 控制器与端点零改动。
- 闸门探测接 `robot.directional_clearance(deg, fov)` (雷达 /scan_mid360);
  室外 GPS/UWB 场景若雷达不可用, 需提供等效净空源, 缺省即 fail-closed。
