# 室外 GPS 航线导航 (feature/outdoor-gps)

> 状态: 功能 A 交付 (纯逻辑核心 + 薄 ROS 壳 + 单测), Web/API 集成由
> 集成执行者 (eI, feature/outdoor-follow-integration) 对接, 见 §5。

## 1. 目标与边界

让机器狗在户外沿一串 WGS-84 航点 (lat/lon) 逐点导航:
GPS 航点 → 地图系 (x, y) → 复用现有 Nav2 点导航链路
(`PointNavigationController` → arbiter → Nav2), 不新建运动链路。

**不在本功能范围内**: 地图构建仍由 FAST_LIO 负责 (GPS 不参与 odom/TF);
室内无 GPS 场景不适用 (受理即被门禁拒绝)。

## 2. 分层 (无 ROS 可测铁律)

| 文件 | 角色 | ROS 依赖 |
|------|------|----------|
| `web/nx_gps_nav.py` | 纯逻辑核心: NMEA 解析 / 质量门禁 / 大地→ENU→map 变换 / 航线状态机 | **零 rclpy** |
| `web/nx_gps_nav_ros.py` | 薄壳: 订阅 `/gps/fix` (NavSatFix) → `GpsFix` 喂核心; tick 定时器; 独立 bringup 入口 | rclpy (顶层 import) |
| `web/tests/test_gps_nav.py` | 核心行为测试 + 文件分层契约测试 | 无 |

数据流:

```
GPS 天线 (USB/网口)
  ├─ nmea_navsat_driver / 厂商驱动 → /gps/fix (NavSatFix, 推荐)
  └─ 串口 NMEA 原始语句 → parse_nmea_sentence() (GGA/RMC)
        ↓ (到达本机的 monotonic 时刻做新鲜度基准)
GpsRouteController (受理门禁 → 锚定 → 逐航点提交/推进/停车)
        ↓ point_port (鸭子类型: get_state/submit/cancel)
PointNavigationController → NavigationArbiter → Nav2 → 底盘
        ↘ park_hook (停车钩子, 生产接 arbiter 停车路径)
```

## 3. fail-closed 停车/拒绝条件 (项目铁律)

| # | 条件 | 动作 | 代码锚点 |
|---|------|------|----------|
| 1 | GPS 无定位 / 过期 (>max_age 2s) / 卫星数不足 / HDOP 超限 | 取消当前 Nav2 目标 + park | `GpsFixGate` + `tick()` |
| 2 | 北向未标定 (map 系与真北夹角未知) | **受理拒绝** `heading_not_calibrated` —— 未标定时整条航线被旋转未知角度, 属盲走 | `submit_route()` |
| 3 | 单航点超时 (默认 300s) / Nav2 失败终态 (aborted/rejected/…) | 整线中止 + park (后续航点不再走) | `tick()` |
| 4 | 航点所有权被抢 (port generation 被外部推进, 如面板点选) | 中止本航线, **不 cancel 不 park** —— 运动所有权已移交新 owner, 停车义务随所有权走 | `tick()` |
| 5 | 航点距离 > max_waypoint_range_m (默认 2km) | 受理拒绝 (防 lat/lon 手误) | `submit_route()` |
| 6 | 航点提交异常 / 端口拒绝 (planner_failed 等) | 整线中止 + park | `_submit_current_waypoint()` |
| 7 | 航线正常完成 | park (Nav2 succeeded ≠ 底盘静止, 对齐 arbiter.on_point_state) | `_finish_locked()` |

操作员 `cancel()` 只撤 Nav2 目标不 park: 操作员可能要接管手动,
停车由 arbiter 的操作员路径负责。

## 4. 坐标变换与北向标定

- **ENU**: `enu_from_latlon()` WGS-84 局部切平面, ~10km 内误差 <1m。
- **map 系目标**: 真北方向角 H (map 系中真北的方位, 从 map+x 逆时针,
  度) 由 `map_goal_for_waypoint()` 把航点 ENU 偏移旋转进 map 系:
  `x = e·sinH + n·cosH ; y = -e·cosH + n·sinH`。
- **标定流程** (每场地一次):
  1. 狗置于 A 点, 记 `(GPS_A, 地图位姿_A)`;
  2. 手动遥控向北行驶 ≥10m 到 B 点, 记 `(GPS_B, 地图位姿_B)`;
  3. `derive_heading_from_track(A, B)` → H;
  4. `controller.set_heading_calibration(H)` 后航线才可受理。
  注意实现坑 (已修复并有回归测试): H = θ_map − θ_enu + 90°,
  直接取方位角差会得到 H−90°。

## 5. eI 集成点 (对接说明)

```python
controller = GpsRouteController(
    point_port=point_nav,                 # PointNavigationController
                                            # 或包 arbiter 的适配器
    park_hook=lambda reason: arbiter.stop_all(reason),  # 或专门 park 路径
    state_callback=lambda s: ws.broadcast({"type": "gps_route", "data": s}),
)
# /gps/fix 订阅回调: controller.update_fix(nav_sat_fix_to_gps_fix(msg, time.monotonic()))
# 5Hz 定时器: controller.tick()
# API: POST /api/gps/route  → controller.submit_route(waypoints, map_pose=当前定位)
```

关键约定: point_port 必须暴露 `generation` (PointNavigationController
天然满足), 所有权被抢检测依赖它。

## 6. 参数默认值

| 参数 | 默认 | 说明 |
|------|------|------|
| `max_age_sec` | 2.0 | GPS 观测过期阈值 |
| `min_satellites` | None | 显式设置后严格模式 (未知也不放行) |
| `max_hdop` | None | 同上; NavSatFix 无此字段, NMEA 路径建议开启 |
| `arrive_timeout` | 300s | 单航点到达超时 |
| `max_waypoint_range_m` | 2000 | 航点离锚点距离上限 |
| `heading_calibrated` | False | 北向未标定默认拒绝受理 |

## 7. 测试

`python -m pytest web/tests/test_gps_nav.py -q` (74 项):
NMEA 严格解析 / 大地数学往返 / 门禁全维度 / 航线状态机全停车路径 /
文件分层契约 (核心零 rclpy、壳→核心依赖方向)。

## 8. 遗留风险

- 北向标定依赖人工流程, 标错角度会把航线整体旋转 (有 §4 自洽性检查
  与正交测试, 但无法全自动防呆)。
- `arrive_timeout` 300s 对长腿户外航线偏保守, 实车验证后可调。
- 多径/城市峡谷下 GPS 漂移不触发"过期", 只能靠 HDOP/卫星数门禁
  (NMEA 路径) 兜底; NavSatFix 路径建议驱动层开启质量字段。
