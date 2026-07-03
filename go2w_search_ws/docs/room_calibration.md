# 房间标定流程 (Go2W 阶段E 运维 SOP)

> 部署阶段E 后, 用本流程把 `config/rooms.yaml` 的占位坐标替换成真 Nav2 地图上的真实坐标。

## 前置

- 阶段D Nav2 已上线: `ros2 action list` 含 `/navigate_to_pose` (bt_navigator 起来了)
- FAST_LIO + map_server 建图完成 (静态地图 yaml + 全局坐标系 map)
- nx_motion_node + nx_sensor_node 跑着, `/odom` `/imu` 有数据
- mock_nav2_action.py 已 kill (用真 Nav2, 不再用 mock)

## 步骤 (每个房间)

1. **遥控狗到房间入口**
   - PC 浏览器开 `http://<NX_IP>:8000`
   - 用键盘/手柄控狗站到该房间的"搜索起点" (一般是门口或房间一角)

2. **记录位姿**
   ```bash
   # NX 上
   ros2 topic echo /odom --once
   ```
   记录 `pose.pose.position.x` / `pose.pose.position.y` / 四元数转 yaw:
   ```bash
   # yaw (弧度) 从四元数算 (qz, qw):
   python3 -c "import math; qz=0.xxxx; qw=0.xxxx; print('yaw_rad=', 2*math.atan2(qz, qw))"
   ```
   把这三个值填进 `config/rooms.yaml` 对应 room 的 `nav_pose`:
   ```yaml
   nav_pose: {x: <odom_x>, y: <odom_y>, yaw: <yaw_rad>}
   ```

3. **标定搜索矩形 search_area**
   - 遥控狗到房间对角的另一个顶点, 记录 x/y
   - `width = |对角x - 入口x|`, `height = |对角y - 入口y|`
   - `origin_x = min(入口x, 对角x)`, `origin_y = min(入口y, 对角y)`
   - `spacing`: 房间越大行间距越大 (客厅 1.5m, 厨房 1.0m), 太密航点多搜索慢
   - `pattern`: 大房间用 `lawnmower` (弓字形), 小房间用 `spiral` (螺旋)

4. **热加载 (无需重启 nx_web)**
   ```bash
   curl http://<NX_IP>:8000/api/reload_rooms
   # 期望: {"ok": true}
   curl http://<NX_IP>:8000/api/rooms
   # 确认房间列表含新标定的房间
   ```

5. **验证搜索**
   - 浏览器输入 "搜索客厅" (走 VLM/解析) 或 `fetch('/api/search_room?room=客厅', {method:'POST'})`
   - F12 Console 看 `type=search_room` 的 phase 推送序列
   - 地图看狗真走到客厅 + 房间内覆盖搜索
   - Console 看 `type=mission_report` 报告 (含检测到的目标)

## 常见问题

- **搜索时狗不动**: 检查 Nav2 action 在线 (`ros2 action list`), 检查 goal 被 reject (坐标超出地图边界)
- **导航到错误位置**: `nav_pose` 坐标系是 `map` 不是 `odom`, 确认 `/odom` 在 map 系下 (TF map→odom→base_link 链通)
- **yaw 方向反了**: ROS REP-103 正=左转 (counterclockwise), Go2 SDK 正=右转——nx_motion_node 内部已反转, 编排层用 REP-103 弧度即可
- **航点太多搜不完**: 增大 `search_area.spacing` (减少行数), 或减小 `width/height`

## 示例 (客厅)

```yaml
- name: 客厅
  aliases: ["living room", "living", "起居室"]
  nav_pose:
    x: 2.53        # ros2 topic echo /odom --once → pose.x
    y: 1.82
    yaw: 0.05      # 2*atan2(qz, qw), 略偏左
  search_area:
    width: 4.8     # 实测对角距离
    height: 3.9
    origin_x: 1.1  # min(入口x, 对角x)
    origin_y: 0.4
    spacing: 1.5
    pattern: lawnmower
  target_classes: []
```

## 检查清单

- [ ] Nav2 在线 (`ros2 action list` 含 `/navigate_to_pose`)
- [ ] mock_nav2_action 已 kill
- [ ] `/odom` 在 map 系 (TF 链通)
- [ ] `config/rooms.yaml` 至少 3 个房间, 坐标非占位值
- [ ] `/api/reload_rooms` 返回 `ok:true`
- [ ] `/api/search_room?room=客厅` 狗真走到客厅
- [ ] F12 Console 看到 `type=mission_report` 报告

## Product Person Search Requirement

- Product command: `去搜索这个房间，把所有人标注出来`.
- The command parser should create a `search_room` task for the active room with `room: "__current__"`, `target_classes: ['person']`, `require_photos: true`, `mark_on_map: true`, `search_strategy: "next_best_view"`, and `use_lidar_person_range: true`.
- `__current__` resolves at mission start from the calibrated room that contains, or is nearest to, the robot pose. If no calibrated room can be resolved, the product flow should fail clearly instead of launching an unbounded search.
- Rooms used by this command must have calibrated `search_area` values in `config/rooms.yaml`: `origin_x`, `origin_y`, `width`, `height`, `spacing`, and `pattern`. The `next_best_view` planner should keep coverage goals inside that calibrated area.
- Person detection is constrained to `target_classes: ['person']`; reported people should use LiDAR-backed localization when range is available and publish map `person_markers` so operators can see every marked person.
