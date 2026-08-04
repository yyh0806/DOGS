# 房间标定流程 (Go2W 阶段E 运维 SOP)

> 部署阶段E 后, 用本流程把 `config/rooms.yaml` 的占位坐标替换成真 Nav2 地图上的真实坐标。

## 前置

- 阶段D Nav2 已上线: `ros2 action list` 含 `/navigate_to_pose` (bt_navigator 起来了)
- FAST_LIO + map_server 建图完成 (静态地图 yaml + 全局坐标系 map)
- nx_motion_node + nx_sensor_node 跑着，`map→odom→base_link` TF 链健康
- mock_nav2_action.py 已 kill (用真 Nav2, 不再用 mock)

## 步骤 (每个房间)

1. **遥控狗到房间入口**
   - PC 浏览器开 `http://<NX_IP>:8000`
   - 用键盘/手柄控狗站到该房间的"搜索起点" (一般是门口或房间一角)

2. **记录 map 位姿（只读，不使用 `/odom` 坐标冒充 `map`）**
   ```bash
   # NX 上；工具只读取 tf2，不创建发布者、不发导航/运动命令
   source /opt/ros/humble/setup.bash
   python3 /home/nx/go2w/current/payload/tools/capture_map_pose.py \
     --room 客厅 --output /home/nx/go2w/validation/客厅-entry.json
   ```
   把输出中的 `yaml_nav_pose` 原样填进 `config/rooms.yaml` 对应房间。该坐标直接来自 `map→base_link`，不会受非零 `map→odom` 变换影响：
   ```yaml
   calibrated: false
   nav_pose: {x: <map_x>, y: <map_y>, yaw: <map_yaw>}
   ```

3. **标定搜索矩形 search_area**
   - 遥控狗到房间对角的另一个顶点，再运行一次 `capture_map_pose.py` 记录 map x/y
   - `width = |对角x - 入口x|`, `height = |对角y - 入口y|`
   - `origin_x = min(入口x, 对角x)`, `origin_y = min(入口y, 对角y)`
   - `spacing`: 房间越大行间距越大 (客厅 1.5m, 厨房 1.0m), 太密航点多搜索慢
   - `pattern`: 大房间用 `lawnmower` (弓字形), 小房间用 `spiral` (螺旋)
   - 所有值复核完成后，才把该房间的 `calibrated` 改为 `true`；缺失或为 `false` 时服务会在发任何导航目标前以 `room_uncalibrated` 拒绝

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
   - 地图看房间边界、候选/已访问视点与覆盖率；覆盖按相机水平视锥累计，默认达到 90% 才允许完成
   - Console 看 `type=mission_report` 报告 (含检测到的目标与覆盖率)

6. **标定 C13 水平视场角和 yaw 外参**
   - 将狗固定在已知 map 位姿，让一人站在正前方和已知左右角度，确保 MID360 对应方向有有效量程
   - 测出 C13 可见水平角并记录光轴相对 `base_link` 的 yaw 偏角(ROS 左转为正)
   - 用 `systemctl edit go2w-web` 写入 `GO2W_CAMERA_HFOV_C13_VIS_DEG` 与 `GO2W_CAMERA_YAW_OFFSET_C13_VIS_DEG`
   - 重启 web 后复测，地图人员 marker 应与人的已知位置重合；不要把默认 `70°/0°` 当作标定结果

## 常见问题

- **搜索时狗不动**: 检查 Nav2 action 在线 (`ros2 action list`), 检查 goal 被 reject (坐标超出地图边界)
- **导航到错误位置**: `nav_pose` 坐标系是 `map` 不是 `odom`；必须使用 `capture_map_pose.py`，并确认 TF `map→odom→base_link` 链通
- **yaw 方向反了**: ROS REP-103 正=左转 (counterclockwise), Go2 SDK 正=右转——nx_motion_node 内部已反转, 编排层用 REP-103 弧度即可
- **航点太多搜不完**: 增大 `search_area.spacing` (减少行数), 或减小 `width/height`
- **任务有照片但没有 marker**: 查看失败原因是否为 `no_lidar_range`;检查 C13 yaw/HFOV 外参、检测帧时间戳和对应方向的 MID360 量程
- **任务提前失败 `coverage_incomplete`**: 房间边界可能包含不可达区域，或默认 12 个视点不足；先修正边界/障碍与标定，再谨慎增加 `max_views`，不要直接降低覆盖阈值掩盖漏搜

## 示例 (客厅)

```yaml
- name: 客厅
  calibrated: true  # 仅在下列坐标全部实测并复核后设置
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
- [ ] 待搜索房间明确写有 `calibrated: true`；占位房间保持 `false`
- [ ] `/api/reload_rooms` 返回 `ok:true`
- [ ] `/api/search_room?room=客厅` 狗真走到客厅
- [ ] C13 的 HFOV/yaw 外参已经实测并写入 go2w-web drop-in
- [ ] 地图覆盖率达到阈值且人员 marker 与真实位置重合
- [ ] F12 Console 看到 `type=mission_report` 报告；未达覆盖率时看到明确失败而不是完成

## Product Person Search Requirement

- Product command: `去搜索这个房间，把所有人标注出来`.
- The command parser creates one canonical `search_room` request with `room: "__current__"`, `target_classes: ['person']`, `require_photos: true`, `mark_on_map: true`, `search_strategy: "frontier_explore"`, `max_radius_m: 6.0`, `max_time: 480.0`, and `use_lidar_person_range: true`.
- Current-room exploration does not consume placeholder room coordinates. It is bounded by radius, time, frontier-attempt budget, live map reachability and cancellation; it may therefore run before named rooms are calibrated.
- Named-room requests such as “去客厅搜索所有人” use `next_best_view` and must have `calibrated: true` plus measured `nav_pose`/`search_area`. Uncalibrated named rooms fail before any navigation goal is sent.
- Person detection is constrained to `target_classes: ['person']`; reported people should use LiDAR-backed localization when range is available and publish map `person_markers` so operators can see every marked person.
