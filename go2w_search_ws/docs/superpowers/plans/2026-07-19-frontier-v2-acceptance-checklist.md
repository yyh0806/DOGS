# 封闭区域 Frontier 探索 v2 — 实机验收清单

> 日期: 2026-07-19 | 分支: `codex/product-room-person-search` | HEAD: `150529a` | 48/48 离线测试通过
> 配套: spec/plan v2 (`docs/superpowers/{specs,plans}/2026-07-19-bounded-frontier-room-exploration-*.md`)
> 目的: 验证 v2 三大核心（时间对齐 / ROI 覆盖率 / completion 四态）在实机成立，并采集 deferred 项（I3 stall watchdog）的设计依据。

## 0. 前置（记忆引用）

- NX 在线：ARP 找 IP（`network-topology-1x.md`），ssh 验真（ping/ARP 可能假阳性）
- livox 稳 10Hz：`livox-rmem-drop.md`（rmem 32MB/8MB 已部署）
- FastLIO 不发散：`fastlio-ntp-jump-divergence.md`（等 `NTPSynchronized=yes` 再起 fastlio）
- nav2-3d active（导航团队负责）：`systemctl is-active go2w-slam-nav`
- map_padding_bridge 跑着（产 `/map_frontier`，2m padding）：`ros2 topic echo /map_frontier --once | head`

## 1. 部署 v2 到 NX

```bash
cd go2w_search_ws
# build_release.sh 的 copy_web_runtime() 用 glob web/nx_*.py
# → 自动收录新增 nx_coverage_metrics.py (无需手动加清单)
bash docker/build_release.sh web                    # 产出 web release artifact
# deploy_release.sh 含 verify/install/switch/health-check/回滚
NX_HOST=<NX_IP> NX_USER=nx bash docker/deploy_release.sh <artifact.tar.gz>
```

**部署验证**（确认新代码到位）：
```bash
ssh nx@<NX_IP> "systemctl is-active go2w-web && \
  python3 -c 'import sys; sys.path.insert(0,\"/home/nx/go2w/current/payload/web\"); \
  import nx_coverage_metrics; print(\"coverage OK\", nx_coverage_metrics.compute_coverage)' && \
  grep -c _derive_completion_status /home/nx/go2w/current/payload/web/nx_room_orchestrator.py"
```
期望：`active` + `coverage OK <function>` + `_derive_completion_status` 出现 ≥1 次。

## 2. 触发封闭房间探索

- 前端 panel "搜索当前房间"，或语音 "搜索房间里所有人"
- `nx_product_command.py:244` current_room → `frontier_explore` 策略，`max_radius_m=6.0`，`max_time_s=480`
- 狗从进门位姿开始，frontier 探索预期 5-15 个 viewpoint，frontier 耗尽退出

## 3. 验收点 — mission_report 字段

任务结束前端推 `type=mission_report`。**关键字段 + 期望**：

| 字段 | 期望 | 异常含义 |
|---|---|---|
| `completion_status` | `completed` 或 `completed_with_gaps` | `coverage_unverified`=地图/ROI 缺；`incomplete`=预算耗尽 |
| `explored_ratio` | ≥ 0.85（实机 ROI 校准，可能 < 0.90 → completed_with_gaps） | 远低于此 = ROI 设置错或 padding 没裁掉 |
| `coverage_valid` | true | false = /map_frontier 无数据 |
| `roi` | `{"type":"circle","center":[x,y],"radius":6.0}` | 命名房间应是 polygon |
| `enclosed_unknown_regions` | 0 或少量小 bbox（被障碍围死的死角） | 大量 = 判定过宽，需调 inflation |
| marker 列表 `source` | 含 `en_route`（不止到点的） | 全空 = en-route 路径没走通 |
| marker `position_quality` | 多数 `range_lidar` | 全 `bearing_only` = lidar range 没对齐 |

**诊断命令**：
```bash
ssh nx@<NX_IP> "journalctl -u go2w-web -n 500 --no-pager | \
  grep -E 'completion_status|explored_ratio|enclosed|en-route|coverage|frontier'"
```

## 4. v2 核心验证（必看）

1. **时间对齐（审核 #1）**：en-route marker 的 `(world_x, world_y)` vs 人真实站位，误差应 < 0.5m。让人站在已知位置，狗经过，看 marker 落点。
2. **移动中标注**：marker 列表里必须有 `source: en_route` 的项（不止 `source` 缺失/到点的）。证明 worker 在导航期间真的采到。
3. **ROI 覆盖率（审核 #3）**：`explored_ratio` 应反映真实房间覆盖（走完一间房应 > 0.7），**不是被 padding 灌到 ~0.31**。若仍 ~0.31 说明 ROI 没生效或 max_radius 太小。
4. **completion 四态（审核 #5）**：frontier 耗尽时，若房间走完 → `completed`；有 enclosed 死角 → `completed_with_gaps`。**不应**只看 frontier 耗尽就 `completed` 而 explored_ratio 很低。

## 5. deferred 项实机观察（采集设计依据）

- **I3 feeder stall**：log grep `observation synchronization failed` / `en-route bundle build failed` 频率。若高频 → 实机需加 30s 无-fresh-bundle watchdog。典型 stall 源：`livox-rmem-drop.md` / `fastlio-ntp-jump-divergence.md` 描述的场景。
- **I2 drain on cancel**：手动 cancel 任务，对比 cancel 时刻 vs mission_report 的 marker 数（应不丢最后 50ms 的检测）。
- **cloud 证据（I1 已修）**：log 里 `bundle.cloud` 不应全 None；人员 marker 若有 z 高度信息说明 cloud 生效。

## 6. 失败应对

| 现象 | 排查 |
|---|---|
| `completion_status=coverage_unverified` | `/map_frontier` 无数据：查 map_padding_bridge + nav2-3d；或 ROI 圆心/半径异常 |
| `completion_status=incomplete` | 预算耗尽：`max_time_s=480` 不够？房间太大调大；或单 goal 卡死吃预算 |
| `explored_ratio` 异常低（<0.4） | ROI 没裁掉 padding：确认 `roi.radius` 是 6.0 不是整图；或 mission_origin 错 |
| 无 `en_route` marker | observation_sync 没 pose/scan：查 nx_web_server 的 `/localization_pose` + `/scan_mid360` 订阅；或 worker 线程没起（log grep `en-route worker`） |
| marker 位置飘 | 时间对齐失效：log grep `bundle_for_detection` + `tolerance`；captured_at 是否超 0.20s 容差 |
| en-route 大量 frame 囤积 | bounded queue 失效：确认 `GO2W_EN_ROUTE_MAX_SAMPLES` 默认 12；log 看内存 |

## 7. 实机跑完贴这些回来，我帮分析

1. **mission_report JSON 全文**（completion_status / explored_ratio / enclosed_unknown_regions / coverage_*_cells / marker 列表）
2. `journalctl -u go2w-web | grep -E 'en-route|coverage|completion|frontier|observation_sync'` 关键行
3. 任何 ERROR/WARNING + 现象描述（狗走了几个 viewpoint？人在哪？marker 落点对不对？）

## 8. 验收通过的判据（v2 三核心 + 不回归）

- ✅ `completion_status ∈ {completed, completed_with_gaps}` 且 `explored_ratio` 合理（>0.7）
- ✅ en-route marker 存在且位置误差 < 0.5m（时间对齐成立）
- ✅ frontier 耗尽正常 REPORT，输出含 coverage 的 mission_report
- ✅ 不回归：狗不发 `/cmd_vel`（走 Nav2 goal）、pose 丢失能恢复、park/estop 不误触

通过后：deferred 项（I3 watchdog 等）按实机 log 设计，开 follow-up Task。
