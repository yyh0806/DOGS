# M3 验收记录 — 安全层 (feature/brain-m3-safety)

日期: 2026-08-31 · 基座: v1.0.0-alpha.2.1 · 目标 tag: v1.0.0-alpha.3

## 交付: 三层防御 + 一个一致性修复

| 层 | 机制 | 位置 |
|---|---|---|
| ① 规划期 | 航点距水域多边形 ≥5m 一致性推出 (veto2.0+margin2.0+1m 余量) | lake_plan/route_api |
| ② 受理期 | follow_route 前置 water_guard_armed + 逐航点禁区校验 (waypoint_in_keepout) | brain dispatcher+工具 |
| ③ 运行期 | WaterGuardClient 速度否决/限速 (注入 motion_controller nav 路径, 不经 Nav2) + nx_water_guard_node ROS 心跳 (5Hz, 过龄 armed 即 fail-closed) | bridge + web |

审批: disarm_water_guard 为 approve 级, 令牌 GO2W_BRAIN_APPROVAL_TOKEN /
GO2W_WATER_GUARD_TOKEN (操作员同源下发); web 服务器新增
POST /api/water_guard/{arm,disarm} (审计路径)。

## 验收结果

| # | 验收项 | 结果 |
|---|---|---|
| 1 | WaterGuard 纯逻辑 (12 项: 三档裁决/有符号距离/margin/过龄/审批) | ✅ |
| 2 | WaterGuardClient (8 项: 否决/限幅/过龄 fail-closed/解除恢复/坏 JSON) | ✅ |
| 3 | bridge 回归 (motion_safety 6 + motion_controller 9, 可选注入零破坏) | ✅ 23/23 |
| 4 | 全量本地 | ✅ 151/151 (brain 52 + guard 12 + lake_plan 13 + GPS 74) |
| 5 | **恶意航线拦截** | ✅ 禁区内航点 → waypoint_in_keepout, 零下发, 干跑照拦 |
| 6 | 守卫前置 | ✅ 未布防 follow_route 双闸拒 (dispatcher + 工具自检) |
| 7 | 审批解除 | ✅ 无令牌/错令牌拒, 对令牌过 |
| 8 | LLM 闭环演示 | ✅ 三幕: ①撞前置→自行布防→再撞禁区校验(1.9m)→正确停手请示; ②修复后完整通过 (加载技能→布防先于走线→受理→报告) |

## 过程中发现并修复的问题 (演示驱动)

1. **规划-守卫阈值不一致** (核心发现): offset 15m 环线圆角处贴水 1.9m,
   守卫正确拒收 → 规划端 ≥5m 一致性推出; 3m 不够 (布防 margin 抬高
   有效阈值), 实测修正为 5m;
2. LLM 三次调用不存在的 load_skill 被硬边界拒绝 (正确), 但暴露接口
   缺失 → 补 load_skill 工具 (DSH 懒加载的模型接口本就该存在);
3. 测试基础设施: M2 测试默认守卫环与园区水域重叠 → 挪远;
   _NullLog 同签名化后 kind= 冲突类 bug 单测即炸。

## 目标机待验证 (WSL2/NX, 无法在 Windows 执行)

- [ ] lake_patrol.world 冒烟: 水面无激光回波 (scan 空洞), 假人可见;
- [ ] nx_water_guard_node 心跳 → motion 状态灯 water_guard_reason;
- [ ] 守卫布防后向水域 waypoint 发Nav2 goal → 绕行 (keepout 层) +
      强推 cmd_vel → 清零 (motion 层);
- [ ] kill 守卫节点 → 心跳过龄 → nav 速度清零 (fail-closed);
- [ ] nav2_params_water_guard.yaml 叠加进 outdoor profile 后室内回归。
