# M7.1 验收记录 — 记忆层深化 (feature/brain-m7p1-memory-deep)

日期: 2026-09-01 · 基座: v1.0.0-alpha.6 · 目标 tag: v1.0.0-alpha.6.1

## 交付 (用户委托: 非真机路线全部推进)

| 组件 | 内容 |
|---|---|
| 守卫多环 | WaterGuard arm=追加语义, evaluate/distance_m 取全环最小有符号距离, disarm(审批)清空, state 带 rings 计数 —— 主禁区与记忆危险带并存 |
| **hazard 自动布防** | arm_water_guard(from_plan) 确定性消费记忆 (不经 LLM): 规划区域内的 hazard 条目自动叠加 15m 圆形禁区环; 事件 hazard_auto_armed 留痕 —— "记忆先让狗更安全" |
| **L1 几何持久化** | plan_memory.py: 规划结果 → geometry 记忆 (环线/扫描点/多边形/统计, 半衰期 30 天, 新规划覆盖旧); try_reuse 命中即复用 |
| **二次任务免重复规划** | plan 工具先查记忆复用 (score≥0.8 且 kind 匹配) → 零瓦片/零 Overpass, 完全离线也可规划; 事件 geometry_reused |
| **计划失败重规划** | 步骤失败 → LLM 有且仅一次修订机会 (修订计划仍过六类强校验) → 继续执行; 失败/无 LLM → 诚实停手; 事件 plan_replanned |
| 控制台记忆面板 | /api/memory 端点 + 地图叠加层 (hazard 红/blocked 紫/vantage 绿/detection 橙/geometry 蓝色虚线环), 任务完成自动刷新 |

## 验收结果

| # | 验收项 | 结果 |
|---|---|---|
| 1 | M7.1 单测 (hazard 自动布防×3/几何复用×2/多环语义/重规划桩) | ✅ 7 项 |
| 2 | 全量回归 | ✅ 200/200 (brain 92) |
| 3 | **live 双任务演示** | ✅ 任务2: memory_retrieved=2 → LLM 计划 → GEOMETRY_REUSED → done |
| 4 | hazard 拦截语义 | ✅ 航点落在 hazard 环内 → waypoint_in_keepout 受理前拦截 |

## 过程中发现并修复

1. arm 工具取 plan_store 少一层 last_route → hazard 查询落空 (测试抓出);
2. hazard 拦截测试把 hazard 放在水塘中心 (环线本就不经过 → 不拦是正确
   行为), 改为放航点正上方 —— 测试自身语义修正;
3. geometry_persisted 事件在无记忆时也记录 → 仅在真实持久化成功时留痕。

## 已知边界 (转 M7.2/真机)

- nx 侧运行期守卫节点只同步主环 (hazard 环本地生效; 同步待目标机);
- geometry 复用保守: 30 天半衰期 + score≥0.8 才复用, 过期自动重规划;
- 重规划只执行一次, 失败即停 (确定性优先于灵活性)。
