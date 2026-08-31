# M2 验收记录 — 规划接入 + 航线执行 (feature/brain-m2-lake-route)

日期: 2026-08-31 · 基座: v1.0.0-alpha.1 · 目标 tag: v1.0.0-alpha.2

## 交付范围

- `web/lake_plan/` 新包: 自 lake_loop 迁移的确定性规划核心
  (geo/tiles[urllib 化]/water/planner + route_api.plan_route), 零 LLM 依赖;
- `go2w_brain` M2 工具五件套: plan_lake_loop / follow_route / cancel_route /
  calibrate_heading / get_route_state;
- platform 写端口: NxHttpAdapter POST (Bearer GO2W_CONTROL_TOKEN) +
  MockAdapter 航线状态机;
- 任务锁 (mission_lock) 接入主循环: act 工具须在任务上下文内;
- plan_store 引用传递: 规划全量结果不进 LLM 上下文, follow_route(from_plan)
  直接引用 (上下文经济);
- 封闭测试夹具: 蠡湖 145 瓦片 (2.1MB) + golden 航线基准入库;
  `GO2W_LAKE_OFFLINE=1` 强制封闭开关;
- **nx_gps_nav.py / nx_web_server.py 零改动** (POST 契约已完备, 计划中的
  patrol 参数推迟到 M3 随仿真一起做)。

## 验收项与结果

| # | 验收项 | 口径 | 结果 |
|---|---|---|---|
| 1 | lake_plan 单测 (封闭) | `pytest lake_plan/tests` | ✅ 10/10, 2.6s, 零网络 |
| 2 | golden 航线回归 | 蠡湖固定瓦片 → 逐航点偏差 <1e-5°, 长度差 <0.5% | ✅ 跨重生成一致 |
| 3 | 选湖正确性 | 质心距中心 <2km (选中蠡湖而非远处水体) | ✅ 1.18km |
| 4 | 路线质量 | closed / water_cross<5% / 航点契约 lat-lon-name | ✅ 0.0 跨水 |
| 5 | brain 单测 | `pytest go2w_brain/tests` | ✅ 43/43 (M1 29 全保留) |
| 6 | GPS 回归 (存量模块) | `pytest tests/test_gps_nav.py` | ✅ 74/74, 零改动 |
| 7 | POST 契约 | 本地假服务器: 令牌校验/202/409 reason 透传/不可达 | ✅ 4 用例 |
| 8 | 任务锁前置 | follow_route 无锁 → mission_lock_required | ✅ |
| 9 | 干跑零下发 | dry 模式 mock.calls 无 submit | ✅ |
| 10 | **LLM 全链路演示** | 真 DeepSeek function-calling: 电量→规划→标定→干跑受理→汇报 | ✅ 4 工具 10.7s |

## 演示记录 (验收产物 runs/brain/20260831_143314_038467_038467_brain.jsonl)

LLM 自主序列: get_battery(72.5%) → plan_lake_loop(离岸15m,
313 航点/12.38km/闭合/0 跨水/选湖 1.18km) → calibrate_heading(12°) →
follow_route(from_plan, dry 受理 313 点无损耗) → 汇总表 +
主动安全提示 ("转真跑需确认 GPS 健康")。llm_used=True, steps=2。

## 过程中发现并修复的问题

1. 选湖上限 8km 误杀蠡湖 (粗扫周长估计 ~11km) → 上限调 12km,
   续航约束归还 M5 电量模型 (职责归位);
2. 负面测试前提错误 (太湖视野内仍有合规小水体) → 换成合成纯陆地图的
   monkeypatch 测试, 零瓦片依赖;
3. 测试曾从网络补瓦片污染夹具目录 → `GO2W_LAKE_OFFLINE` 强制封闭开关
   (把封闭性从纪律变成机制);
4. 313 航点经 LLM 上下文搬运会截断 → plan_store 引用传递;
5. M1 测试与 M2 内置工具撞名 (follow_route) → 注册表同层重名拒绝正确触发,
   测试改名。

## 已知限制 (转 M3+)

- min_shore_m ≈ 5.9m 偏小: 贴水安全由 M3 keepout 层兜底 (规划层只保证
  不入水, 不保证离岸精确 15m);
- Gazebo 湖景仿真未建 (M3): "仿真走线" 验收项以 GPS 控制器 74 项纯逻辑
  回归 + mock 状态机替代, 真仿真验收推迟至 M3;
- 规则退化模式仍只懂状态报告 (LLM 不可用时巡查任务诚实拒绝, 不硬编);
- NxHttpAdapter 字段仍为宽容提取, M3 随仿真状态契约收紧。
