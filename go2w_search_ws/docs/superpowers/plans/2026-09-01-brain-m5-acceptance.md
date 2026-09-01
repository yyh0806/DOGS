# M5 验收记录 — 巡逻语义闭环 (feature/brain-m5-patrol-semantics)

日期: 2026-09-01 · 基座: v1.0.0-alpha.4 · 目标 tag: v1.0.0-alpha.5

## 交付

| 组件 | 内容 |
|---|---|
| lake_plan 扫描点 | `_scan_points`: 沿环线每 150m 一个, 附朝目标质心的方位角 (湖法线, 云台扫视基准); 双 kind 输出 `scan_points` 字段 |
| patrol_math | `endurance_check` (能耗 4%/km × 1.25 停顿系数 → GO/SEGMENT/REFUSE + ETA); `route_completion`; `nearest_route_point` |
| approach_vantage 工具 | 接近点 = 环线距告警最近航点 (构造性在 keepout 外) + 守卫矢量兜底; 校验不过 → 放弃接近 |
| patrol_report 工具 | 目标/环线/扫描点/进度%/守卫/电量/两级告警清单 → markdown + 轨迹留痕 |
| 规划工具升级 | plan_lake_loop / plan_campus_loop 输出 endurance 判决与扫描点数 |

## 验收结果

| # | 验收项 | 结果 |
|---|---|---|
| 1 | 单测 | ✅ 全量 171/171 (brain 63 含 M5 新 8 项) |
| 2 | 扫描点 | ✅ 数量随环长, 方位字段 [0,360) |
| 3 | 续航三档 | ✅ GO/SEGMENT/REFUSE/unknown 边界用例 |
| 4 | 接近点安全 | ✅ 距告警>5m + 守卫距离达标; 无规划/守卫兜底不过均拒绝 |
| 5 | 报告完整性 | ✅ 含进度%/扫描点/告警坐标清单 |
| 6 | **LLM 全流程演示** | ✅ 8 步 16 工具调用 53.6s: 电量→规划(引用续航 72.5%→70.5%)→标定→布防→受理→扫描(confirmed 0.99)→approach_vantage→patrol_report; 主动引用"布防先于走线"铁律, 干跑声明诚实 |

## 目标机/仿真待接入 (M6)

- [ ] 云台调度器消费 scan_points.look_bearing_deg (nx_gimbal_node);
- [ ] approach_vantage 接 go-to 执行 (实机运动链);
- [ ] 告警 WS 推送 + Web UI 任务卡 (nx_web_server 前端);
- [ ] speak 接 TTS/狗喇叭;
- [ ] lake_patrol.world 端到端 (语音→巡逻→检测→报告);
- [ ] 续航系数实机标定 (4%/km 为估值)。
