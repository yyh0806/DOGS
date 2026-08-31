# M1 验收记录 — 大脑骨架 (feature/brain-m1-skeleton)

日期: 2026-08-31 · 基座: v0.97 (0815demo) · 目标 tag: v1.0.0-alpha.1

## 交付范围

`web/go2w_brain/` 新增包 (纯 stdlib, 零 ROS/业务依赖):
registry / prompt_assembler / dispatcher / session_log / llm / platform /
brain_loop / tools(get_gps, get_battery, get_pose, speak 桩) /
skills(status-report, lake-patrol 占位) / run_brain CLI / README /
tests(29 项) + `tools/ci_gate.sh` 门禁脚本。

## 验收项与结果

| # | 验收项 | 口径 | 结果 |
|---|---|---|---|
| 1 | pytest 全绿 (离线) | `python -m pytest go2w_brain/tests -q` | ✅ 29 passed |
| 2 | 干跑状态报告全链路 | mock + no-llm: 3 个只读工具调用, 中文汇总含经纬度/电量/位姿 | ✅ 见 runs/brain 轨迹 |
| 3 | 未知任务诚实拒绝 | "把门打开" → 拒绝且 tool_call 计数 = 0 | ✅ 轨迹 kinds 无 tool_call |
| 4 | 轨迹可回放 + 会话分离 | jsonl per-session (微秒时间戳), load_messages 重建 | ✅ 2 文件独立 |
| 5 | 未知动词是硬边界 | dispatcher 拒绝未注册工具 (open_door) | ✅ test_unknown_tool_rejected |
| 6 | 前置 fail-closed | 未实现的前置 → 拒绝 | ✅ test_missing_precondition_fails_closed |
| 7 | 审批分级机制 | approve 级无令牌拒绝 | ✅ test_approve_tool_requires_token |

## 已知限制 (下一里程碑处理)

- speak 为桩 (TTS/狗喇叭 M5 接入);
- NxHttpAdapter 字段宽容提取, M2 随 NX 状态契约收紧;
- 规则退化只懂"状态报告"类任务, 其余需 LLM 或后续里程碑扩展;
- 事件管道就绪但暂无外部事件源 (M2 follow_route 进度接入)。

## 回归

本里程碑未触碰任何既有模块 (全部新增), 室内搜索/GPS 航线/UWB 功能面
无改动; 完整回归安排在 M2 (首次触碰 nx_gps_nav) 与 M3 (首次触碰
nx_motion_node) 时按 ci_gate 扩展执行。

## 附: 本次提交

- feat(brain): 核心骨架 (registry/prompt_assembler/dispatcher/session_log/llm/loop)
- feat(brain): 平台适配 + M1 工具四件套 + 技能库 + CLI 入口
- test(brain): M1 单测 29 项 + ci_gate 门禁 + README
