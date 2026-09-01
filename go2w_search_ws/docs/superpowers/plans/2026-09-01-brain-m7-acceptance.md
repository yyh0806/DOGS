# M7 验收记录 — 语义记忆层 + 任务列表 (feature/brain-m7-semantic-memory)

日期: 2026-09-01 · 基座: v1.0.0-alpha.5 · 目标 tag: v1.0.0-alpha.6

## 用户架构决策

> 地图式记忆; 建图转成记忆; 指令 × 记忆 → 任务列表 (动词+参数级; 经验层起步)

## 交付

| 组件 | 内容 |
|---|---|
| `go2w_brain/memory.py` | 地图式经验记忆库: geo 键控网格索引 + append-only + 置信×半衰期衰减 (hazard 7d/blocked 3d/vantage 14d/fp_zone 3d/detection 7d) + 同类同源邻近新观测覆盖旧假设 (跨源不覆盖) + 跨会话持久 |
| `go2w_brain/task_plan.py` | 任务列表 (强语义): PlanStep DAG + **确定性强校验** (动词在注册表/参数过 schema/前置已注册/依赖可解析/无环/memory_refs 存在) + 规则组合器 (退化路径) + LLM 起草提示词与解析 |
| 动词 `get_memory` / `record_observation` | 记忆入能力边界: 检索/写入过 dispatcher, 全程留痕 |
| 回写管线 | scan_water→detection 自动记忆; approach_vantage→vantage 自动记忆; scan 前读 fp_zone/blocked 经验提示 |
| brain_loop 计划式执行 | 计划任务 (绕湖/绕园区/状态): 记忆检索注入 → LLM 起草 (结构化 JSON) → 强校验 (失败重试→规则兜底) → 拓扑序逐条过**既有 DispatchGate** 执行 → 失败诚实停手 → LLM/确定性汇报 |

## 验收结果

| # | 验收项 | 结果 |
|---|---|---|
| 1 | 记忆库单测 (写入/检索/过滤/衰减/冲突/跨源/持久) | ✅ 7 项 |
| 2 | 任务计划单测 (强校验 6 类拒绝/组合器/草稿解析) | ✅ 12 项 |
| 3 | 端到端规则模式 (绕湖/绕园区/状态/未知任务回退自由循环) | ✅ 4 项 |
| 4 | **记忆复利 (核心)**: 任务1 检索 0→写入 detection; 任务2 检索 1 | ✅ 双演示 + 单测 |
| 5 | campus 5m 一致性补漏 (follow_route 曾被守卫正确拦截) | ✅ golden 重基 |
| 6 | **LLM 起草计划 live 通过** (source=llm, 6 步含 load_skill) | ✅ 8.9s 全流程 done |
| 7 | 全量回归 | ✅ 193/193 |

## 过程中发现并修复

1. 环检测失效 (拓扑排序卡死时静默附加) → 部分序返回 + 长度比对判环;
2. `kind=` 关键字冲突第三例 (scan_water/approach_vantage/record 回写) —
   M2.1 的老坑, 现已三处清零;
3. run() finally 引用未定义 end_meta 掩盖真实异常 → 初始化兜底;
4. campus 分支缺规划-安全 5m 一致性 (水的修了园的漏) → 守卫正确拦截
   follow_route 暴露 → 补齐并重基 golden;
5. LLM 草稿三连败的真因 (逐一修提示词): from_plan 填成步骤 id →
   布尔语义提示; 发明未注册前置 → 白名单; load_skill 参数写成 skill →
   参数名提示。每次失败都是校验器在正确工作, 失败即退化规则计划, 全程
   可回放 (draft 原始输出入轨迹)。

## 设计纪律 (与用户共识)

- 记忆只**建议**, 安全类观测即时布防 (M7.1 起自动进 keepout);
- 任务列表与自由循环共用一条安检链, 计划失败诚实停手;
- 无 LLM 时规则组合器可产出并执行完整巡逻计划 (退化能力随 M7 首次
  覆盖巡逻类任务)。

## 后续 (M7.1+)

- [ ] hazard 类记忆自动进 keepout (安全层消费记忆);
- [ ] 执行期观测 (可走/堵) 在实机运动链的回写钩子;
- [ ] 计划失败后的 LLM 重规划 (当前诚实停手);
- [ ] L1 几何层持久化 (lake_plan 产物入库, 二次免重新规划)。
