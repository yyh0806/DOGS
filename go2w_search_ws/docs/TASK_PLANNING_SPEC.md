# 任务级自主规划 — 架构规格

> 创建于 feature/fetch-task-planner 分支。本文档是"取咖啡/找人/跟踪"系列任务的架构基线。
> 所有设计遵循项目核心哲学：**大模型只做高层决策（有超时、不危险），执行走确定性代码**。

## 1. 地图分层架构（L1/L2/L3）

| 层 | 内容 | 状态 | 用途 |
|----|------|------|------|
| L1 局部 SLAM | FastLIO 厘米级地图 | ✅ 已有 | 避障/DWB 唯一权威，无图区域由 frontier 探索自动补图 |
| L2 全局参考 | 狗 gnss/uslam → UTM 锚点 | ❌ 未接入（有意延迟） | 消除大范围 SLAM 漂移、跨任务坐标对齐。**等实测漂移证据再做** |
| L3 语义知识 | rooms.yaml + 地标注册表 + mission 报告 | 部分已有 | LLM 摘要的数据源："门口在哪、上次看到咖啡在哪" |

- **不做**：高德/百度地图 API（无室内语义 + GPS 误差致命）、自建瓦片服务器（解决渲染不是定位）。
- 取咖啡 demo 只依赖 L1 + L3 最小版（rooms.yaml 加 pickup 标定点）。

## 2. 任务模型（object 类型无关）

```
{intent: "fetch" | "find" | "follow",
 object: 任意自然语言描述,      # "咖啡杯" / "人" / "穿黑衣服的人"
 location: 地标名,              # "门口" → rooms.yaml 标定点
 deliver: 地标名 | "原位"}
```

| intent | 终态 |
|--------|------|
| fetch | 找到 → 人把物放上狗 → 返回交付（无臂送递模式） |
| find | 到达目标位置 → 报告（拍照/语音/面板标注） |
| follow | 持续跟踪目标（TargetTracker 已实现核心状态机） |

## 3. 视觉链路（两级 + 选择）

```
日常感知:  YOLO 8fps 常驻 (person/已知类别) —— 资源可控
目标定位:  locate-anything 按需单次 (任意自然语言, CPU GGUF, 串行锁)
属性选择:  "穿黑衣服的人" → locate prompt 属性短语 (模板已有 precedent)
指代消解:  "那个" → 多候选时 VLM 单帧问答选 bbox → tracker 锚定 (待开发)
跟踪:      ai/tracker.py TargetTracker (SEARCHING→TRACKING→RECOVERING) ✅ 已有
```

## 4. LLM 约束（fail-closed 模板填槽）

- LLM 输出只能填预定义模板槽位（`fetch/find/follow` + object + location），窄正则/解析器强校验。
- 任何解析失败/超时/网络错误 → fail-closed 拒绝，绝不产生自由动作。
- 物理安全第二道闸：motion 安全看门狗 + costmap 避障 + velocity_authorized。

## 5. Memory 桥接（LLM 与知识互动）

```
rooms.yaml / mission 报告 / frontier 状态
   → 确定性摘要生成器 (纯代码, 可测试)
   → 自然语言描述 ("门口已标定, 历史报告无咖啡, 需要现场寻找")
   → LLM 决策 (只读摘要)
   → 结构化意图 → 确定性执行器
```

- LLM 不直接读原始 JSON/点云；摘要生成器保证知识正确性。

## 6. 开发顺序（feature/fetch-task-planner）

| Commit | 内容 |
|--------|------|
| ① feat(nlu) | fetch/find/follow 意图模板 + NLU 契约扩展 |
| ② feat(planner) | nx_task_planner.py 状态机骨架（fetch: 导航→找→装载确认→返回） |
| ③ feat(nav) | S1 导航（复用 navigation_gateway + rooms.yaml 地标） |
| ④ feat(locate) | S2 视觉定位（locate-anything 单次 + 云台对准） |
| ⑤ feat(confirm) | S3/S4 装载确认（WS 提示 + 语音 + 等待确认） |
| ⑥ feat(return) | S5 返回交付 + fetch 端到端集成测试 |
| ⑦ feat(select-track) | 指代消解 select_and_track（"跟踪那个穿黑衣服的人"） |
| ⑧ feat(l3-summary) | Memory 摘要生成器 + LLM 桥接 |

- master 不直接修改；每个 commit 后契约测试必须绿；合并后打 tag。
