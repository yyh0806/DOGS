# M7.2 验收记录 — 观测回写 + 感知桥接 + 告警/播报通道 (feature/brain-m7p2-observation-alerts)

日期: 2026-09-01 · 基座: v1.0.0-alpha.6.1 · 目标 tag: v1.0.0-alpha.6.2

## 交付 (非真机路线推进)

| 组件 | 内容 |
|---|---|
| 观测回写钩子 | 事件契约 `{kind:"observation", mem_kind, geo, confidence, data}` → brain_loop 确定性写入记忆 (自由循环首轮 + 计划执行步骤间隙两处), 事件 observation_recorded 留痕; 非法种类/geo 静默忽略。真机来源 (运动链堵点回调/导航恢复/操作员标注) 只需 submit_event |
| TTS 接入点 | `go2w_brain/tts.py`: TtsBackend 协议 + ConsoleTtsBackend (演练) + EdgeTtsBackend (NX 真发声, edge-tts CLI 子进程, fail-soft); speak 工具经 ctx["tts"] 发声 (GO2W_TTS=console|edge), 无后端保持诚实桩语义 |
| nx_ai 检测注入适配 | `web/nx_detector_bridge.py`: make_yolo_person_detector (防御式归一化 bbox/xyxy/box×conf/confidence/score×cls/class/name, 元组形态, COCO class 0, 非法框换序容错, 只收 person 类) + make_vlm_verifier (chat→JSON 宽容解析, 关键词兜底, 解析失败保守否决) —— 契约闭合测试证明适配产物可直接驱动 DrowningDetector 五级管线 |
| 告警 WS 推送 | nx_web_server `POST /api/alerts` (audited, ws_broadcast force 广播 `lake_alert`) + `push_alert` 动词 + NxHttpAdapter.post_alert / Mock 记录 |

## 验收结果

| # | 验收项 | 结果 |
|---|---|---|
| 1 | M7.2 单测 (观测回写×2/TTS×4/nx适配×5/告警×2) | ✅ 13 项 |
| 2 | 全量回归 | ✅ 213/213 (brain 105) |
| 3 | 适配契约闭合 | ✅ 仿形 YOLO 输出直接驱动检测管线出告警 |
| 4 | nx_web_server 编译 | ✅ |

## 过程中发现并修复

- 观测事件自带 kind 字段 → SessionLog.append 首参冲突 (**第四次**同型坑) →
  统一 `_log_event` 折叠为 event_kind, 从此类事件形态全部走该助手;
- YOLO 元组形态 class id 0 (COCO person) 被字符串化过滤 → PERSON_CLASS_NAMES
  补 "0";
- 测试断言漏 .get() 于无 event 键的折叠事件 → 修正。

## 目标机待验证 (代码已备, 见各验收文档清单)

- [ ] nx_ai_node 装配: DrowningDetector(person_detector=make_yolo..., vlm_verify=make_vlm...) 接入 brain 会话;
- [ ] edge-tts CLI 安装与发声验证 (NX);
- [ ] 运动链堵点/恢复事件 → submit_event(observation) 接线;
- [ ] /api/alerts → 前端 lake_alert 卡片渲染。
