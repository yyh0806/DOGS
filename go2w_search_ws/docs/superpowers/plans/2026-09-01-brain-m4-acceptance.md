# M4 验收记录 — 落水检测管线 (feature/brain-m4-drowning-detect)

日期: 2026-09-01 · 基座: v1.0.0-alpha.3 · 目标 tag: v1.0.0-alpha.4

## 交付: 五级流水线 (纯逻辑核心, 可插拔 AI 壳)

`web/nx_drowning_detect.py` — DrowningDetector 有状态引擎:

| 级 | 拦截 | 实现 |
|---|---|---|
| A 检出 | 漏检 | 可插拔 person_detector (合成域=色彩候选; 生产=YOLO 注入) |
| B 水域 | 岸上的人 | 相机视角水面掩膜 (HSV 蓝窗+暗度) + bbox 邻域水占比 ≥0.55 |
| C 语义 | 漂浮物/游泳者 | 可插拔 vlm_verify (VLM_VERIFY_PROMPT 内置; 兜底=保守启发式) |
| D 时序 | 单帧幻觉 | 连续 2 帧 / 8s 窗 / 10m 世界位置簇 → 才 confirmed |
| E 定位 | — | bbox 方位+高度测距 → ENU → WGS-84 (≤120m, 低置信封顶) |

告警两级: suspect (A+B) / confirmed (+C+D)。配套: `scan_water` /
`get_detection_events` 工具, 合成巡逻帧源 (mock/演练), 技能方法论
M4 节, ci_gate 扩展。

## 验收结果 (合成数据集, 种子固定, 零网络)

| # | 验收项 | 阈值 | 结果 |
|---|---|---|---|
| 1 | recall @≤40m (10 例 3 帧序列) | ≥ 2/3 | ✅ 10/10 |
| 2 | 空水面误报 (10 例) | 0 事件 | ✅ 0 |
| 3 | 反光干扰 (6 例) | 0 confirmed | ✅ 0 |
| 4 | 漂浮白箱 (6 例) | 0 事件 (A 级色彩窗拦) | ✅ 0 |
| 5 | 岸上行人 (5 例) | 0 事件 (B 级水域拦) | ✅ 0 |
| 6 | 单帧只 suspect | D 级时序 | ✅ |
| 7 | VLM 否决压制 confirmed | C 级 | ✅ |
| 8 | 告警定位误差 | < 25m | ✅ |
| 9 | 工具级: scan_water 两级演进/无引擎 fail-closed/事件过滤 | — | ✅ 4 项 |
| 10 | 全量回归 | — | ✅ 164/164 |
| 11 | LLM 全链路演示 | 巡逻+扫描+汇报 | ✅ confirmed (31.488313, 120.369460) 方位-10.5° 距 13.6m 置信 0.99 + 同簇 suspect; LLM 主动声明 approach_vantage 缺失 (M5) 不硬编 |

## 过程中发现并修复

1. 岸上行人首版合成位置骑在岸线斜坡上 (头部悬于水面) → B 级判"在水中"
   是正确行为, 修的是生成器几何 (行人明确入绿地);
2. 测试种子用 list 哈希 → TypeError, 元组化。

## 目标机待验证 (M6)

- [ ] nx_ai_node 注入 YOLO person_detector + VLM worker 到本引擎
      (接口已备: person_detector / vlm_verify 可插拔);
- [ ] lake_patrol.world 双假人 (20m/70m) 实拍帧跑 recall;
- [ ] 真实水面反光/波浪数据回归校准 B 级阈值。
