# M7.3 验收记录 — 卫星底图 + VLM 语义锚定 (feature/brain-m7p3-semantic-anchor)

日期: 2026-09-01 · 基座: v1.0.0-alpha.6.2 · 目标 tag: v1.0.0-alpha.6.3

## 目标 (用户原话落地)

"卫星地图, 比较高层级的, 通过多模态大模型或 locate-anything 进行语义分割;
**首先确定我们本体在哪, 湖指的是哪个 —— 语义没有分歧**, 再规划。"

## 交付

| 组件 | 内容 |
|---|---|
| Esri 卫星底图 | `config.PROVIDERS["esri"]` (World_Imagery, WGS-84); 控制台/回放页底图切 esri, `/tiles/{provider}/{z}/{x}/{y}` 端点 provider 感知 + 魔数嗅探 content-type (esri 实为 JPEG) |
| 语义锚定核心 | `lake_plan/semantic_anchor.py`: 卫星拼接图上绘制本体绿圈("本体"字标)与候选水体红圈(大字号编号) → 多模态大模型回答 `{self_near_water, target_idx, target_name, ambiguity[], why}`; VLM 缺席/失败/卫星覆盖<50% 一律诚实降级规则锚定 (最近水体 + GNSS=本体), `source=rule` + `why` 全程留痕 |
| VLM 客户端 | `go2w_brain/vlm.py`: OpenAI 兼容 vision 调用 (key 复用 BrainConfig 凭据, `GO2W_VLM_MODEL` 可换), `parse_json_loose` 宽容解析 (围栏/前缀/推理尾巴) |
| 语义纠正闭环 | VLM 认定任务所指是另一候选水体 → 以其质心 `prefer` 重规划一次 (至多一次), 事件 `anchor_replan`; 第二次仍分歧则 `resolved_to_plan=false` 如实上报, 绝不静默 |
| 锚定入记忆 | geometry 记忆条目增存 `anchor` (哪片水体=任务湖、来源、歧义清单) —— 复用几何时锚定证据随行 |
| 接线 | `BrainSession(vlm=...)` → ctx["vlm"]/ctx["task"]; `run_brain.build_session` 构造 VLMClient; 控制台事件 zh 标签 (`语义锚定`/`锚定纠正重规划`, 含来源/目标/歧义摘要) |

## 验收结果

| # | 验收项 | 结果 |
|---|---|---|
| 1 | M7.3 单测 (lake_plan 语义锚定×14 + brain 接入×2: 规则退化/VLM 解析宽容/覆盖拒喂/prefer 纠正选湖) | ✅ 16 项 |
| 2 | 全量回归 | ✅ 229/229 web + 23 bridge = 252 |
| 3 | 真 VLM 锚定实测 | ✅ 见下 (source=vlm, 目标/歧义与几何一致) |
| 4 | 实时控制台全链路 (LLM 计划 → VLM 锚定 → 布防 → 走线 → 扫描 → 报告) | ✅ 轨迹 `runs/brain/20260901_233348_510097_live.jsonl`, 回放页 `/runs/brain/20260901_233348_510097_live.html` |

### 真 VLM 锚定实测 (无锡太科园, 7 个候选水体)

```
semantic_anchor: source=vlm, target_idx=0, resolved_to_plan=true,
  ambiguity=[3,4,6],
  why="绿色圆圈（机器人）紧邻编号0的圆形小湖泊，位于公园内；
      编号1、2、5为线状河道，不是湖泊；编号3、4、6也是小水塘/小湖，可能产生歧义。"
```

- 本体确认: GNSS (31.488192, 120.369486) 与影像绿圈一致; 湖确认: #0=公园内湖 (规则选湖同解), 歧义水塘如实列出;
- 全程事件留痕: `semantic_anchor` → `geometry_persisted` → `water_guard_armed` → `follow_route_dry` → `patrol_report`。

## 过程中发现并修复

- **推理型视觉模型 token 预算坑** (最重要): `deepseek-v4-flash-vision-exp` 是推理模型, `max_tokens=400` 全被 `reasoning_content` 吃掉, `finish_reason=length` 且 `content=""` → 锚定从未真正生效 (被规则兜底掩盖)。修复: 默认 1024 + reasoning 尾巴兜底解析, 锚定调用给 4096;
- **标记不可辨**: z16 上本体绿圈与最近水体红圈近乎重叠、编号用默认位图小字 → VLM 两次给出互相矛盾的 idx(3 vs 4)。修复: 重绘标记 (绿圈 r18+字标 / 红圈 r20+42px 白字红描边编号), 提示词明确"圈大小不代表水体大小", 修复后稳定 target_idx=0;
- 占位灰底图 (tiles_ok=0) 曾直接喂 VLM → 覆盖率<50% 拒喂, 降级规则锚定;
- VLM 与规则分支 `why` 字段层级不一致 → 统一顶层;
- esri 瓦片 JPEG 按 image/png 输出 → 魔数嗅探; Windows GBK 控制台 emoji 崩 print → stdout reconfigure utf-8;
- 运行时瓦片缓存与测试夹具分离: `fixtures/cache` 保持封闭无 esri (离线锚定测试依赖其缺席), 运行时用 `lake_plan/cache` (被 `*.png` gitignore 覆盖), 演示级 esri z16 80 张已预热。

## 目标机待验证 (代码已备)

- [ ] NX 上以 locate-anything 类分割引擎并联/替代 VLM (`map_segmenter` 接口已预留: `segment_water(image) -> [{bbox,score,label}]`);
- [ ] 真机 GNSS 定位置信度接入 `self.confirmed` (当前 dry-run 恒 true);
- [ ] 部署环境卫星瓦片离线预取策略 (大面积巡逻前按候选 bbox 预热);
- [ ] VLM 供应商/模型的 NX 端选型与限流预算 (当前 deepseek-v4-flash-vision-exp)。
