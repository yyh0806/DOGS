---
name: lake-patrol
description: 绕湖巡查任务的方法论骨架 (M1 占位, M2 起随规划/航线工具逐步填充完整流程)
when_to_use: 任务包含"绕湖/环湖/湖面巡查/落水"等字样时加载
---

# 绕湖巡查方法论 (骨架, 里程碑逐版填充)

> M1 只有只读工具, 本技能当前只描述流程形态; M2 接入
> plan_lake_loop/follow_route, M3 接入 arm_water_guard, M4 接入
> scan_water/detect_person_in_water, M5 接入 approach_vantage 后,
> 本文件按节增补, 每次增补都过验收门禁。

## 流程形态 (不变的部分)

1. **规划前核对电量**: get_battery() 与环线长度 × 单位能耗对照:
   - 余量 > 70%: 直接执行;
   - 40~70%: 分段执行并设定返航点;
   - < 40%: 只规划不执行, 报告原因。

2. **布防先于走线**: 运动类工具执行前必须确认水域 keepout 已装载
   (M3 起由 dispatcher 前置条件强制, 大脑不依赖自己记得)。

3. **告警两档** (M4 起):
   - suspect (仅检出): 记录, 不停步;
   - confirmed (VLM+多帧确认): 允许告警与接近观察, 全程不得进入 keepout。

4. **任务结束必须报告**: 环线闭合情况 / 检测结果 / 覆盖率 / 告警清单。
