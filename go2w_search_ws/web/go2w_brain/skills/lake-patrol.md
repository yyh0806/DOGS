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

3. **检测处置** (M4 起):
   - 扫描节奏: 每到扫描点调 scan_water(frames=3); 行进间单帧监视;
   - suspect (疑似): 记录不停步, 下个扫描点继续观察;
   - confirmed (确认, 含 WGS-84 坐标): speak 声明 + 报位置 →
     approach_vantage 选安全观察点 → 近距 scan_water 复查;
   - 汇报: 任务结束 get_detection_events(confirmed) 出清单
     (坐标/方位/距离/置信度), 配合守卫状态与航线完成度。

4. **任务结束必须报告**: patrol_report() 出完整清单 —— 环线/进度/
   扫描点/守卫/电量/两级告警 (confirmed 含坐标)。

5. **续航与接近** (M5 起):
   - 续航判决看规划输出里的 endurance: GO 直接走; SEGMENT 分段
     并设定返航点; REFUSE 只报告不上路; unknown 如实说明并请示;
   - 接近点取自环线 (构造性安全), 守卫兜底校验不过就放弃接近、
     保持远距观察。
