# go2w_brain — DSH 式机器狗任务大脑 (M1 骨架)

移植 DeepSeek Harness 思想的机器狗大脑: LLM 只在慢回路做任务级决策,
能力 = 确定性工具 (动词) + 懒加载技能 (方法论), 安全层在快回路与大脑无关。

## 分层 (铁律)

```
慢回路 (本包): 秒级决策, 事件驱动, 断网退规则图, 全程 jsonl 轨迹
快回路 (Nav2/EKF/看门狗/运动链): LLM 永不进入, 大脑无权绕过
```

## 结构

| 文件 | 作用 |
|---|---|
| `registry.py` | 工具注册表 (scope 分层遮蔽) + Skill 目录 (Markdown 懒加载) |
| `prompt_assembler.py` | 有序分节提示词组装, 严格变量替换, 前缀稳定 |
| `dispatcher.py` | 派发安检: 未知动词拒绝 / 风险分级 / 前置 fail-closed |
| `session_log.py` | jsonl 轨迹: 可恢复可回放 |
| `llm.py` | DeepSeek function-calling (stdlib urllib, 零依赖) |
| `platform.py` | 世界状态接入: 读 (GET) + M2 运动写端口 (POST+令牌) |
| `brain_loop.py` | 事件驱动自由循环 + 规则退化 + 任务锁 + plan_store 引用传递 |
| `tools/` | M1: get_gps/get_battery/get_pose/speak(桩); M2: plan_lake_loop/follow_route/cancel_route/calibrate_heading/get_route_state; M2.1: plan_campus_loop(绕园区) |
| `skills/` | status-report / lake-patrol(占位) |

## 运行

```bash
cd go2w_search_ws/web

# 干跑 (无网络/无 ROS 可跑):
python -m go2w_brain.run_brain --task "报告当前状态" --platform mock --no-llm

# 真机/仿真 (NX 上, 读 nx_web_server 状态, 运动指令需 GO2W_CONTROL_TOKEN):
python -m go2w_brain.run_brain --task "..." --platform nx \
    --nx-url http://localhost:8000

# LLM function-calling 全链路干跑预演 (规划→标定→受理, 不下发运动):
python -m go2w_brain.run_brain --task "绕湖巡查预演..." \
    --platform mock --dry

# 有 DEEPSEEK_API_KEY 时省略 --no-llm; --dry = 运动类工具只记录不下发
```

## 测试

```bash
python -m pytest go2w_brain/tests -q     # 29 项, 全离线
```

## M1 验收口径

1. pytest 全绿 (注册表/安检/轨迹/组装/端到端干跑);
2. 离线规则模式"报告当前状态"全链路: 3 个只读工具调用 + 中文汇总;
3. 未知任务诚实拒绝, 零工具执行;
4. 每会话独立 jsonl 轨迹, 可回放 (`SessionLog.load_messages`)。

## 后续里程碑 (M2+)

M2 plan_lake_loop/follow_route (lake_loop 迁移为 web/lake_plan);
M3 arm_water_guard + keepout + 看门狗; M4 scan_water/detect_person_in_water;
M5 巡逻语义 + 告警闭环; M6 实机验收。技能文件按里程碑逐节增补。
