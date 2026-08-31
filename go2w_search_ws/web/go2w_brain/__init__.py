"""go2w_brain — DSH 式机器狗任务大脑 (M1: 骨架)。

设计原则 (移植 DeepSeek Harness 的思想, 非其代码):
- 工具 = 动词: 确定性代码 + JSON schema, 能力的硬边界; LLM 只能选择与填参;
- Skill = 方法论: Markdown 懒加载, 目录常驻上下文, 全文按需进场;
- LLM 只活在慢回路: 秒级决策, 事件驱动, 不轮询; 快回路/安全层绝不经过 LLM;
- 断网退化: LLM 不可用时走规则图, 工具照常可用, 不懂就老实报告;
- 全程 jsonl 轨迹: 可恢复、可回放 (对齐 lake_loop trace.json 思想)。

本包零第三方依赖、不 import 任何 ROS/业务模块: 世界状态只经
platform.py 的适配器进入 (M1: HTTP 只读 + Mock)。
"""
__version__ = "0.1.0"
