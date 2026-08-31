"""lake_plan — 绕湖航线几何规划 (M2: 自 lake_loop 迁移的服务化核心)。

与 lake_loop 原型的关系:
- lake_loop/ 保留为 PC 试验场 (LangGraph + LLM 全流程演示);
- 本包是嵌入 NX 部署链 (web/ 目录) 的确定性核心: 无 LLM、无 langgraph
  依赖, 只做 瓦片(缓存优先) → 水域分割 → 选湖 → 细化 → 离岸环线。

依赖: numpy + pillow (NX 镜像内已有); 网络仅规划期瓦片补抓, 巡逻执行零网络。
"""
from .route_api import plan_route  # noqa: F401
