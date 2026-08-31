# lake_plan — 绕湖航线几何规划核心 (M2)

自 `lake_loop/` 原型迁移的服务化核心 (web/ 部署链内, NX 可跑)。
**确定性、无 LLM、无 langgraph**：lake_loop 仍是 PC 试验场, 本包只取其
规则路径 (粗扫感知 → 选湖 → 高分辨率细化 → 离岸环线)。

依赖: `numpy` + `pillow` (NX 镜像已有); 网络仅规划期瓦片补抓。

## 用法

```python
from lake_plan import plan_route

result = plan_route(31.5163, 120.2673, offset_m=15.0)
# {"ok": true, "waypoints": [{"lat","lon","name"}...],   # 闭合环, 直接喂
#  "water_polygon": [[lat,lng]...],                       #   POST /api/gps/route
#  "lake": {...}, "stats": {...}}
```

## 环境变量

| 变量 | 作用 |
|---|---|
| `GO2W_LAKE_CACHE_DIR` | 瓦片缓存目录 (默认包内 `cache/`) |
| `GO2W_LAKE_OFFLINE=1` | 缓存未命中直接报错, 绝不联网 (测试封闭开关) |

## 测试 (封闭, 零网络)

```bash
python -m pytest lake_plan/tests -q
```

golden 回归: 固定蠡湖瓦片夹具 (145 张, 入库) → 逐航点与基准比对
(<1e-5 度 ≈ 1.1m)。有意变更规划行为时: `LAKE_PLAN_REGEN_GOLDEN=1`
重跑并更新基准, commit message 必须说明原因。
