# lake_plan — 环线几何规划核心 (M2 / M2.1)

自 `lake_loop/` 原型迁移的服务化核心 (web/ 部署链内, NX 可跑)。
**确定性、无 LLM、无 langgraph**：lake_loop 仍是 PC 试验场, 本包只取其
规则路径并服务化。

## 双目标 (kind)

| kind | 语义 | 数据源 | 网络 |
|---|---|---|---|
| `water` | 绕**距离最近的合格水体**(小到园区景观湖, 大到天然湖泊) | 瓦片水域分割 (缓存优先) | 仅补瓦片 |
| `campus` | 绕**园区**(产业园/厂区: landuse 地块聚类 + 凸包轮廓) | Overpass 向量 (缓存优先) | 仅查 Overpass |

两者共用离岸外扩管线 (感知阶梯 **先近后远** z16→z14→z12,
让"最近合格目标"真正最近 —— 2026-08-31 园区坐标实测修复)。

依赖: `numpy` + `pillow`; 规划期才可能联网 (瓦片/Overpass 均带缓存)。

## 用法

```python
from lake_plan import plan_route

# 绕湖 (园区里的景观湖也能选中)
r1 = plan_route(31.488192, 120.369486, kind="water", offset_m=15.0)
# 绕园区 (当前点位所在的地块聚类凸包)
r2 = plan_route(31.488192, 120.369486, kind="campus", offset_m=15.0)
# 共同输出契约: waypoints[{"lat","lon","name"}](闭合) + polygon + target + stats
```

## 环境变量

| 变量 | 作用 |
|---|---|
| `GO2W_LAKE_CACHE_DIR` | 缓存根 (瓦片 `osm/` + Overpass `overpass/`) |
| `GO2W_LAKE_OFFLINE=1` | 缓存未命中直接报错, 绝不联网 (测试封闭开关) |

## 测试 (封闭, 零网络)

```bash
python -m pytest lake_plan/tests -q
```

golden 回归: 园区基准点双目标 (绕湖 12 航点/0.4km + 绕园区 27 航点/1.0km),
夹具 = 64 张 z16 瓦片 + 1 份 Overpass 响应, 全部入库。
有意变更规划行为时: `LAKE_PLAN_REGEN_GOLDEN=1` 重跑并更新基准,
commit message 必须说明原因。
