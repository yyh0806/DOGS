# Lake Loop — 瓦片地图服务器 + LangGraph 绕湖规划

以「当前坐标」为中心的瓦片地图服务器；瓦片拼接图直接作为多模态大模型输入；
用 LangGraph 编排思维链，对「绕湖走一圈」这类任务自动感知地图、选湖、
规划离岸环线并校验闭合性。

## 架构

```
                 ┌────────────────────────────────────────────┐
 用户任务 ──▶    │  LangGraph 思维链 (agent.py)                │
 "绕湖走一圈"    │  intent → perceive → select_lake → refine   │
                 │              ↑ (扩视野/搬家重扫)             │
                 │            plan → verify ──(不合格重规划)──┐ │
                 │              └────────────▶ output ◀──────┘ │
                 └───────┬──────────────┬───────────┬─────────┘
                         │ 瓦片          │ 结构化     │ 产物
                 ┌───────▼──────┐ ┌─────▼─────┐ ┌───▼──────────┐
                 │ tiles.py     │ │ water.py  │ │ planner.py   │
                 │ 瓦片服务器+缓存│ │ 水域分割   │ │ 外扩环线/导出  │
                 └───────┬──────┘ └───────────┘ └──────────────┘
                         │ PNG 拼接图（Georef 地理参照）
                 ┌───────▼──────────────────────┐
                 │ DeepSeek 多模态 (llm.py)      │
                 │ vision: 瓦片直接作为图像输入    │
                 │ text: reasoning_content 思维链 │
                 └──────────────────────────────┘
```

## 组件

| 文件 | 作用 |
|---|---|
| `server.py` | FastAPI 瓦片地图服务器（:8077）：/tile /stitch /georef /water /cache/stats |
| `tiles.py` | 多源瓦片抓取（OSM/CARTO/高德）+ 本地缓存 + 区域拼接，支持 GCJ-02↔WGS-84 |
| `geo.py` | Web-Mercator 像素↔经纬度、GCJ-02 转换、StitchGeoref 地理参照 |
| `water.py` | HSV+参考色水域分割、连通域、Moore 边界追踪、RDP 简化 |
| `llm.py` | DeepSeek 客户端：chat（含思维链）/ vision（图像输入）/ chat_json |
| `planner.py` | 湖岸多边形向外偏移(米)、重采样、闭合性/跨水率校验、GeoJSON/GPX/预览渲染 |
| `agent.py` | LangGraph 状态机：7 个节点 + 条件边（扩视野、搬家、重规划回路） |
| `run_demo.py` | 命令行入口 |

## 快速开始

```bash
cd lake_loop
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt

# 1) 启动瓦片地图服务器（保持运行）
.venv\Scripts\python server.py

# 2) 另开终端，运行绕湖任务（默认中心=无锡太科园，近太湖）
.venv\Scripts\python run_demo.py --task "绕湖走一圈"

# 指定坐标 / 底图 / 纯几何模式
.venv\Scripts\python run_demo.py --lat 31.5163 --lng 120.2673 --name 蠡湖
.venv\Scripts\python run_demo.py --provider amap
.venv\Scripts\python run_demo.py --no-llm
```

大模型 key：环境变量 `DEEPSEEK_API_KEY`，缺省读取 `~/.dsh/.credentials.yaml`。

## 产物（runs/<时间戳>/）

- `route.geojson` / `route.gpx` — 环线（含水域多边形），可直接导入地图软件
- `preview.png` — 底图上的路线预览（水域轮廓/路线/起点/公里标记/比例尺）
- `preview.html` — Leaflet 交互页（底图即本地瓦片服务器）
- `trace.json` / `summary.md` — 完整思维链轨迹与报告

## 瓦片服务器 API

```
GET /tile/{provider}/{z}/{x}/{y}.png       单瓦片（缓存）
GET /stitch?lat=&lng=&z=14&nx=6&ny=6       以坐标为中心拼接 PNG（附 X-Lake-Georef 头）
GET /stitch?bbox=west,south,east,north&z=  按边界拼接
GET /georef?...                            拼接图地理参照 JSON
GET /water?...                             水域掩码可视化
GET /cache/stats                           缓存统计
```

## 说明

- 太湖这类超大水体不适合步行绕圈：智能体会先扩视野确认，再迁移到
  附近合适的小湖（蠡湖等），全过程记录在思维链里。
- `--no-llm` 时全部决策退化为规则引擎，流水线仍然完整可跑。
- 底图版权：OSM (ODbL) / CARTO / 高德，仅本地演示用途。
