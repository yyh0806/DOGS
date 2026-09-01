"""lake_plan 配置: 缓存目录环境变量化 (测试用 fixtures 缓存的关键)。"""
from __future__ import annotations

import os
from pathlib import Path

_PACKAGE_DIR = Path(__file__).resolve().parent

# 瓦片缓存目录: 默认包内 cache/, 测试/部署用 GO2W_LAKE_CACHE_DIR 覆盖。
# 每次调用时读取 (而非 import 时固化), 便于测试注入与部署迁移。


def cache_dir() -> Path:
    return Path(os.environ.get("GO2W_LAKE_CACHE_DIR",
                               _PACKAGE_DIR / "cache"))


def offline() -> bool:
    """GO2W_LAKE_OFFLINE=1: 缓存未命中直接报错, 绝不联网。

    测试封闭性的强制开关 —— 防止"意外从网络补瓦片"让 golden 回归
    失去意义。规划期在线运行时不要设置。
    """
    return os.environ.get("GO2W_LAKE_OFFLINE", "") == "1"


# 瓦片源 (与 lake_loop 保持一致; water_rgb 是水域分割参考色)
PROVIDERS = {
    "osm": {
        "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        "crs": "wgs84",
        "water_rgb": (170, 211, 223),   # #aad3df
        "attribution": "(c) OpenStreetMap contributors",
    },
    "carto": {
        "url": "https://basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png",
        "crs": "wgs84",
        "water_rgb": (165, 191, 221),
        "attribution": "(c) OpenStreetMap contributors (c) CARTO",
    },
    "amap": {
        "url": "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}",
        "crs": "gcj02",                  # 高德瓦片是火星坐标
        "water_rgb": (166, 208, 238),
        "attribution": "(c) 高德地图",
    },
}
DEFAULT_PROVIDER = "osm"

MAX_TILES_PER_STITCH = 400      # 单次拼接瓦片上限 (防失控)
TILE_TIMEOUT = 20               # 单瓦片下载超时 (秒)
TILE_SLEEP = 0.05               # 相邻下载间隔 (秒), 礼貌抓取
USER_AGENT = "go2w-lake-plan/1.0 (robot patrol planner)"

# 规划默认值 (M2 巡查语义)
# 注: 周长上限只做"荒谬过滤"(几十km的大湖), 真正的续航约束由 M5 的
# 电量模型在任务层做 (分段/返航), 不在选湖阶段一刀切。
DEFAULT_LOOP_OFFSET_M = 15.0    # 离岸 15m (人行原型为 140m)
DEFAULT_STEP_M = 40.0           # 航点间距
DEFAULT_MAX_PERIM_KM = 12.0     # 中型城市湖泊 (~10km 周长) 在内; 特大湖被拒
DEFAULT_MIN_PERIM_KM = 0.3
COMPACT_MIN = 0.12              # 紧凑度下限 (低于视为河流/水渠)
DEFAULT_SCAN_SPACING_M = 150.0  # M5: 扫描点间距 (近岸密远岸疏, 后续自适应)
