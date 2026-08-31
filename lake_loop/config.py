"""全局配置：默认坐标、瓦片源、大模型、限制。"""
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
CACHE_DIR = BASE_DIR / "cache"
RENDER_DIR = CACHE_DIR / "renders"
RUNS_DIR = BASE_DIR / "runs"
for _d in (CACHE_DIR, RENDER_DIR, RUNS_DIR):
    _d.mkdir(parents=True, exist_ok=True)

# 『当前坐标』：无锡·新吴区·太科园片区（近太湖北岸）
DEFAULT_CENTER = (31.475, 120.372)
DEFAULT_LOCATION_NAME = "无锡·太科园"

# 瓦片服务器
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8077
SERVER_URL = f"http://{SERVER_HOST}:{SERVER_PORT}"
USER_AGENT = "lake-loop-agent/1.0 (local demo; contact: none)"

# 瓦片源。crs: 该源瓦片所用的坐标系统
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

MAX_TILES_PER_STITCH = 400      # 单次拼接瓦片上限（防失控）
TILE_TIMEOUT = 20               # 单瓦片下载超时(秒)
TILE_SLEEP = 0.05               # 相邻下载间隔(秒)，礼貌抓取

# 大模型（DeepSeek, OpenAI 兼容协议）
LLM_BASE_URL = "https://api.deepseek.com"
TEXT_MODEL = "deepseek-v4-flash"              # 文本推理（含思维链 reasoning_content）
VISION_MODEL = "deepseek-v4-flash-vision-exp"  # 视觉：瓦片直接作为图像输入
CREDENTIALS_FILE = Path.home() / ".dsh" / ".credentials.yaml"

# 规划默认值
DEFAULT_LOOP_OFFSET_M = 140      # 环湖线离岸默认距离(米)
DEFAULT_MAX_LAKE_PERIM_KM = 35   # 「步行绕一圈」可接受的最大湖周长(千米)
