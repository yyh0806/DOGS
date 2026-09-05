"""prefetch_esri.py — 预热园区周边 esri 卫星瓦片到运行时缓存。

覆盖: 以园区中心为基准, z15/z16/z17 各 8×8, z18/z19 湖心 8×8/4×4。
用于回放页/控制台离线渲染 (在线抓取不稳时地图不灰)。
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "web"))
from lake_plan import tiles

CENTER = (31.488192, 120.369486)
POND = (31.489009, 120.369261)
PLANS = [
    ("esri", CENTER, 15, 8, 8),
    ("esri", CENTER, 16, 10, 8),
    ("esri", CENTER, 17, 8, 8),
    ("esri", POND, 18, 8, 8),
    ("esri", POND, 19, 4, 4),
]

ok = miss = 0
for provider, (lat, lng), z, nx, ny in PLANS:
    img, detail = tiles.stitch_centered(provider, lat, lng, z, nx, ny)
    ok += detail["tiles_ok"]
    miss += len(detail["tiles_miss"])
    print(f"{provider} z{z} {nx}x{ny}: ok={detail['tiles_ok']} "
          f"miss={len(detail['tiles_miss'])}", flush=True)
    time.sleep(0.3)
print(f"done: {ok} cached, {miss} missing")
