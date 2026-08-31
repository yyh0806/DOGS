"""瓦片获取与拼接：本地缓存 -> 在线抓取 -> 失败占位。"""
import hashlib
import io
import time
from pathlib import Path

import requests
from PIL import Image

import config
from config import PROVIDERS, CACHE_DIR, USER_AGENT, MAX_TILES_PER_STITCH, TILE_TIMEOUT, TILE_SLEEP
from geo import StitchGeoref

_session = requests.Session()
_session.headers.update({"User-Agent": USER_AGENT,
                         "Accept": "image/png,image/*;q=0.8"})


class TileError(Exception):
    pass


def tile_cache_path(provider: str, z: int, x: int, y: int) -> Path:
    return CACHE_DIR / provider / f"{z}_{x}_{y}.png"


def fetch_tile(provider: str, z: int, x: int, y: int, use_cache=True) -> bytes:
    """取一张瓦片 PNG 字节。优先本地缓存。"""
    if not 0 <= x < 2 ** z or not 0 <= y < 2 ** z:
        raise TileError(f"瓦片坐标越界 z={z} x={x} y={y}")
    cache = tile_cache_path(provider, z, x, y)
    if use_cache and cache.exists():
        data = cache.read_bytes()
        if data:
            return data
    prov = PROVIDERS.get(provider)
    if not prov:
        raise TileError(f"未知瓦片源 {provider}")
    url = prov["url"].format(z=z, x=x, y=y)
    last_err = None
    for attempt in range(3):
        try:
            r = _session.get(url, timeout=TILE_TIMEOUT)
            r.raise_for_status()
            data = r.content
            if not data or len(data) < 100:
                raise TileError("空瓦片响应")
            cache.parent.mkdir(parents=True, exist_ok=True)
            cache.write_bytes(data)
            time.sleep(TILE_SLEEP)
            return data
        except Exception as e:  # noqa: BLE001
            last_err = e
            time.sleep(0.4 * (attempt + 1))
    raise TileError(f"下载失败 {provider}/{z}/{x}/{y}: {last_err}")


def fetch_tile_image(provider: str, z: int, x: int, y: int, use_cache=True) -> Image.Image:
    return Image.open(io.BytesIO(fetch_tile(provider, z, x, y, use_cache))).convert("RGB")


def stitch_area(provider: str, z: int, x0: int, y0: int, nx: int, ny: int,
                use_cache=True, placeholder=True):
    """拼接 nx*ny 张瓦片。返回 (PIL.Image, dict)。dict 为地理参照 detail。"""
    if nx * ny > MAX_TILES_PER_STITCH:
        raise TileError(f"拼接瓦片数 {nx * ny} 超过上限 {MAX_TILES_PER_STITCH}")
    W = H = 256
    canvas = Image.new("RGB", (nx * W, ny * H), (240, 240, 234))
    n_ok = 0
    misses = []
    for row in range(ny):
        for col in range(nx):
            x, y = x0 + col, y0 + row
            try:
                tile = fetch_tile_image(provider, z, x, y, use_cache)
                canvas.paste(tile, (col * W, row * H))
                n_ok += 1
            except TileError:
                misses.append((x, y))
                if not placeholder:
                    raise
    detail = {"provider": provider, "z": z, "x0": x0, "y0": y0, "nx": nx, "ny": ny,
              "w": nx * W, "h": ny * H, "tiles_ok": n_ok, "tiles_miss": misses,
              "crs": PROVIDERS.get(provider, {}).get("crs", "wgs84")}
    return canvas, detail


def stitch_bbox(provider: str, z: int, west, south, east, north, pad_tiles=0,
                use_cache=True):
    """按经纬度边界拼接（wgs84 或该源 crs 的等价边界交给 StitchGeoref 处理）。"""
    from geo import lng_to_global_px, lat_to_global_px
    gx0 = int(lng_to_global_px(west, z) // 256)
    gx1 = int(lng_to_global_px(east, z) // 256)
    gy0 = int(lat_to_global_px(north, z) // 256)
    gy1 = int(lat_to_global_px(south, z) // 256)
    x0, nx = gx0 - pad_tiles, gx1 - gx0 + 1 + 2 * pad_tiles
    y0, ny = gy0 - pad_tiles, gy1 - gy0 + 1 + 2 * pad_tiles
    return stitch_area(provider, z, x0, y0, nx, ny, use_cache=use_cache)


def stitch_centered(provider: str, lat: float, lng: float, z: int, nx: int, ny: int,
                    use_cache=True):
    """以 (lat,lng) 为中心拼接 nx*ny 张瓦片。"""
    from geo import latlon_to_tile
    cx, cy = latlon_to_tile(lat, lng, z)
    x0 = cx - nx // 2
    y0 = cy - ny // 2
    return stitch_area(provider, z, x0, y0, nx, ny, use_cache=use_cache)


def georef_from_detail(detail: dict) -> StitchGeoref:
    return StitchGeoref(detail["z"], detail["x0"], detail["y0"],
                        detail["w"], detail["h"], detail.get("crs", "wgs84"))


def cache_stats():
    total = size = 0
    for p in CACHE_DIR.rglob("*.png"):
        total += 1
        size += p.stat().st_size
    return {"tiles": total, "bytes": size,
            "dir": str(CACHE_DIR), "providers": sorted(
                [d.name for d in CACHE_DIR.iterdir() if d.is_dir()])}
