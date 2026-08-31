"""瓦片获取与拼接: 本地缓存优先 → 在线补抓 (stdlib urllib, 零 requests 依赖)。"""
from __future__ import annotations

import io
import time
import urllib.error
import urllib.request
from pathlib import Path

from PIL import Image

from . import config
from .config import MAX_TILES_PER_STITCH, PROVIDERS, TILE_SLEEP, TILE_TIMEOUT, USER_AGENT
from .geo import StitchGeoref, lat_to_global_px, latlon_to_tile, lng_to_global_px


class TileError(Exception):
    pass


def tile_cache_path(provider: str, z: int, x: int, y: int) -> Path:
    return config.cache_dir() / provider / f"{z}_{x}_{y}.png"


def fetch_tile(provider: str, z: int, x: int, y: int, use_cache=True) -> bytes:
    """取一张瓦片 PNG 字节。优先本地缓存。"""
    if not 0 <= x < 2 ** z or not 0 <= y < 2 ** z:
        raise TileError(f"瓦片坐标越界 z={z} x={x} y={y}")
    cache = tile_cache_path(provider, z, x, y)
    if use_cache and cache.exists():
        data = cache.read_bytes()
        if data:
            return data
    if config.offline():
        raise TileError(f"offline_cache_miss {provider}/{z}/{x}/{y}")
    prov = PROVIDERS.get(provider)
    if not prov:
        raise TileError(f"未知瓦片源 {provider}")
    url = prov["url"].format(z=z, x=x, y=y)
    last_err: Exception | None = None
    for attempt in range(3):
        try:
            request = urllib.request.Request(
                url, headers={"User-Agent": USER_AGENT,
                              "Accept": "image/png,image/*;q=0.8"})
            with urllib.request.urlopen(request,
                                        timeout=TILE_TIMEOUT) as resp:
                data = resp.read()
            if not data or len(data) < 100:
                raise TileError("空瓦片响应")
            cache.parent.mkdir(parents=True, exist_ok=True)
            cache.write_bytes(data)
            time.sleep(TILE_SLEEP)
            return data
        except (urllib.error.URLError, OSError, TileError) as exc:
            last_err = exc
            time.sleep(0.4 * (attempt + 1))
    raise TileError(f"下载失败 {provider}/{z}/{x}/{y}: {last_err}")


def fetch_tile_image(provider: str, z: int, x: int, y: int,
                     use_cache=True) -> Image.Image:
    return Image.open(io.BytesIO(
        fetch_tile(provider, z, x, y, use_cache))).convert("RGB")


def stitch_area(provider: str, z: int, x0: int, y0: int, nx: int, ny: int,
                use_cache=True, placeholder=True):
    """拼接 nx*ny 张瓦片。返回 (PIL.Image, detail dict)。"""
    if nx * ny > MAX_TILES_PER_STITCH:
        raise TileError(
            f"拼接瓦片数 {nx * ny} 超过上限 {MAX_TILES_PER_STITCH}")
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
    detail = {"provider": provider, "z": z, "x0": x0, "y0": y0,
              "nx": nx, "ny": ny, "w": nx * W, "h": ny * H,
              "tiles_ok": n_ok, "tiles_miss": misses,
              "crs": PROVIDERS.get(provider, {}).get("crs", "wgs84")}
    return canvas, detail


def stitch_bbox(provider: str, z: int, west, south, east, north,
                pad_tiles=0, use_cache=True):
    """按经纬度边界拼接。"""
    gx0 = int(lng_to_global_px(west, z) // 256)
    gx1 = int(lng_to_global_px(east, z) // 256)
    gy0 = int(lat_to_global_px(north, z) // 256)
    gy1 = int(lat_to_global_px(south, z) // 256)
    x0, nx = gx0 - pad_tiles, gx1 - gx0 + 1 + 2 * pad_tiles
    y0, ny = gy0 - pad_tiles, gy1 - gy0 + 1 + 2 * pad_tiles
    return stitch_area(provider, z, x0, y0, nx, ny, use_cache=use_cache)


def stitch_centered(provider: str, lat: float, lng: float, z: int,
                    nx: int, ny: int, use_cache=True):
    """以 (lat,lng) 为中心拼接 nx*ny 张瓦片。"""
    cx, cy = latlon_to_tile(lat, lng, z)
    x0 = cx - nx // 2
    y0 = cy - ny // 2
    return stitch_area(provider, z, x0, y0, nx, ny, use_cache=use_cache)


def georef_from_detail(detail: dict) -> StitchGeoref:
    return StitchGeoref(detail["z"], detail["x0"], detail["y0"],
                        detail["w"], detail["h"], detail.get("crs", "wgs84"))


def cache_stats():
    root = config.cache_dir()
    total = 0
    size = 0
    for p in root.rglob("*.png"):
        total += 1
        size += p.stat().st_size
    providers = sorted([d.name for d in root.iterdir()
                        if d.is_dir()]) if root.exists() else []
    return {"tiles": total, "bytes": size, "dir": str(root),
            "providers": providers}
