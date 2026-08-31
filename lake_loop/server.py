"""瓦片地图服务器 (FastAPI)。

端点：
  GET /                     服务信息
  GET /tile/{provider}/{z}/{x}/{y}.png        单张瓦片（带缓存）
  GET /stitch?lat=&lng=&z=&nx=&ny=&provider=  以坐标为中心拼接，返回 PNG
  GET /stitch?bbox=w,s,e,n&z=                 按边界拼接，返回 PNG
  GET /georef?...同 /stitch                   返回拼接图地理参照 JSON
  GET /water?...同 /stitch                    水域掩码可视化 PNG
  GET /static/renders/{name}                  渲染结果（HTML/预览图）
  GET /cache/stats                            缓存统计

说明：/stitch 同时写入 X-Lake-Georef 响应头（URL-safe base64 JSON），
     瓦片图可以直接作为大模型的图像输入。
"""
import base64
import json
import urllib.parse

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from PIL import Image, ImageDraw

import config
import tiles
import water
from config import DEFAULT_CENTER, DEFAULT_PROVIDER, PROVIDERS

app = FastAPI(title="Lake Loop Tile Server", version="1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"],
                   allow_headers=["*"])


def _georef_header(detail: dict) -> str:
    raw = json.dumps(detail, ensure_ascii=False).encode("utf-8")
    return base64.urlsafe_b64encode(raw).decode("ascii")


@app.get("/")
def root():
    return {
        "service": "lake-loop tile server",
        "center": {"lat": DEFAULT_CENTER[0], "lng": DEFAULT_CENTER[1],
                   "name": config.DEFAULT_LOCATION_NAME},
        "providers": {k: {"crs": v["crs"], "attribution": v["attribution"]}
                      for k, v in PROVIDERS.items()},
        "endpoints": ["/tile/{provider}/{z}/{x}/{y}.png", "/stitch", "/georef",
                      "/water", "/cache/stats", "/static/renders/{name}"],
        "note": "/stitch 输出可直接作为多模态大模型的图像输入",
    }


@app.get("/tile/{provider}/{z}/{x}/{y}.png")
def tile(provider: str, z: int, x: int, y: int):
    if provider not in PROVIDERS:
        raise HTTPException(404, f"未知瓦片源 {provider}")
    try:
        data = tiles.fetch_tile(provider, z, x, y)
        return Response(data, media_type="image/png",
                        headers={"Cache-Control": "public, max-age=86400"})
    except tiles.TileError as e:
        raise HTTPException(502, str(e))


def _parse_stitch_args(lat, lng, z, nx, ny, bbox, provider, pad):
    if provider not in PROVIDERS:
        raise HTTPException(404, f"未知瓦片源 {provider}")
    if bbox:
        try:
            w, s, e, n = [float(v) for v in bbox.split(",")]
        except ValueError:
            raise HTTPException(400, "bbox 格式: west,south,east,north")
        img, detail = tiles.stitch_bbox(provider, z, w, s, e, n, pad_tiles=pad)
    else:
        if lat is None or lng is None:
            lat, lng = DEFAULT_CENTER
        img, detail = tiles.stitch_centered(provider, float(lat), float(lng),
                                            z, nx, ny)
    return img, detail


@app.get("/stitch")
def stitch(lat: float | None = None, lng: float | None = None,
           z: int = 14, nx: int = 6, ny: int = 6,
           bbox: str | None = None, provider: str = DEFAULT_PROVIDER,
           pad: int = 0):
    img, detail = _parse_stitch_args(lat, lng, z, nx, ny, bbox, provider, pad)
    buf = _png_bytes(img)
    return Response(buf, media_type="image/png",
                    headers={"X-Lake-Georef": _georef_header(detail)})


@app.get("/georef")
def georef(lat: float | None = None, lng: float | None = None,
           z: int = 14, nx: int = 6, ny: int = 6,
           bbox: str | None = None, provider: str = DEFAULT_PROVIDER,
           pad: int = 0):
    img, detail = _parse_stitch_args(lat, lng, z, nx, ny, bbox, provider, pad)
    g = tiles.georef_from_detail(detail)
    w, s, e, n = g.bbox()
    return {"detail": detail, "bbox_wgs84": [w, s, e, n],
            "meters_per_px": g.mppx()}


@app.get("/water")
def water_overlay(lat: float | None = None, lng: float | None = None,
                  z: int = 14, nx: int = 6, ny: int = 6,
                  bbox: str | None = None, provider: str = DEFAULT_PROVIDER,
                  pad: int = 0):
    img, detail = _parse_stitch_args(lat, lng, z, nx, ny, bbox, provider, pad)
    ref = PROVIDERS[provider]["water_rgb"]
    mask = water.water_mask(img, ref_rgb=ref)
    # 半透明红色叠加显示水域掩码
    import numpy as np
    coords = np.argwhere(mask)
    out = img.copy().convert("RGBA")
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    for y, x in coords[::2]:
        od.point((x, y), fill=(230, 60, 60, 200))
    out = Image.alpha_composite(out, overlay).convert("RGB")
    buf = _png_bytes(out)
    return Response(buf, media_type="image/png",
                    headers={"X-Lake-Georef": _georef_header(detail)})


@app.get("/cache/stats")
def cache_stats():
    return tiles.cache_stats()


@app.get("/static/renders/{name}")
def renders(name: str):
    path = (config.RENDER_DIR / name).resolve()
    if not str(path).startswith(str(config.RENDER_DIR.resolve())) or not path.exists():
        raise HTTPException(404, "not found")
    media = "text/html" if name.endswith((".html", ".htm")) else "image/png"
    return FileResponse(path, media_type=media)


def _png_bytes(img: Image.Image) -> bytes:
    import io
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def main():
    import uvicorn
    uvicorn.run(app, host=config.SERVER_HOST, port=config.SERVER_PORT, log_level="info")


if __name__ == "__main__":
    main()
