"""地理/瓦片坐标数学：WGS-84 Web-Mercator (slippy map) 与 GCJ-02 转换。"""
import math

R_EARTH = 6378137.0


def lng_to_global_px(lng: float, z: int) -> float:
    return (lng + 180.0) / 360.0 * (2 ** z) * 256.0


def lat_to_global_px(lat: float, z: int) -> float:
    s = math.sin(math.radians(lat))
    return (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * (2 ** z) * 256.0


def global_px_to_lng(px: float, z: int) -> float:
    return px / ((2 ** z) * 256.0) * 360.0 - 180.0


def global_px_to_lat(py: float, z: int) -> float:
    n = math.pi - 2 * math.pi * py / ((2 ** z) * 256.0)
    return math.degrees(math.atan(0.5 * (math.exp(n) - math.exp(-n))))


def latlon_to_tile(lat: float, lng: float, z: int):
    """返回包含 (lat,lng) 的瓦片整数坐标 (x, y)。"""
    return (int(math.floor(lng_to_global_px(lng, z) / 256.0)),
            int(math.floor(lat_to_global_px(lat, z) / 256.0)))


def meters_per_px(lat: float, z: int) -> float:
    return 156543.03392 * math.cos(math.radians(lat)) / (2 ** z)


def haversine_m(lat1, lng1, lat2, lng2) -> float:
    ra = math.radians
    dlat, dlng = ra(lat2 - lat1), ra(lng2 - lng1)
    a = math.sin(dlat / 2) ** 2 + math.cos(ra(lat1)) * math.cos(ra(lat2)) * math.sin(dlng / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))


# ---------------- GCJ-02 <-> WGS-84 (火星坐标偏移) ----------------

def _out_of_china(lat, lng):
    return not (73.66 < lng < 135.05 and 3.86 < lat < 53.55)


def _transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret


def _transform_lng(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret


def wgs84_to_gcj02(lat: float, lng: float):
    if _out_of_china(lat, lng):
        return lat, lng
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - 0.00669342162296594323 * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((6378245.0 * (1 - 0.00669342162296594323)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (6378245.0 / sqrtmagic * math.cos(radlat) * math.pi)
    return lat + dlat, lng + dlng


def gcj02_to_wgs84(lat: float, lng: float):
    if _out_of_china(lat, lng):
        return lat, lng
    glat, glng = lat, lng
    for _ in range(3):  # 不动点迭代逼近
        wlat, wlng = wgs84_to_gcj02(glat, glng)
        glat, glng = glat - (wlat - lat), glng - (wlng - lng)
    return glat, glng


class StitchGeoref:
    """一张拼接图的地理参照：图像像素 <-> WGS-84 经纬度。

    detail: {z, x0, y0, w, h, crs}  x0/y0 为左上角瓦片索引。
    """

    def __init__(self, z: int, x0: int, y0: int, w: int, h: int, crs: str = "wgs84"):
        self.z, self.x0, self.y0, self.w, self.h, self.crs = z, x0, y0, w, h, crs

    def pixel_to_latlon(self, px: float, py: float):
        gx = self.x0 * 256.0 + px
        gy = self.y0 * 256.0 + py
        lat = global_px_to_lat(gy, self.z)
        lng = global_px_to_lng(gx, self.z)
        if self.crs == "gcj02":
            lat, lng = gcj02_to_wgs84(lat, lng)
        return lat, lng

    def latlon_to_pixel(self, lat: float, lng: float):
        if self.crs == "gcj02":
            lat, lng = wgs84_to_gcj02(lat, lng)
        gx = lng_to_global_px(lng, self.z)
        gy = lat_to_global_px(lat, self.z)
        return gx - self.x0 * 256.0, gy - self.y0 * 256.0

    def bbox(self):
        """(west, south, east, north) WGS-84"""
        w, n = self.pixel_to_latlon(0, 0)
        e, s = self.pixel_to_latlon(self.w, self.h)
        return w, s, e, n

    def mppx(self):
        lat, _ = self.pixel_to_latlon(self.w / 2, self.h / 2)
        return meters_per_px(lat, self.z)
