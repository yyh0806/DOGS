"""Small obstacle-grid accumulator for nx_web_server.

Input points are already in world coordinates. The accumulator quantizes them to
a fixed grid so the browser receives a bounded, stable obstacle map instead of
an ever-growing raw scan stream.

M4 fix (2026-07-01): 加 threading.Lock 保护 — broadcast_loop 线程写 update(),
其他广播线程可能读 points()。OrderedDict 的 move_to_end/popitem 非原子。
行为/契约不变 (构造签名、update/points 返回值同前), 仅加锁。
"""

import threading
from collections import OrderedDict


class ObstacleGridAccumulator:
    def __init__(self, resolution=0.1, max_points=5000):
        self.resolution = float(resolution)
        self.max_points = int(max_points)
        self._cells = OrderedDict()
        self._lock = threading.Lock()

    def _cell_key(self, x, y):
        return (round(float(x) / self.resolution), round(float(y) / self.resolution))

    def _cell_point(self, key):
        x = round(key[0] * self.resolution, 2)
        y = round(key[1] * self.resolution, 2)
        if x == -0.0:
            x = 0.0
        if y == -0.0:
            y = 0.0
        return [x, y]

    def update(self, points):
        with self._lock:
            for pt in points or []:
                if len(pt) < 2:
                    continue
                key = self._cell_key(pt[0], pt[1])
                if key in self._cells:
                    self._cells.move_to_end(key)
                self._cells[key] = self._cell_point(key)
                while len(self._cells) > self.max_points:
                    self._cells.popitem(last=False)
            return list(self._cells.values())

    def points(self):
        with self._lock:
            return list(self._cells.values())
