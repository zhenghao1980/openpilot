"""Offline raster tile layer for the map panel (Web Mercator, raylib textures).

Tiles come from the PC-side pipeline (tools/map_tiles): tileserver-gl rendered
Mapbox navigation-night style PNGs, copied to the device as /data/mapd_tiles/z/x/y.png.
Vector data (matched path / nearby roads) still comes from mapd and is drawn as
overlay on top of the tiles.
"""
from __future__ import annotations  # pyray.Texture2D is a function at runtime, not a type
import math
import os
from collections import OrderedDict

import pyray as rl

TILE_SIZE = 256
EARTH_R = 6378137.0  # Web Mercator radius
TILE_DIR_CANDIDATES = ("/data/mapd_tiles", "./mapd_tiles", "/tmp/mapd_tiles")
CACHE_CAP = 128


def world_px(lat: float, lon: float, z: int) -> tuple[float, float]:
  n = TILE_SIZE * (2 ** z)
  x = (lon + 180.0) / 360.0 * n
  lat = max(min(lat, 85.05), -85.05)
  r = math.radians(lat)
  y = (1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * n
  return x, y


def mpp_at_lat(lat: float, z: float) -> float:
  """real-world meters per tile pixel at latitude/zoom."""
  return math.cos(math.radians(lat)) * 2 * math.pi * EARTH_R / (TILE_SIZE * (2 ** z))


class TileLayer:
  def __init__(self):
    self.root = next((d for d in TILE_DIR_CANDIDATES if os.path.isdir(d)), None)
    self._cache: OrderedDict[tuple, rl.Texture2D] = OrderedDict()

  def available(self, z: int, x: int, y: int) -> bool:
    return self.root is not None and os.path.exists(os.path.join(self.root, str(z), str(x), f"{y}.png"))

  def _texture(self, z: int, x: int, y: int) -> rl.Texture2D | None:
    key = (z, x, y)
    tex = self._cache.get(key)
    if tex is not None:
      self._cache.move_to_end(key)
      return tex
    if not self.available(z, x, y):
      return None
    try:
      img = rl.load_image(os.path.join(self.root, str(z), str(x), f"{y}.png"))
      tex = rl.load_texture_from_image(img)
      rl.unload_image(img)
    except Exception:
      return None
    self._cache[key] = tex
    self._cache.move_to_end(key)
    while len(self._cache) > CACHE_CAP:
      _, old = self._cache.popitem(last=False)
      rl.unload_texture(old)
    return tex

  def pick_zoom(self, lat: float, want_mpp: float) -> int:
    """选最近可用 zoom；本级别中心瓦片缺失则逐级降（数据只导出了部分级别时优雅退化）。"""
    ideal = math.log2(math.cos(math.radians(lat)) * 2 * math.pi * EARTH_R / (TILE_SIZE * want_mpp))
    z = max(10, min(17, int(round(ideal))))
    while z > 10:
      wx, wy = world_px(lat, 0.0, z)
      if self.available(z, int(wx) // TILE_SIZE, int(wy) // TILE_SIZE):
        break
      z -= 1
    return z
