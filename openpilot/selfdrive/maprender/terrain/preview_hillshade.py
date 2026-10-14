#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
preview_hillshade.py — 地形包效果预览/验收工具（hillshade 方案 §6）

不依赖车端：直接读 terrain_*.mbtiles（terrarium raster-dem），用与
maplibre hillshade 图层一致的光照模型（方位 315°、高度 45°，光源方位锚定
map——即 style 里的 hillshade-illumination-anchor: map）渲染山体阴影，
叠加在 style_text.json 的 background 底色上出图，用于：
  - exaggeration 三档对比（0.3 / 0.5 / 0.8，方案 §4.1 验收项）；
  - z12→z13→z14 overzoom 平滑性目检（z>maxzoom 时自动取父级瓦片放大，
    模拟 maplibre overzoom 行为）；
  - 缺瓦片区域目检（不规则裁剪后被跳过的瓦片应保持底色，无黑块）。

示例：
  python preview_hillshade.py terrain_gongga.mbtiles \
      --center 101.88,29.60 --zooms 10,12,13,14 --exaggerations 0.3,0.5,0.8 \
      --outdir previews
"""
import argparse
import io
import math
import os
import sqlite3

import numpy as np
from PIL import Image

TILE = 256
BG = np.array([16, 20, 28], dtype=np.float64) / 255.0  # style background #10141c


def terrarium_decode(png_bytes):
    img = np.asarray(Image.open(io.BytesIO(png_bytes)).convert("RGB"), dtype=np.float64)
    return img[:, :, 0] * 256.0 + img[:, :, 1] + img[:, :, 2] / 256.0 - 32768.0


def lonlat2tile(lon, lat, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(min(85.05112878, max(-85.05112878, lat)))
    y = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


class TerrainPackage:
    def __init__(self, path):
        self.db = sqlite3.connect(path)
        row = self.db.execute(
            "SELECT MIN(zoom_level), MAX(zoom_level) FROM tiles").fetchone()
        self.minz, self.maxz = row

    def get_tile(self, z, x, y):
        """取瓦片高程；z>maxz 时递归取父级并裁剪放大（模拟 maplibre overzoom）"""
        if z > self.maxz:
            parent = self.get_tile(z - 1, x // 2, y // 2)
            if parent is None:
                return None
            ox, oy = (x % 2) * (TILE // 2), (y % 2) * (TILE // 2)
            crop = parent[oy:oy + TILE // 2, ox:ox + TILE // 2]
            return np.asarray(Image.fromarray(crop).resize((TILE, TILE), Image.BILINEAR))
        tms_y = (1 << z) - 1 - y
        row = self.db.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? "
            "AND tile_row=?", (z, x, tms_y)).fetchone()
        if not row:
            return None
        return terrarium_decode(row[0])


def hillshade(elev, cellsize, azimuth=315.0, altitude=45.0, exaggeration=0.5):
    """Horn 法山体阴影，光照模型与 maplibre hillshade 图层一致"""
    az = math.radians(360.0 - azimuth + 90.0)
    alt = math.radians(altitude)
    padded = np.pad(elev, 1, mode="edge")
    dz_dx = ((padded[:-2, 2:] + 2 * padded[1:-1, 2:] + padded[2:, 2:]) -
             (padded[:-2, :-2] + 2 * padded[1:-1, :-2] + padded[2:, :-2])) / (8 * cellsize)
    dz_dy = ((padded[2:, :-2] + 2 * padded[2:, 1:-1] + padded[2:, 2:]) -
             (padded[:-2, :-2] + 2 * padded[:-2, 1:-1] + padded[:-2, 2:])) / (8 * cellsize)
    dz_dx *= exaggeration * 2.0  # exaggeration 对标 maplibre 观感的经验系数
    dz_dy *= exaggeration * 2.0
    slope = np.arctan(np.hypot(dz_dx, dz_dy))
    aspect = np.arctan2(-dz_dy, dz_dx)
    hs = (np.sin(alt) * np.cos(slope) +
          np.cos(alt) * np.sin(slope) * np.cos(az - aspect))
    return np.clip(hs, 0.0, 1.0)


def render_view(pkg, center, zoom, exaggeration, size=640, opacity=0.55):
    """渲染指定中心/zoom 的一帧：底色 + hillshade 叠加（缺瓦片处保持底色）"""
    lon, lat = center
    xt, yt = lonlat2tile(lon, lat, zoom)
    half = size / 2 / TILE
    x0, x1 = int(math.floor(xt - half)), int(math.floor(xt + half))
    y0, y1 = int(math.floor(yt - half)), int(math.floor(yt + half))
    W, H = (x1 - x0 + 1) * TILE, (y1 - y0 + 1) * TILE
    elev = np.full((H, W), np.nan)
    missing = 0
    for ty in range(y0, y1 + 1):
        for tx in range(x0, x1 + 1):
            t = pkg.get_tile(zoom, tx, ty)
            if t is None:
                missing += 1
                continue
            elev[(ty - y0) * TILE:(ty - y0 + 1) * TILE,
                 (tx - x0) * TILE:(tx - x0 + 1) * TILE] = t
    # 中心裁剪 size×size
    cx, cy = (xt - x0) * TILE, (yt - y0) * TILE
    r0, c0 = int(cy - size / 2), int(cx - size / 2)
    elev = elev[r0:r0 + size, c0:c0 + size]

    frame = np.ones((size, size, 3)) * BG
    valid = ~np.isnan(elev)
    if valid.any():
        # 瓦片地面分辨率（米/像素）
        ground = 156543.03392 * math.cos(math.radians(lat)) / (2 ** zoom)
        filled = np.where(valid, elev, np.nanmean(elev[valid]))
        hs = hillshade(filled, ground, exaggeration=exaggeration)
        hs3 = np.stack([hs] * 3, axis=-1)
        a = np.where(valid, opacity, 0.0)[:, :, None]
        frame = frame * (1 - a) + hs3 * a
    img = Image.fromarray((np.clip(frame, 0, 1) * 255).astype(np.uint8))
    return img, missing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mbtiles")
    ap.add_argument("--center", required=True, help="lon,lat")
    ap.add_argument("--zooms", default="12")
    ap.add_argument("--exaggerations", default="0.5")
    ap.add_argument("--size", type=int, default=640)
    ap.add_argument("--opacity", type=float, default=0.55)
    ap.add_argument("--outdir", default="previews")
    args = ap.parse_args()

    pkg = TerrainPackage(args.mbtiles)
    center = tuple(float(v) for v in args.center.split(","))
    os.makedirs(args.outdir, exist_ok=True)
    print(f"package zoom range: z{pkg.minz}-z{pkg.maxz}")

    for z in [int(v) for v in args.zooms.split(",")]:
        for ex in [float(v) for v in args.exaggerations.split(",")]:
            img, missing = render_view(pkg, center, z, ex,
                                       size=args.size, opacity=args.opacity)
            tag = "overzoom" if z > pkg.maxz else "native"
            name = f"z{z}_ex{ex:.1f}_{tag}.png"
            img.save(os.path.join(args.outdir, name))
            print(f"  {name}  (missing tiles in view: {missing})")


if __name__ == "__main__":
    main()
