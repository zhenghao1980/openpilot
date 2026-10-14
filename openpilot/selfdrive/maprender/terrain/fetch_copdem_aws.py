#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fetch_copdem_aws.py — 从 AWS Open Data 公共桶批量下载 Copernicus DEM GLO-30

免注册。按省界 GeoJSON（或 bbox）算出所需 1°×1° COG 瓦片清单，并发下载、
断点续传、校验非空，产物直接喂 build_terrain_mbtiles.py --source geotiff。

桶结构（registry.opendata.aws/copernicus-dem）：
  s3://copernicus-dem-30m/Copernicus_DSM_COG_10_N26_00_E097_00_DEM/
                         Copernicus_DSM_COG_10_N26_00_E097_00_DEM.tif

示例（四川省）：
  curl -O https://geo.datav.aliyun.com/areas_v3/bound/510000_full.json
  python fetch_copdem_aws.py --geojson 510000_full.json -o dem/sichuan
"""
import argparse
import math
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed

BASE = ("https://copernicus-dem-30m.s3.amazonaws.com/"
        "Copernicus_DSM_COG_10_{lat}_{lon}_DEM/"
        "Copernicus_DSM_COG_10_{lat}_{lon}_DEM.tif")


def cell_name(lat, lon):
    la = f"{'N' if lat >= 0 else 'S'}{abs(lat):02d}_00"
    lo = f"{'E' if lon >= 0 else 'W'}{abs(lon):03d}_00"
    return la, lo


def load_cells(geojson_path, bbox):
    """1°×1° 单元与省界多边形相交判定（粗筛，宁多勿漏）"""
    import json
    import numpy as np
    from matplotlib.path import Path
    polys = []
    if geojson_path:
        with open(geojson_path, encoding="utf-8") as f:
            gj = json.load(f)
        feats = gj.get("features", [gj])
        for ft in feats:
            geom = ft.get("geometry", ft)
            t = geom.get("type")
            coords = geom.get("coordinates", [])
            ps = coords if t == "MultiPolygon" else [coords] if t == "Polygon" else []
            for p in ps:
                ring = np.asarray(p[0], dtype=np.float64)
                if ring.ndim == 2 and ring.shape[0] >= 3:
                    polys.append(ring)
        if not polys:
            sys.exit("geojson 中没有多边形")
        allr = np.concatenate(polys)
        minlon, minlat = allr[:, 0].min(), allr[:, 1].min()
        maxlon, maxlat = allr[:, 0].max(), allr[:, 1].max()
    else:
        minlon, minlat, maxlon, maxlat = bbox

    cells = []
    for lat in range(math.floor(minlat), math.ceil(maxlat)):
        for lon in range(math.floor(minlon), math.ceil(maxlon)):
            if not polys:
                cells.append((lat, lon))
                continue
            pts = np.array([[lon, lat], [lon, lat + 1], [lon + 1, lat],
                            [lon + 1, lat + 1], [lon + 0.5, lat + 0.5]])
            keep = False
            for ring in polys:
                if (ring[:, 0].max() < lon or ring[:, 0].min() > lon + 1 or
                        ring[:, 1].max() < lat or ring[:, 1].min() > lat + 1):
                    continue
                if Path(ring).contains_points(pts).any():
                    keep = True
                    break
                inside = ((ring[:, 0] >= lon) & (ring[:, 0] <= lon + 1) &
                          (ring[:, 1] >= lat) & (ring[:, 1] <= lat + 1))
                if inside.any():
                    keep = True
                    break
            if keep:
                cells.append((lat, lon))
    return cells


def fetch_one(lat, lon, outdir, retries=3):
    import urllib.request
    la, lo = cell_name(lat, lon)
    url = BASE.format(lat=la, lon=lo)
    name = f"Copernicus_DSM_COG_10_{la}_{lo}_DEM.tif"
    path = os.path.join(outdir, name)
    if os.path.exists(path) and os.path.getsize(path) > 1024 * 100:
        return name, "cached", os.path.getsize(path)
    tmp = path + ".part"
    last = None
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=120) as r, open(tmp, "wb") as f:
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
            os.replace(tmp, path)
            return name, "ok", os.path.getsize(path)
        except Exception as e:  # noqa: BLE001
            last = e
            if "404" in str(e):
                break
    if os.path.exists(tmp):
        os.remove(tmp)
    return name, f"fail({last})", 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--geojson", help="省界 GeoJSON")
    ap.add_argument("--bbox", help="minlon,minlat,maxlon,maxlat（无 geojson 时用）")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("-o", "--outdir", required=True)
    args = ap.parse_args()
    if not args.geojson and not args.bbox:
        sys.exit("需要 --geojson 或 --bbox")
    bbox = tuple(float(v) for v in args.bbox.split(",")) if args.bbox else None
    os.makedirs(args.outdir, exist_ok=True)

    cells = load_cells(args.geojson, bbox)
    print(f"1x1 degree cells to fetch: {len(cells)}")

    ok = fail = cached = 0
    total = 0
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(fetch_one, lat, lon, args.outdir): (lat, lon)
                for lat, lon in cells}
        for i, fut in enumerate(as_completed(futs), 1):
            name, status, size = fut.result()
            total += size
            if status == "ok":
                ok += 1
            elif status == "cached":
                cached += 1
            else:
                fail += 1
                print(f"  FAIL {name}: {status}")
            if i % 10 == 0 or i == len(cells):
                print(f"  progress {i}/{len(cells)} "
                      f"(ok={ok} cached={cached} fail={fail}, {total/1e9:.2f} GB)",
                      flush=True)
    print(f"\ndone: ok={ok} cached={cached} fail={fail}, total {total/1e9:.2f} GB "
          f"-> {args.outdir}")


if __name__ == "__main__":
    main()
