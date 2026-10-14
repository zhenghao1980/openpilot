#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_terrain_mbtiles.py — 离线高程地形包构建工具（hillshade 方案 §3）

把 DEM 高程数据切成 terrarium 编码 PNG 瓦片，按标准 schema 打包成
terrain_<省>.mbtiles（TMS 行序、PNG blob 不 gzip），供 maprenderd 的
vendor MBTilesFileSource 以 raster-dem 源直读（方案 §3.4 / §3.5 / §4.3 已验证）。

两种数据源：
  1) --source aws-terrarium : AWS Open Data 公共 terrarium 瓦片
     (https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png)
     免注册，适合试点；注意其 z>13 无原生数据。
  2) --source geotiff --dem a.tif b.tif ... : 本地 Copernicus DEM GLO-30
     GeoTIFF（需 pip install rasterio），正式全国切分用（方案 §3.1）。

裁剪：
  --bbox minlon,minlat,maxlon,maxlat     矩形范围（试点）
  --geojson province.geojson             省级多边形（正式，需 matplotlib）
  --relief-threshold M                   不规则裁剪：瓦片高程极差 < M 米则
                                         不出片（方案 §3.3，端上靠 overzoom 降级）

示例（贡嘎山试点，z4-z13）：
  python build_terrain_mbtiles.py --source aws-terrarium \
      --bbox 101.5,29.1,102.5,30.1 --minzoom 4 --maxzoom 13 \
      --relief-threshold 50 -o terrain_gongga.mbtiles
"""
import argparse
import io
import math
import os
import sqlite3
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

import numpy as np
from PIL import Image

AWS_TERRARIUM = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"


# ---------------- Web Mercator 瓦片数学 ----------------

def lonlat2tile(lon, lat, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(min(85.05112878, max(-85.05112878, lat)))
    y = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def tile2lonlat(x, y, z):
    """瓦片西北角经纬度"""
    n = 2 ** z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


def tiles_for_bbox(bbox, z):
    minlon, minlat, maxlon, maxlat = bbox
    x0f, y0f = lonlat2tile(minlon, maxlat, z)  # 西北
    x1f, y1f = lonlat2tile(maxlon, minlat, z)  # 东南
    n = 2 ** z
    xs = range(max(0, int(math.floor(x0f))), min(n - 1, int(math.floor(x1f))) + 1)
    ys = range(max(0, int(math.floor(y0f))), min(n - 1, int(math.floor(y1f))) + 1)
    return [(z, x, y) for x in xs for y in ys]


def tile_bbox(x, y, z):
    lon0, lat1 = tile2lonlat(x, y, z)
    lon1, lat0 = tile2lonlat(x + 1, y + 1, z)
    return (lon0, lat0, lon1, lat1)


# ---------------- terrarium 编解码 ----------------

def terrarium_decode(png_bytes):
    img = np.asarray(Image.open(io.BytesIO(png_bytes)).convert("RGB"), dtype=np.float64)
    return img[:, :, 0] * 256.0 + img[:, :, 1] + img[:, :, 2] / 256.0 - 32768.0


def terrarium_encode(elev):
    v = np.clip(elev + 32768.0, 0.0, 65535.0 + 255.0 / 256.0)
    r = np.floor(v / 256.0)
    g = np.floor(v - r * 256.0)
    b = np.floor((v - r * 256.0 - g) * 256.0)
    rgb = np.stack([r, g, b], axis=-1).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(rgb, "RGB").save(buf, format="PNG", optimize=True)
    return buf.getvalue()


# ---------------- 并行瓦片渲染（geotiff 模式，进程池） ----------------

_WORKER = {}


def _worker_init(dem_paths):
    import json
    import rasterio
    # 源文件 bounds 缓存：全国 1149 个 DEM，逐文件开句柄取 bounds 很慢，
    # 缓存到 dem 目录下一次全国各省切分共用
    cache_path = os.path.join(os.path.dirname(os.path.abspath(dem_paths[0])),
                              "_bounds_cache.json")
    cached = {}
    if os.path.exists(cache_path):
        with open(cache_path, encoding="utf-8") as f:
            cached = json.load(f)
    srcs = []
    dirty = False
    for p in dem_paths:
        ap = os.path.abspath(p)
        if ap in cached:
            b = cached[ap]
            srcs.append((p, rasterio.coords.BoundingBox(*b)))
        else:
            s = rasterio.open(p)
            srcs.append((p, s.bounds))
            cached[ap] = list(s.bounds)
            s.close()
            dirty = True
    if dirty:
        with open(cache_path, "w", encoding="utf-8") as f:
            json.dump(cached, f)
    _WORKER["srcs"] = srcs


def _worker_render(task):
    """task = (z, x, y, relief_threshold, quantize)
    返回 (z, x, y, status, blob_or_none)"""
    import rasterio
    from rasterio.warp import reproject, Resampling
    from rasterio.transform import from_bounds
    z, x, y, relief_threshold, quantize = task
    dbg = os.environ.get("TILE_DEBUG_LOG")
    if dbg:
        with open(dbg, "a", encoding="utf-8") as f:
            f.write(f"{os.getpid()} start {z} {x} {y}\n")
    lon0, lat0, lon1, lat1 = tile_bbox(x, y, z)

    def to3857(lon, lat):
        X = lon * 20037508.34 / 180.0
        Y = math.log(math.tan(math.radians(90 + lat) / 2)) * 6378137.0
        return X, Y
    x0, ys = to3857(lon0, lat0)
    x1, yn = to3857(lon1, lat1)
    dst = np.full((256, 256), np.nan, dtype=np.float32)
    dst_transform = from_bounds(x0, ys, x1, yn, 256, 256)
    for p, b in _WORKER["srcs"]:
        if b.right < lon0 or b.left > lon1 or b.top < lat0 or b.bottom > lat1:
            continue
        try:
            with rasterio.open(p) as src:
                tmp = np.full((256, 256), np.nan, dtype=np.float32)
                reproject(
                    source=rasterio.band(src, 1),
                    destination=tmp,
                    dst_transform=dst_transform,
                    dst_crs="EPSG:3857",
                    resampling=Resampling.bilinear,
                    src_nodata=src.nodata,
                    dst_nodata=np.nan,
                )
        except Exception:
            # 瓦片与源边缘相切时窗口退化为零宽/零高，GDAL 报
            # "Chunk and warp failed"——跳过该源即可（其他源补足）
            continue
        dst = np.where(np.isnan(dst), tmp, dst)
    if np.isnan(dst).all():
        return z, x, y, "nodata", None
    if relief_threshold > 0 and float(np.nanmax(dst) - np.nanmin(dst)) < relief_threshold:
        return z, x, y, "relief", None
    if quantize > 0:
        dst = np.round(dst / quantize) * quantize
    dst = np.where(np.isnan(dst), 0.0, dst)
    if dbg:
        with open(dbg, "a", encoding="utf-8") as f:
            f.write(f"{os.getpid()} done {z} {x} {y}\n")
    return z, x, y, "ok", terrarium_encode(dst)

class PolyMask:
    """省级多边形裁剪：瓦片外包框角点/中心在任一多边形内即保留（粗筛，
    边界瓦片宁多勿漏——多出的瓦片只是体积，缺了会出无底图洞）。"""

    def __init__(self, geojson_path):
        import json
        from matplotlib.path import Path
        with open(geojson_path, encoding="utf-8") as f:
            gj = json.load(f)
        self._Path = Path
        self.polys = []
        feats = gj.get("features", [gj])
        for ft in feats:
            geom = ft.get("geometry", ft)
            t = geom.get("type")
            coords = geom.get("coordinates", [])
            polys = coords if t == "MultiPolygon" else [coords] if t == "Polygon" else []
            for p in polys:
                ring = np.asarray(p[0], dtype=np.float64)  # 外环
                if ring.ndim == 2 and ring.shape[0] >= 3:
                    self.polys.append(ring)
        if not self.polys:
            raise ValueError(f"no polygon found in {geojson_path}")

    def covers_tile(self, x, y, z):
        lon0, lat0, lon1, lat1 = tile_bbox(x, y, z)
        pts = np.array([
            [lon0, lat0], [lon0, lat1], [lon1, lat0], [lon1, lat1],
            [(lon0 + lon1) / 2, (lat0 + lat1) / 2],
        ])
        for ring in self.polys:
            # 快速包围盒
            if (ring[:, 0].max() < lon0 or ring[:, 0].min() > lon1 or
                    ring[:, 1].max() < lat0 or ring[:, 1].min() > lat1):
                continue
            if self._Path(ring).contains_points(pts).any():
                return True
            # 多边形顶点落在瓦片内也算相交
            inside = ((ring[:, 0] >= lon0) & (ring[:, 0] <= lon1) &
                      (ring[:, 1] >= lat0) & (ring[:, 1] <= lat1))
            if inside.any():
                return True
        return False


# ---------------- 数据源 ----------------

class AwsTerrariumSource:
    """AWS Open Data 公共 terrarium 瓦片源（原生数据 z<=13 左右，全球）"""

    def __init__(self, cache_dir=None, workers=16, retries=3):
        self.cache_dir = cache_dir
        self.workers = workers
        self.retries = retries
        if cache_dir:
            os.makedirs(cache_dir, exist_ok=True)

    def fetch_one(self, zxy):
        import urllib.request
        z, x, y = zxy
        cache_path = None
        if self.cache_dir:
            cache_path = os.path.join(self.cache_dir, str(z), str(x))
            os.makedirs(cache_path, exist_ok=True)
            cache_path = os.path.join(cache_path, f"{y}.png")
            if os.path.exists(cache_path):
                with open(cache_path, "rb") as f:
                    return zxy, f.read()
        url = AWS_TERRARIUM.format(z=z, x=x, y=y)
        last = None
        for attempt in range(self.retries):
            try:
                with urllib.request.urlopen(url, timeout=30) as r:
                    data = r.read()
                if cache_path:
                    with open(cache_path, "wb") as f:
                        f.write(data)
                return zxy, data
            except Exception as e:  # noqa: BLE001
                last = e
                time.sleep(1.5 * (attempt + 1))
        if "404" in str(last):
            return zxy, ("__404__", last)
        raise last

    def fetch(self, tile_list):
        out = {}
        with ThreadPoolExecutor(max_workers=self.workers) as ex:
            futs = {ex.submit(self.fetch_one, t): t for t in tile_list}
            done = 0
            for fut in as_completed(futs):
                zxy, data = fut.result()
                done += 1
                if done % 200 == 0 or done == len(tile_list):
                    print(f"  downloaded {done}/{len(tile_list)}", flush=True)
                out[zxy] = data
        return out


class GeoTiffSource:
    """本地 GeoTIFF（Copernicus GLO-30 等）逐瓦片重投影采样，需要 rasterio。
    源文件按空间范围建索引，每个输出瓦片只 warp 相交的 1-4 个源。"""

    def __init__(self, dem_paths):
        try:
            import rasterio  # noqa: F401
            from rasterio.warp import reproject, Resampling  # noqa: F401
        except ImportError:
            sys.exit("geotiff 模式需要 rasterio：pip install rasterio "
                     "（或用 --source aws-terrarium）")
        import rasterio
        self.srcs = []
        for p in dem_paths:
            s = rasterio.open(p)
            self.srcs.append((s, s.bounds))  # bounds: (left, bottom, right, top)
        from rasterio.warp import Resampling  # noqa
        self.Resampling = Resampling
        print(f"geotiff sources: {len(self.srcs)} files indexed")

    def render_tile(self, z, x, y, size=256):
        """从 DEM 重投影出一张 size×size 的 EPSG:3857 瓦片，返回高程数组"""
        import rasterio
        from rasterio.warp import reproject
        from rasterio.transform import from_bounds
        lon0, lat0, lon1, lat1 = tile_bbox(x, y, z)
        # 3857 下瓦片边界
        def to3857(lon, lat):
            X = lon * 20037508.34 / 180.0
            Y = math.log(math.tan(math.radians(90 + lat) / 2)) * 6378137.0
            return X, Y
        x0, ys = to3857(lon0, lat0)   # lat0 = 南缘 → ys = 南端 Y（小值）
        x1, yn = to3857(lon1, lat1)   # lat1 = 北缘 → yn = 北端 Y（大值）
        dst = np.full((size, size), np.nan, dtype=np.float32)
        # from_bounds(left, bottom, right, top)：bottom=南 top=北，不能传反
        dst_transform = from_bounds(x0, ys, x1, yn, size, size)
        for src, b in self.srcs:
            # 空间索引：源范围（经纬度）与瓦片经纬度范围相交才处理
            if b.right < lon0 or b.left > lon1 or b.top < lat0 or b.bottom > lat1:
                continue
            tmp = np.full((size, size), np.nan, dtype=np.float32)
            reproject(
                source=rasterio.band(src, 1),
                destination=tmp,
                dst_transform=dst_transform,
                dst_crs="EPSG:3857",
                resampling=self.Resampling.bilinear,
                src_nodata=src.nodata,
                dst_nodata=np.nan,
            )
            dst = np.where(np.isnan(dst), tmp, dst)
        return dst


# ---------------- mbtiles 打包 ----------------

def create_mbtiles(path, resume=False):
    if os.path.exists(path) and not resume:
        os.remove(path)
    db = sqlite3.connect(path)
    if not resume:
        db.execute("CREATE TABLE metadata (name text, value text)")
        db.execute("CREATE TABLE tiles (zoom_level integer, tile_column integer, "
                   "tile_row integer, tile_data blob)")
    else:
        # 续切：确保表结构存在（上次可能没跑到 finalize 就中断）
        db.execute("CREATE TABLE IF NOT EXISTS metadata (name text, value text)")
        db.execute("CREATE TABLE IF NOT EXISTS tiles (zoom_level integer, "
                   "tile_column integer, tile_row integer, tile_data blob)")
    # 跳过台账：低起伏/无数据的瓦片也记录（无 tile_data），否则中断续切会
    # 反复重算这些不出片的瓦片——队列头部一旦被判跳过瓦片占满就会原地踏步
    db.execute("CREATE TABLE IF NOT EXISTS skipped (zoom_level integer, "
               "tile_column integer, tile_row integer)")
    return db


def finalize_mbtiles(db, meta):
    # 防御性去重：tiles 表在 finalize 前没有唯一约束，并发写/异常续切可能
    # 留下重复行，直接建唯一索引会失败；保留每瓦片首行即可
    db.execute("DELETE FROM tiles WHERE rowid NOT IN "
               "(SELECT MIN(rowid) FROM tiles "
               "GROUP BY zoom_level, tile_column, tile_row)")
    db.execute("DELETE FROM metadata")
    db.executemany("INSERT INTO metadata (name, value) VALUES (?, ?)",
                   list(meta.items()))
    db.execute("CREATE UNIQUE INDEX IF NOT EXISTS tile_index ON tiles "
               "(zoom_level, tile_column, tile_row)")
    db.commit()
    db.close()


def main():
    ap = argparse.ArgumentParser(description="构建 terrarium raster-dem mbtiles 地形包")
    ap.add_argument("--source", choices=["aws-terrarium", "geotiff"], required=True)
    ap.add_argument("--dem", nargs="*", help="geotiff 模式的 DEM 文件列表")
    ap.add_argument("--dem-list", help="DEM 清单文件（每行一个路径，替代 --dem，"
                                       "规避 Windows 命令行长度上限）")
    ap.add_argument("--bbox", help="minlon,minlat,maxlon,maxlat")
    ap.add_argument("--geojson", help="省级多边形 GeoJSON（与 bbox 可叠加）")
    ap.add_argument("--minzoom", type=int, default=4)
    ap.add_argument("--maxzoom", type=int, default=12)
    ap.add_argument("--relief-threshold", type=float, default=0.0,
                    help="瓦片高程极差低于该米数则不出片（不规则裁剪）")
    ap.add_argument("--quantize", type=float, default=0.0,
                    help="高程量化步长（米）。双线性重投影产生的亚米级噪声会让 "
                         "PNG 体积膨胀 ~2.7 倍；GLO-30 垂直精度为米级，建议 1")
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--cache-dir", default=None, help="下载瓦片缓存目录")
    ap.add_argument("--name", default="offline-terrain")
    ap.add_argument("--resume", action="store_true",
                    help="续切：保留已有 mbtiles，跳过已存在的瓦片")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    if args.bbox:
        bbox = tuple(float(v) for v in args.bbox.split(","))
    elif args.geojson:
        gj_bbox = None
        import json
        with open(args.geojson, encoding="utf-8") as f:
            gj = json.load(f)
        gj_bbox = gj.get("bbox")
        if not gj_bbox:
            # 从多边形顶点推
            mask_tmp = PolyMask(args.geojson)
            allr = np.concatenate(mask_tmp.polys)
            gj_bbox = [allr[:, 0].min(), allr[:, 1].min(),
                       allr[:, 0].max(), allr[:, 1].max()]
        bbox = tuple(float(v) for v in gj_bbox[:4])
    else:
        sys.exit("需要 --bbox 或 --geojson 之一")

    mask = PolyMask(args.geojson) if args.geojson else None

    # 收集全部待切瓦片
    all_tiles = []
    for z in range(args.minzoom, args.maxzoom + 1):
        for (zz, x, y) in tiles_for_bbox(bbox, z):
            if mask and not mask.covers_tile(x, y, z):
                continue
            all_tiles.append((zz, x, y))
    print(f"tiles to process: {len(all_tiles)} "
          f"(z{args.minzoom}-z{args.maxzoom}, bbox={bbox})")

    db = create_mbtiles(args.output, resume=args.resume)
    if args.resume:
        done_tiles = {(z, x, (1 << z) - 1 - y) for z, x, y in
                      db.execute("SELECT zoom_level, tile_column, tile_row FROM tiles")}
        done_tiles |= {(z, x, (1 << z) - 1 - y) for z, x, y in
                       db.execute("SELECT zoom_level, tile_column, tile_row FROM skipped")}
        before = len(all_tiles)
        all_tiles = [t for t in all_tiles if t not in done_tiles]
        print(f"resume: skip {before - len(all_tiles)} existing tiles, "
              f"{len(all_tiles)} remaining")
    stats = {"written": 0, "skipped_relief": 0, "skipped_nodata": 0}
    bytes_total = 0

    # 重排：高级别瓦片（海量、轻快）先跑，z4-z8 低级别瓦片（少而重，
    # 单片要 warp 数百个 DEM 源）放最后并行收尾——避免按序阻塞首轮提交
    all_tiles.sort(key=lambda t: (-t[0], t[1], t[2]))

    if args.source == "aws-terrarium":
        src = AwsTerrariumSource(cache_dir=args.cache_dir, workers=args.workers)
        data = src.fetch(all_tiles)
        for (z, x, y), blob in sorted(data.items()):
            if isinstance(blob, tuple):  # 404
                stats["skipped_nodata"] += 1
                db.execute("INSERT INTO skipped VALUES (?,?,?)",
                           (z, x, (1 << z) - 1 - y))
                continue
            if args.relief_threshold > 0:
                elev = terrarium_decode(blob)
                if float(elev.max() - elev.min()) < args.relief_threshold:
                    stats["skipped_relief"] += 1
                    db.execute("INSERT INTO skipped VALUES (?,?,?)",
                               (z, x, (1 << z) - 1 - y))
                    continue
            tms_y = (1 << z) - 1 - y
            db.execute("INSERT OR REPLACE INTO tiles VALUES (?,?,?,?)",
                       (z, x, tms_y, blob))
            stats["written"] += 1
            bytes_total += len(blob)
    else:
        dem_paths = list(args.dem or [])
        if args.dem_list:
            with open(args.dem_list, encoding="utf-8") as f:
                dem_paths += [ln.strip() for ln in f if ln.strip()]
        if not dem_paths:
            sys.exit("geotiff 模式需要 --dem <tif 文件...> 或 --dem-list <清单>")
        args.dem = dem_paths
        # 并发模型：用线程池而非进程池——本机安全软件会狙杀高频 spawn 的
        # python 子进程（实测 BrokenProcessPool / WinError 5，与任务内容无关）。
        # rasterio/GDAL 对独立句柄线程安全，warp 与 PNG 编码释放 GIL，
        # 每瓦片独立 open/close 数据集；sqlite 只由主线程写入，可安全地
        # 边渲染边落库（周期提交，中断后 --resume 续切不丢进度）。
        from concurrent.futures import ThreadPoolExecutor
        _worker_init(list(args.dem))  # bounds 索引建在主线程，worker 只读
        tasks = [(z, x, y, args.relief_threshold, args.quantize)
                 for (z, x, y) in all_tiles]
        done = 0
        with ThreadPoolExecutor(max_workers=args.workers) as ex:
            for z, x, y, status, blob in ex.map(_worker_render, tasks,
                                                chunksize=32):
                done += 1
                if status == "nodata":
                    stats["skipped_nodata"] += 1
                    db.execute("INSERT INTO skipped VALUES (?,?,?)",
                               (z, x, (1 << z) - 1 - y))
                elif status == "relief":
                    stats["skipped_relief"] += 1
                    db.execute("INSERT INTO skipped VALUES (?,?,?)",
                               (z, x, (1 << z) - 1 - y))
                else:
                    tms_y = (1 << z) - 1 - y
                    db.execute("INSERT OR REPLACE INTO tiles VALUES (?,?,?,?)",
                               (z, x, tms_y, blob))
                    stats["written"] += 1
                    bytes_total += len(blob)
                if done % 400 == 0:
                    db.commit()  # 周期提交，中断后 --resume 不丢进度
                    print(f"  rendered {done}/{len(tasks)}", flush=True)
        db.commit()

    meta = {
        "name": args.name,
        "format": "png",           # vendor MBTilesFileSource 据此生成 tiles 模板
        "type": "overlay",
        "version": "1",
        "description": "terrarium raster-dem tiles for offline hillshade",
        "encoding": "terrarium",   # 信息字段；解码方式以 style 源声明为准
        "minzoom": str(args.minzoom),
        "maxzoom": str(args.maxzoom),
        "bounds": ",".join(f"{v:.6f}" for v in bbox),
        "center": f"{(bbox[0]+bbox[2])/2:.6f},{(bbox[1]+bbox[3])/2:.6f},{args.minzoom}",
        "scheme": "tms",
    }
    finalize_mbtiles(db, meta)

    size_mb = os.path.getsize(args.output) / 1e6
    print(f"\ndone: {args.output}")
    print(f"  written={stats['written']}  skipped(relief<{args.relief_threshold}m)="
          f"{stats['skipped_relief']}  skipped(nodata)={stats['skipped_nodata']}")
    print(f"  package size = {size_mb:.1f} MB (tile payload {bytes_total/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
