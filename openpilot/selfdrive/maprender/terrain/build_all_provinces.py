#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_all_provinces.py — 全国省级地形包批量切分驱动（hillshade 方案 §6 步骤 4）

按方案 §3.3 分级表逐省切分 terrarium raster-dem 省包：
  z13 山区省 / z12 中低山丘陵省 / z11 平原沿海省，统一 relief-threshold 50m
  不规则裁剪 + 1m 高程量化（试点校准结论，见 README）。

依赖：
  - dem/china/*.tif  全国 GLO-30 原始 DEM（fetch_copdem_aws.py 先行下载）
  - 省界 GeoJSON 自动从 DataV.GeoAtlas 下载缓存到 geojson/

断点续跑：每省完成后写 output/terrain_<省>.done 标记，重跑自动跳过；
省内中断由 build_terrain_mbtiles.py --resume 续切。可反复执行本脚本直至
全部完成。

用法：
  python build_all_provinces.py --dem-dir dem/china --outdir output \
      --workers 14 [--only yunnan,xizang]
"""
import argparse
import os
import subprocess
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
BUILDER = os.path.join(HERE, "build_terrain_mbtiles.py")
GEOJSON_URL = "https://geo.datav.aliyun.com/areas_v3/bound/{adcode}_full.json"

# (拼音, adcode, maxzoom) —— 分级依据方案 §3.3 初版分级表
# z13：高原山地省；z12：中低山丘陵省；z11：平原沿海省
# 方案中跨省混合情形（河南/新疆/内蒙古/浙江/安徽）取高值封顶 + 不规则裁剪自动瘦身
PROVINCES = [
    # ---- z13 山区省（观感主要来源，优先）----
    ("chongqing", 500000, 13),
    ("guizhou", 520000, 13),
    ("shaanxi", 610000, 13),
    ("shanxi", 140000, 13),
    ("gansu", 620000, 13),
    ("yunnan", 530000, 13),
    ("qinghai", 630000, 13),
    ("sichuan", 510000, 13),   # 已切过会自动跳过（.done 标记）
    ("xizang", 540000, 13),
    ("xinjiang", 650000, 13),
    # ---- z12 中低山丘陵省 ----
    ("guangxi", 450000, 12),
    ("hunan", 430000, 12),
    ("hubei", 420000, 12),
    ("henan", 410000, 12),
    ("jiangxi", 360000, 12),
    ("zhejiang", 330000, 12),
    ("anhui", 340000, 12),
    ("jilin", 220000, 12),
    ("heilongjiang", 230000, 12),
    ("neimenggu", 150000, 12),
    ("hainan", 460000, 12),
    # ---- z11 平原/沿海 ----
    ("hebei", 130000, 11),
    ("shandong", 370000, 11),
    ("liaoning", 210000, 11),
    ("jiangsu", 320000, 11),
    ("guangdong", 440000, 11),
    ("fujian", 350000, 11),
    ("ningxia", 640000, 11),
    ("beijing", 110000, 11),
    ("tianjin", 120000, 11),
    ("shanghai", 310000, 11),
    ("taiwan", 710000, 11),
    ("xianggang", 810000, 11),
    ("aomen", 820000, 11),
]


def fetch_geojson(adcode, cache_dir):
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, f"{adcode}_full.json")
    if os.path.exists(path) and os.path.getsize(path) > 1000:
        return path
    url = GEOJSON_URL.format(adcode=adcode)
    with urllib.request.urlopen(url, timeout=60) as r:
        data = r.read()
    with open(path, "wb") as f:
        f.write(data)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dem-dir", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--relief-threshold", type=float, default=50)
    ap.add_argument("--quantize", type=float, default=1)
    ap.add_argument("--minzoom", type=int, default=4)
    ap.add_argument("--only", help="只切指定省（拼音逗号分隔）")
    args = ap.parse_args()

    dem_paths = sorted(
        os.path.join(args.dem_dir, f) for f in os.listdir(args.dem_dir)
        if f.endswith(".tif"))
    if not dem_paths:
        sys.exit(f"no .tif in {args.dem_dir}")
    print(f"DEM sources: {len(dem_paths)} tiles in {args.dem_dir}")
    # 命令行长度上限（WinError 206）：DEM 清单走文件传递
    dem_list_path = os.path.join(args.outdir, "_dem_list.txt")
    with open(dem_list_path, "w", encoding="utf-8") as f:
        f.write("\n".join(os.path.abspath(p) for p in dem_paths))

    only = set(args.only.split(",")) if args.only else None
    geojson_dir = os.path.join(args.outdir, "geojson")
    os.makedirs(args.outdir, exist_ok=True)

    pending = []
    for name, adcode, maxz in PROVINCES:
        if only and name not in only:
            continue
        done_marker = os.path.join(args.outdir, f"terrain_{name}.done")
        if os.path.exists(done_marker):
            print(f"[skip] {name} (done marker exists)")
            continue
        pending.append((name, adcode, maxz))

    print(f"pending provinces: {len(pending)}")
    for name, adcode, maxz in pending:
        print(f"\n===== {name} (adcode={adcode}, z{args.minzoom}-z{maxz}) =====",
              flush=True)
        gj = fetch_geojson(adcode, geojson_dir)
        out = os.path.join(args.outdir, f"terrain_{name}.mbtiles")
        cmd = [sys.executable, BUILDER,
               "--source", "geotiff",
               "--dem-list", dem_list_path,
               "--geojson", gj,
               "--minzoom", str(args.minzoom),
               "--maxzoom", str(maxz),
               "--relief-threshold", str(args.relief_threshold),
               "--quantize", str(args.quantize),
               "--workers", str(args.workers),
               "--resume",
               "--name", f"terrain_{name}",
               "-o", out]
        ret = subprocess.run(cmd).returncode
        if ret != 0:
            print(f"[ABORT] {name} failed rc={ret}; rerun this script to resume",
                  flush=True)
            sys.exit(ret)
        with open(os.path.join(args.outdir, f"terrain_{name}.done"), "w") as f:
            f.write("ok\n")
        size_mb = os.path.getsize(out) / 1e6
        print(f"===== {name} done: {size_mb:.0f} MB =====", flush=True)

    print("\nALL PROVINCES DONE")


if __name__ == "__main__":
    main()
