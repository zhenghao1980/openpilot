#!/usr/bin/env python3
"""从本地 tileserver-gl 批量导出栅格瓦片（设备离线用）。

用法：
  python export_tiles.py --bbox 39.75 116.15 40.05 116.55 --zooms 14 15 16 --out ./tiles
  # bbox = lat_min lon_min lat_max lon_max（先小范围试跑！z16 一个城区约数千张）

产出 z/x/y.png 目录树，scp 到设备：/data/mapd_tiles/
"""
import argparse, math, os, sys, time, urllib.request

def lonlat_to_tile(lon, lat, z):
  n = 2 ** z
  x = int((lon + 180.0) / 360.0 * n)
  lat_r = math.radians(max(min(lat, 85.05), -85.05))
  y = int((1.0 - math.log(math.tan(lat_r) + 1 / math.cos(lat_r)) / math.pi) / 2.0 * n)
  return max(0, min(n - 1, x)), max(0, min(n - 1, y))

def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--bbox", type=float, nargs=4, required=True, metavar=("LAT_MIN", "LON_MIN", "LAT_MAX", "LON_MAX"))
  ap.add_argument("--zooms", type=int, nargs="+", default=[14, 15, 16])
  ap.add_argument("--out", default="./tiles")
  ap.add_argument("--url", default="http://localhost:8080/styles/nav-night/{z}/{x}/{y}.png")
  ap.add_argument("--sleep", type=float, default=0.02)
  a = ap.parse_args()
  lat0, lon0, lat1, lon1 = a.bbox

  total = done = fail = skip = 0
  for z in a.zooms:
    x0, y1 = lonlat_to_tile(lon0, lat0, z)   # min lon, min lat -> x0, y1(南)
    x1, y0 = lonlat_to_tile(lon1, lat1, z)
    for x in range(x0, x1 + 1):
      for y in range(y0, y1 + 1):
        total += 1
        d = os.path.join(a.out, str(z), str(x))
        fp = os.path.join(d, f"{y}.png")
        if os.path.exists(fp):
          skip += 1; continue
        os.makedirs(d, exist_ok=True)
        try:
          with urllib.request.urlopen(a.url.format(z=z, x=x, y=y), timeout=10) as r:
            if r.status == 200:
              open(fp, "wb").write(r.read()); done += 1
            else:
              fail += 1
        except Exception:
          fail += 1
        time.sleep(a.sleep)
    print(f"z{z} 完成 | 累计 total={total} new={done} skip={skip} fail={fail}", flush=True)
  print(f"== 导出结束：total={total} new={done} skip={skip} fail={fail} ==")
  print(f"设备部署：scp -r {a.out} comma:/data/mapd_tiles")
  sys.exit(1 if fail else 0)

if __name__ == "__main__":
  main()
