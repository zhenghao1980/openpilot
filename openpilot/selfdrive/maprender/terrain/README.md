# terrain/ — 离线高程地形包构建与预览工具

配套《离线地图高程渲染（hillshade）方案》§3 / §5.2 / §6。构建侧（PC）运行，
产出 `terrain_<省>.mbtiles`（terrarium 编码 raster-dem，标准 mbtiles schema，
TMS 行序，PNG blob 不压缩），拷贝到车端 `/data/mapd_render/` 即可被
maprenderd 经 `TerrainMbtilesPath` / argv[3] 加载。

## 工具

| 脚本 | 用途 |
|---|---|
| `fetch_copdem_aws.py` | 从 AWS Open Data 公共桶按省界批量下载 GLO-30 原始 GeoTIFF（免注册、断点续传） |
| `build_terrain_mbtiles.py` | DEM → terrarium PNG → mbtiles 省包 |
| `preview_hillshade.py` | 不出车端，直接从地形包渲染 hillshade 预览图，用于 exaggeration 三档对比与 overzoom 平滑性验收 |

正式切分一条命令（四川实例，已实测）：

```bash
curl -O https://geo.datav.aliyun.com/areas_v3/bound/510000_full.json   # 省界
python fetch_copdem_aws.py --geojson 510000_full.json -o dem/sichuan   # 原始 DEM，长期保留（方案 §3.1）
python build_terrain_mbtiles.py --source geotiff --dem dem/sichuan/*.tif \
    --geojson 510000_full.json --minzoom 4 --maxzoom 13 \
    --relief-threshold 50 --quantize 1 --workers 14 --resume \
    -o terrain_sichuan.mbtiles
```

## 关键参数（方案 §3.3 / §5.2，部分经试点校准）

- `--maxzoom`：山区省 13，中低山 12，平原省 11；z14 排除（对 30m DEM 无信息增量）
- `--relief-threshold 50`：不规则裁剪，瓦片高程极差 < 50m 不出片，
  端上靠 overzoom 降级（平原大省可再省 30%+，成都平原实测 38%）
- `--quantize 1`：**高程量化为 1 米**。geotiff 模式双线性重投影产生的亚米级
  噪声会使 terrarium PNG 体积膨胀约 2.7 倍（实测 134→50 KB/片）；GLO-30
  垂直精度为米级，1m 量化对山体阴影视觉无损。geotiff 正式切分务必开启
- `--resume`：断点续切（周期提交，全国切分中断后直接重跑同命令）
- `--workers`：geotiff 模式进程池并行，建议设为本机核数
- `--minzoom 4`：与 style 中 hillshade `minzoom: 4` 对齐

## 数据源

1. **试点**：`--source aws-terrarium`，AWS Open Data 公共 terrarium 瓦片
   （`elevation-tiles-prod`，免注册）。注意其为多源融合数据，精度不足以
   替代正式 DEM，仅用于效果验证。
2. **正式**：`--source geotiff --dem dem/sichuan/*.tif`，Copernicus DEM
   GLO-30 原始 GeoTIFF，需 `pip install rasterio`。推荐用
   `fetch_copdem_aws.py` 从 AWS Open Data 公共桶免注册批量下载
   （中国区覆盖完整，已实测）；要最新 2023_1 修订版再走 CDSE 注册账号。
   **原始 GeoTIFF 在构建侧长期保留**（方案 §3.1 / §7.1，为 Valhalla skadi
   高程服务预留）。

## 省级多边形裁剪

省界 GeoJSON 用 DataV.GeoAtlas：
`https://geo.datav.aliyun.com/areas_v3/bound/<adcode>_full.json`
（四川 adcode=510000），完整流程见上方"正式切分一条命令"。

## 验收预览

```bash
python preview_hillshade.py terrain_sichuan.mbtiles \
    --center 101.879,29.595 --zooms 10,12,13,14 \
    --exaggerations 0.3,0.5,0.8 --outdir previews
```

- z > 包 maxzoom 时自动取父级瓦片放大，模拟 maplibre overzoom；
- 被不规则裁剪跳过的瓦片在预览中保持底色，目检无黑块即可。
