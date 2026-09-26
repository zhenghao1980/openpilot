#!/usr/bin/env bash
# 下载 style.json 所需字体栈的 glyph PBF 到 ./fonts/{fontstack}/{range}.pbf
# 字体栈需与 style.json 的 text-font 一致（当前: Noto Sans Regular / Noto Sans Bold）。
# CJK 路名标注需要含 CJK 的字体栈；fonts.openmaptiles.org 可用栈见其根路径列表，
# 若无 CJK 栈，改用 node fontnik 从 Noto Sans CJK SC 生成（见 README §4.6 备注）。
set -e
OUT=${1:-./fonts}
BASE="https://fonts.openmaptiles.org"
mkdir -p "$OUT"
for STACK in "Noto Sans Regular" "Noto Sans Bold"; do
  ENC=$(python3 -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$STACK")
  for RANGE in $(seq 0 256 65535); do
    END=$((RANGE+255))
    DIR="$OUT/$STACK"; mkdir -p "$DIR"
    curl -sf "$BASE/$ENC/$RANGE-$END.pbf" -o "$DIR/$RANGE-$END.pbf" || true
  done
  echo "fontstack done: $STACK"
done
echo "glyphs -> $OUT （部署：scp -r $OUT comma:/data/mapd_render/glyphs）"
