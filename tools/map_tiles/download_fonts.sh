#!/usr/bin/env bash
# 下载并"校验"字体栈 glyph PBF（v2：下载后逐文件验证 PBF magic，失败即报错退出，
# 不再静默存 HTML 错误页——v1 的 curl -sf || true 会把代理错误页存成 .pbf，
# 导致 maplibre glyph 解析崩溃）。
set -euo pipefail
OUT=${1:-./fonts}
BASE=${FONTS_BASE:-https://fonts.openmaptiles.org}
mkdir -p "$OUT"
fail=0
for STACK in "Noto Sans Regular" "Noto Sans Bold"; do
  ENC=$(python3 -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$STACK")
  for RANGE in $(seq 0 256 65535); do
    END=$((RANGE+255))
    DIR="$OUT/$STACK"; mkdir -p "$DIR"
    FP="$DIR/$RANGE-$END.pbf"
    curl -sf "$BASE/$ENC/$RANGE-$END.pbf" -o "$FP" || { echo "DL FAIL: $FP"; fail=1; continue; }
    # 校验：glyph pbf 首字节应为 0x0a（field1 LEN）；HTML '<' = 0x3c
    FIRST=$(xxd -p -l1 "$FP")
    if [ "$FIRST" != "0a" ]; then
      echo "BAD PBF (first=0x$FIRST, 疑似HTML错误页): $FP"
      rm -f "$FP"; fail=1
    fi
  done
  echo "fontstack done: $STACK"
done
if [ "$fail" != "0" ]; then
  echo "== 存在失败/损坏文件（见上）。网络不可达时请改用 fontnik 本地生成（见 gen_glyphs_local.md）=="
  exit 1
fi
echo "glyphs -> $OUT （部署：scp -r $OUT comma:/data/mapd_render/glyphs）"
