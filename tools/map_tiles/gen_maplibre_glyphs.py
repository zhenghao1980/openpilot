import sys, os
import numpy as np
import freetype
from scipy.ndimage import distance_transform_edt

# Generate MapLibre-compatible SDF glyph PBFs, following the exact convention
# of mapbox/node-fontnik as validated against mbgl's glyph_pbf.cpp:
#   - glyph rendered at 24px, tight bbox (w0 x h0)
#   - bitmap = (w0 + 2*3) x (h0 + 2*3), glyph placed at offset (3,3)
#   - width=w0, height=h0 (bitmap must be exactly (w+6)*(h+6) bytes)
#   - left = freetype bitmap_left
#   - top  = bitmap_top - 24 - 3
#   - SDF byte = clip(round(255 * (0.75 - sd/8)))  (mbgl tiny_sdf radius=8 cutoff=.25
#     encoding: edge=191, interior=255, exterior=0; matches symbol_sdf.fragment.glsl
#     inner_edge = (256-64)/256 = 0.75)

FONT = sys.argv[1]
OUT = sys.argv[2]
SIZE = 24
BORDER = 3
SDF_RADIUS = 8.0

def varint(n):
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n: out.append(b | 0x80)
        else:
            out.append(b); break
    return bytes(out)
def fb(num, data): return varint(num << 3 | 2) + varint(len(data)) + data
def fv(num, n): return varint(num << 3 | 0) + varint(n)
def fs(num, n):
    zz = ((n << 1) ^ (n >> 31)) & 0xFFFFFFFF
    return varint(num << 3 | 0) + varint(zz)

face = freetype.Face(FONT)
face.set_pixel_sizes(0, SIZE)
os.makedirs(OUT, exist_ok=True)

blocks = [(0x0000, 0xD7FF), (0xE000, 0xFFEF)]
nglyph = 0
for lo, hi in blocks:
    for start in range(lo, hi + 1, 256):
        glyphs = []
        for cp in range(start, min(start + 256, hi + 1)):
            idx = face.get_char_index(cp)
            if idx == 0: continue
            try:
                face.load_glyph(idx, freetype.FT_LOAD_RENDER | freetype.FT_LOAD_TARGET_NORMAL)
            except Exception:
                continue
            g = face.glyph
            bm = g.bitmap
            w0, h0 = bm.width, bm.rows
            left = g.bitmap_left
            top = g.bitmap_top - SIZE - BORDER
            adv = int(round(g.advance.x / 64.0))
            if w0 == 0 or h0 == 0:
                # zero-area glyph (e.g. space): metrics only, no bitmap
                glyphs.append(fv(1, cp) + fv(3, 0) + fv(4, 0) + fs(5, left) + fs(6, top) + fv(7, adv))
                nglyph += 1
                continue
            alpha = np.frombuffer(bytes(bm.buffer), dtype=np.uint8).reshape(h0, w0).astype(np.float32) / 255.0
            canvas = np.zeros((h0 + 2 * BORDER, w0 + 2 * BORDER), np.float32)
            canvas[BORDER:BORDER + h0, BORDER:BORDER + w0] = alpha
            inside = canvas > 0.5
            dt_in = distance_transform_edt(inside)
            dt_out = distance_transform_edt(~inside)
            sd = dt_out - dt_in  # positive OUTSIDE, px
            sdf = np.clip(np.round(255.0 * (0.75 - sd / SDF_RADIUS)), 0, 255).astype(np.uint8)
            gm = fv(1, cp) + fb(2, sdf.tobytes()) + fv(3, w0) + fv(4, h0)
            gm += fs(5, left) + fs(6, top) + fv(7, adv)
            glyphs.append(gm)
            nglyph += 1
        # always write the range file, even when empty, so mbgl never 404s
        stack = fb(1, os.path.basename(OUT).encode()) + fb(2, ("%d-%d" % (start, start + 255)).encode())
        for gm in glyphs:
            stack += fb(3, gm)
        open(os.path.join(OUT, "%d-%d.pbf" % (start, start + 255)), "wb").write(fb(1, stack))
print("GLYPHS_DONE", OUT, nglyph)
