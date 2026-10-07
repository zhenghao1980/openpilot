/* qoi_dec.c - fast QOI decoder for the UI maprender client.
 * Byte-for-byte compatible with the pure-Python decoder in
 * openpilot/selfdrive/ui/onroad/sp_maprender_client.py (standard QOI spec:
 * initial pixel (0,0,0,255), 64-entry zeroed index).
 *
 * Build:  gcc -O2 -shared -fPIC -o libqoi_dec.so qoi_dec.c
 * The Python client dlopens libqoi_dec.so when present and falls back to the
 * pure-Python decoder otherwise.
 */
#include <stdlib.h>
#include <string.h>

/* Returns 0 on success; decodes straight into caller-provided `out`
 * (must hold at least w*h*4 bytes). */
int qoi_decode_rgba_into(const unsigned char *data, int size,
                         int *out_w, int *out_h, unsigned char *out) {
  if (!data || !out_w || !out_h || !out) return -1;
  if (size < 14 || memcmp(data, "qoif", 4) != 0) return -1;
  const int w = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
  const int h = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
  const int channels = data[12];
  if (w <= 0 || h <= 0 || w > 8192 || h > 8192) return -1;
  if (channels != 3 && channels != 4) return -1;
  const long npix = (long)w * (long)h;
  if (npix * 4 > 256L * 1024 * 1024) return -1;

  unsigned char index[64][4];
  memset(index, 0, sizeof(index));
  int r = 0, g = 0, b = 0, a = 255;
  long p = 14, o = 0;
  int run = 0;

  for (long i = 0; i < npix; i++) {
    if (run > 0) {
      run--;
    } else {
      if (p >= size) return -1;
      const unsigned char b1 = data[p++];
      if (b1 == 0xFE) {
        if (p + 3 > size) return -1;
        r = data[p]; g = data[p + 1]; b = data[p + 2]; p += 3;
      } else if (b1 == 0xFF) {
        if (p + 4 > size) return -1;
        r = data[p]; g = data[p + 1]; b = data[p + 2]; a = data[p + 3]; p += 4;
      } else {
        const unsigned char tag = b1 & 0xC0;
        if (tag == 0x00) {
          r = index[b1 & 0x3F][0]; g = index[b1 & 0x3F][1];
          b = index[b1 & 0x3F][2]; a = index[b1 & 0x3F][3];
        } else if (tag == 0x40) {
          r = (r + ((b1 >> 4) & 3) - 2) & 0xFF;
          g = (g + ((b1 >> 2) & 3) - 2) & 0xFF;
          b = (b + (b1 & 3) - 2) & 0xFF;
        } else if (tag == 0x80) {
          if (p + 1 > size) return -1;
          const unsigned char b2 = data[p++];
          const int dg = (b1 & 0x3F) - 32;
          r = (r + dg + ((b2 >> 4) & 0xF) - 8) & 0xFF;
          g = (g + dg) & 0xFF;
          b = (b + dg + (b2 & 0xF) - 8) & 0xFF;
        } else {
          run = b1 & 0x3F;
        }
      }
      index[(r * 3 + g * 5 + b * 7 + a * 11) % 64][0] = (unsigned char)r;
      index[(r * 3 + g * 5 + b * 7 + a * 11) % 64][1] = (unsigned char)g;
      index[(r * 3 + g * 5 + b * 7 + a * 11) % 64][2] = (unsigned char)b;
      index[(r * 3 + g * 5 + b * 7 + a * 11) % 64][3] = (unsigned char)a;
    }
    out[o] = (unsigned char)r; out[o + 1] = (unsigned char)g;
    out[o + 2] = (unsigned char)b; out[o + 3] = (unsigned char)a;
    o += 4;
  }

  *out_w = w;
  *out_h = h;
  return 0;
}

/* Returns 0 on success; *out gets a malloc'ed RGBA buffer (w*h*4) that the
 * caller must free(). */
int qoi_decode_rgba(const unsigned char *data, int size,
                    int *out_w, int *out_h, unsigned char **out) {
  if (!out) return -1;
  *out = NULL;
  if (!data || size < 14 || memcmp(data, "qoif", 4) != 0) return -1;
  const int w = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
  const int h = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
  const long npix = (long)w * (long)h;
  if (npix * 4 > 256L * 1024 * 1024) return -1;
  unsigned char *px = (unsigned char *)malloc((size_t)npix * 4);
  if (!px) return -1;
  if (qoi_decode_rgba_into(data, size, out_w, out_h, px) != 0) {
    free(px);
    return -1;
  }
  *out = px;
  return 0;
}
