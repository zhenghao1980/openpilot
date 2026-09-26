/* qoi.h - QOI (Quite OK Image) encoder, API-compatible with the reference
 * implementation by Dominic Szablewski (public domain, https://qoiformat.org).
 * Only encoding is needed on-device (decoding lives in Python on the UI side,
 * see sp_maprender_client.py). Written to match that decoder byte-for-byte.
 */
#ifndef MAPD_QOI_H
#define MAPD_QOI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  unsigned int width;
  unsigned int height;
  unsigned char channels;    /* 3 = RGB, 4 = RGBA */
  unsigned char colorspace;  /* 0 = sRGB (linear alpha), 1 = linear */
} qoi_desc;

/* Returns malloc'ed buffer (caller frees with free()); *out_len set. */
void *qoi_encode(const void *data, const qoi_desc *desc, int *out_len);

#ifdef __cplusplus
}
#endif

#endif /* MAPD_QOI_H */

#ifdef QOI_IMPLEMENTATION

#include <stdlib.h>
#include <string.h>

#define QOI_OP_INDEX 0x00 /* 00xxxxxx */
#define QOI_OP_DIFF  0x40 /* 01xxxxxx */
#define QOI_OP_LUMA  0x80 /* 10xxxxxx */
#define QOI_OP_RUN   0xc0 /* 11xxxxxx */
#define QOI_OP_RGB   0xfe
#define QOI_OP_RGBA  0xff
#define QOI_MASK_2   0xc0

#define QOI_MAX_RUN 62

static int qoi_hash(int r, int g, int b, int a) {
  return (r * 3 + g * 5 + b * 7 + a * 11) % 64;
}

void *qoi_encode(const void *data, const qoi_desc *desc, int *out_len) {
  if (!data || !desc || !out_len) return NULL;
  const int w = (int)desc->width, h = (int)desc->height;
  const int channels = desc->channels;
  if (w <= 0 || h <= 0 || (channels != 3 && channels != 4)) return NULL;
  const int npix = w * h;
  const int cap = npix * (channels + 1) + 14 + 8;
  unsigned char *out = (unsigned char *)malloc(cap);
  if (!out) return NULL;

  unsigned char *o = out;
  memcpy(o, "qoif", 4); o += 4;
  unsigned int be;
  be = (unsigned int)w; o[0] = be >> 24; o[1] = be >> 16; o[2] = be >> 8; o[3] = be; o += 4;
  be = (unsigned int)h; o[0] = be >> 24; o[1] = be >> 16; o[2] = be >> 8; o[3] = be; o += 4;
  o[0] = (unsigned char)channels;
  o[1] = desc->colorspace;
  o += 2;

  unsigned char index[64 * 4];  /* 4B/entry: r,g,b,a */
  memset(index, 0, sizeof(index));

  int pr = 0, pg = 0, pb = 0, pa = 255;   /* previous pixel */
  int run = 0;
  const unsigned char *px = (const unsigned char *)data;

  for (int i = 0; i < npix; i++, px += channels) {
    const int r = px[0], g = px[1], b = px[2];
    const int a = (channels == 4) ? px[3] : 255;

    if (r == pr && g == pg && b == pb && a == pa) {
      run++;
      if (run == QOI_MAX_RUN || i == npix - 1) {
        *o++ = QOI_OP_RUN | (run - 1);
        run = 0;
      }
      continue;
    }

    if (run > 0) {
      *o++ = QOI_OP_RUN | (run - 1);
      run = 0;
    }

    const int h = qoi_hash(r, g, b, a);
    if (index[h * 4 + 0] == r && index[h * 4 + 1] == g &&
        index[h * 4 + 2] == b && index[h * 4 + 3] == a) {
      *o++ = QOI_OP_INDEX | h;
    } else {
      index[h * 4 + 0] = (unsigned char)r;
      index[h * 4 + 1] = (unsigned char)g;
      index[h * 4 + 2] = (unsigned char)b;
      index[h * 4 + 3] = (unsigned char)a;
      if (a == pa) {
        const int vr = (int)(signed char)(r - pr);
        const int vg = (int)(signed char)(g - pg);
        const int vb = (int)(signed char)(b - pb);
        const int vg_r = vr - vg;
        const int vg_b = vb - vg;
        if (vr > -3 && vr < 2 && vg > -3 && vg < 2 && vb > -3 && vb < 2) {
          *o++ = QOI_OP_DIFF | ((vr + 2) << 4) | ((vg + 2) << 2) | (vb + 2);
        } else if (vg > -33 && vg < 32 && vg_r > -9 && vg_r < 8 && vg_b > -9 && vg_b < 8) {
          *o++ = QOI_OP_LUMA | (vg + 32);
          *o++ = ((vg_r + 8) << 4) | (vg_b + 8);
        } else {
          *o++ = QOI_OP_RGB;
          *o++ = (unsigned char)r;
          *o++ = (unsigned char)g;
          *o++ = (unsigned char)b;
        }
      } else {
        *o++ = QOI_OP_RGBA;
        *o++ = (unsigned char)r;
        *o++ = (unsigned char)g;
        *o++ = (unsigned char)b;
        *o++ = (unsigned char)a;
      }
    }
    pr = r; pg = g; pb = b; pa = a;
  }

  for (int k = 0; k < 7; k++) *o++ = 0;
  *o++ = 1;

  *out_len = (int)(o - out);
  return out;
}

#endif /* QOI_IMPLEMENTATION */
