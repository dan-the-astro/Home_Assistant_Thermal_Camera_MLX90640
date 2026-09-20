#include "thermal_image.h"

#include <Arduino.h>
#include <algorithm>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

namespace {

// "Ironbow" style gradient: black -> purple -> red -> orange -> yellow -> white.
struct Stop { float pos; uint8_t r, g, b; };
const Stop kStops[] = {
  {0.00f,   0,   0,   0},
  {0.10f,  20,   0,  80},
  {0.30f, 120,   0, 150},
  {0.50f, 210,  30,  90},
  {0.70f, 245, 120,  20},
  {0.85f, 255, 205,  40},
  {1.00f, 255, 255, 255},
};
const int kStopCount = sizeof(kStops) / sizeof(kStops[0]);

inline bool plausible(float t) {
  return !isnan(t) && !isinf(t) && t >= VALID_TEMP_MIN_C && t <= VALID_TEMP_MAX_C;
}

// Scratch for range(). Static so a 3 kB copy of the frame does not land on the
// Arduino loop task's stack once per frame.
float gSorted[MLX_W * MLX_H];

}  // namespace

bool ThermalImage::begin() {
  if (_pixels) return true;
  const size_t bytes = pixelBytes();
  if (psramFound()) {
    _pixels = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!_pixels) {
    _pixels = (uint8_t *)malloc(bytes);
  }
  if (!_pixels) return false;
  memset(_pixels, 0, bytes);
  buildPalette();
  return true;
}

void ThermalImage::buildPalette() {
  for (int i = 0; i < 256; i++) {
    const float p = i / 255.0f;
    int s = 0;
    while (s < kStopCount - 2 && p > kStops[s + 1].pos) s++;
    const Stop &a = kStops[s];
    const Stop &b = kStops[s + 1];
    const float t = (p - a.pos) / (b.pos - a.pos);
    const uint8_t r = (uint8_t)lroundf(a.r + (b.r - a.r) * t);
    const uint8_t g = (uint8_t)lroundf(a.g + (b.g - a.g) * t);
    const uint8_t bl = (uint8_t)lroundf(a.b + (b.b - a.b) * t);
    // The esp32-camera JPEG encoder expects PIXFORMAT_RGB888 buffers with the
    // bytes in memory as B, G, R (the layout its own converters produce).
#if IMAGE_SWAP_RB
    _palette[i][0] = r; _palette[i][1] = g; _palette[i][2] = bl;
#else
    _palette[i][0] = bl; _palette[i][1] = g; _palette[i][2] = r;
#endif
  }
}

bool ThermalImage::stats(const float *frame, float &minC, float &maxC) {
  bool any = false;
  float lo = 0, hi = 0;
  for (int i = 0; i < MLX_W * MLX_H; i++) {
    const float t = frame[i];
    if (!plausible(t)) continue;
    if (!any) { lo = hi = t; any = true; }
    else { if (t < lo) lo = t; if (t > hi) hi = t; }
  }
  if (any) { minC = lo; maxC = hi; }
  return any;
}

bool ThermalImage::range(const float *frame, float lowPercent, float highPercent, float &lo, float &hi) {
  int n = 0;
  for (int i = 0; i < MLX_W * MLX_H; i++) {
    if (plausible(frame[i])) gSorted[n++] = frame[i];
  }
  if (n == 0) return false;

  int loIdx = (int)lroundf(lowPercent * 0.01f * (n - 1));
  int hiIdx = (int)lroundf(highPercent * 0.01f * (n - 1));
  loIdx = constrain(loIdx, 0, n - 1);
  hiIdx = constrain(hiIdx, loIdx, n - 1);

  // After the first partition everything from loIdx on is already >= the low
  // end, so the second one only has to look at that tail.
  std::nth_element(gSorted, gSorted + loIdx, gSorted + n);
  lo = gSorted[loIdx];
  std::nth_element(gSorted + loIdx, gSorted + hiIdx, gSorted + n);
  hi = gSorted[hiIdx];
  return true;
}

void ThermalImage::render(const float *frame, float lo, float hi) {
  if (!_pixels) return;
  float span = hi - lo;
  if (span < 0.01f) span = 0.01f;
  const float scale = 255.0f / span;

  // Precompute horizontal sampling positions (shared by every output row).
  static int x0[IMG_W], x1[IMG_W];
  static float fx[IMG_W];
  for (int x = 0; x < IMG_W; x++) {
    float sx = (x + 0.5f) / IMAGE_SCALE - 0.5f;
#if FLIP_HORIZONTAL
    sx = (MLX_W - 1) - sx;
#endif
    if (sx < 0) sx = 0;
    if (sx > MLX_W - 1) sx = MLX_W - 1;
    x0[x] = (int)sx;
    x1[x] = x0[x] < MLX_W - 1 ? x0[x] + 1 : x0[x];
    fx[x] = sx - x0[x];
  }

  uint8_t *dst = _pixels;
  for (int y = 0; y < IMG_H; y++) {
    float sy = (y + 0.5f) / IMAGE_SCALE - 0.5f;
#if FLIP_VERTICAL
    sy = (MLX_H - 1) - sy;
#endif
    if (sy < 0) sy = 0;
    if (sy > MLX_H - 1) sy = MLX_H - 1;
    const int y0 = (int)sy;
    const int y1 = y0 < MLX_H - 1 ? y0 + 1 : y0;
    const float fy = sy - y0;
    const float *row0 = frame + y0 * MLX_W;
    const float *row1 = frame + y1 * MLX_W;

    for (int x = 0; x < IMG_W; x++) {
      float a = row0[x0[x]], b = row0[x1[x]];
      float c = row1[x0[x]], d = row1[x1[x]];
      // Replace glitched pixels with a neighbour so they don't produce black/white specks.
      if (!plausible(a)) a = plausible(b) ? b : lo;
      if (!plausible(b)) b = a;
      if (!plausible(c)) c = a;
      if (!plausible(d)) d = c;
      const float top = a + (b - a) * fx[x];
      const float bot = c + (d - c) * fx[x];
      const float t = top + (bot - top) * fy;
      int idx = (int)((t - lo) * scale + 0.5f);
      if (idx < 0) idx = 0;
      if (idx > 255) idx = 255;
      const uint8_t *p = _palette[idx];
      *dst++ = p[0];
      *dst++ = p[1];
      *dst++ = p[2];
    }
  }
}
