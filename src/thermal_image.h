// Renders a 32x24 MLX90640 temperature frame into an upscaled false-colour bitmap.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "config.h"

#define MLX_W 32
#define MLX_H 24
#define IMG_W (MLX_W * IMAGE_SCALE)
#define IMG_H (MLX_H * IMAGE_SCALE)

class ThermalImage {
public:
  // Allocates the pixel buffer (PSRAM when available) and builds the palette.
  bool begin();

  // Scans a frame for the minimum / maximum plausible temperature.
  // Returns false if the frame contained no valid pixels.
  static bool stats(const float *frame, float &minC, float &maxC);

  // Renders `frame` mapping lo..hi degrees C onto the palette. The result is a
  // tightly packed 24-bit buffer in the byte order expected by fmt2jpg().
  void render(const float *frame, float lo, float hi);

  const uint8_t *pixels() const { return _pixels; }
  size_t pixelBytes() const { return (size_t)IMG_W * IMG_H * 3; }
  int width() const { return IMG_W; }
  int height() const { return IMG_H; }

private:
  void buildPalette();

  uint8_t *_pixels = nullptr;
  uint8_t _palette[256][3];  // stored in output byte order
};
