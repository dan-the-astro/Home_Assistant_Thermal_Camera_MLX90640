#include "frame_filter.h"

#include <algorithm>
#include <math.h>
#include <string.h>

#include "config.h"
#include "thermal_image.h"  // MLX_W / MLX_H

namespace {

constexpr int kPixels = MLX_W * MLX_H;
constexpr int kMaxLine = MLX_W > MLX_H ? MLX_W : MLX_H;

inline bool plausible(float t) {
  return !isnan(t) && !isinf(t) && t >= VALID_TEMP_MIN_C && t <= VALID_TEMP_MAX_C;
}

// +1 on even indices, -1 on odd ones, i.e. (-1)^k.
inline float alt(int k) { return (k & 1) ? -1.0f : 1.0f; }

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Frame-sized scratch for the dead pixel pass. Static rather than stack, so
// 3 kB does not land on the Arduino loop task every frame.
float gCopy[kPixels];

// Learned offsets, carried between frames so a single frame the scene happens
// to confuse cannot swing the correction.
float gRowAmp[MLX_H];
float gColCycle[FPN_COLUMN_PERIOD];
float gChessAmp = 0;
bool gPrimed = false;

// Rearranges v and returns its median. O(n), no allocation.
float median(float *v, int n) {
  if (n <= 0) return 0.0f;
  float *mid = v + n / 2;
  std::nth_element(v, mid, v + n);
  return *mid;
}

// Copies the nearest measured value into the two ends, which have no second
// difference of their own, then runs a [1 2 1] smoother along the profile.
// The amplitude envelope is a property of the readout electronics and varies
// slowly, so smoothing it costs nothing and steadies a median taken over only
// half a row.
void finishProfile(float *p, int n, int first, int last) {
  if (last < first) return;
  for (int i = 0; i < first; i++) p[i] = p[first];
  for (int i = last + 1; i < n; i++) p[i] = p[last];

#if FPN_ROW_SMOOTHING
  float prev = p[0];
  for (int i = 1; i < n - 1; i++) {
    const float cur = p[i];
    p[i] = (prev + 2.0f * cur + p[i + 1]) * 0.25f;
    prev = cur;
  }
#endif
}

// Recovers the repeating column offsets from their second difference.
//
// A column pass can only see d = o - (o_left + o_right)/2, never o itself,
// because a constant added to every column is the scene's own level and must
// not be touched. Over one period that relation is a circulant matrix whose
// mode k scales by 1 - cos(2*pi*k/P). For P = 4 those factors are 0, 1, 2, 1:
// the zero is the untouchable DC level, and inverting the other three is a
// handful of multiplies. The result sums to zero by construction, so this only
// ever redistributes between the four columns of a group and never shifts the
// picture as a whole.
void solveColumnCycle(const float d[FPN_COLUMN_PERIOD], float o[FPN_COLUMN_PERIOD]) {
  const float u = d[0] - d[2];
  const float v = d[1] - d[3];
  const float w = (d[0] - d[1] + d[2] - d[3]) * 0.125f;
  o[0] = u * 0.5f + w;
  o[1] = v * 0.5f - w;
  o[2] = -u * 0.5f + w;
  o[3] = -v * 0.5f - w;
}

// A scene feature can only mislead the estimator on the lines it actually
// crosses, while the sensor's banding is on every line at a similar size. So
// compare each line with the middle of the profile and pull in whatever stands
// far out: a sharp edge shows up as one large entry among small ones. Without
// this a hard step gets read as banding and subtracted back out of the picture.
void rejectProfileOutliers(float *p, int first, int last, float *scratch) {
  const int n = last - first + 1;
  if (n < 4) return;
  for (int i = 0; i < n; i++) scratch[i] = fabsf(p[first + i]);
  const float limit = median(scratch, n) * FPN_PROFILE_OUTLIER;
  for (int i = first; i <= last; i++) p[i] = clampf(p[i], -limit, limit);
}

#if REPAIR_DEAD_PIXELS
// A few MLX90640 pixels read tens of degrees off. The factory marks them in
// EEPROM, but Adafruit's getFrame() never applies that correction and keeps
// both the pixel list and MLX90640_BadPixelsCorrection() private, so find them
// here instead: anything implausible, or far from the pixels around it, becomes
// the median of its neighbours.
//
// DEAD_PIXEL_DELTA_C is deliberately wide. At 32x24 a genuinely hot object can
// land on a single pixel, and a tighter threshold would erase it.
void repairOutliers(float *frame) {
  // Neighbours are read from a copy so one repair cannot seed the next.
  memcpy(gCopy, frame, sizeof(gCopy));

  for (int y = 0; y < MLX_H; y++) {
    for (int x = 0; x < MLX_W; x++) {
      float neighbours[8];
      int count = 0;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) continue;
          const int ny = y + dy, nx = x + dx;
          if (ny < 0 || ny >= MLX_H || nx < 0 || nx >= MLX_W) continue;
          const float v = gCopy[ny * MLX_W + nx];
          if (plausible(v)) neighbours[count++] = v;
        }
      }
      if (count < 3) continue;  // nothing trustworthy to compare against

      const int i = y * MLX_W + x;
      const float med = median(neighbours, count);
      if (!plausible(gCopy[i]) || fabsf(gCopy[i] - med) > DEAD_PIXEL_DELTA_C) frame[i] = med;
    }
  }
}
#endif  // REPAIR_DEAD_PIXELS

#if FPN_FILTER
// Measures and removes the offsets that repeat every two pixels.
//
// Write the pattern as A(y)*(-1)^y + B(x) + C*(-1)^(x+y): banding between odd
// and even rows, a fixed offset per column, and the checkerboard between the
// two subpages. A is an amplitude that drifts slowly across the array - on a
// real part the row banding fades from one edge to the other. C, being one
// difference between two readouts of the whole array, is a single number.
//
// B is the one that is easy to get wrong. The columns do not simply alternate:
// the array's column electronics work in groups of four, and B repeats with a
// period of four, a cycle a two-pixel model cannot represent at all. So B is
// four numbers, one per column modulo four, shared by the whole array. That is
// also what makes it safe - a room is not periodic across all thirty-two
// columns, so the scene cannot pretend to be this pattern, and the median
// within each group throws out the columns a warm object happens to cross.
//
// Take V, a pixel minus the mean of the two pixels above and below it. The
// pure-column term B is identical in all three, so it cancels; the other two
// alternate with y, so both come out doubled:
//
//     V * (-1)^y = 2A + 2C*(-1)^x
//
// which is 2A+2C on even columns and 2A-2C on odd ones. Their sum gives A and
// their difference gives C, each measured over a set with a single centre - the
// part that matters, because a median only reports the middle of what it is
// given. Medianing a set with two symmetric peaks, which is what mixing the
// components into one estimate produces, lands on whichever peak rounding
// picks rather than between them. Doing the same left to right gives B, and C
// a second time.
//
// Running that per row rather than over the whole frame is what lets A follow
// the array; the pattern it removes is still confined to the sensor's Nyquist
// frequency, which is the safety argument for the whole filter. A real thermal
// scene has almost nothing left at Nyquist, because the lens spreads a point
// source over more than one pixel, so whatever alternates pixel by pixel is the
// sensor and not the room.
void removeParityNoise(float *frame) {
  float rowAmp[MLX_H] = {0};
  float colDiff[MLX_W] = {0};
  bool colOk[MLX_W] = {false};
  float chessRow[MLX_H] = {0};
  float chessCol[MLX_W] = {0};
  float even[kMaxLine], odd[kMaxLine];

  // Top to bottom: the two components that alternate with y, one row at a time.
  int firstRow = -1, lastRow = -1;
  for (int y = 1; y < MLX_H - 1; y++) {
    int nEven = 0, nOdd = 0;
    for (int x = 0; x < MLX_W; x++) {
      const int i = y * MLX_W + x;
      const float c = frame[i], up = frame[i - MLX_W], down = frame[i + MLX_W];
      if (!plausible(c) || !plausible(up) || !plausible(down)) continue;
      const float v = (c - (up + down) * 0.5f) * alt(y);
      if (x & 1) {
        odd[nOdd++] = v;
      } else {
        even[nEven++] = v;
      }
    }
    if (nEven < 4 || nOdd < 4) continue;  // too little of this row to trust
    const float e = median(even, nEven), o = median(odd, nOdd);
    rowAmp[y] = (e + o) * 0.25f;
    chessRow[y] = (e - o) * 0.25f;
    if (firstRow < 0) firstRow = y;
    lastRow = y;
  }
  if (firstRow < 0) return;

  // Left to right. This pass yields the second difference of the column
  // offsets rather than the offsets themselves; solveColumnCycle inverts it.
  int firstCol = -1, lastCol = -1;
  for (int x = 1; x < MLX_W - 1; x++) {
    int nEven = 0, nOdd = 0;
    for (int y = 0; y < MLX_H; y++) {
      const int i = y * MLX_W + x;
      const float c = frame[i], left = frame[i - 1], right = frame[i + 1];
      if (!plausible(c) || !plausible(left) || !plausible(right)) continue;
      const float v = (c - (left + right) * 0.5f) * alt(x);
      if (y & 1) {
        odd[nOdd++] = v;
      } else {
        even[nEven++] = v;
      }
    }
    if (nEven < 4 || nOdd < 4) continue;
    const float e = median(even, nEven), o = median(odd, nOdd);
    colDiff[x] = (e + o) * 0.5f * alt(x);
    colOk[x] = true;
    chessCol[x] = (e - o) * 0.25f;
    if (firstCol < 0) firstCol = x;
    lastCol = x;
  }
  if (firstCol < 0) return;

  // Rejected before smoothing, so a spike is not first spread over its
  // neighbours and then measured as if it belonged to all of them. Only the
  // rows need this; the column groups are already medians over eight columns
  // spread across the array.
  rejectProfileOutliers(rowAmp, firstRow, lastRow, even);
  finishProfile(rowAmp, MLX_H, firstRow, lastRow);

  float quad[FPN_COLUMN_PERIOD];
  for (int r = 0; r < FPN_COLUMN_PERIOD; r++) {
    int n = 0;
    for (int x = 1; x < MLX_W - 1; x++) {
      if ((x % FPN_COLUMN_PERIOD) == r && colOk[x]) even[n++] = colDiff[x];
    }
    if (n < 3) return;  // too little of this group to tell it from the scene
    quad[r] = median(even, n);
  }
  float colCycle[FPN_COLUMN_PERIOD];
  solveColumnCycle(quad, colCycle);

  // The checkerboard is one offset between two readouts of the whole array, so
  // both passes are measuring the same number; take the middle of each and
  // average them.
  const float chess =
      (median(chessRow + firstRow, lastRow - firstRow + 1) + median(chessCol + firstCol, lastCol - firstCol + 1)) *
      0.5f;

  // The residual offsets are tenths of a degree. A larger estimate means the
  // scene fooled the estimator, not that the sensor drifted that far.
  for (int y = 0; y < MLX_H; y++) rowAmp[y] = clampf(rowAmp[y], -FPN_MAX_OFFSET_C, FPN_MAX_OFFSET_C);
  for (int r = 0; r < FPN_COLUMN_PERIOD; r++) colCycle[r] = clampf(colCycle[r], -FPN_MAX_OFFSET_C, FPN_MAX_OFFSET_C);
  const float chessClamped = clampf(chess, -FPN_MAX_OFFSET_C, FPN_MAX_OFFSET_C);

  // The pattern drifts with die temperature, so it has to be tracked;
  // re-measuring it from scratch every frame would chase noise instead.
  if (!gPrimed) {
    memcpy(gRowAmp, rowAmp, sizeof(gRowAmp));
    memcpy(gColCycle, colCycle, sizeof(gColCycle));
    gChessAmp = chessClamped;
    gPrimed = true;
  } else {
    for (int y = 0; y < MLX_H; y++) gRowAmp[y] += (rowAmp[y] - gRowAmp[y]) * FPN_ADAPT_RATE;
    for (int r = 0; r < FPN_COLUMN_PERIOD; r++) gColCycle[r] += (colCycle[r] - gColCycle[r]) * FPN_ADAPT_RATE;
    gChessAmp += (chessClamped - gChessAmp) * FPN_ADAPT_RATE;
  }

  for (int y = 0; y < MLX_H; y++) {
    for (int x = 0; x < MLX_W; x++) {
      const int i = y * MLX_W + x;
      if (!plausible(frame[i])) continue;
      frame[i] -= gRowAmp[y] * alt(y) + gColCycle[x % FPN_COLUMN_PERIOD] + gChessAmp * alt(x + y);
    }
  }
}
#endif  // FPN_FILTER

}  // namespace

void frameFilterApply(float *frame) {
#if REPAIR_DEAD_PIXELS
  repairOutliers(frame);
#endif
#if FPN_FILTER
  removeParityNoise(frame);
#endif
  (void)frame;
}

void frameFilterReset() {
  gPrimed = false;
  gChessAmp = 0;
  memset(gRowAmp, 0, sizeof(gRowAmp));
  memset(gColCycle, 0, sizeof(gColCycle));
}

void frameFilterGetStats(FrameFilterStats &out) {
  float sum = 0;
  for (int y = 0; y < MLX_H; y++) sum += fabsf(gRowAmp[y]);
  out.rowAmp = sum / MLX_H;
  for (int r = 0; r < FPN_COLUMN_PERIOD; r++) out.colCycle[r] = gColCycle[r];
  out.chess = gChessAmp;
  out.primed = gPrimed;
}
