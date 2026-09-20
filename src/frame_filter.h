// Cleans the MLX90640's own fixed-pattern noise out of a raw temperature frame.
#pragma once

// What the filter last measured, for the periodic status log. Seeing these
// settle to small, steady numbers is how you tell the correction is running and
// has locked on; all zeroes means it never ran.
struct FrameFilterStats {
  float rowAmp;       // mean |A(y)| of the odd/even row banding, degrees C
  float colCycle[4];  // the repeating four-column offsets, degrees C
  float chess;        // offset between the two subpages, degrees C
  bool primed;
};

// Repairs dead pixels and subtracts the sensor's fixed-pattern offsets, in
// place. Call once per frame, before anything else reads the temperatures.
void frameFilterApply(float *frame);

// Forgets the learned offsets. Call after the sensor is re-initialised or the
// refresh rate changes, because both shift the pattern.
void frameFilterReset();

void frameFilterGetStats(FrameFilterStats &out);
