#include "ros_free/AdaptiveStrideController.h"

#include <cassert>
#include <cmath>
#include <iostream>

using ov_msckf::AdaptiveStrideController;
using ov_msckf::AdaptiveStrideInput;

static AdaptiveStrideInput nominal(double t, double h, double gyro = 0.03,
                                   double roll = 1.0, int tracks = 330,
                                   int slam = 50, int configured = 400) {
  AdaptiveStrideInput in;
  in.timestamp = t;
  in.initialized = true;
  in.relative_height_m = h;
  in.horizontal_speed_mps = 40.0;
  in.roll_deg = roll;
  in.pitch_deg = 2.0;
  in.gyro_norm_radps = gyro;
  in.active_msckf_features = tracks;
  in.active_slam_features = slam;
  in.configured_feature_count = configured;
  return in;
}

// Drive a fresh controller through `n` quiet cruise frames at 10 Hz and return
// the converged recommendation.  Asserts every step changes by at most one.
static int settle(AdaptiveStrideController &c, double h, int n) {
  int last = c.recommended_stride();
  for (int i = 0; i < n; ++i) {
    auto d = c.update(nominal(0.1 * i, h));
    assert(std::abs(d.recommended_stride - last) <= 1);
    last = d.recommended_stride;
  }
  return last;
}

int main() {
  // ---- 500 m stable cruise -> stride 5, monotone single-step ramp ----
  {
    AdaptiveStrideController c;
    auto d = c.update(nominal(0.0, 500.0));
    assert(d.recommended_stride == 1);
    int last = d.recommended_stride;
    for (int i = 1; i <= 180; ++i) {
      d = c.update(nominal(0.1 * i, 500.0));
      assert(std::abs(d.recommended_stride - last) <= 1);
      last = d.recommended_stride;
    }
    assert(d.recommended_stride == 5);
  }

  // ---- severe turn forces stride 1; recovery is slow ----
  {
    AdaptiveStrideController c;
    for (int i = 0; i <= 180; ++i)
      c.update(nominal(0.1 * i, 500.0));
    int last = c.recommended_stride();
    for (int i = 181; i <= 210; ++i) {
      auto d = c.update(nominal(0.1 * i, 500.0, 0.30, 10.0));
      assert(std::abs(d.recommended_stride - last) <= 1);
      last = d.recommended_stride;
    }
    assert(last == 1);
    // Recovery is intentionally slow: four quiet seconds plus upward holds.
    for (int i = 211; i <= 240; ++i)
      c.update(nominal(0.1 * i, 500.0));
    assert(c.recommended_stride() <= 2);
  }

  // ---- V2: 300 m stable cruise -> base stride 3 (NOT 4) ----
  {
    AdaptiveStrideController c;
    int s = settle(c, 300.0, 121);
    assert(s == 3);
    // Severe feature starvation (80 tracks / 10 slam) collapses to stride 1.
    for (int i = 121; i <= 145; ++i)
      c.update(nominal(0.1 * i, 300.0, 0.03, 1.0, 80, 10));
    assert(c.recommended_stride() == 1);
  }

  // ---- V2: ~200 m cruise -> stride 2 (fly1/fly3 band) ----
  {
    AdaptiveStrideController c;
    assert(settle(c, 200.0, 121) == 2);
  }

  // ---- V2: 500 m -> stride 5 via settle() helper ----
  {
    AdaptiveStrideController c;
    assert(settle(c, 500.0, 181) == 5);
  }

  // ---- V2: at 300 m, a normal turn drops stride 3 -> 2, a severe turn -> 1 ----
  {
    AdaptiveStrideController c;
    assert(settle(c, 300.0, 121) == 3);
    // Normal turn: gyro/roll above the enter gates but below the severe gates.
    for (int i = 121; i <= 140; ++i)
      c.update(nominal(0.1 * i, 300.0, 0.18, 7.0));
    assert(c.recommended_stride() == 2);
    // Escalate to a severe turn -> stride 1.
    for (int i = 141; i <= 160; ++i)
      c.update(nominal(0.1 * i, 300.0, 0.30, 12.0));
    assert(c.recommended_stride() == 1);
    // Quiet cruise recovers slowly back toward the base (2 then 3): the turn
    // latch needs 4 quiet seconds to clear, then each up-step holds 3 s.
    for (int i = 161; i <= 320; ++i)
      c.update(nominal(0.1 * i, 300.0));
    assert(c.recommended_stride() == 3);
  }

  // ---- Height-boundary stability: dithering around the 2/3 edge must not
  //      produce rapid up/down reversals (hysteresis must hold). ----
  {
    AdaptiveStrideController c;
    assert(settle(c, 300.0, 121) == 3);
    int reversals = 0;
    int prev = c.recommended_stride();
    int prev_dir = 0;
    // Oscillate effective height across the 2->3 down edge (down at 205 m) and
    // 3->4 up edge: stay inside the stride-3 hysteresis band.
    for (int i = 121; i <= 320; ++i) {
      double h = (i % 2 == 0) ? 300.0 : 260.0;
      auto d = c.update(nominal(0.1 * i, h));
      assert(std::abs(d.recommended_stride - prev) <= 1);
      int dir = (d.recommended_stride > prev) - (d.recommended_stride < prev);
      if (dir != 0 && prev_dir != 0 && dir != prev_dir)
        reversals++;
      if (dir != 0)
        prev_dir = dir;
      prev = d.recommended_stride;
    }
    assert(c.recommended_stride() == 3);
    assert(reversals == 0);
  }

  std::cout << "adaptive stride controller tests passed (V2 height bands)\n";
  return 0;
}
