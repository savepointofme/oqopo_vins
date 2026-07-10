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

static AdaptiveStrideInput nominal_parallax(AdaptiveStrideController &c,
                                            double t, double h,
                                            double raw_parallax_px = 2.5,
                                            double gyro = 0.03,
                                            double roll = 1.0,
                                            int tracks = 330,
                                            int slam = 50,
                                            int configured = 400) {
  auto in = nominal(t, h, gyro, roll, tracks, slam, configured);
  in.median_parallax_px = raw_parallax_px * std::max(1, c.recommended_stride());
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

static int settle_parallax(AdaptiveStrideController &c, double h, int n,
                           double raw_parallax_px = 2.5) {
  int last = c.recommended_stride();
  for (int i = 0; i < n; ++i) {
    auto d = c.update(nominal_parallax(c, 0.1 * i, h, raw_parallax_px));
    assert(std::abs(d.recommended_stride - last) <= 1);
    last = d.recommended_stride;
  }
  return last;
}

int main() {
  // ---- No parallax yet: 500 m stable cruise falls back to height stride 5. ----
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

  // ---- Parallax-primary cruise reaches the stride12 globalbaseline scale. ----
  {
    AdaptiveStrideController c;
    assert(settle_parallax(c, 300.0, 361) == 12);
  }

  // ---- Severe turn limits stride, but does not force full-rate by itself. ----
  {
    AdaptiveStrideController c;
    for (int i = 0; i <= 360; ++i)
      c.update(nominal_parallax(c, 0.1 * i, 500.0));
    int last = c.recommended_stride();
    assert(last == 12);
    for (int i = 361; i <= 400; ++i) {
      auto d = c.update(nominal_parallax(c, 0.1 * i, 500.0, 2.5, 0.30, 10.0));
      assert(!d.force_fullrate_tracking);
      assert(std::abs(d.recommended_stride - last) <= 1);
      last = d.recommended_stride;
    }
    assert(last == 6);
    // Recovery is intentionally slow: four quiet seconds plus upward holds.
    for (int i = 401; i <= 650; ++i)
      c.update(nominal_parallax(c, 0.1 * i, 500.0));
    assert(c.recommended_stride() == 12);
  }

  // ---- V2: 300 m stable cruise -> base stride 3 (NOT 4) ----
  {
    AdaptiveStrideController c;
    int s = settle(c, 300.0, 121);
    assert(s == 3);
    // Severe feature starvation forces full-rate tracking even without a
    // parallax packet; preserving tracks is more important than height stride.
    bool saw_fullrate = false;
    for (int i = 121; i <= 145; ++i)
      saw_fullrate = c.update(nominal(0.1 * i, 300.0, 0.03, 1.0, 80, 10)).force_fullrate_tracking || saw_fullrate;
    assert(saw_fullrate);
    assert(c.recommended_stride() == 1);
  }

  // ---- With parallax target stride12, severe feature starvation forces stride 1. ----
  {
    AdaptiveStrideController c;
    assert(settle_parallax(c, 300.0, 361) == 12);
    bool saw_fullrate = false;
    for (int i = 361; i <= 420; ++i)
      saw_fullrate = c.update(nominal_parallax(c, 0.1 * i, 300.0, 2.5, 0.03, 1.0, 80, 10)).force_fullrate_tracking || saw_fullrate;
    assert(saw_fullrate);
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

  // ---- V3: at 300 m, turn-only motion does not override the height fallback. ----
  {
    AdaptiveStrideController c;
    assert(settle(c, 300.0, 121) == 3);
    // Normal turn: gyro/roll above the enter gates but below the severe gates.
    for (int i = 121; i <= 140; ++i)
      c.update(nominal(0.1 * i, 300.0, 0.18, 7.0));
    assert(c.recommended_stride() == 3);
    // Escalate to a severe turn.
    for (int i = 141; i <= 160; ++i) {
      auto d = c.update(nominal(0.1 * i, 300.0, 0.30, 12.0));
      assert(!d.force_fullrate_tracking);
    }
    assert(c.recommended_stride() == 3);
    // Quiet cruise keeps the same fallback target.
    for (int i = 161; i <= 320; ++i)
      c.update(nominal(0.1 * i, 300.0));
    assert(c.recommended_stride() == 3);
  }

  // ---- Severe parallax forces stride 1 immediately instead of walking down
  //      one stride at a time while KLT is already under stress. ----
  {
    AdaptiveStrideController c;
    assert(settle_parallax(c, 300.0, 361) == 12);
    auto d = c.update(nominal_parallax(c, 36.2, 300.0, 80.0));
    assert(d.force_fullrate_tracking);
    assert(d.reason_parallax_high);
    assert(d.severe_parallax_high);
    assert(d.recommended_stride == 1);
    assert(d.switch_reason == "parallax_severe_fullrate");
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

  std::cout << "adaptive stride controller tests passed (V3 parallax primary)\n";
  return 0;
}
