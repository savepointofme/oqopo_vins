#include "core/DynamicTurnRoiPolicy.h"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  using namespace ov_msckf;

  DynamicTurnRoiPolicy policy;
  double timestamp = 0.0;
  auto decision = policy.update(timestamp, 0.0, true);
  assert(decision.direction == TurnDirection::STRAIGHT);
  for (int i = 0; i < 40; ++i) {
    timestamp += 0.05;
    decision = policy.update(timestamp, -0.25, true);
  }
  assert(decision.direction == TurnDirection::LEFT);
  assert(decision.preferred_side == PreferredImageSide::RIGHT);
  assert(decision.strength > 0.95);
  assert(decision.preferred_fraction > 0.795);
  assert(decision.detection_exclusion_fraction > 0.344);

  const DynamicTurnRoiQuota balanced =
      DynamicTurnRoiPolicy::quota(40, 100, 100, 0.80);
  assert(balanced.preferred == 32 && balanced.other == 8);
  const DynamicTurnRoiQuota shortage =
      DynamicTurnRoiPolicy::quota(40, 10, 100, 0.80);
  assert(shortage.preferred == 10 && shortage.other == 30);
  const DynamicTurnRoiQuota mirror_shortage =
      DynamicTurnRoiPolicy::quota(40, 100, 5, 0.80);
  assert(mirror_shortage.preferred == 35 && mirror_shortage.other == 5);

  // Recovery is deliberately slower than entry, preventing side-flapping.
  for (int i = 0; i < 10; ++i) {
    timestamp += 0.05;
    decision = policy.update(timestamp, 0.0, true);
  }
  assert(decision.direction == TurnDirection::LEFT);
  for (int i = 0; i < 20; ++i) {
    timestamp += 0.05;
    decision = policy.update(timestamp, 0.0, true);
  }
  assert(decision.direction == TurnDirection::STRAIGHT);

  for (int i = 0; i < 40; ++i) {
    timestamp += 0.05;
    decision = policy.update(timestamp, 0.25, true);
  }
  assert(decision.direction == TurnDirection::RIGHT);
  assert(decision.preferred_side == PreferredImageSide::LEFT);

  const double accepted_timestamp = decision.timestamp;
  const auto duplicate = policy.update(accepted_timestamp, 0.25, true);
  assert(duplicate.timestamp == accepted_timestamp);
  assert(duplicate.reason == "non_monotonic_timestamp");

  std::cout << "dynamic turn ROI policy tests passed\n";
  return 0;
}
