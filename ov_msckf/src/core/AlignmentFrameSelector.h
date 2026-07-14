/* Bounded information-oriented frame selection for online alignment graphs. */

#ifndef OV_MSCKF_ALIGNMENT_FRAME_SELECTOR_H
#define OV_MSCKF_ALIGNMENT_FRAME_SELECTOR_H

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>

namespace ov_msckf {

struct AlignmentFrameSelectorConfig {
  double minimum_interval_s = 0.10;
  double maximum_interval_s = 0.60;
  double target_compensated_parallax_px = 3.0;
  double minimum_survival_ratio = 0.45;
  int minimum_common_tracks = 20;
  int maximum_selected_frames = 36;
};

struct AlignmentFrameSelectionInput {
  double timestamp = -1.0;
  double compensated_parallax_px = 0.0;
  double baseline_proxy = 0.0;
  double survival_ratio = 0.0;
  double excitation_score = 0.0;
  int common_tracks = 0;
  bool force_tail = false;
  bool high_angular_rate = false;
};

struct AlignmentFrameSelectionDecision {
  bool selected = false;
  bool replace_tail = false;
  double information_score = 0.0;
  std::string reason;
};

class AlignmentFrameSelector {
public:
  explicit AlignmentFrameSelector(
      const AlignmentFrameSelectorConfig &config = AlignmentFrameSelectorConfig())
      : config_(config) {}

  AlignmentFrameSelectionDecision
  evaluate(const AlignmentFrameSelectionInput &input) {
    AlignmentFrameSelectionDecision out;
    if (!std::isfinite(input.timestamp)) {
      out.reason = "timestamp_invalid";
      return out;
    }
    if (input.high_angular_rate) {
      out.reason = "high_angular_rate";
      return out;
    }
    if (last_selected_time_ < 0.0) {
      out.selected = true;
      out.reason = "first_supported_frame";
    } else {
      const double dt = input.timestamp - last_selected_time_;
      out.information_score =
          std::max(0.0, input.compensated_parallax_px) +
          0.5 * std::max(0.0, input.baseline_proxy) +
          0.25 * std::max(0.0, input.excitation_score);
      const bool healthy_tracks =
          input.common_tracks >= config_.minimum_common_tracks &&
          input.survival_ratio >= config_.minimum_survival_ratio;
      const bool enough_motion =
          input.compensated_parallax_px >=
          config_.target_compensated_parallax_px;
      const bool maximum_gap = dt >= config_.maximum_interval_s;
      const bool ordinary = dt >= config_.minimum_interval_s && healthy_tracks &&
                            enough_motion;
      out.selected = input.force_tail || maximum_gap || ordinary;
      out.reason = input.force_tail
                       ? "window_tail"
                       : maximum_gap ? "maximum_interval"
                                     : ordinary ? "information_gain"
                                                : "insufficient_increment";
    }
    if (out.selected) {
      last_selected_time_ = input.timestamp;
      selected_times_.push_back(input.timestamp);
      while (selected_times_.size() >
             static_cast<size_t>(std::max(2, config_.maximum_selected_frames)))
        selected_times_.pop_front();
    }
    return out;
  }

  void prune_before(double timestamp) {
    while (!selected_times_.empty() && selected_times_.front() < timestamp)
      selected_times_.pop_front();
    if (selected_times_.empty())
      last_selected_time_ = -1.0;
  }

  void reset() {
    selected_times_.clear();
    last_selected_time_ = -1.0;
  }

  size_t selected_count() const { return selected_times_.size(); }

private:
  AlignmentFrameSelectorConfig config_;
  std::deque<double> selected_times_;
  double last_selected_time_ = -1.0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ALIGNMENT_FRAME_SELECTOR_H
