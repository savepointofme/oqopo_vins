/*
 * Diagnostic-only visual residual CSV writer.
 *
 * This file intentionally has no estimator-side decisions: it only records
 * residual and chi2 values that the visual updaters already computed.
 */

#ifndef OV_MSCKF_VISUAL_RESIDUAL_DIAG_H
#define OV_MSCKF_VISUAL_RESIDUAL_DIAG_H

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "utils/colors.h"
#include "utils/print.h"

namespace ov_msckf {

class VisualResidualDiag {
public:
  struct Context {
    double yaw_rate = std::numeric_limits<double>::quiet_NaN();
    double gpsz_residual = std::numeric_limits<double>::quiet_NaN();
    std::string gpsz_status;
    double course_error = std::numeric_limits<double>::quiet_NaN();
    double cross_error = std::numeric_limits<double>::quiet_NaN();
  };

  struct FeatureRow {
    double time = std::numeric_limits<double>::quiet_NaN();
    std::string update_type;
    size_t feature_id = 0;
    std::string feature_kind;
    int track_age = 0;
    int num_measurements = 0;
    double u_mean = std::numeric_limits<double>::quiet_NaN();
    double v_mean = std::numeric_limits<double>::quiet_NaN();
    std::string image_side = "unknown";
    std::string image_quadrant = "unknown";
    double residual_norm = std::numeric_limits<double>::quiet_NaN();
    double whitened_residual_norm = std::numeric_limits<double>::quiet_NaN();
    double chi2 = std::numeric_limits<double>::quiet_NaN();
    double chi2_threshold = std::numeric_limits<double>::quiet_NaN();
    bool accepted_by_chi2 = false;
    bool rejected_by_chi2 = false;
    std::string reject_reason;
    double measurement_sigma_px = std::numeric_limits<double>::quiet_NaN();
    double roll = std::numeric_limits<double>::quiet_NaN();
    double bg_z = std::numeric_limits<double>::quiet_NaN();
  };

  struct FrameSummary {
    double time = std::numeric_limits<double>::quiet_NaN();
    std::string update_type;
    int num_features = 0;
    int num_accepted = 0;
    int num_rejected = 0;
    double mean_residual_norm = std::numeric_limits<double>::quiet_NaN();
    double p95_residual_norm = std::numeric_limits<double>::quiet_NaN();
    double mean_whitened_residual_norm = std::numeric_limits<double>::quiet_NaN();
    double p95_whitened_residual_norm = std::numeric_limits<double>::quiet_NaN();
    double mean_chi2 = std::numeric_limits<double>::quiet_NaN();
    double p95_chi2 = std::numeric_limits<double>::quiet_NaN();
    double chi2_threshold = std::numeric_limits<double>::quiet_NaN();
    double accepted_ratio = std::numeric_limits<double>::quiet_NaN();
    int high_residual_accepted_count = 0;
    double high_residual_accepted_ratio = std::numeric_limits<double>::quiet_NaN();
    double u_mean = std::numeric_limits<double>::quiet_NaN();
    double left_frac = std::numeric_limits<double>::quiet_NaN();
    double roll = std::numeric_limits<double>::quiet_NaN();
  };

  void set_window(double t0, double t1) {
    t0_ = t0;
    t1_ = t1;
  }

  void set_context(const Context &ctx) { context_ = ctx; }

  void open(const std::string &feature_path, const std::string &summary_path) {
    if (feature_csv_.is_open())
      feature_csv_.close();
    if (summary_csv_.is_open())
      summary_csv_.close();

    if (!feature_path.empty()) {
      boost::filesystem::path p(feature_path);
      if (!p.parent_path().empty())
        boost::filesystem::create_directories(p.parent_path());
      feature_csv_.open(feature_path, std::ofstream::out | std::ofstream::trunc);
      if (!feature_csv_.is_open()) {
        PRINT_WARNING(YELLOW "[VISUAL-RESIDUAL-DIAG] failed to open %s\n" RESET, feature_path.c_str());
      } else {
        feature_csv_
            << "time,update_type,feature_id,feature_kind_msckf_or_slam,"
            << "track_age,num_measurements,u_mean,v_mean,image_side,image_quadrant,"
            << "residual_norm,whitened_residual_norm,chi2,chi2_threshold,"
            << "accepted_by_chi2,rejected_by_chi2,reject_reason,measurement_sigma_px,"
            << "roll,yaw_rate,bg_z,gpsz_residual_nearest,gpsz_status_nearest,"
            << "course_error_nearest,cross_error_nearest\n";
        feature_csv_.flush();
        PRINT_INFO(GREEN "[VISUAL-RESIDUAL-DIAG] writing feature rows %s window=[%.3f, %.3f]\n" RESET,
                   feature_path.c_str(), t0_, t1_);
      }
    }

    if (!summary_path.empty()) {
      boost::filesystem::path p(summary_path);
      if (!p.parent_path().empty())
        boost::filesystem::create_directories(p.parent_path());
      summary_csv_.open(summary_path, std::ofstream::out | std::ofstream::trunc);
      if (!summary_csv_.is_open()) {
        PRINT_WARNING(YELLOW "[VISUAL-RESIDUAL-DIAG] failed to open %s\n" RESET, summary_path.c_str());
      } else {
        summary_csv_
            << "time,update_type,num_features,num_accepted,num_rejected,"
            << "mean_residual_norm,p95_residual_norm,"
            << "mean_whitened_residual_norm,p95_whitened_residual_norm,"
            << "mean_chi2,p95_chi2,chi2_threshold,accepted_ratio,"
            << "high_residual_accepted_count,high_residual_accepted_ratio,"
            << "u_mean,left_frac,roll,yaw_rate,gpsz_residual_nearest,"
            << "course_error_nearest,cross_error_nearest\n";
        summary_csv_.flush();
        PRINT_INFO(GREEN "[VISUAL-RESIDUAL-DIAG] writing frame summary %s window=[%.3f, %.3f]\n" RESET,
                   summary_path.c_str(), t0_, t1_);
      }
    }
  }

  bool enabled(double time) const {
    return time >= t0_ && time <= t1_ &&
           (feature_csv_.is_open() || summary_csv_.is_open());
  }

  bool feature_enabled(double time) const {
    return time >= t0_ && time <= t1_ && feature_csv_.is_open();
  }

  bool summary_enabled(double time) const {
    return time >= t0_ && time <= t1_ && summary_csv_.is_open();
  }

  void log_feature(const FeatureRow &r) {
    if (!feature_enabled(r.time))
      return;
    feature_csv_ << std::fixed << std::setprecision(9)
                 << r.time << "," << r.update_type << "," << r.feature_id << ","
                 << r.feature_kind << "," << r.track_age << "," << r.num_measurements << ","
                 << std::setprecision(6)
                 << r.u_mean << "," << r.v_mean << "," << r.image_side << ","
                 << r.image_quadrant << "," << r.residual_norm << ","
                 << r.whitened_residual_norm << "," << r.chi2 << ","
                 << r.chi2_threshold << "," << (r.accepted_by_chi2 ? 1 : 0) << ","
                 << (r.rejected_by_chi2 ? 1 : 0) << "," << r.reject_reason << ","
                 << r.measurement_sigma_px << "," << r.roll << ","
                 << context_.yaw_rate << "," << r.bg_z << ","
                 << context_.gpsz_residual << "," << context_.gpsz_status << ","
                 << context_.course_error << "," << context_.cross_error << "\n";
  }

  void log_summary(const FrameSummary &s) {
    latest_summary_ = s;
    have_latest_summary_ = true;
    if (!summary_enabled(s.time))
      return;
    summary_csv_ << std::fixed << std::setprecision(9)
                 << s.time << "," << s.update_type << ","
                 << s.num_features << "," << s.num_accepted << "," << s.num_rejected << ","
                 << std::setprecision(6)
                 << s.mean_residual_norm << "," << s.p95_residual_norm << ","
                 << s.mean_whitened_residual_norm << "," << s.p95_whitened_residual_norm << ","
                 << s.mean_chi2 << "," << s.p95_chi2 << "," << s.chi2_threshold << ","
                 << s.accepted_ratio << "," << s.high_residual_accepted_count << ","
                 << s.high_residual_accepted_ratio << "," << s.u_mean << "," << s.left_frac << ","
                 << s.roll << "," << context_.yaw_rate << "," << context_.gpsz_residual << ","
                 << context_.course_error << "," << context_.cross_error << "\n";
  }

  bool get_latest_summary(FrameSummary &summary) const {
    if (!have_latest_summary_)
      return false;
    summary = latest_summary_;
    return true;
  }

  static double mean(const std::vector<double> &values) {
    std::vector<double> finite;
    finite.reserve(values.size());
    for (double v : values) {
      if (std::isfinite(v))
        finite.push_back(v);
    }
    if (finite.empty())
      return std::numeric_limits<double>::quiet_NaN();
    return std::accumulate(finite.begin(), finite.end(), 0.0) / (double)finite.size();
  }

  static double percentile(std::vector<double> values, double q) {
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double v) { return !std::isfinite(v); }),
                 values.end());
    if (values.empty())
      return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const double idx = std::max(0.0, std::min(1.0, q)) * (double)(values.size() - 1);
    const size_t lo = (size_t)std::floor(idx);
    const size_t hi = std::min(values.size() - 1, lo + 1);
    const double a = idx - (double)lo;
    return (1.0 - a) * values[lo] + a * values[hi];
  }

private:
  std::ofstream feature_csv_;
  std::ofstream summary_csv_;
  double t0_ = -std::numeric_limits<double>::infinity();
  double t1_ = std::numeric_limits<double>::infinity();
  Context context_;
  FrameSummary latest_summary_;
  bool have_latest_summary_ = false;
};

} // namespace ov_msckf

#endif // OV_MSCKF_VISUAL_RESIDUAL_DIAG_H
