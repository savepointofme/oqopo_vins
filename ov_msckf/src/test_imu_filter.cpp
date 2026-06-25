#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/ImuFilter.h"
#include "thirdparty/px4/Px4NotchFilter.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

using ov_msckf::ImuFilter;
using ov_msckf::ImuFilterOptions;
using Px4LowPass = px4::math::LowPassFilter2p<float>;
using Px4Notch = px4::math::NotchFilter<float>;

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr const char *kNotchSha256 = "b7e2d2143bfdfe371546b5393bd125f25a4141c30d37d92d1f8fafd06dfa8464";

class InspectableLowPass : public Px4LowPass {
public:
  using Px4LowPass::LowPassFilter2p;
  float delay1() const { return _delay_element_1; }
  float delay2() const { return _delay_element_2; }
  float a1() const { return _a1; }
  float a2() const { return _a2; }
  float b0() const { return _b0; }
  float b1() const { return _b1; }
  float b2() const { return _b2; }
};

class InspectableNotch : public Px4Notch {
public:
  float delay1() const { return _delay_element_1; }
  float delay2() const { return _delay_element_2; }
  float out1() const { return _delay_element_output_1; }
  float out2() const { return _delay_element_output_2; }
  float a1() const { return _a1; }
  float a2() const { return _a2; }
  float b0() const { return _b0; }
  float b1() const { return _b1; }
  float b2() const { return _b2; }
  float sample_freq() const { return _sample_freq; }
};

static_assert(std::is_same_v<decltype(std::declval<const Px4LowPass &>().getMagnitudeResponse(0.f)), float>);
static_assert(std::is_same_v<decltype(std::declval<const Px4Notch &>().getMagnitudeResponse(1.f)), float>);

bool same_bits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }
bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }
bool close(float a, float b, float tolerance = 2e-5f) { return std::fabs(a - b) <= tolerance; }
bool close(double a, double b, double tolerance = 1e-6) { return std::abs(a - b) <= tolerance; }

ov_core::ImuData sample(double t, double gyro, double accel = 9.81) {
  ov_core::ImuData value;
  value.timestamp = t;
  value.wm = Eigen::Vector3d(gyro, -2.0 * gyro, 0.5 * gyro);
  value.am = Eigen::Vector3d(accel, -0.25 * accel, 0.1 * accel);
  return value;
}

void assert_same_imu_bits(const ov_core::ImuData &a, const ov_core::ImuData &b) {
  assert(same_bits(a.timestamp, b.timestamp));
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    assert(same_bits(a.wm(axis), b.wm(axis)));
    assert(same_bits(a.am(axis), b.am(axis)));
  }
}

std::vector<uint8_t> read_file(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("unable to read " + path.string());
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32U - n)); }

std::string sha256_hex(const std::vector<uint8_t> &data) {
  static const uint32_t k[64] = {
      0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
      0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
      0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
      0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
      0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
      0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
      0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
      0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
  std::vector<uint8_t> msg = data;
  const uint64_t bit_len = static_cast<uint64_t>(msg.size()) * 8U;
  msg.push_back(0x80U);
  while ((msg.size() % 64U) != 56U)
    msg.push_back(0U);
  for (int i = 7; i >= 0; --i)
    msg.push_back(static_cast<uint8_t>((bit_len >> (8U * i)) & 0xffU));

  uint32_t h[8] = {0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                   0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
  for (size_t off = 0; off < msg.size(); off += 64) {
    uint32_t w[64] = {};
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<uint32_t>(msg[off + 4 * i]) << 24U) |
             (static_cast<uint32_t>(msg[off + 4 * i + 1]) << 16U) |
             (static_cast<uint32_t>(msg[off + 4 * i + 2]) << 8U) |
             static_cast<uint32_t>(msg[off + 4 * i + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3U);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10U);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ ((~e) & g);
      const uint32_t temp1 = hh + s1 + ch + k[i] + w[i];
      const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp2 = s0 + maj;
      hh = g; g = f; f = e; e = d + temp1; d = c; c = b; b = a; a = temp1 + temp2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (uint32_t x : h)
    out << std::setw(8) << x;
  return out.str();
}

std::filesystem::path source_dir() {
  return std::filesystem::path(__FILE__).parent_path();
}

void verify_px4_notch_source() {
  const auto px4_dir = source_dir() / "thirdparty" / "px4";
  const auto vendored = read_file(px4_dir / "NotchFilter.hpp");
  const auto upstream = read_file(px4_dir / "upstream_3042f906abaab7ab59ae838ad5a530a9ef3df9a6" / "NotchFilter.hpp");
  assert(vendored == upstream);
  assert(sha256_hex(vendored) == kNotchSha256);
}

std::vector<float> make_signal_suite() {
  std::vector<float> values;
  values.reserve(6000);
  values.insert(values.end(), 256, 0.25f);
  values.insert(values.end(), 256, 1.25f);
  for (int i = 0; i < 1024; ++i)
    values.push_back(std::sin(2.f * kPi * 7.f * static_cast<float>(i) / 200.f));
  for (int i = 0; i < 2048; ++i) {
    const float t = static_cast<float>(i) / 200.f;
    values.push_back(0.5f * std::sin(2.f * kPi * 5.f * t) + 0.2f * std::sin(2.f * kPi * 40.f * t) +
                     0.1f * std::cos(2.f * kPi * 83.f * t));
  }
  values.insert(values.end(), 64, 0.f);
  values.push_back(3.f);
  values.insert(values.end(), 64, 0.f);
  std::mt19937 generator(0x50483432u);
  std::uniform_real_distribution<float> distribution(-2.f, 2.f);
  for (int i = 0; i < 1024; ++i)
    values.push_back(distribution(generator));
  return values;
}

double rms_after(std::vector<float> values, size_t skip) {
  double acc = 0.0;
  size_t n = 0;
  for (size_t i = skip; i < values.size(); ++i) {
    acc += static_cast<double>(values[i]) * values[i];
    n++;
  }
  return std::sqrt(acc / static_cast<double>(n));
}

std::vector<float> sine(float frequency_hz, float sample_rate_hz, int count) {
  std::vector<float> values;
  values.reserve(count);
  for (int i = 0; i < count; ++i)
    values.push_back(std::sin(2.f * kPi * frequency_hz * static_cast<float>(i) / sample_rate_hz));
  return values;
}

void verify_lowpass_official_paths(const std::vector<float> &input) {
  InspectableLowPass constructed(200.f, 25.f);
  InspectableLowPass configured;
  configured.set_cutoff_frequency(200.f, 25.f);
  assert(same_bits(constructed.a1(), configured.a1()));
  assert(same_bits(constructed.a2(), configured.a2()));
  assert(same_bits(constructed.b0(), configured.b0()));
  assert(same_bits(constructed.b1(), configured.b1()));
  assert(same_bits(constructed.b2(), configured.b2()));
  for (float value : input) {
    const float a = constructed.apply(value);
    const float b = configured.apply(value);
    assert(same_bits(a, b));
    assert(same_bits(constructed.delay1(), configured.delay1()));
    assert(same_bits(constructed.delay2(), configured.delay2()));
  }
  InspectableLowPass array_filter(200.f, 25.f);
  InspectableLowPass scalar_filter(200.f, 25.f);
  std::vector<float> array_output = input;
  std::vector<float> scalar_output;
  for (float value : input)
    scalar_output.push_back(scalar_filter.apply(value));
  array_filter.applyArray(array_output.data(), static_cast<int>(array_output.size()));
  for (size_t i = 0; i < input.size(); ++i)
    assert(same_bits(array_output[i], scalar_output[i]));
}

void verify_notch_official_paths(const std::vector<float> &input) {
  InspectableNotch configured;
  assert(configured.setParameters(200.f, 40.f, 8.f));
  InspectableNotch array_filter;
  InspectableNotch scalar_filter;
  assert(array_filter.setParameters(200.f, 40.f, 8.f));
  assert(scalar_filter.setParameters(200.f, 40.f, 8.f));
  std::vector<float> array_output = input;
  std::vector<float> scalar_output;
  for (float value : input)
    scalar_output.push_back(scalar_filter.apply(value));
  array_filter.applyArray(array_output.data(), static_cast<int>(array_output.size()));
  for (size_t i = 0; i < input.size(); ++i)
    assert(same_bits(array_output[i], scalar_output[i]));
  assert(same_bits(array_filter.delay1(), scalar_filter.delay1()));
  assert(same_bits(array_filter.out1(), scalar_filter.out1()));
}

void verify_notch_frequency_behavior() {
  auto center = sine(40.f, 200.f, 4000);
  auto away = sine(12.f, 200.f, 4000);
  InspectableNotch notch_center;
  InspectableNotch notch_away;
  assert(notch_center.setParameters(200.f, 40.f, 8.f));
  assert(notch_away.setParameters(200.f, 40.f, 8.f));
  for (float &v : center)
    v = notch_center.apply(v);
  for (float &v : away)
    v = notch_away.apply(v);
  assert(rms_after(center, 1000) < 0.08);
  assert(rms_after(away, 1000) > 0.60);
  const float exact_center_response = notch_center.getMagnitudeResponse(40.f);
  assert(!std::isfinite(exact_center_response) || exact_center_response < 0.02f);
  assert(notch_center.getMagnitudeResponse(12.f) > 0.90f);
}

void verify_px4_reset_disable_invalid() {
  InspectableNotch notch;
  assert(notch.setParameters(200.f, 40.f, 8.f));
  notch.apply(1.f);
  notch.reset();
  assert(!notch.initialized());
  notch.reset(std::numeric_limits<float>::quiet_NaN());
  assert(notch.initialized());
  assert(same_bits(notch.delay1(), 0.f));
  assert(same_bits(notch.delay2(), 0.f));
  notch.disable();
  assert(same_bits(notch.getNotchFreq(), 0.f));
  assert(same_bits(notch.getBandwidth(), 0.f));
  assert(same_bits(notch.sample_freq(), 0.f));
  assert(same_bits(notch.apply(-1.75f), -1.75f));

  const std::vector<std::array<float, 3>> invalid = {{
      {0.f, 40.f, 8.f}, {200.f, 0.f, 8.f}, {200.f, -1.f, 8.f}, {200.f, 100.f, 8.f},
      {200.f, 40.f, 0.f}, {std::numeric_limits<float>::quiet_NaN(), 40.f, 8.f},
      {200.f, std::numeric_limits<float>::infinity(), 8.f}, {200.f, 40.f, std::numeric_limits<float>::quiet_NaN()}}};
  for (const auto &p : invalid) {
    InspectableNotch f;
    assert(!f.setParameters(p[0], p[1], p[2]));
    assert(same_bits(f.apply(0.625f), 0.625f));
  }

  InspectableNotch runtime;
  assert(runtime.setParameters(200.f, 40.f, 8.f));
  for (int i = 0; i < 30; ++i)
    runtime.apply(0.5f);
  assert(runtime.initialized());
  assert(runtime.setParameters(200.f, 42.f, 8.f));
  assert(runtime.initialized());
  assert(runtime.setParameters(200.f, 70.f, 8.f));
  assert(!runtime.initialized());
  assert(runtime.setParameters(200.f, 70.f, 10.f));
  assert(!runtime.initialized());
}

void verify_openvins_wrapper_paths() {
  ImuFilterOptions missing_node_defaults;
  ImuFilterOptions explicit_false;
  explicit_false.enabled = false;
  ImuFilterOptions all_child_off;
  all_child_off.enabled = true;
  ImuFilter missing_node_filter(missing_node_defaults);
  ImuFilter explicit_false_filter(explicit_false);
  ImuFilter all_child_off_filter(all_child_off);
  for (int i = 0; i < 32; ++i) {
    const auto raw = sample(12.5 + 0.005 * i, 0.73 + 0.01 * i, 8.9 - 0.02 * i);
    assert_same_imu_bits(missing_node_filter.process(raw), raw);
    assert_same_imu_bits(explicit_false_filter.process(raw), raw);
    assert_same_imu_bits(all_child_off_filter.process(raw), raw);
  }

  ImuFilterOptions options;
  options.enabled = true;
  options.sample_rate_hz = 200.0;
  options.gyro_notch0 = {true, 40.0, 8.0};
  options.gyro_notch1 = {true, 72.0, 6.0};
  options.gyro_lowpass = {true, 25.0};
  ImuFilter filter(options);
  Px4Notch nf0[3];
  Px4Notch nf1[3];
  Px4LowPass lp[3];
  for (int axis = 0; axis < 3; ++axis) {
    assert(nf0[axis].setParameters(200.f, 40.f, 8.f));
    assert(nf1[axis].setParameters(200.f, 72.f, 6.f));
    lp[axis].set_cutoff_frequency(200.f, 25.f);
  }
  for (int i = 0; i < 200; ++i) {
    const double t = 1.0 + 0.005 * i;
    const double g = 0.4 * std::sin(2.0 * M_PI * 12.0 * i / 200.0) +
                     0.3 * std::sin(2.0 * M_PI * 40.0 * i / 200.0) +
                     0.2 * std::sin(2.0 * M_PI * 72.0 * i / 200.0);
    auto raw = sample(t, g);
    auto out = filter.process(raw);
    Eigen::Vector3d expected = raw.wm;
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      expected(axis) = nf0[axis].apply(static_cast<float>(expected(axis)));
      expected(axis) = nf1[axis].apply(static_cast<float>(expected(axis)));
      expected(axis) = (i == 0) ? lp[axis].reset(static_cast<float>(expected(axis)))
                                : lp[axis].apply(static_cast<float>(expected(axis)));
      assert(same_bits(static_cast<float>(out.wm(axis)), static_cast<float>(expected(axis))));
      assert(same_bits(out.am(axis), raw.am(axis)));
    }
    assert(same_bits(out.timestamp, raw.timestamp));
  }
  assert(filter.stats().coefficient_update_count == 1);
  assert(filter.stats().reset_count == 1);

  auto bad = options;
  bad.gyro_notch0.frequency_hz = 100.0;
  ImuFilter invalid_filter(bad);
  const auto raw = sample(0.0, 3.0);
  const auto passed = invalid_filter.process(raw);
  assert(passed.wm.allFinite());
  assert(invalid_filter.stats().disable_count >= 3);

  auto timestamped = options;
  timestamped.sample_rate_hz = 0.0;
  ImuFilter timestamp_filter(timestamped);
  assert_same_imu_bits(timestamp_filter.process(sample(1.000, 0.2)), sample(1.000, 0.2));
  assert(close(timestamp_filter.process(sample(1.005, 0.4)).timestamp, 1.005, 1e-15));
  assert(close(timestamp_filter.stats().active_sample_rate_hz, 200.0, 1e-9));

  ImuFilter anomaly_filter(options);
  assert(close(anomaly_filter.process(sample(0.0, 1.0)).wm.x(), 1.0));
  anomaly_filter.process(sample(0.005, 2.0));
  assert(close(anomaly_filter.process(sample(0.005, 3.0)).wm.x(), 3.0));
  assert(close(anomaly_filter.process(sample(0.004, 4.0)).wm.x(), 4.0));
  assert(close(anomaly_filter.process(sample(0.200, 5.0)).wm.x(), 5.0));
  assert(anomaly_filter.stats().timestamp_duplicate_count == 1);
  assert(anomaly_filter.stats().timestamp_backward_count == 1);
  assert(anomaly_filter.stats().timestamp_gap_count == 1);

  ImuFilter finite_filter(options);
  finite_filter.process(sample(0.0, 1.0));
  auto nonfinite = sample(0.005, 2.0);
  nonfinite.wm.x() = std::numeric_limits<double>::quiet_NaN();
  const auto finite = finite_filter.process(nonfinite);
  assert(finite.wm.allFinite());
  assert(same_bits(static_cast<float>(finite.wm.x()), 0.f));
}

void verify_cascade_effect() {
  auto values = sine(40.f, 200.f, 4000);
  auto second = sine(72.f, 200.f, 4000);
  for (size_t i = 0; i < values.size(); ++i)
    values[i] += 0.75f * second[i];
  Px4Notch nf0;
  Px4Notch nf1;
  assert(nf0.setParameters(200.f, 40.f, 8.f));
  assert(nf1.setParameters(200.f, 72.f, 6.f));
  for (float &v : values)
    v = nf1.apply(nf0.apply(v));
  assert(rms_after(values, 1000) < 0.15);
}

} // namespace

int main() {
  verify_px4_notch_source();
  const auto suite = make_signal_suite();
  verify_lowpass_official_paths(suite);
  verify_notch_official_paths(suite);
  verify_notch_frequency_behavior();
  verify_px4_reset_disable_invalid();
  verify_cascade_effect();
  verify_openvins_wrapper_paths();
  std::cout << "PX4 NotchFilter/LowPassFilter2p and OpenVINS IMU filter-chain tests passed\n";
  return 0;
}
