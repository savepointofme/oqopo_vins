#include "core/p4/factors/Factor_P4ImuSharedBias.h"

#include <array>
#include <vector>

namespace ov_msckf {
namespace p4 {

Factor_P4ImuSharedBias::Factor_P4ImuSharedBias(
    double delta_time, Eigen::Vector3d &gravity, Eigen::Vector3d &alpha,
    Eigen::Vector3d &beta, Eigen::Vector4d &q_k_to_k1,
    Eigen::Vector3d &ba_linearization, Eigen::Vector3d &bg_linearization,
    Eigen::Matrix3d &J_q, Eigen::Matrix3d &J_beta,
    Eigen::Matrix3d &J_alpha, Eigen::Matrix3d &H_beta,
    Eigen::Matrix3d &H_alpha,
    Eigen::Matrix<double, 15, 15> &covariance)
    : base_factor_(delta_time, gravity, alpha, beta, q_k_to_k1,
                   ba_linearization, bg_linearization, J_q, J_beta, J_alpha,
                   H_beta, H_alpha, covariance) {
  set_num_residuals(15);
  mutable_parameter_block_sizes()->push_back(4); // q_k
  mutable_parameter_block_sizes()->push_back(3); // shared bg
  mutable_parameter_block_sizes()->push_back(3); // v_k
  mutable_parameter_block_sizes()->push_back(3); // shared ba
  mutable_parameter_block_sizes()->push_back(3); // p_k
  mutable_parameter_block_sizes()->push_back(4); // q_k1
  mutable_parameter_block_sizes()->push_back(3); // v_k1
  mutable_parameter_block_sizes()->push_back(3); // p_k1
}

bool Factor_P4ImuSharedBias::Evaluate(double const *const *parameters,
                                      double *residuals,
                                      double **jacobians) const {
  const double *base_parameters[10] = {
      parameters[0], parameters[1], parameters[2], parameters[3],
      parameters[4], parameters[5], parameters[1], parameters[6],
      parameters[3], parameters[7]};

  if (jacobians == nullptr)
    return base_factor_.Evaluate(base_parameters, residuals, nullptr);

  constexpr std::array<int, 10> kBaseBlockSizes = {4, 3, 3, 3, 3,
                                                   4, 3, 3, 3, 3};
  std::array<std::vector<double>, 10> storage;
  std::array<double *, 10> base_jacobians{};
  for (std::size_t index = 0; index < storage.size(); ++index) {
    storage[index].resize(15 * kBaseBlockSizes[index], 0.0);
    base_jacobians[index] = storage[index].data();
  }
  if (!base_factor_.Evaluate(base_parameters, residuals,
                             base_jacobians.data()))
    return false;

  const auto copy_block = [&](int output_index, int base_index,
                              int columns) {
    if (jacobians[output_index] == nullptr)
      return;
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                             Eigen::RowMajor>>
        output(jacobians[output_index], 15, columns);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                   Eigen::RowMajor>>
        input(storage[base_index].data(), 15, columns);
    output = input;
  };
  const auto sum_blocks = [&](int output_index, int first_base_index,
                              int second_base_index) {
    if (jacobians[output_index] == nullptr)
      return;
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> output(
        jacobians[output_index]);
    Eigen::Map<const Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> first(
        storage[first_base_index].data());
    Eigen::Map<const Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> second(
        storage[second_base_index].data());
    output = first + second;
  };

  copy_block(0, 0, 4);
  sum_blocks(1, 1, 6);
  copy_block(2, 2, 3);
  sum_blocks(3, 3, 8);
  copy_block(4, 4, 3);
  copy_block(5, 5, 4);
  copy_block(6, 7, 3);
  copy_block(7, 9, 3);
  return true;
}

} // namespace p4
} // namespace ov_msckf
