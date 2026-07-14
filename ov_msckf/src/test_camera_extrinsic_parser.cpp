/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "utils/opencv_yaml_parse.h"

#include <Eigen/Core>
#include <boost/filesystem.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

class TemporaryDirectory {
public:
  TemporaryDirectory()
      : path_(boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("openvins-extrinsic-%%%%-%%%%")) {
    if (!boost::filesystem::create_directories(path_))
      throw std::runtime_error("failed to create temporary directory");
  }

  ~TemporaryDirectory() {
    boost::system::error_code error;
    boost::filesystem::remove_all(path_, error);
  }

  const boost::filesystem::path &path() const { return path_; }

private:
  boost::filesystem::path path_;
};

void require_matrix_near(const Eigen::Matrix4d &actual, const Eigen::Matrix4d &expected, double tolerance,
                         const std::string &message) {
  const double max_error = (actual - expected).cwiseAbs().maxCoeff();
  if (max_error > tolerance) {
    std::ostringstream detail;
    detail << message << ": max_error=" << max_error << "\nactual:\n" << actual << "\nexpected:\n" << expected;
    throw std::runtime_error(detail.str());
  }
}

void write_main_config(const boost::filesystem::path &path, const std::string &external_filename) {
  std::ofstream output(path.string());
  if (!output.is_open())
    throw std::runtime_error("failed to create main YAML config");
  output << "%YAML:1.0\n";
  output << "relative_config_imucam: \"" << external_filename << "\"\n";
}

void write_camera_config(const boost::filesystem::path &path, const std::string &transform_name,
                         const Eigen::Matrix4d &transform) {
  std::ofstream output(path.string());
  if (!output.is_open())
    throw std::runtime_error("failed to create camera YAML config");
  output << "%YAML:1.0\n";
  output << "cam0:\n";
  output << "  " << transform_name << ":\n";
  output << std::setprecision(17);
  for (int row = 0; row < 4; ++row) {
    output << "    - [";
    for (int col = 0; col < 4; ++col) {
      if (col > 0)
        output << ", ";
      output << transform(row, col);
    }
    output << "]\n";
  }
}

} // namespace

int main() {
  try {
    TemporaryDirectory temp;
    const boost::filesystem::path direct_main = temp.path() / "direct_main.yaml";
    const boost::filesystem::path inverse_main = temp.path() / "inverse_main.yaml";
    const boost::filesystem::path direct_camera = temp.path() / "direct_camera.yaml";
    const boost::filesystem::path inverse_camera = temp.path() / "inverse_camera.yaml";

    Eigen::Matrix4d T_imu_cam;
    T_imu_cam << 0.6, -0.8, 0.0, 0.17,
                 0.8, 0.6, 0.0, -0.31,
                 0.0, 0.0, 1.0, 0.52,
                 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4d T_cam_imu;
    T_cam_imu << 0.6, 0.8, 0.0, 0.146,
                 -0.8, 0.6, 0.0, 0.322,
                 0.0, 0.0, 1.0, -0.52,
                 0.0, 0.0, 0.0, 1.0;

    write_main_config(direct_main, direct_camera.filename().string());
    write_main_config(inverse_main, inverse_camera.filename().string());
    write_camera_config(direct_camera, "T_imu_cam", T_imu_cam);
    write_camera_config(inverse_camera, "T_cam_imu", T_cam_imu);

    ov_core::YamlParser direct_parser(direct_main.generic_string());
    ov_core::YamlParser inverse_parser(inverse_main.generic_string());

    Eigen::Matrix4d direct_as_imu_cam = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d inverse_as_imu_cam = Eigen::Matrix4d::Zero();
    direct_parser.parse_external("relative_config_imucam", "cam0", "T_imu_cam", direct_as_imu_cam);
    inverse_parser.parse_external("relative_config_imucam", "cam0", "T_imu_cam", inverse_as_imu_cam);

    require_matrix_near(direct_as_imu_cam, T_imu_cam, 1e-12, "direct T_imu_cam parse changed the transform");
    require_matrix_near(inverse_as_imu_cam, T_imu_cam, 1e-12, "T_cam_imu fallback was not inverted");
    require_matrix_near(direct_as_imu_cam, inverse_as_imu_cam, 1e-12,
                        "direct and inverse-key T_imu_cam requests disagree");

    Eigen::Matrix4d inverse_as_cam_imu = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d direct_as_cam_imu = Eigen::Matrix4d::Zero();
    inverse_parser.parse_external("relative_config_imucam", "cam0", "T_cam_imu", inverse_as_cam_imu);
    direct_parser.parse_external("relative_config_imucam", "cam0", "T_cam_imu", direct_as_cam_imu);

    require_matrix_near(inverse_as_cam_imu, T_cam_imu, 1e-12, "direct T_cam_imu parse changed the transform");
    require_matrix_near(direct_as_cam_imu, T_cam_imu, 1e-12, "T_imu_cam fallback was not inverted");
    require_matrix_near(inverse_as_cam_imu, direct_as_cam_imu, 1e-12,
                        "direct and inverse-key T_cam_imu requests disagree");

    if (!direct_parser.successful() || !inverse_parser.successful())
      throw std::runtime_error("parser reported missing or invalid required parameters");
  } catch (const std::exception &error) {
    std::cerr << "test_camera_extrinsic_parser failed: " << error.what() << std::endl;
    return 1;
  }

  std::cout << "test_camera_extrinsic_parser passed" << std::endl;
  return 0;
}
