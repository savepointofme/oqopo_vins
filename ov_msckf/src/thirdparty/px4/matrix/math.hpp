#pragma once

// OPENVINS COMPATIBILITY ADAPTATION:
// The upstream filter includes PX4 matrix/math.hpp so vector instantiations can
// use PX4 matrix types. OpenVINS instantiates the preserved NotchFilter only
// with scalar float gyro data, so no PX4 matrix type is required.
