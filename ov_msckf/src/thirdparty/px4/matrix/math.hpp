#pragma once

// OPENVINS COMPATIBILITY ADAPTATION:
// The upstream filter includes PX4 matrix/math.hpp so vector instantiations can
// use PX4 matrix types. OpenVINS deliberately instantiates the preserved
// template only as LowPassFilter2p<float>; no PX4 matrix type is required.
