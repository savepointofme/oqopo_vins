#pragma once

#include "LowPassFilter2p.hpp"

// OPENVINS COMPATIBILITY ADAPTATION:
// The upstream class lives in ::math. This alias provides the explicit px4::math
// vendor namespace used by the OpenVINS wrapper without modifying the byte-for-
// byte upstream source file or its class/state semantics.
namespace px4
{
namespace math
{

template<typename T>
using LowPassFilter2p = ::math::LowPassFilter2p<T>;

} // namespace math
} // namespace px4
