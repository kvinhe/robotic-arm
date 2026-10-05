#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace arm {

constexpr size_t kJointCount = 4;
constexpr std::array<uint8_t, kJointCount> kJointIds{1, 2, 3, 4};
constexpr std::array<const char*, kJointCount> kJointNames{"base", "shoulder", "elbow", "wrist"};

using Pose = std::array<int, kJointCount>;   // encoder counts 0-4095, in kJointIds order

inline int max_abs_difference(const Pose& a, const Pose& b) {
    int worst = 0;
    for (size_t j = 0; j < kJointCount; ++j) worst = std::max(worst, std::abs(a[j] - b[j]));
    return worst;
}

inline Pose clamp(const Pose& pose, const Pose& lo, const Pose& hi) {
    Pose out{};
    for (size_t j = 0; j < kJointCount; ++j) out[j] = std::clamp(pose[j], lo[j], hi[j]);
    return out;
}

}  // namespace arm
