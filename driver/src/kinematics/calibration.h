#pragma once

#include "kinematics/kinematics.h"
#include "kinematics/joints.h"

#include <array>
#include <string>

namespace arm {

// Converts between the kinematics' joint angles and the servos' encoder counts.
struct JointCalibration {
    int zero = 2048;     // counts at 0 rad
    int direction = 1;   // +1 if positive angles mean increasing counts, else -1
    int min = 0;         // safe range, in counts
    int max = 4095;
};

struct Calibration {
    std::array<JointCalibration, kJointCount> joint;

    Joints to_joints(const Pose& counts) const;
    // False, with the joint named in `why`, if an angle lands outside the safe range.
    bool to_counts(const Joints& q, Pose* counts, std::string* why) const;

    bool save(const std::string& path, std::string* error) const;
    static bool load(const std::string& path, Calibration* out, std::string* error);
};

constexpr const char* kCalibrationFile = "calibration.txt";
constexpr double kCountsPerRad = 4096.0 / (2.0 * 3.14159265358979323846);

}  // namespace arm
