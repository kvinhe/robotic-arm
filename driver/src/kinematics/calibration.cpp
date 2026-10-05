#include "kinematics/calibration.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace arm {
namespace {

constexpr const char* kHeader = "# arm calibration v1";

// Joints and Pose list the joints in the same order: pan, shoulder, elbow, wrist.
std::array<double, kJointCount> as_array(const Joints& q) {
    return {q.pan, q.shoulder, q.elbow, q.wrist};
}

}  // namespace

Joints Calibration::to_joints(const Pose& counts) const {
    std::array<double, kJointCount> rad{};
    for (size_t j = 0; j < kJointCount; ++j) {
        rad[j] = (counts[j] - joint[j].zero) * joint[j].direction / kCountsPerRad;
    }
    return {rad[0], rad[1], rad[2], rad[3]};
}

bool Calibration::to_counts(const Joints& q, Pose* counts, std::string* why) const {
    const auto rad = as_array(q);
    for (size_t j = 0; j < kJointCount; ++j) {
        const JointCalibration& c = joint[j];
        const int n = c.zero + c.direction * static_cast<int>(std::lround(rad[j] * kCountsPerRad));
        if (n < c.min || n > c.max) {
            *why = std::string(kJointNames[j]) + " would leave its calibrated range";
            return false;
        }
        (*counts)[j] = n;
    }
    return true;
}

bool Calibration::save(const std::string& path, std::string* error) const {
    std::ofstream file(path);
    file << kHeader << "\n# joint zero direction min max\n";
    for (size_t j = 0; j < kJointCount; ++j) {
        const JointCalibration& c = joint[j];
        file << kJointNames[j] << ' ' << c.zero << ' ' << c.direction << ' ' << c.min << ' '
             << c.max << '\n';
    }
    if (!file) {
        *error = "cannot write " + path;
        return false;
    }
    return true;
}

bool Calibration::load(const std::string& path, Calibration* out, std::string* error) {
    std::ifstream file(path);
    std::string line;
    if (!file || !std::getline(file, line) || line != kHeader) {
        *error = "no calibration at " + path + " -- run `servo_tool calibrate` first";
        return false;
    }
    size_t j = 0;
    while (std::getline(file, line) && j < kJointCount) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        std::string name;
        JointCalibration& c = out->joint[j];
        fields >> name >> c.zero >> c.direction >> c.min >> c.max;
        if (fields.fail() || name != kJointNames[j] || (c.direction != 1 && c.direction != -1) ||
            c.min >= c.max) {
            *error = path + ": bad line for " + kJointNames[j] + ": \"" + line + "\"";
            return false;
        }
        ++j;
    }
    if (j != kJointCount) {
        *error = path + " is missing joints";
        return false;
    }
    return true;
}

}  // namespace arm
