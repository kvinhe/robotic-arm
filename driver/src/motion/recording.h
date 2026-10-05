#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace arm {

constexpr size_t kJointCount = 4;
constexpr std::array<uint8_t, kJointCount> kJointIds{1, 2, 3, 4};
constexpr std::array<const char*, kJointCount> kJointNames{"base", "shoulder", "elbow", "wrist"};

using Pose = std::array<int, kJointCount>;   // encoder counts 0-4095, in kJointIds order

struct Sample {
    double t;   // seconds from the start
    Pose pose;
};

// A motion recorded by hand: timestamped poses, starting at t = 0.
class Recording {
public:
    Recording() = default;
    explicit Recording(std::vector<Sample> samples) : samples_(std::move(samples)) {}

    const std::vector<Sample>& samples() const { return samples_; }
    double duration() const { return samples_.empty() ? 0.0 : samples_.back().t; }
    const Pose& first() const { return samples_.front().pose; }
    const Pose& last() const { return samples_.back().pose; }
    Pose min() const;
    Pose max() const;

    Pose at(double t) const;                    // interpolated, clamped to [0, duration]
    Recording reversed() const;                 // end to start
    Recording retraced() const;                 // out and back: ends where it began
    Recording scaled_to(double seconds) const;  // same path, new duration

    // Fastest joint speed in counts/s, over windows of at least `window` seconds so that
    // single-sample jitter does not read as a spike.
    double peak_speed(double window = 0.1) const;

    bool save(const std::string& path, std::string* error) const;
    static bool load(const std::string& path, Recording* out, std::string* error);

private:
    std::vector<Sample> samples_;
};

int max_abs_difference(const Pose& a, const Pose& b);
Pose clamp(const Pose& pose, const Pose& lo, const Pose& hi);

// The `servo_tool record` and `servo_tool replay` commands (record_replay.cpp).
// `port` may be empty to auto-detect; args are the command's own flags.
int record_command(const std::string& port, const std::vector<std::string>& args);
int replay_command(const std::string& port, const std::vector<std::string>& args);

}  // namespace arm
