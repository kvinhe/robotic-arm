#pragma once

#include "kinematics/calibration.h"
#include "kinematics/kinematics.h"
#include "motion/recording.h"
#include "bus/connection.h"
#include "bus/sts3215.h"

#include <array>
#include <chrono>
#include <csignal>
#include <string>
#include <vector>

namespace arm {

using Clock = std::chrono::steady_clock;

constexpr double kDegPerRad = 57.29577951308232;
constexpr auto kRecordTick = std::chrono::milliseconds(20);   // 50 Hz
constexpr auto kControlTick = std::chrono::milliseconds(40);  // 25 Hz

// Servo settings for every motion.
constexpr int kSpeedCap = 1500;          // steps/s
constexpr double kSpeedHeadroom = 0.8;   // a plan may use 80 % of the cap, leaving room to catch up
constexpr int kAcceleration = 50;        // x100 steps/s^2
constexpr int kTapAcceleration = 150;    // quick taps need a sharp start
constexpr int kTorqueLimit = 500;        // 50 %

extern volatile std::sig_atomic_t g_stop;   // set by Ctrl+C after watch_ctrl_c()
void watch_ctrl_c();
double seconds_since(Clock::time_point start);
bool wait_for_enter(const std::string& prompt);   // false if Ctrl+C was pressed
int wrapped_delta(int a, int b);   // shortest signed step between encoder counts
bool open_bus(const std::string& name, Connection* link);   // prints the error
bool load_calibration(Calibration* cal);
void use_calibration(const Calibration& cal);   // instead of calibration.txt (the simulator)
void print_pose(const char* label, const Pose& actual, const Pose& goal);
void print_tip(const char* label, const TipPose& p);

// The four joints, read and commanded together.
class Arm {
public:
    Arm(sts3215::Bus& bus, std::string port) : bus_(bus), port_(std::move(port)) {}

    bool read(Pose* pose);
    bool command(const Pose& goal);

    // The goal starts where the arm already is, so enabling torque holds it still instead
    // of jumping to a stale goal.
    bool prepare(const Pose& present, int acceleration = kAcceleration);
    bool torque(bool on);

    // Follows `path` in real time, holding if a joint falls too far behind. Goals are
    // clamped to [lo, hi] when given. Returns the worst lag, in counts.
    int follow(const Recording& path, const Pose* lo, const Pose* hi, bool report,
               std::vector<Pose>* actuals = nullptr);
    Pose settle(const Pose& goal, const Pose* lo = nullptr, const Pose* hi = nullptr);

    // On any trouble, freeze with torque on: a limp shoulder would drop the arm.
    [[noreturn]] void hold(const std::string& why);

    // The servo's position offset (EEPROM): present position = raw encoder - offset.
    bool read_offset(uint8_t id, int* offset);
    bool write_offset(uint8_t id, int offset);

    std::string error() const { return bus_.last_error(); }

private:
    Pose corrected(const Pose& goal, const Pose* lo, const Pose* hi) const;
    void learn(const Pose& goal, const Pose& actual);

    sts3215::Bus& bus_;
    std::string port_;
    bool torque_on_ = false;
    std::array<double, kJointCount> correction_{};
};

}  // namespace arm
