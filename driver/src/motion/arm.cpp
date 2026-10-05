#include "motion/arm.h"

#include "bus/serial_port.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace arm {
namespace {

constexpr int kLagLimit = 250;        // counts a joint may trail its goal...
constexpr double kLagGrace = 0.5;     // ...for this long before we hold
constexpr int kArrivedTolerance = 20;
constexpr double kSettleSeconds = 2.0;
constexpr int kMaxCorrection = 40;    // counts of sag correction
constexpr double kCorrectionGain = 0.2;

void on_sigint(int) { g_stop = 1; }

}  // namespace

volatile std::sig_atomic_t g_stop = 0;

void watch_ctrl_c() { std::signal(SIGINT, on_sigint); }

double seconds_since(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

bool wait_for_enter(const std::string& prompt) {
    std::cout << prompt << std::flush;
    std::string ignored;
    std::getline(std::cin, ignored);
    return !g_stop;
}

int wrapped_delta(int a, int b) {
    const int d = ((b - a) % 4096 + 4096) % 4096;
    return d > 2048 ? d - 4096 : d;
}

bool open_port(const std::string& name, SerialPort* port) {
    if (port->open(name, sts3215::kDefaultBaudRate)) return true;
    std::cout << port->last_error() << "\n";
    return false;
}

bool load_calibration(Calibration* cal) {
    std::string error;
    if (Calibration::load(kCalibrationFile, cal, &error)) return true;
    std::cout << error << "\n";
    return false;
}

void print_pose(const char* label, const Pose& actual, const Pose& goal) {
    std::printf("  %-5s", label);
    for (size_t j = 0; j < kJointCount; ++j) {
        std::printf("  %s %4d/%4d", kJointNames[j], actual[j], goal[j]);
    }
    std::printf("\n");
}

void print_tip(const char* label, const TipPose& p) {
    std::printf("  %-7s x %6.1f  y %6.1f  z %6.1f mm   pitch %6.1f deg\n", label, p.x, p.y, p.z,
                p.pitch * kDegPerRad);
}

bool Arm::read(Pose* pose) {
    for (size_t j = 0; j < kJointCount; ++j) {
        uint16_t p = 0;
        if (!bus_.read_u16(kJointIds[j], sts3215::kRegPresentPosition, &p)) return false;
        (*pose)[j] = p;
    }
    return true;
}

bool Arm::command(const Pose& goal) {
    for (size_t j = 0; j < kJointCount; ++j) {
        if (!bus_.write_u16(kJointIds[j], sts3215::kRegGoalPosition, static_cast<uint16_t>(goal[j]))) {
            return false;
        }
    }
    return true;
}

bool Arm::prepare(const Pose& present, int acceleration) {
    for (uint8_t id : kJointIds) {
        if (!bus_.write_u16(id, sts3215::kRegGoalSpeed, kSpeedCap) ||
            !bus_.write_u8(id, sts3215::kRegAcceleration, acceleration) ||
            !bus_.write_u16(id, sts3215::kRegTorqueLimit, kTorqueLimit)) {
            return false;
        }
    }
    return command(present);
}

bool Arm::torque(bool on) {
    bool ok = true;
    for (uint8_t id : kJointIds) {
        ok = bus_.write_u8(id, sts3215::kRegTorqueEnable, on ? 1 : 0) && ok;
    }
    torque_on_ = on && ok;
    return ok;
}

int Arm::follow(const Recording& path, const Pose* lo, const Pose* hi, bool report,
                std::vector<Pose>* actuals) {
    const auto start = Clock::now();
    auto next = start;
    double lagging_since = -1.0;
    int last_report = -1;
    int worst_lag = 0;
    Pose previous_goal = path.at(0.0);
    for (double t = 0.0; t <= path.duration(); t = seconds_since(start)) {
        if (g_stop) hold("interrupted");
        const Pose goal = lo ? clamp(path.at(t), *lo, *hi) : path.at(t);
        Pose actual{};
        if (!command(corrected(goal, lo, hi)) || !read(&actual)) hold("lost the arm mid-move: " + error());
        // Only learn the sag while the goal is still: mid-motion the gap is lag, not sag.
        if (max_abs_difference(goal, previous_goal) <= 3) learn(goal, actual);
        previous_goal = goal;
        if (actuals) actuals->push_back(actual);

        const int lag = max_abs_difference(actual, goal);
        worst_lag = std::max(worst_lag, lag);
        if (lag <= kLagLimit) {
            lagging_since = -1.0;
        } else if (lagging_since < 0.0) {
            lagging_since = t;
        } else if (t - lagging_since > kLagGrace) {
            print_pose("", actual, goal);
            hold("a joint is " + std::to_string(lag) + " counts behind -- blocked?");
        }

        if (report && static_cast<int>(t) != last_report) {
            last_report = static_cast<int>(t);
            print_pose((std::to_string(last_report) + "s").c_str(), actual, goal);
        }
        next += kControlTick;
        std::this_thread::sleep_until(next);
    }
    return worst_lag;
}

Pose Arm::settle(const Pose& goal, const Pose* lo, const Pose* hi) {
    const auto start = Clock::now();
    Pose actual{};
    for (;;) {
        if (!command(corrected(goal, lo, hi)) || !read(&actual)) hold("lost the arm at the end: " + error());
        if (max_abs_difference(actual, goal) <= kArrivedTolerance / 2) return actual;
        if (seconds_since(start) > kSettleSeconds) {
            if (max_abs_difference(actual, goal) <= kArrivedTolerance) return actual;
            hold("did not settle at the end");
        }
        learn(goal, actual);
        std::this_thread::sleep_for(kControlTick);
    }
}

void Arm::hold(const std::string& why) {
    std::cout << "\nSTOPPED: " << why << "\n";
    Pose present{};
    if (torque_on_ && read(&present) && command(present)) {
        std::cout << "The arm is HOLDING with torque on. Support it, then run:\n"
                     "  build/servo_tool " << port_ << " torque-off\n";
    }
    std::exit(1);
}

// Sign-magnitude, with bit 11 as the sign.
bool Arm::read_offset(uint8_t id, int* offset) {
    uint16_t v = 0;
    if (!bus_.read_u16(id, sts3215::kRegPositionOffset, &v)) return false;
    *offset = (v & 0x800) ? -static_cast<int>(v & 0x7FF) : static_cast<int>(v & 0x7FF);
    return true;
}

bool Arm::write_offset(uint8_t id, int offset) {
    const uint16_t v = static_cast<uint16_t>(offset < 0 ? (-offset | 0x800) : offset);
    const bool ok = bus_.write_u8(id, sts3215::kRegLock, 0) &&
                    bus_.write_u16(id, sts3215::kRegPositionOffset, v);
    const bool relocked = bus_.write_u8(id, sts3215::kRegLock, 1);   // even on failure
    return ok && relocked;
}

// Gravity leaves the servos a little short of their goal. Learn that per joint and add it
// to what we command (integral correction), capped and kept in range.
Pose Arm::corrected(const Pose& goal, const Pose* lo, const Pose* hi) const {
    Pose out{};
    for (size_t j = 0; j < kJointCount; ++j) out[j] = goal[j] + static_cast<int>(std::lround(correction_[j]));
    return lo ? clamp(out, *lo, *hi) : out;
}

void Arm::learn(const Pose& goal, const Pose& actual) {
    for (size_t j = 0; j < kJointCount; ++j) {
        correction_[j] = std::clamp(correction_[j] + kCorrectionGain * (goal[j] - actual[j]),
                                    -double(kMaxCorrection), double(kMaxCorrection));
    }
}

}  // namespace arm
