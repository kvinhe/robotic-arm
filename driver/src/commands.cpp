#include "recording.h"
#include "serial_port.h"
#include "sts3215.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace arm {
namespace {

using Clock = std::chrono::steady_clock;

constexpr const char* kDefaultFile = "recording.txt";
constexpr auto kRecordTick = std::chrono::milliseconds(20);   // 50 Hz
constexpr auto kReplayTick = std::chrono::milliseconds(40);   // 25 Hz

constexpr int kSpeedCap = 1500;           // steps/s (~130 deg/s), written to every servo
constexpr double kSpeedHeadroom = 0.8;    // a replay may use at most 80 % of the cap
constexpr int kAcceleration = 50;         // 5000 steps/s^2
constexpr int kTorqueLimit = 500;         // 50 %
constexpr int kStartTolerance = 150;      // counts (~13 deg) from a replay's first pose
constexpr double kApproachSeconds = 1.0;  // slow move onto the first pose
constexpr int kLagLimit = 250;            // counts a joint may trail its goal...
constexpr double kLagGrace = 0.5;         // ...for this many seconds before replay holds
constexpr int kArrivedTolerance = 20;
constexpr double kSettleSeconds = 2.0;

volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int) { g_stop = 1; }

double seconds_since(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

struct Options {
    double seconds = 0.0;
    std::string file = kDefaultFile;
    bool reverse = false;
    bool yes = false;
};

bool parse(const std::vector<std::string>& args, bool replay, Options* o) {
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        const bool has_value = i + 1 < args.size();
        if (a == "--seconds" && has_value) {
            o->seconds = std::atof(args[++i].c_str());
            if (o->seconds <= 0.0 || o->seconds > 600.0) return false;
        } else if (a == (replay ? "--in" : "--out") && has_value) {
            o->file = args[++i];
        } else if (replay && a == "--reverse") {
            o->reverse = true;
        } else if (replay && a == "--yes") {
            o->yes = true;
        } else {
            return false;
        }
    }
    return true;
}

void print_ranges(const Recording& r) {
    const Pose lo = r.min();
    const Pose hi = r.max();
    for (size_t j = 0; j < kJointCount; ++j) {
        std::printf("  %-8s %4d .. %4d  (%.0f deg)\n", kJointNames[j], lo[j], hi[j],
                    (hi[j] - lo[j]) * 360.0 / 4096.0);
    }
}

void print_pose(const char* label, const Pose& actual, const Pose& goal) {
    std::printf("  %-5s", label);
    for (size_t j = 0; j < kJointCount; ++j) {
        std::printf("  %s %4d/%4d", kJointNames[j], actual[j], goal[j]);
    }
    std::printf("\n");
}

// All four joints, read and commanded together.
class Arm {
public:
    Arm(sts3215::Bus& bus, std::string port) : bus_(bus), port_(std::move(port)) {}

    bool read(Pose* pose) {
        for (size_t j = 0; j < kJointCount; ++j) {
            uint16_t p = 0;
            if (!bus_.read_u16(kJointIds[j], sts3215::kRegPresentPosition, &p)) return false;
            (*pose)[j] = p;
        }
        return true;
    }

    bool command(const Pose& goal) {
        for (size_t j = 0; j < kJointCount; ++j) {
            if (!bus_.write_u16(kJointIds[j], sts3215::kRegGoalPosition,
                                static_cast<uint16_t>(goal[j]))) {
                return false;
            }
        }
        return true;
    }

    // Speed, acceleration and torque limits, with the goal set to where the arm already
    // is -- so enabling torque holds it still instead of jumping to a stale goal.
    bool prepare(const Pose& present) {
        for (uint8_t id : kJointIds) {
            if (!bus_.write_u16(id, sts3215::kRegGoalSpeed, kSpeedCap) ||
                !bus_.write_u8(id, sts3215::kRegAcceleration, kAcceleration) ||
                !bus_.write_u16(id, sts3215::kRegTorqueLimit, kTorqueLimit)) {
                return false;
            }
        }
        return command(present);
    }

    bool torque(bool on) {
        bool ok = true;
        for (uint8_t id : kJointIds) {
            ok = bus_.write_u8(id, sts3215::kRegTorqueEnable, on ? 1 : 0) && ok;
        }
        torque_on_ = on && ok;
        return ok;
    }

    // Freeze in place with torque on. A limp shoulder would drop the arm, so on any
    // trouble mid-move we hold rather than let go.
    [[noreturn]] void hold(const std::string& why) {
        std::cout << "\nSTOPPED: " << why << "\n";
        Pose present{};
        if (torque_on_ && read(&present) && command(present)) {
            std::cout << "The arm is HOLDING with torque on. Support it, then run:\n"
                         "  build/servo_tool " << port_ << " torque-off\n";
        }
        std::exit(1);
    }

    std::string error() const { return bus_.last_error(); }

private:
    sts3215::Bus& bus_;
    std::string port_;
    bool torque_on_ = false;
};

}  // namespace

int record_command(const std::string& port_name, const std::vector<std::string>& args) {
    Options o;
    o.seconds = 15.0;
    if (!parse(args, false, &o)) {
        std::cout << "usage: servo_tool [PORT] record [--seconds N] [--out FILE]\n";
        return 1;
    }

    SerialPort port;
    if (!port.open(port_name, sts3215::kDefaultBaudRate)) {
        std::cout << port.last_error() << "\n";
        return 1;
    }
    sts3215::Bus bus(port);
    Arm robot(bus, port.name());
    if (!robot.torque(false)) {
        std::cout << "could not reach every joint: " << robot.error() << "\n";
        return 1;
    }

    std::signal(SIGINT, on_sigint);
    std::printf("Recording %.0f s on %s -- move the arm by hand now. Ctrl+C stops early.\n",
                o.seconds, port.name().c_str());

    std::vector<Sample> samples;
    const auto start = Clock::now();
    auto next = start;
    while (!g_stop && seconds_since(start) < o.seconds) {
        Sample s{seconds_since(start), {}};
        if (robot.read(&s.pose)) samples.push_back(s);   // a bad read drops one sample
        next += kRecordTick;
        std::this_thread::sleep_until(next);
    }

    if (samples.size() < 2) {
        std::cout << "too few samples to save -- check the bus\n";
        return 1;
    }
    const double t0 = samples.front().t;
    for (Sample& s : samples) s.t -= t0;
    const Recording recording(std::move(samples));

    std::string error;
    if (!recording.save(o.file, &error)) {
        std::cout << error << "\n";
        return 1;
    }
    const size_t n = recording.samples().size();
    std::printf("Saved %zu samples over %.1f s (%.0f Hz) to %s\n", n, recording.duration(),
                (n - 1) / recording.duration(), o.file.c_str());
    print_ranges(recording);
    if (max_abs_difference(recording.first(), recording.last()) > kStartTolerance) {
        std::cout << "Note: it ends away from where it started, so replay will retrace it back.\n";
    }
    return 0;
}

int replay_command(const std::string& port_name, const std::vector<std::string>& args) {
    Options o;
    if (!parse(args, true, &o)) {
        std::cout << "usage: servo_tool [PORT] replay [--seconds N] [--in FILE] [--reverse] [--yes]\n";
        return 1;
    }

    // Plan everything before touching the bus.
    Recording recording;
    std::string error;
    if (!Recording::load(o.file, &recording, &error)) {
        std::cout << error << "\n";
        return 1;
    }
    const Pose lo = recording.min();
    const Pose hi = recording.max();

    Recording plan = o.reverse ? recording.reversed() : recording;
    if (o.seconds > 0.0) plan = plan.scaled_to(o.seconds);
    const double motion_seconds = plan.duration();

    // Always finish at rest. Reversed already ends at the recorded start; otherwise a
    // recording that ends elsewhere is retraced back along its own path.
    const bool retrace =
        !o.reverse && max_abs_difference(plan.first(), plan.last()) > kStartTolerance;
    if (retrace) plan = plan.retraced();

    const double peak = plan.peak_speed();
    const double allowed = kSpeedCap * kSpeedHeadroom;
    if (peak > allowed) {
        std::printf("Too fast: a joint would need %.0f counts/s, over the %.0f allowed.\n"
                    "Use --seconds %.1f or more.\n",
                    peak, allowed, motion_seconds * peak / allowed + 0.05);
        return 1;
    }

    std::printf("\nreplay %s%s: %.1f s%s, peak speed %.0f%% of the cap\n", o.file.c_str(),
                o.reverse ? " reversed" : "", motion_seconds,
                retrace ? " out and the same back" : "", 100.0 * peak / kSpeedCap);
    print_ranges(recording);

    SerialPort port;
    if (!port.open(port_name, sts3215::kDefaultBaudRate)) {
        std::cout << port.last_error() << "\n";
        return 1;
    }
    sts3215::Bus bus(port);
    Arm robot(bus, port.name());
    std::signal(SIGINT, on_sigint);

    Pose present{};
    if (!robot.read(&present)) robot.hold("could not read the arm: " + robot.error());
    if (max_abs_difference(present, plan.first()) > kStartTolerance) {
        std::cout << "\nThe arm is not at the replay's first pose:\n";
        print_pose("now", present, plan.first());
        std::cout << "Move it there by hand (within ~13 deg per joint)"
                  << (o.reverse ? "" : ", or use --reverse if it is at the end pose") << ".\n";
        return 1;
    }

    if (!robot.prepare(present)) robot.hold("could not configure the servos: " + robot.error());
    if (!o.yes) {
        std::cout << "\nPress Enter to enable torque and replay, or Ctrl+C to abort. " << std::flush;
        std::string ignored;
        std::getline(std::cin, ignored);
        if (g_stop) return 1;
    }
    if (!robot.torque(true)) robot.hold("could not enable torque: " + robot.error());

    int worst_lag = 0;
    // Follows `path` in real time. `clamped` keeps goals inside the recorded range; it is
    // off for the approach, which starts wherever the arm rests, maybe just outside it.
    auto follow = [&](const Recording& path, bool clamped, bool report) {
        const auto start = Clock::now();
        auto next = start;
        double lagging_since = -1.0;
        int last_report = -1;
        for (double t = 0.0; t <= path.duration(); t = seconds_since(start)) {
            if (g_stop) robot.hold("interrupted");
            const Pose goal = clamped ? clamp(path.at(t), lo, hi) : path.at(t);
            Pose actual{};
            if (!robot.command(goal) || !robot.read(&actual)) {
                robot.hold("lost the arm mid-move: " + robot.error());
            }

            const int lag = max_abs_difference(actual, goal);
            worst_lag = std::max(worst_lag, lag);
            if (lag <= kLagLimit) {
                lagging_since = -1.0;
            } else if (lagging_since < 0.0) {
                lagging_since = t;
            } else if (t - lagging_since > kLagGrace) {
                print_pose("", actual, goal);
                robot.hold("a joint is " + std::to_string(lag) + " counts behind -- blocked?");
            }

            if (report && static_cast<int>(t) != last_report) {
                last_report = static_cast<int>(t);
                print_pose((std::to_string(last_report) + "s").c_str(), actual, goal);
            }
            next += kReplayTick;
            std::this_thread::sleep_until(next);
        }
    };

    const Recording approach({{0.0, present}, {kApproachSeconds, plan.first()}});
    follow(approach, false, false);
    follow(plan, true, true);

    // Let every joint arrive before letting go.
    if (!robot.command(plan.last())) robot.hold("lost the arm at the end: " + robot.error());
    const auto settle_start = Clock::now();
    Pose actual{};
    while (!robot.read(&actual) || max_abs_difference(actual, plan.last()) > kArrivedTolerance) {
        if (seconds_since(settle_start) > kSettleSeconds) robot.hold("did not settle at the end");
        std::this_thread::sleep_for(kReplayTick);
    }
    print_pose("end", actual, plan.last());
    robot.torque(false);
    std::printf("\nDone. Worst lag %d counts. Torque is off.\n", worst_lag);
    return 0;
}

}  // namespace arm
