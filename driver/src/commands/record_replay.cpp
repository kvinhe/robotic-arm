#include "motion/arm.h"
#include "commands/commands.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace arm {
namespace {

constexpr const char* kDefaultRecording = "recording.txt";
constexpr int kStartTolerance = 150;   // counts (~13 deg) from a replay's first pose
constexpr double kApproachSeconds = 1.0;

struct Options {
    double seconds = 0.0;
    std::string file = kDefaultRecording;
    bool reverse = false;
    bool yes = false;
};

bool parse_options(const std::vector<std::string>& args, bool replay, Options* o) {
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

}  // namespace

int record_command(const std::string& port_name, const std::vector<std::string>& args) {
    Options o;
    o.seconds = 15.0;
    if (!parse_options(args, false, &o)) {
        std::cout << "usage: servo_tool [PORT] record [--seconds N] [--out FILE]\n";
        return 1;
    }

    Connection link;
    if (!open_bus(port_name, &link)) return 1;
    sts3215::Bus bus(*link.port);
    Arm robot(bus, link.name);
    if (!robot.torque(false)) {
        std::cout << "could not reach every joint: " << robot.error() << "\n";
        return 1;
    }

    watch_ctrl_c();
    std::printf("Recording %.0f s on %s -- move the arm by hand now. Ctrl+C stops early.\n",
                o.seconds, link.name.c_str());

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
    if (!parse_options(args, true, &o)) {
        std::cout << "usage: servo_tool [PORT] replay [--seconds N] [--in FILE] [--reverse] [--yes]\n";
        return 1;
    }

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

    // Always finish at rest: a recording that ends elsewhere is retraced back.
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

    Connection link;
    if (!open_bus(port_name, &link)) return 1;
    sts3215::Bus bus(*link.port);
    Arm robot(bus, link.name);
    watch_ctrl_c();

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
    if (!o.yes && !wait_for_enter("\nPress Enter to enable torque and replay, or Ctrl+C to abort. ")) {
        return 1;
    }
    if (!robot.torque(true)) robot.hold("could not enable torque: " + robot.error());

    // The approach starts wherever the arm rests, possibly outside the recorded range,
    // so it is not clamped.
    robot.follow(Recording({{0.0, present}, {kApproachSeconds, plan.first()}}), nullptr, nullptr,
                 false);
    const int worst_lag = robot.follow(plan, &lo, &hi, true);
    print_pose("end", robot.settle(plan.last()), plan.last());
    robot.torque(false);
    std::printf("\nDone. Worst lag %d counts. Torque is off.\n", worst_lag);
    return 0;
}

}  // namespace arm
