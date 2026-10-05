#include "motion/paths.h"

#include "bus/serial_port.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace arm {
namespace {

constexpr double kPlanStep = 0.02;   // seconds between planned samples

double distance(const TipPose& a, const TipPose& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// `speed` mm/s, at most 45 deg/s of pitch, at least 0.3 s.
double calm_seconds(const TipPose& a, const TipPose& b, double speed) {
    return std::max({0.3, distance(a, b) / speed, std::abs(b.pitch - a.pitch) / (45.0 / kDegPerRad)});
}

}  // namespace

TipPose along_stylus(const TipPose& p, double dz) {
    const double length = dz / std::sin(p.pitch);
    const double out = length * std::cos(p.pitch);   // outward from the base axis
    const double radial = std::hypot(p.x, p.y);
    return {p.x + out * p.x / radial, p.y + out * p.y / radial, p.z + dz, p.pitch};
}

bool plan_path(const Calibration& cal, const Pose& present, std::vector<Waypoint> waypoints,
               double approach_speed, Recording* out, std::string* why) {
    TipPose from = forward(cal.to_joints(present));
    if (waypoints.front().seconds <= 0.0) {
        waypoints.front().seconds = calm_seconds(from, waypoints.front().pose, approach_speed);
    }
    std::vector<Sample> samples;
    double t0 = 0.0;
    for (const Waypoint& w : waypoints) {
        const int steps = std::max(1, static_cast<int>(std::ceil(w.seconds / kPlanStep)));
        for (int i = samples.empty() ? 0 : 1; i <= steps; ++i) {
            const double s = static_cast<double>(i) / steps;
            const double e = s * s * (3 - 2 * s);   // smoothstep
            const TipPose p{from.x + e * (w.pose.x - from.x), from.y + e * (w.pose.y - from.y),
                            from.z + e * (w.pose.z - from.z), from.pitch + e * (w.pose.pitch - from.pitch)};
            Joints q;
            Pose counts{};
            if (!inverse(p, &q, why) || !cal.to_counts(q, &counts, why)) {
                char where[64];
                std::snprintf(where, sizeof(where), "no safe path to (%.0f, %.0f, %.0f): ", p.x, p.y, p.z);
                *why = where + *why;
                return false;
            }
            if (!samples.empty() && max_abs_difference(counts, samples.back().pose) > 60) {
                *why = "no smooth path: the arm would have to flip to its other elbow configuration";
                return false;
            }
            samples.push_back({t0 + s * w.seconds, counts});
        }
        t0 += w.seconds;
        from = w.pose;
    }
    Recording path(std::move(samples));
    if (max_abs_difference(path.first(), present) > 50) {
        *why = "the arm's current configuration is not the one IK would choose -- move it\n"
               "roughly into the L pose (see `servo_tool calibrate`) and try again";
        return false;
    }
    if (path.peak_speed(0.04) > kSpeedCap * kSpeedHeadroom) {
        *why = "too fast for the servos";
        return false;
    }
    *out = std::move(path);
    return true;
}

int execute_path(Arm& robot, const Calibration& cal, const Pose& present, const Recording& path,
                 double deepest_goal, bool release, bool yes, int acceleration) {
    Pose lo{}, hi{};
    for (size_t j = 0; j < kJointCount; ++j) {
        lo[j] = cal.joint[j].min;
        hi[j] = cal.joint[j].max;
    }
    if (!robot.prepare(present, acceleration)) robot.hold("could not configure the servos: " + robot.error());
    if (!yes && !wait_for_enter("Press Enter to enable torque and move, or Ctrl+C to abort. ")) return 1;
    if (!robot.torque(true)) robot.hold("could not enable torque: " + robot.error());

    std::vector<Pose> actuals;
    const int worst_lag = robot.follow(path, &lo, &hi, false, &actuals);
    const Pose end = robot.settle(path.last(), &lo, &hi);
    double deepest = 1e9;
    for (const Pose& a : actuals) deepest = std::min(deepest, forward(cal.to_joints(a)).z);
    std::printf("  deepest tip z %.1f mm (commanded %.1f)\n", deepest, deepest_goal);
    print_tip("reached", forward(cal.to_joints(end)));
    if (release) robot.torque(false);
    std::printf("Done in %.1f s, worst lag %d counts. %s\n", path.duration(), worst_lag,
                release ? "Torque is off." : "Holding with torque on.");
    return 0;
}

int run_path(const std::string& port_name, const std::vector<Waypoint>& waypoints, bool release,
             bool yes, double approach_speed, int acceleration) {
    Calibration cal;
    if (!load_calibration(&cal)) return 1;
    SerialPort port;
    if (!open_port(port_name, &port)) return 1;
    sts3215::Bus bus(port);
    Arm robot(bus, port.name());
    watch_ctrl_c();

    Pose present{};
    if (!robot.read(&present)) {
        std::cout << "could not read the arm: " << robot.error() << "\n";
        return 1;
    }
    Recording path;
    std::string why;
    if (!plan_path(cal, present, waypoints, approach_speed, &path, &why)) {
        print_reason(why);
        return 1;
    }
    return execute_path(robot, cal, present, path, lowest_z(waypoints), release, yes, acceleration);
}

void print_reason(std::string why) {
    why[0] = static_cast<char>(std::toupper(why[0]));
    std::cout << why << ".\n";
}

double lowest_z(const std::vector<Waypoint>& waypoints) {
    double z = 1e9;
    for (const Waypoint& w : waypoints) z = std::min(z, w.pose.z);
    return z;
}

void travel_to(std::vector<Waypoint>* path, const TipPose& pose) {
    path->push_back({pose, path->empty() ? 0.0 : calm_seconds(path->back().pose, pose, 80.0)});
}

void add_pause(std::vector<Waypoint>* path, double seconds) {
    path->push_back({path->back().pose, seconds});
}

void add_tap(std::vector<Waypoint>* path, const TipPose& surface, double depth, double press,
             double drop_speed) {
    const TipPose above = along_stylus(surface, kHover);
    const TipPose pressed = along_stylus(surface, -depth);
    const double drop = (kHover + depth) / drop_speed;
    travel_to(path, above);
    path->push_back({pressed, drop});
    path->push_back({pressed, press});
    path->push_back({above, drop});
    add_pause(path, 0.25);
}

void add_swipe(std::vector<Waypoint>* path, const TipPose& from, const TipPose& to, double depth,
               double speed) {
    const TipPose start = along_stylus(from, -depth);
    const TipPose end = along_stylus(to, -depth);
    const double drop = (kHover + depth) / kDropSpeed;
    travel_to(path, along_stylus(from, kHover));
    path->push_back({start, drop});
    add_pause(path, 0.1);
    path->push_back({end, std::max(0.2, distance(start, end) / speed)});
    path->push_back({along_stylus(to, kHover), drop});
}

}  // namespace arm
