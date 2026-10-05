#include "motion/arm.h"
#include "commands/commands.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace arm {

int calibrate_command(const std::string& port_name, const std::vector<std::string>& args) {
    std::string path = kCalibrationFile;
    if (args.size() == 2 && args[0] == "--out") {
        path = args[1];
    } else if (!args.empty()) {
        std::cout << "usage: servo_tool [PORT] calibrate [--out FILE]\n";
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
    auto read_or_die = [&](Pose* p) {
        if (!robot.read(p)) {
            std::cout << "lost the arm: " << robot.error() << "\n";
            std::exit(1);
        }
    };
    int old_offset[kJointCount];
    for (size_t j = 0; j < kJointCount; ++j) {
        if (!robot.read_offset(kJointIds[j], &old_offset[j])) {
            std::cout << "could not read " << kJointNames[j] << "'s position offset: " << robot.error() << "\n";
            return 1;
        }
    }

    // 1. A reference pose that is easy to judge by eye. Its angles in the kinematic model
    //    come from the geometry.
    std::cout << "\nTorque is off. Hold the arm in the L pose:\n"
                 "  - base facing straight ahead (this defines \"forward\" for everything)\n"
                 "  - shoulder horn centre directly below the elbow horn centre\n"
                 "  - elbow horn centre level with the wrist horn centre\n"
                 "  - stylus hanging straight down\n";
    if (!wait_for_enter("Press Enter while holding it there. ")) return 1;
    Pose l_pose{};
    read_or_die(&l_pose);
    const double pi = 3.14159265358979323846;
    const Joints q_l = joints_for(0.0, pi / 2, 0.0, -pi / 2);
    const double l_angles[kJointCount] = {q_l.pan, q_l.shoulder, q_l.elbow, q_l.wrist};

    // 2. Directions: nudge each joint the way the model calls positive.
    const char* nudge[kJointCount] = {
        "turn the BASE to the arm's right (clockwise seen from above)",
        "tilt the UPPER ARM forward",
        "lower the FOREARM (wrist end down)",
        "swing the STYLUS TIP back toward the base",
    };
    Calibration cal;
    for (size_t j = 0; j < kJointCount; ++j) {
        for (;;) {
            Pose before{};
            read_or_die(&before);
            std::cout << "\nFrom there, " << nudge[j] << " by about 20 degrees and hold it.\n";
            if (!wait_for_enter("Press Enter while holding it. ")) return 1;
            Pose after{};
            read_or_die(&after);
            const int delta = wrapped_delta(before[j], after[j]);
            if (std::abs(delta) >= 100) {
                cal.joint[j].direction = delta > 0 ? 1 : -1;
                break;
            }
            std::cout << "The " << kJointNames[j] << " only moved " << delta
                      << " counts -- move it further and try again.\n";
        }
    }

    // 3. Safe range, tracked through the 4095 -> 0 wrap: at 50 Hz a hand-moved joint moves
    //    far less than half a turn between readings, so the shortest step is the real one.
    std::cout << "\nNow move every joint slowly to BOTH ends of where it can safely go -- stop\n"
                 "short of hitting anything, including the table. Take your time.\n";
    if (!wait_for_enter("Press Enter to start, and Enter again when every joint is done. ")) return 1;
    Pose previous = l_pose;
    int travel[kJointCount] = {};   // counts from the L pose
    int lo[kJointCount] = {}, hi[kJointCount] = {};
    std::atomic<bool> done{false};
    std::thread enter([&] {
        std::string ignored;
        std::getline(std::cin, ignored);
        done = true;
    });
    while (!done && !g_stop) {
        Pose p{};
        if (robot.read(&p)) {
            for (size_t j = 0; j < kJointCount; ++j) {
                travel[j] += wrapped_delta(previous[j], p[j]);
                lo[j] = std::min(lo[j], travel[j]);
                hi[j] = std::max(hi[j], travel[j]);
            }
            previous = p;
        }
        std::printf("\r  swept:");
        for (size_t j = 0; j < kJointCount; ++j) {
            std::printf("  %s %3.0f deg", kJointNames[j], (hi[j] - lo[j]) / kCountsPerRad * kDegPerRad);
        }
        std::fflush(stdout);
        std::this_thread::sleep_for(kRecordTick);
    }
    std::printf("\n");
    enter.detach();   // still waiting for a line after Ctrl+C; the process is exiting anyway
    if (g_stop) return 1;

    constexpr double kMinSweepDeg = 30.0;
    constexpr int kMaxSweep = 3900;   // ~340 deg: past this there is no room left for the wrap
    bool ok = true;
    for (size_t j = 0; j < kJointCount; ++j) {
        const double deg = (hi[j] - lo[j]) / kCountsPerRad * kDegPerRad;
        if (deg < kMinSweepDeg) {
            std::printf("The %s only swept %.0f deg -- move it further both ways.\n", kJointNames[j], deg);
            ok = false;
        } else if (hi[j] - lo[j] > kMaxSweep) {
            std::printf("The %s swept %.0f deg -- keep it within about 340 deg.\n", kJointNames[j], deg);
            ok = false;
        }
    }
    if (!ok) {
        std::cout << "Nothing saved. Run calibrate again.\n";
        return 1;
    }

    // 4. Centre each joint in its encoder: offset the servo so the middle of the swept range
    //    reads 2048, putting the wrap as far as possible from anywhere the joint goes.
    constexpr int kMargin = 30;   // counts kept clear of the ends you showed
    for (size_t j = 0; j < kJointCount; ++j) {
        const int middle = (lo[j] + hi[j]) / 2;
        const int raw_middle = l_pose[j] + old_offset[j] + middle;
        int offset = wrapped_delta(2048, raw_middle);
        if (offset == 2048) offset = 2047;   // the register holds +-2047
        if (!robot.write_offset(kJointIds[j], offset)) {
            std::cout << "could not write " << kJointNames[j] << "'s position offset: " << robot.error() << "\n";
            return 1;
        }
        JointCalibration& c = cal.joint[j];
        const int l_counts = 2048 - middle;
        c.min = 2048 + (lo[j] - middle) + kMargin;
        c.max = 2048 + (hi[j] - middle) - kMargin;
        c.zero = l_counts - c.direction * static_cast<int>(std::lround(l_angles[j] * kCountsPerRad));
    }
    std::cout << "Position offsets written: every joint is now centred in its encoder.\n";

    std::string error;
    if (!cal.save(path, &error)) {
        std::cout << error << "\n";
        return 1;
    }
    std::cout << "\nSaved " << path << ":\n";
    for (size_t j = 0; j < kJointCount; ++j) {
        const JointCalibration& c = cal.joint[j];
        std::printf("  %-8s zero %5d  direction %+d  range %4d .. %4d  (%.0f deg)\n",
                    kJointNames[j], c.zero, c.direction, c.min, c.max,
                    (c.max - c.min) * 360.0 / 4096.0);
    }
    std::cout << "Check it: hold the arm anywhere and run `servo_tool where`.\n";
    return 0;
}

int where_command(const std::string& port_name, const std::vector<std::string>& args) {
    if (!args.empty()) {
        std::cout << "usage: servo_tool [PORT] where\n";
        return 1;
    }
    Calibration cal;
    if (!load_calibration(&cal)) return 1;
    Connection link;
    if (!open_bus(port_name, &link)) return 1;
    sts3215::Bus bus(*link.port);
    Arm robot(bus, link.name);

    Pose counts{};
    if (!robot.read(&counts)) {
        std::cout << "could not read the arm: " << robot.error() << "\n";
        return 1;
    }
    const Joints q = cal.to_joints(counts);
    std::printf("  counts  pan %4d  shoulder %4d  elbow %4d  wrist %4d\n", counts[0], counts[1],
                counts[2], counts[3]);
    std::printf("  angles  pan %6.1f  shoulder %6.1f  elbow %6.1f  wrist %6.1f deg\n",
                q.pan * kDegPerRad, q.shoulder * kDegPerRad, q.elbow * kDegPerRad,
                q.wrist * kDegPerRad);
    print_tip("tip", forward(q));
    return 0;
}

}  // namespace arm
