#include "commands/commands.h"
#include "motion/paths.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <utility>

namespace arm {
namespace {

using Flags = std::vector<std::pair<std::string, double>>;

// Numbers and --flags with values; --release and --yes are switches. False on an unknown flag.
bool split_args(const std::vector<std::string>& args, const std::vector<std::string>& valued,
                std::vector<double>* numbers, Flags* flags, bool* release, bool* yes) {
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a == "--release") {
            *release = true;
        } else if (a == "--yes") {
            *yes = true;
        } else if (a.rfind("--", 0) == 0) {
            if (std::find(valued.begin(), valued.end(), a) == valued.end() || i + 1 >= args.size()) return false;
            flags->push_back({a, std::atof(args[++i].c_str())});
        } else {
            numbers->push_back(std::atof(a.c_str()));
        }
    }
    return true;
}

double flag(const Flags& flags, const char* name, double fallback) {
    for (const auto& f : flags) {
        if (f.first == name) return f.second;
    }
    return fallback;
}

}  // namespace

int move_command(const std::string& port_name, const std::vector<std::string>& args) {
    std::vector<double> n;
    Flags f;
    bool release = false, yes = false;
    const bool ok = split_args(args, {"--speed"}, &n, &f, &release, &yes);
    const double speed = flag(f, "--speed", 40.0);
    if (!ok || n.size() != 4 || speed <= 0.0 || speed > 200.0) {
        std::cout << "usage: servo_tool [PORT] move-to <x> <y> <z> <pitch> [--speed MM_S] [--release] [--yes]\n"
                     "  mm and degrees in the robot frame; speed 1-200 mm/s (default 40)\n";
        return 1;
    }
    return run_path(port_name, {{{n[0], n[1], n[2], n[3] / kDegPerRad}, 0.0}}, release, yes, speed);
}

int tap_command(const std::string& port_name, const std::vector<std::string>& args) {
    std::vector<double> n;
    Flags f;
    bool release = false, yes = false;
    const bool ok = split_args(args, {"--depth", "--speed", "--pitch", "--count", "--press"}, &n, &f, &release, &yes);
    const double depth = flag(f, "--depth", kTapDepth);
    const double speed = flag(f, "--speed", kDropSpeed);
    const double pitch = flag(f, "--pitch", -90.0) / kDegPerRad;
    const int count = static_cast<int>(flag(f, "--count", 1));
    const double press = flag(f, "--press", kTapPress);
    if (!ok || n.size() != 3 || depth < 0 || depth > 12 || speed <= 0 || speed > 250 || count < 1 ||
        count > 20 || press < 0.05 || press > 2.0) {
        std::cout << "usage: servo_tool [PORT] tap <x> <y> <z> [--depth MM] [--press S] [--speed MM_S] [--count N] [--pitch DEG]\n"
                     "  z is the screen surface; the tip drops from 20 mm above to `depth` below it\n"
                     "  (default 7) at `speed` (default 135), stays `press` s (default 0.1) and lifts, `count` times\n";
        return 1;
    }
    std::vector<Waypoint> path;
    for (int i = 0; i < count; ++i) add_tap(&path, {n[0], n[1], n[2], pitch}, depth, press, speed);
    return run_path(port_name, path, release, yes, 40.0, kTapAcceleration);
}

int swipe_command(const std::string& port_name, const std::vector<std::string>& args) {
    std::vector<double> n;
    Flags f;
    bool release = false, yes = false;
    const bool ok = split_args(args, {"--depth", "--speed", "--pitch"}, &n, &f, &release, &yes);
    const double depth = flag(f, "--depth", kSwipeDepth);
    const double speed = flag(f, "--speed", kSwipeSpeed);
    const double pitch = flag(f, "--pitch", -90.0) / kDegPerRad;
    if (!ok || n.size() != 5 || depth < 0 || depth > 12 || speed <= 0 || speed > 250) {
        std::cout << "usage: servo_tool [PORT] swipe <x1> <y1> <x2> <y2> <z> [--depth MM] [--speed MM_S] [--pitch DEG]\n"
                     "  drops onto the glass at (x1, y1), drags to (x2, y2) without lifting, lifts\n";
        return 1;
    }
    std::vector<Waypoint> path;
    add_swipe(&path, {n[0], n[1], n[4], pitch}, {n[2], n[3], n[4], pitch}, depth, speed);
    return run_path(port_name, path, release, yes, 40.0, kTapAcceleration);
}

}  // namespace arm
