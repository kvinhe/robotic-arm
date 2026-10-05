#include "commands/commands.h"
#include "motion/paths.h"
#include "commands/phone.h"
#include "bus/serial_port.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace arm {
namespace {

// A point `left` and `up` mm from the middle of the screen, oriented as in phone.h.
TipPose on_screen(const TipPose& centre, double left, double up, double pitch) {
    const double a = phone::kTopDirectionDeg / kDegPerRad;
    return {centre.x + up * std::cos(a) - left * std::sin(a),
            centre.y + up * std::sin(a) + left * std::cos(a), centre.z, pitch};
}

const phone::Model* find_model(const std::string& name) {
    std::string key;
    for (char c : name) {
        if (std::isalnum(static_cast<unsigned char>(c))) key += static_cast<char>(std::tolower(c));
    }
    for (const phone::Model& m : phone::kModels) {
        if (key == m.name) return &m;
    }
    return nullptr;
}

// Wake tap, swipe up, then the pin -- or, with `check`, a hover over its first digit.
std::vector<Waypoint> unlock_waypoints(const TipPose& centre, const phone::Model& m,
                                       const std::string& pin, bool check, double pitch) {
    const double mm = m.mm_per_pt;
    auto key = [&](char c) {
        const int d = c - '0';   // keypad rows: 1 2 3 / 4 5 6 / 7 8 9 / _ 0 _
        const int row = d == 0 ? 3 : (d - 1) / 3;
        const int col = d == 0 ? 1 : (d - 1) % 3;
        return on_screen(centre, (phone::kKey1LeftPt - col * phone::kKeyColumnPt) * mm,
                         (phone::kKey1UpPt - row * phone::kKeyRowPt) * mm, pitch);
    };
    const TipPose middle = on_screen(centre, 0, 0, pitch);
    const double swipe_start = (m.height_pt / 2 - phone::kSwipeStartAboveBottomPt) * mm;

    std::vector<Waypoint> path;
    add_tap(&path, middle, kTapDepth, kTapPress, kDropSpeed);
    add_pause(&path, phone::kPauseAfterWake);
    add_swipe(&path, on_screen(centre, 0, -swipe_start, pitch), middle, kSwipeDepth, kSwipeSpeed);
    add_pause(&path, phone::kPauseAfterSwipe);
    if (check) {
        travel_to(&path, along_stylus(key(pin[0]), 10.0));
    } else {
        for (const char c : pin) add_tap(&path, key(c), kTapDepth, kTapPress, kDropSpeed);
    }
    return path;
}

}  // namespace

int unlock_command(const std::string& port_name, const std::vector<std::string>& args) {
    std::vector<std::string> words;
    bool check = false, release = false, yes = false;
    for (const std::string& a : args) {
        if (a == "--check") check = true;
        else if (a == "--release") release = true;
        else if (a == "--yes") yes = true;
        else words.push_back(a);
    }
    const phone::Model* model = words.size() == 2 ? find_model(words[0]) : nullptr;
    const std::string pin = words.size() == 2 ? words[1] : "";
    const bool pin_ok = (pin.size() == 4 || pin.size() == 6) &&
        std::all_of(pin.begin(), pin.end(), [](char c) { return c >= '0' && c <= '9'; });
    if (!model || !pin_ok) {
        std::cout << "usage: servo_tool [PORT] unlock <model> <pin> [--check] [--release] [--yes]\n"
                     "  e.g. servo_tool unlock iphone15 1111. The pin is 4 or 6 digits. --check\n"
                     "  wakes and swipes, then hovers over the first digit instead of typing.\n";
        if (words.size() == 2 && !model) {
            std::cout << "unknown model \"" << words[0] << "\". Known:";
            for (const phone::Model& m : phone::kModels) std::cout << ' ' << m.name;
            std::cout << "\n";
        }
        return 1;
    }

    Calibration cal;
    if (!load_calibration(&cal)) return 1;
    SerialPort port;
    if (!open_port(port_name, &port)) return 1;
    sts3215::Bus bus(port);
    Arm robot(bus, port.name());
    watch_ctrl_c();

    // Where the phone is: the stylus tip, placed by hand on the middle of the screen.
    if (!robot.torque(false)) {
        std::cout << "could not reach every joint: " << robot.error() << "\n";
        return 1;
    }
    if (!yes && !wait_for_enter("Torque is off. Put the stylus tip on the middle of the screen, then press Enter. ")) {
        return 1;
    }
    Pose present{};
    if (!robot.read(&present)) {
        std::cout << "could not read the arm: " << robot.error() << "\n";
        return 1;
    }
    const TipPose centre = forward(cal.to_joints(present));
    print_tip("centre", centre);

    // The most vertical stylus angle that keeps every touch in range.
    Recording path;
    std::vector<Waypoint> waypoints;
    std::string why;
    double chosen = 0.0;
    for (const double deg : phone::kPitchesDeg) {
        waypoints = unlock_waypoints(centre, *model, pin, check, deg / kDegPerRad);
        if (plan_path(cal, present, waypoints, 40.0, &path, &why)) {
            chosen = deg;
            break;
        }
    }
    if (chosen == 0.0) {
        print_reason(why);
        std::cout << "No stylus angle works from here: move the phone closer to the arm.\n";
        return 1;
    }
    std::printf("%s, stylus at %.0f deg, %.1f s.\n",
                check ? "Wake, swipe, then hover over the first digit" : "Wake, swipe, then the passcode",
                chosen, path.duration());
    return execute_path(robot, cal, present, path, lowest_z(waypoints), release, yes, kTapAcceleration);
}

}  // namespace arm
