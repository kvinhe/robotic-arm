#pragma once

#include <array>

// Settings for `servo_tool unlock <model> <pin>`. The screen centre is measured at the
// start of every run, so only the orientation and the layout live here. After editing,
// rebuild with `cmake --build build`.
namespace phone {

// Which way the top of the phone (the Dynamic Island) points, seen from above: degrees from
// the arm's forward direction, counterclockwise. 0 = away from the arm, 90 = the arm's
// left, -90 = the arm's right (charger toward the arm's left), 180 = toward the arm.
constexpr double kTopDirectionDeg = -90.0;

// Stylus angles to try, most vertical first: the first one that keeps every touch inside
// the calibrated range is used.
constexpr std::array<double, 7> kPitchesDeg{-90, -85, -80, -75, -70, -65, -60};

// Screen sizes in iOS points, and mm per point (= pixel scale x 25.4 / pixels per inch).
struct Model {
    const char* name;   // lowercase, no spaces: "iPhone 15 Pro" -> "iphone15pro"
    double width_pt, height_pt, mm_per_pt;
};
constexpr double k460ppi = 3 * 25.4 / 460;
constexpr std::array<Model, 22> kModels{{
    {"iphone12mini", 375, 812, 3 * 25.4 / 476},   {"iphone13mini", 375, 812, 3 * 25.4 / 476},
    {"iphone12", 390, 844, k460ppi},              {"iphone12pro", 390, 844, k460ppi},
    {"iphone13", 390, 844, k460ppi},              {"iphone13pro", 390, 844, k460ppi},
    {"iphone14", 390, 844, k460ppi},              {"iphone12promax", 428, 926, 3 * 25.4 / 458},
    {"iphone13promax", 428, 926, 3 * 25.4 / 458}, {"iphone14plus", 428, 926, 3 * 25.4 / 458},
    {"iphone14pro", 393, 852, k460ppi},           {"iphone15", 393, 852, k460ppi},
    {"iphone15pro", 393, 852, k460ppi},           {"iphone16", 393, 852, k460ppi},
    {"iphone14promax", 430, 932, k460ppi},        {"iphone15plus", 430, 932, k460ppi},
    {"iphone15promax", 430, 932, k460ppi},        {"iphone16plus", 430, 932, k460ppi},
    {"iphone16pro", 402, 874, k460ppi},           {"iphone16promax", 440, 956, k460ppi},
    {"iphone11", 414, 896, 2 * 25.4 / 326},       {"iphonexr", 414, 896, 2 * 25.4 / 326},
}};

// The swipe to unlock starts this many points above the bottom edge and ends at the middle.
constexpr double kSwipeStartAboveBottomPt = 32;

// Passcode keypad, in points from the middle of the screen. Tuned on an iPhone 15; other
// models use the same layout as an estimate -- try `unlock --check` first on a new model.
constexpr double kKey1LeftPt = 115.3;   // the "1" key, left of the middle
constexpr double kKey1UpPt = 111.1;     // ...and above it
constexpr double kKeyColumnPt = 102.7;  // between key columns
constexpr double kKeyRowPt = 93.0;      // between key rows

// Pauses, in seconds: after the wake tap, and after the swipe while the keypad slides in.
constexpr double kPauseAfterWake = 0.5;
constexpr double kPauseAfterSwipe = 0.8;

}  // namespace phone
