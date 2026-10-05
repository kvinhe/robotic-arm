#pragma once

#include "motion/arm.h"

#include <string>
#include <vector>

namespace arm {

// Touching a screen. A slow press does not register, so taps drop fast and leave quickly.
constexpr double kHover = 20.0;        // mm above the glass between touches
constexpr double kDropSpeed = 135.0;   // mm/s
constexpr double kTapDepth = 7.0;      // mm past the surface
constexpr double kTapPress = 0.1;      // s on the glass
constexpr double kSwipeDepth = 4.0;
constexpr double kSwipeSpeed = 108.0;  // mm/s along the glass

// Reach `pose` in a straight line, `seconds` after the previous waypoint.
struct Waypoint {
    TipPose pose;
    double seconds;
};

// Moves the tip along the stylus axis until its height changes by `dz`. A straight-down
// move with a tilted stylus would drag the tip across the glass; this pushes straight in.
TipPose along_stylus(const TipPose& p, double dz);

// IK every 20 ms along each straight segment, eased in and out. False, with the reason,
// if any point is unreachable or out of range, the elbow would flip, or it is too fast.
// A first waypoint with `seconds` 0 is timed from the current pose at `approach_speed`.
bool plan_path(const Calibration& cal, const Pose& present, std::vector<Waypoint> waypoints,
               double approach_speed, Recording* out, std::string* why);

int execute_path(Arm& robot, const Calibration& cal, const Pose& present, const Recording& path,
                 double deepest_goal, bool release, bool yes, int acceleration);

// Plans and runs waypoints, refusing before torque goes on if any check fails.
int run_path(const std::string& port_name, const std::vector<Waypoint>& waypoints, bool release,
             bool yes, double approach_speed = 40.0, int acceleration = kAcceleration);

void print_reason(std::string why);
double lowest_z(const std::vector<Waypoint>& waypoints);

// Building blocks. Each continues from wherever the path left the tip.
void travel_to(std::vector<Waypoint>* path, const TipPose& pose);
void add_pause(std::vector<Waypoint>* path, double seconds);
void add_tap(std::vector<Waypoint>* path, const TipPose& surface, double depth, double press,
             double drop_speed);   // `surface` is the touch point on the glass
void add_swipe(std::vector<Waypoint>* path, const TipPose& from, const TipPose& to, double depth,
               double speed);      // drops at `from`, drags to `to` without lifting, lifts

}  // namespace arm
