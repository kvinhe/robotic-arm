#pragma once

#include <string>

namespace arm {

// Joint angles in radians. All zero = upper arm up, forearm and stylus pointing forward.
struct Joints {
    double pan = 0, shoulder = 0, elbow = 0, wrist = 0;
};

// Where the stylus tip is, in mm: x forward, y left, z up from the bottom of the base.
// `pitch` is the stylus angle in radians: 0 = pointing forward, -pi/2 = straight down.
struct TipPose {
    double x = 0, y = 0, z = 0, pitch = 0;
};

// A point in the arm's side view: r = out from the base, z = up.
struct Vec2 {
    double r, z;
};

// Arm dimensions in mm, from the SO-101 URDF.
struct Geometry {
    Vec2 shoulder{30.399, 116.6};   // base -> shoulder joint
    Vec2 upper_arm{28.0, 112.57};   // shoulder -> elbow
    Vec2 forearm{134.9, 5.2};       // elbow -> wrist
    Vec2 tool{80.0, 0.0};           // wrist -> stylus tip
    Joints min{-1.91986, -1.74533, -1.69, -1.65806};
    Joints max{1.91986, 1.74533, 1.69, 1.65806};
};

// Forward kinematics: joint angles -> stylus tip.
TipPose forward(const Joints& q, const Geometry& g = Geometry{});

// Inverse kinematics: stylus tip -> joint angles. False, with the reason in `why`, if the
// tip is out of reach, past a joint limit, or below the table.
bool inverse(const TipPose& target, Joints* out, std::string* why,
             const Geometry& g = Geometry{});

// Joint angles that point the base at `yaw` and each link in the given direction.
Joints joints_for(double yaw, double upper_dir, double fore_dir, double tool_dir,
                  const Geometry& g = Geometry{});

}  // namespace arm
