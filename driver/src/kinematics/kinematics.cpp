#include "kinematics/kinematics.h"

#include <cmath>

namespace arm {
namespace {

constexpr double kPi = 3.14159265358979323846;

Vec2 rotate(Vec2 v, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {v.r * c - v.z * s, v.r * s + v.z * c};
}

double angle_of(Vec2 v) { return std::atan2(v.z, v.r); }
double length(Vec2 v) { return std::hypot(v.r, v.z); }

// Into (-pi, pi], so equivalent angles compare equal.
double wrap(double a) {
    a = std::fmod(a + kPi, 2 * kPi);
    return (a <= 0 ? a + 2 * kPi : a) - kPi;
}

bool within_limits(const Joints& q, const Geometry& g) {
    return q.pan >= g.min.pan && q.pan <= g.max.pan &&
           q.shoulder >= g.min.shoulder && q.shoulder <= g.max.shoulder &&
           q.elbow >= g.min.elbow && q.elbow <= g.max.elbow &&
           q.wrist >= g.min.wrist && q.wrist <= g.max.wrist;
}

}  // namespace

// The shoulder, elbow and wrist axes all point along +y, so a positive joint angle turns
// everything after it clockwise in the (r, z) plane. Each link's rotation from its zero
// pose is therefore minus the sum of the joint angles before it. The pan axis points down,
// so positive pan also turns the arm clockwise seen from above.
TipPose forward(const Joints& q, const Geometry& g) {
    const double upper_rot = -q.shoulder;
    const double fore_rot = upper_rot - q.elbow;
    const double tool_rot = fore_rot - q.wrist;

    const Vec2 u = rotate(g.upper_arm, upper_rot);
    const Vec2 f = rotate(g.forearm, fore_rot);
    const Vec2 t = rotate(g.tool, tool_rot);
    const double r = g.shoulder.r + u.r + f.r + t.r;
    const double z = g.shoulder.z + u.z + f.z + t.z;

    // Pitch in the arm's own plane. If the arm is folded back over the base (r < 0), "away
    // from the base" is the opposite way, which mirrors the angle.
    const double plane_pitch = tool_rot + angle_of(g.tool);
    const double yaw = -q.pan;
    return {r * std::cos(yaw), r * std::sin(yaw), z, wrap(r >= 0 ? plane_pitch : kPi - plane_pitch)};
}

// Undo each link's zero-pose angle, then invert the "minus the sum of the joints before
// it" relation from forward().
Joints joints_for(double yaw, double upper_dir, double fore_dir, double tool_dir,
                  const Geometry& g) {
    const double upper_rot = upper_dir - angle_of(g.upper_arm);
    const double fore_rot = fore_dir - angle_of(g.forearm);
    const double tool_rot = tool_dir - angle_of(g.tool);
    return {wrap(-yaw), wrap(-upper_rot), wrap(upper_rot - fore_rot), wrap(fore_rot - tool_rot)};
}

namespace {

enum class Solve { kOk, kOutOfReach, kPastLimit };

// IK in the arm's vertical plane, once the base direction (`yaw`) is chosen. Coordinates
// are (r, z): r = distance out from the base axis along that direction, z = height.
//
//              elbow
//               /\
//           a  /  \  b            a = upper arm, b = forearm (fixed lengths)
//             /    \              d = shoulder to wrist (depends on the target)
//    shoulder ------- wrist
//                d     \
//                       \  stylus, at angle `pitch`
//                        tip  <- the target
Solve solve_plane(double yaw, double r, double z, double pitch, const Geometry& g, Joints* out) {
    // 1. Wrist. The stylus angle is given, so the wrist sits exactly one stylus length back
    //    from the tip along that angle. This turns a 3-link problem into a 2-link one.
    const double tool_len = length(g.tool);
    const Vec2 wrist{r - tool_len * std::cos(pitch), z - tool_len * std::sin(pitch)};

    // 2. Elbow bend, from the law of cosines. The triangle shoulder-elbow-wrist has all
    //    three sides known: d^2 = a^2 + b^2 + 2ab*cos(bend), where `bend` is how far the
    //    forearm turns away from the upper arm's direction (0 = arm straight).
    const Vec2 to_wrist{wrist.r - g.shoulder.r, wrist.z - g.shoulder.z};
    const double a = length(g.upper_arm);
    const double b = length(g.forearm);
    const double d = length(to_wrist);
    const double cos_bend = (d * d - a * a - b * b) / (2 * a * b);
    // |cos| > 1 means no such triangle: the wrist is farther than a + b (too far) or
    // closer than |a - b| (too close to fold into).
    if (cos_bend > 1.0 || cos_bend < -1.0) return Solve::kOutOfReach;

    // 3. Two mirror-image triangles reach the same wrist: elbow above the shoulder-wrist
    //    line (negative bend, how the SO-101 normally sits) or below it. Try elbow-up first
    //    and keep the first one inside every joint limit.
    for (const double side : {-1.0, 1.0}) {
        const double bend = side * std::acos(cos_bend);

        // 4. Link directions (angles in the plane, 0 = horizontal and outward).
        //    The wrist lies at  a*dir(upper) + b*dir(upper + bend)  from the shoulder, which
        //    points `atan2(b sin bend, a + b cos bend)` beyond the upper arm. So the upper arm
        //    points at the wrist minus that angle, and the forearm adds the bend on top.
        const double upper_dir =
            angle_of(to_wrist) - std::atan2(b * std::sin(bend), a + b * std::cos(bend));
        const double fore_dir = upper_dir + bend;

        // 5. Directions -> motor angles (see joints_for), then check the joint limits.
        const Joints q = joints_for(yaw, upper_dir, fore_dir, pitch, g);
        if (within_limits(q, g)) {
            *out = q;
            return Solve::kOk;
        }
    }
    return Solve::kPastLimit;
}

}  // namespace

// Closed-form IK: four numbers in (x, y, z, pitch), four joint angles out. There are as
// many unknowns as constraints, so it is solved directly with geometry -- no iteration.
bool inverse(const TipPose& target, Joints* out, std::string* why, const Geometry& g) {
    // Never plan into the table.
    if (target.z < 0) {
        *why = "below the table (z < 0)";
        return false;
    }

    // Base. Shoulder, elbow and wrist all bend in one vertical plane, and the tip stays in
    // the plane through the base axis. So seen from above, the base just points at the
    // target, and everything after that is a 2D problem in (r, z).
    const double yaw = std::atan2(target.y, target.x);   // direction to the target
    const double r = std::hypot(target.x, target.y);      // distance out from the base axis

    // Usually the arm faces the target. A target behind the base can also be reached by
    // turning the base the other way and folding the arm back over the top: then r is
    // negative and "away from the base" flips, which mirrors the pitch.
    const Solve facing = solve_plane(yaw, r, target.z, target.pitch, g, out);
    if (facing == Solve::kOk) return true;
    const Solve folded = solve_plane(yaw + kPi, -r, target.z, kPi - target.pitch, g, out);
    if (folded == Solve::kOk) return true;

    // Report the more useful reason: reachable but blocked by a limit beats out of reach.
    const bool reachable = facing == Solve::kPastLimit || folded == Solve::kPastLimit;
    *why = reachable ? "needs a joint past its limit" : "out of reach";
    return false;
}

}  // namespace arm
