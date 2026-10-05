#include "doctest/doctest.h"
#include "kinematics/kinematics.h"

#include <cmath>
#include <random>

using arm::Joints;
using arm::TipPose;

namespace {

constexpr double kPi = 3.14159265358979323846;

void check_same_pose(const TipPose& a, const TipPose& b, double mm = 1e-6) {
    CHECK(std::abs(a.x - b.x) < mm);
    CHECK(std::abs(a.y - b.y) < mm);
    CHECK(std::abs(a.z - b.z) < mm);
    CHECK(std::abs(std::remainder(a.pitch - b.pitch, 2 * kPi)) < 1e-9);
}

}  // namespace

TEST_CASE("forward matches the SO-101 URDF") {
    // Tip positions computed from the full 3D URDF chain (so101_new_calib.urdf), with the
    // stock gripper tip and wrist_roll at 0 -- so this uses the gripper, not our stylus.
    // Tolerance covers the tip's 0.01 mm sideways offset, which the planar model ignores.
    arm::Geometry urdf;
    urdf.tool = {159.227, -7.9};
    struct Case {
        Joints q;
        double x, y, z;
    };
    const Case cases[] = {
        {{0.00, 0.00, 0.00, 0.00}, 352.526, -0.009, 226.470},
        {{0.50, 0.00, 0.00, 0.00}, 309.367, -169.018, 226.469},
        {{0.00, 0.60, -0.40, 0.30}, 386.262, -0.009, 88.723},
        {{-1.00, -0.80, 1.20, -0.90}, 129.115, 201.066, 236.776},
        {{1.20, 1.00, 0.50, 1.40}, -0.558, 1.406, -10.757},
    };
    for (const Case& c : cases) {
        const TipPose p = arm::forward(c.q, urdf);
        CHECK(std::abs(p.x - c.x) < 0.05);
        CHECK(std::abs(p.y - c.y) < 0.05);
        CHECK(std::abs(p.z - c.z) < 0.05);
    }
}

TEST_CASE("inverse undoes forward over random reachable poses") {
    const arm::Geometry g;
    std::mt19937 rng(42);
    auto uniform = [&](double lo, double hi) { return std::uniform_real_distribution<>(lo, hi)(rng); };

    int tested = 0;
    for (int i = 0; i < 5000; ++i) {
        const Joints q{uniform(g.min.pan, g.max.pan), uniform(g.min.shoulder, g.max.shoulder),
                       uniform(g.min.elbow, g.max.elbow), uniform(g.min.wrist, g.max.wrist)};
        const TipPose pose = arm::forward(q);
        if (pose.z < 0) continue;   // below the table: IK rightly refuses these

        Joints solved;
        std::string why;
        // q itself is a valid answer, so IK must find one (maybe the other elbow side).
        REQUIRE_MESSAGE(arm::inverse(pose, &solved, &why), why);
        check_same_pose(arm::forward(solved), pose);
        ++tested;
    }
    CHECK(tested > 3000);
}

TEST_CASE("inverse finds the zero pose from its own tip position") {
    Joints q;
    std::string why;
    REQUIRE(arm::inverse(arm::forward(Joints{}), &q, &why));
    CHECK(std::abs(q.pan) < 1e-9);
    CHECK(std::abs(q.shoulder) < 1e-9);
    CHECK(std::abs(q.elbow) < 1e-9);
    CHECK(std::abs(q.wrist) < 1e-9);
}

TEST_CASE("inverse can point the stylus straight down at a tablet") {
    const TipPose tap{250.0, 40.0, 15.0, -kPi / 2};
    Joints q;
    std::string why;
    REQUIRE_MESSAGE(arm::inverse(tap, &q, &why), why);
    check_same_pose(arm::forward(q), tap);
}

TEST_CASE("inverse refuses what the arm cannot or must not do") {
    Joints q;
    std::string why;

    SUBCASE("too far away") {
        CHECK_FALSE(arm::inverse({1000.0, 0.0, 100.0, 0.0}, &q, &why));
        CHECK(why == "out of reach");
    }
    SUBCASE("below the table") {
        CHECK_FALSE(arm::inverse({250.0, 0.0, -5.0, -kPi / 2}, &q, &why));
        CHECK(why == "below the table (z < 0)");
    }
    SUBCASE("behind the arm, past the base's rotation limit") {
        CHECK_FALSE(arm::inverse({-250.0, 0.0, 150.0, 0.0}, &q, &why));
        CHECK(why == "needs a joint past its limit");
    }
}
