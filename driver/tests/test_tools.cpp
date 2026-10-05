#include "doctest/doctest.h"
#include "motion/recording.h"
#include "bus/serial_port.h"

#include <cstdio>
#include <filesystem>
#include <fstream>

using arm::Pose;
using arm::Recording;

namespace {

// Base sweeps 2000 -> 2400 -> 2000 over 2 s; the other joints sit still.
Recording sweep() {
    return Recording({
        {0.0, {2000, 1700, 3000, 1600}},
        {1.0, {2400, 1700, 3000, 1600}},
        {2.0, {2000, 1700, 3000, 1600}},
    });
}

// A one-way move: base 2000 -> 2400 in 1 s. Ends away from where it started.
Recording one_way() {
    return Recording({
        {0.0, {2000, 1700, 3000, 1600}},
        {0.5, {2200, 1750, 3000, 1600}},
        {1.0, {2400, 1800, 3000, 1600}},
    });
}

std::string temp_path(const char* name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

}  // namespace

TEST_CASE("at() interpolates between samples and clamps outside the recording") {
    const Recording r = sweep();
    CHECK(r.at(0.5)[0] == 2200);
    CHECK(r.at(1.5)[0] == 2200);
    CHECK(r.at(1.0)[0] == 2400);
    CHECK(r.at(-3.0) == r.first());
    CHECK(r.at(99.0) == r.last());
}

TEST_CASE("min and max are per joint over the whole recording") {
    const Recording r = one_way();
    CHECK(r.min() == Pose{2000, 1700, 3000, 1600});
    CHECK(r.max() == Pose{2400, 1800, 3000, 1600});
}

TEST_CASE("reversed plays end to start over the same duration") {
    const Recording r = one_way().reversed();
    CHECK(r.duration() == doctest::Approx(1.0));
    CHECK(r.first() == one_way().last());
    CHECK(r.last() == one_way().first());
    CHECK(r.at(0.25)[0] == 2300);
}

TEST_CASE("retraced goes out and back, ending where it began") {
    const Recording r = one_way().retraced();
    CHECK(r.duration() == doctest::Approx(2.0));
    CHECK(r.first() == r.last());
    CHECK(r.at(1.0) == one_way().last());      // the turnaround
    CHECK(r.at(1.5) == one_way().at(0.5));      // coming back along the same path
    for (size_t i = 1; i < r.samples().size(); ++i) {
        CHECK(r.samples()[i].t >= r.samples()[i - 1].t);
    }
}

TEST_CASE("scaled_to keeps the path and changes only the timing") {
    const Recording slow = sweep().scaled_to(8.0);
    CHECK(slow.duration() == doctest::Approx(8.0));
    CHECK(slow.at(4.0) == sweep().at(1.0));
    CHECK(slow.at(2.0) == sweep().at(0.5));
}

TEST_CASE("peak_speed reports the fastest joint, and scales with the timing") {
    const Recording r = sweep();   // 400 counts per second on the base
    CHECK(r.peak_speed() == doctest::Approx(400.0));
    CHECK(r.scaled_to(1.0).peak_speed() == doctest::Approx(800.0));
    CHECK(r.scaled_to(4.0).peak_speed() == doctest::Approx(200.0));
}

TEST_CASE("peak_speed ignores single-sample jitter shorter than the window") {
    // A one-sample 60-count blip, 10 ms wide, would read as 6000 counts/s sample to sample.
    const Recording r({
        {0.00, {2000, 1700, 3000, 1600}},
        {0.01, {2060, 1700, 3000, 1600}},
        {0.02, {2000, 1700, 3000, 1600}},
        {0.20, {2000, 1700, 3000, 1600}},
    });
    CHECK(r.peak_speed(0.1) < 1000.0);
}

TEST_CASE("clamp and max_abs_difference") {
    const Pose lo{100, 100, 100, 100};
    const Pose hi{200, 200, 200, 200};
    CHECK(arm::clamp({50, 150, 250, 200}, lo, hi) == Pose{100, 150, 200, 200});
    CHECK(arm::max_abs_difference({0, 10, 0, 0}, {5, -20, 0, 0}) == 30);
}

TEST_CASE("save then load round-trips a recording") {
    const std::string path = temp_path("arm_recording_roundtrip.txt");
    std::string error;
    REQUIRE(one_way().save(path, &error));

    Recording loaded;
    REQUIRE(Recording::load(path, &loaded, &error));
    REQUIRE(loaded.samples().size() == one_way().samples().size());
    CHECK(loaded.first() == one_way().first());
    CHECK(loaded.last() == one_way().last());
    CHECK(loaded.duration() == doctest::Approx(1.0));
    std::remove(path.c_str());
}

TEST_CASE("load rejects files that are not valid recordings") {
    const std::string path = temp_path("arm_recording_bad.txt");
    std::string error;
    Recording r;

    SUBCASE("missing header, e.g. the old shell-script format") {
        std::ofstream(path) << "# t_seconds base shoulder elbow wrist\n0 2304 1720 2983 1597\n";
        CHECK_FALSE(Recording::load(path, &r, &error));
        CHECK(error.find("not an arm recording") != std::string::npos);
    }
    SUBCASE("position out of range") {
        std::ofstream(path) << "# arm recording v1\n0.0 2304 1720 2983 1597\n0.1 2304 9999 2983 1597\n";
        CHECK_FALSE(Recording::load(path, &r, &error));
        CHECK(error.find("outside 0-4095") != std::string::npos);
    }
    SUBCASE("time going backwards") {
        std::ofstream(path) << "# arm recording v1\n0.5 2304 1720 2983 1597\n0.1 2304 1720 2983 1597\n";
        CHECK_FALSE(Recording::load(path, &r, &error));
    }
    SUBCASE("wrong number of joints") {
        std::ofstream(path) << "# arm recording v1\n0.0 2304 1720 2983\n0.1 2304 1720 2983\n";
        CHECK_FALSE(Recording::load(path, &r, &error));
    }
    SUBCASE("no motion") {
        std::ofstream(path) << "# arm recording v1\n0.0 2304 1720 2983 1597\n";
        CHECK_FALSE(Recording::load(path, &r, &error));
    }
    std::remove(path.c_str());
}

TEST_CASE("load shifts a recording to start at t = 0") {
    const std::string path = temp_path("arm_recording_offset.txt");
    std::ofstream(path) << "# arm recording v1\n5.0 2000 1700 3000 1600\n6.0 2400 1700 3000 1600\n";
    Recording r;
    std::string error;
    REQUIRE(Recording::load(path, &r, &error));
    CHECK(r.samples().front().t == doctest::Approx(0.0));
    CHECK(r.duration() == doctest::Approx(1.0));
    std::remove(path.c_str());
}

TEST_CASE("looks_like_port recognises port names") {
    CHECK(looks_like_port("/dev/cu.usbmodem5B8E1134991"));
    CHECK(looks_like_port("/dev/ttyUSB0"));
}

TEST_CASE("looks_like_port does not mistake commands or flags for ports") {
    CHECK_FALSE(looks_like_port("scan"));
    CHECK_FALSE(looks_like_port("pos"));
    CHECK_FALSE(looks_like_port("--baud"));
    CHECK_FALSE(looks_like_port("--seconds"));
    CHECK_FALSE(looks_like_port("recording.txt"));
}

TEST_CASE("choose_adapter picks the only adapter") {
    std::string port;
    std::string error;
    REQUIRE(choose_adapter({"/dev/cu.usbmodem1101"}, &port, &error));
    CHECK(port == "/dev/cu.usbmodem1101");
}

TEST_CASE("choose_adapter refuses to guess") {
    std::string port;
    std::string error;

    SUBCASE("nothing plugged in") {
        CHECK_FALSE(choose_adapter({}, &port, &error));
        CHECK(error.find("no USB serial adapter") != std::string::npos);
    }
    SUBCASE("two adapters: both are named in the error") {
        CHECK_FALSE(choose_adapter({"/dev/cu.usbmodem1", "/dev/cu.usbserial2"}, &port, &error));
        CHECK(error.find("/dev/cu.usbmodem1") != std::string::npos);
        CHECK(error.find("/dev/cu.usbserial2") != std::string::npos);
    }
    CHECK(port.empty());
}
