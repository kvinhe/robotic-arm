#include "commands/commands.h"
#include "kinematics/kinematics.h"
#include "bus/connection.h"
#include "bus/serial_port.h"
#include "bus/sts3215.h"
#ifdef WITH_MUJOCO
#include "sim/sim.h"
#endif

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void print_usage() {
    std::cout <<
        "usage: servo_tool [--sim | --sim-headless | PORT] [--baud N] <command> [args]\n"
        "  PORT is optional when exactly one USB serial adapter is plugged in\n"
        "  --sim runs the command on a simulated arm and phone (MuJoCo) instead\n"
        "\n"
        "commands:\n"
        "  scan                 ping every ID from 0 to 253 and list the ones that answer\n"
        "  ping <id>            check a single servo\n"
        "  setid <old> <new>    change a servo's ID (one servo on the bus, please)\n"
        "  pos <id>             read the servo's present position (0-4095)\n"
        "  read <id> <reg> [n]  read register <reg>, n = 1 or 2 bytes (default 1)\n"
        "  write <id> <reg> <v> [n] [--force]   write <v> to <reg>, n = 1 or 2 bytes\n"
        "  assign [count]       interactive: give `count` servos the IDs 1..count\n"
        "  torque-off           disable torque on joints 1-4 (the arm goes limp: support it)\n"
        "  record [--seconds N] [--out FILE]   torque off; record the arm as you move it\n"
        "  replay [--seconds N] [--in FILE] [--reverse] [--yes]   play a recording back\n"
        "  calibrate            guided setup: zero pose, directions, safe ranges -> calibration.txt\n"
        "  where                where the arm is now: counts, joint angles, stylus tip\n"
        "  move-to <x> <y> <z> <pitch> [--speed MM_S] [--release]   straight-line move (mm, deg)\n"
        "  tap <x> <y> <z> [--depth MM] [--count N]      quick tap; z = screen surface\n"
        "  swipe <x1> <y1> <x2> <y2> <z> [--speed MM_S]  drag along the screen without lifting\n"
        "  unlock <model> <pin> [--check]   measure the stylus on the screen centre, then wake,\n"
        "                       swipe up and type the pin (e.g. unlock iphone15 1111)\n"
        "  fk <pan> <shoulder> <elbow> <wrist>   joint angles (deg) -> stylus tip (no arm needed)\n"
        "  ik <x> <y> <z> <pitch>                stylus tip (mm, deg) -> joint angles\n"
        "\n"
        "registers are decimal or 0x hex. Useful ones:\n"
        "  40 torque enable (0/1)   55 EEPROM lock (0/1)   62 voltage (0.1V)\n"
        "  63 temperature (C)       42 goal position (2B)  56 present position (2B)\n"
        "\n"
        "writes to registers below 40 are EEPROM (persistent, write-limited) and\n"
        "need --force. Writing register 6 (baud rate) can make a servo unreachable.\n"
        "\n"
        "examples: servo_tool scan\n"
        "          servo_tool record --seconds 15\n"
        "          servo_tool /dev/cu.usbmodem1101 replay --seconds 8\n";
}

bool parse_id(const char* text, uint8_t* out) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value < 0 || value > 253) {
        std::cout << "'" << text << "' is not a valid servo ID (0-253)\n";
        return false;
    }
    *out = static_cast<uint8_t>(value);
    return true;
}

// Register addresses and values accept decimal or 0x hex.
bool parse_u8(const char* text, const char* what, uint8_t* out) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 0);
    if (end == text || *end != '\0' || value < 0 || value > 255) {
        std::cout << "'" << text << "' is not a valid " << what << " (0-255)\n";
        return false;
    }
    *out = static_cast<uint8_t>(value);
    return true;
}

// Registers below this are EEPROM: persistent, write-limited, and include the ID and
// baud rate (a wrong baud rate makes the servo unreachable). Writes need --force.
constexpr uint8_t kFirstSramRegister = 40;

void wait_for_enter(const std::string& prompt) {
    std::cout << prompt << std::flush;
    std::string ignored;
    std::getline(std::cin, ignored);
}

constexpr double kDegPerRad = 57.29577951308232;

// `servo_tool fk` / `servo_tool ik`: try the kinematics from the command line. Pure math.
int run_kinematics(const std::string& command, int argc, char** argv) {
    if (argc != 4) {
        std::cout << (command == "fk" ? "usage: servo_tool fk <pan> <shoulder> <elbow> <wrist>  (deg)\n"
                                      : "usage: servo_tool ik <x> <y> <z> <pitch>  (mm, deg)\n");
        return 1;
    }
    double v[4];
    for (int i = 0; i < 4; ++i) v[i] = std::atof(argv[i]);

    arm::Joints q;
    if (command == "fk") {
        q = {v[0] / kDegPerRad, v[1] / kDegPerRad, v[2] / kDegPerRad, v[3] / kDegPerRad};
    } else {
        std::string why;
        if (!arm::inverse({v[0], v[1], v[2], v[3] / kDegPerRad}, &q, &why)) {
            std::cout << "no solution: " << why << "\n";
            return 1;
        }
    }
    const arm::TipPose tip = arm::forward(q);
    std::printf("joints (deg): pan %.2f  shoulder %.2f  elbow %.2f  wrist %.2f\n",
                q.pan * kDegPerRad, q.shoulder * kDegPerRad, q.elbow * kDegPerRad,
                q.wrist * kDegPerRad);
    std::printf("tip (mm):     x %.1f  y %.1f  z %.1f  pitch %.1f deg\n", tip.x, tip.y, tip.z,
                tip.pitch * kDegPerRad);
    return 0;
}

// One servo at a time: new servos are all ID 1, so two on the bus would both answer.
int run_assign(sts3215::Bus& bus, int count) {
    std::cout << "\nAssigning IDs 1 to " << count << ".\n"
              << "Plug in exactly ONE servo at a time -- unplug the others.\n";

    for (int target = 1; target <= count; ++target) {
        std::cout << "\n--- servo " << target << " of " << count << " ---\n";
        wait_for_enter("Connect the servo that should become ID " +
                       std::to_string(target) + ", then press Enter: ");

        std::cout << "scanning the bus...\n";
        const std::vector<uint8_t> found = bus.scan();

        if (found.empty()) {
            std::cout << "No servo answered. Check power, the data cable, and the baud rate,\n"
                         "then run the tool again.\n";
            return 1;
        }
        if (found.size() > 1) {
            std::cout << "Found " << found.size()
                      << " servos on the bus. Leave only one connected and try again.\n";
            return 1;
        }

        const uint8_t current = found[0];
        std::cout << "found servo with ID " << static_cast<int>(current) << "\n";

        if (current == target) {
            std::cout << "already ID " << target << ", nothing to do.\n";
            continue;
        }

        if (!bus.set_id(current, static_cast<uint8_t>(target))) {
            std::cout << "failed: " << bus.last_error() << "\n";
            return 1;
        }
        std::cout << "servo is now ID " << target << "\n";
    }

    std::cout << "\nAll done. Plug every servo back in and run `scan` to confirm\n"
                 "you see IDs 1 to " << count << ".\n";
    return 0;
}

}  // namespace

int run_tool(int argc, char** argv) {
    int arg = 1;
    std::string requested_port;
    if (arg < argc && looks_like_port(argv[arg])) {
        requested_port = argv[arg++];
    }

    uint32_t baud_rate = sts3215::kDefaultBaudRate;
    if (arg < argc && std::string(argv[arg]) == "--baud") {
        if (arg + 1 >= argc) {
            std::cout << "--baud needs a number\n";
            return 1;
        }
        baud_rate = static_cast<uint32_t>(std::strtoul(argv[arg + 1], nullptr, 10));
        arg += 2;
    }
    if (arg >= argc) {
        print_usage();
        return 1;
    }

    const std::string command = argv[arg++];
    const int remaining = argc - arg;

    if (command == "fk" || command == "ik") {
        return run_kinematics(command, argc - arg, argv + arg);
    }
    const std::vector<std::string> args(argv + arg, argv + argc);
    if (command == "record") return arm::record_command(requested_port, args);
    if (command == "replay") return arm::replay_command(requested_port, args);
    if (command == "calibrate") return arm::calibrate_command(requested_port, args);
    if (command == "where") return arm::where_command(requested_port, args);
    if (command == "move-to") return arm::move_command(requested_port, args);
    if (command == "tap") return arm::tap_command(requested_port, args);
    if (command == "swipe") return arm::swipe_command(requested_port, args);
    if (command == "unlock") return arm::unlock_command(requested_port, args);

    Connection link;
    std::string open_error;
    if (!open_connection(requested_port, baud_rate, &link, &open_error)) {
        std::cout << open_error << "\n";
        return 1;
    }
    std::cout << "opened " << link.name << " at " << baud_rate << " baud\n";

    sts3215::Bus bus(*link.port);

    if (command == "scan") {
        std::cout << "scanning IDs 0-253, this takes a few seconds...\n";
        const std::vector<uint8_t> found = bus.scan();
        if (found.empty()) {
            std::cout << "no servos found\n";
            return 1;
        }
        std::cout << "found " << found.size() << " servo(s):";
        for (uint8_t id : found) {
            std::cout << " " << static_cast<int>(id);
        }
        std::cout << "\n";
        return 0;
    }

    if (command == "ping") {
        uint8_t id = 0;
        if (remaining < 1 || !parse_id(argv[arg], &id)) {
            print_usage();
            return 1;
        }
        if (!bus.ping(id)) {
            std::cout << "servo " << static_cast<int>(id) << " did not answer ("
                      << bus.last_error() << ")\n";
            return 1;
        }
        std::cout << "servo " << static_cast<int>(id) << " is alive\n";
        return 0;
    }

    if (command == "setid") {
        uint8_t old_id = 0;
        uint8_t new_id = 0;
        if (remaining < 2 || !parse_id(argv[arg], &old_id) || !parse_id(argv[arg + 1], &new_id)) {
            print_usage();
            return 1;
        }
        if (!bus.set_id(old_id, new_id)) {
            std::cout << "failed: " << bus.last_error() << "\n";
            return 1;
        }
        std::cout << "servo " << static_cast<int>(old_id) << " is now servo "
                  << static_cast<int>(new_id) << "\n";
        return 0;
    }

    if (command == "pos") {
        uint8_t id = 0;
        if (remaining < 1 || !parse_id(argv[arg], &id)) {
            print_usage();
            return 1;
        }
        uint16_t position = 0;
        if (!bus.read_u16(id, sts3215::kRegPresentPosition, &position)) {
            std::cout << "failed: " << bus.last_error() << "\n";
            return 1;
        }
        // The STS3215 reports position as 0-4095 over its 360 degree range.
        std::cout << "servo " << static_cast<int>(id) << " position: " << position << " ("
                  << (position * 360.0 / 4096.0) << " degrees)\n";
        return 0;
    }

    if (command == "read") {
        uint8_t id = 0;
        uint8_t reg = 0;
        if (remaining < 2 || !parse_id(argv[arg], &id) ||
            !parse_u8(argv[arg + 1], "register", &reg)) {
            print_usage();
            return 1;
        }

        int width = 1;
        if (remaining >= 3) {
            width = std::atoi(argv[arg + 2]);
            if (width != 1 && width != 2) {
                std::cout << "byte count must be 1 or 2\n";
                return 1;
            }
        }

        if (width == 1) {
            uint8_t value = 0;
            if (!bus.read_u8(id, reg, &value)) {
                std::cout << "failed: " << bus.last_error() << "\n";
                return 1;
            }
            std::cout << "servo " << static_cast<int>(id) << " reg " << static_cast<int>(reg)
                      << " = " << static_cast<int>(value) << "\n";
        } else {
            uint16_t value = 0;
            if (!bus.read_u16(id, reg, &value)) {
                std::cout << "failed: " << bus.last_error() << "\n";
                return 1;
            }
            std::cout << "servo " << static_cast<int>(id) << " reg " << static_cast<int>(reg)
                      << " = " << value << "\n";
        }
        return 0;
    }

    if (command == "write") {
        bool force = false;
        std::vector<const char*> args;
        for (int i = arg; i < argc; ++i) {
            if (std::string(argv[i]) == "--force") {
                force = true;
            } else {
                args.push_back(argv[i]);
            }
        }

        uint8_t id = 0;
        uint8_t reg = 0;
        if (args.size() < 3 || !parse_id(args[0], &id) || !parse_u8(args[1], "register", &reg)) {
            print_usage();
            return 1;
        }

        int width = 1;
        if (args.size() >= 4) {
            width = std::atoi(args[3]);
            if (width != 1 && width != 2) {
                std::cout << "byte count must be 1 or 2\n";
                return 1;
            }
        }

        char* end = nullptr;
        const long raw = std::strtol(args[2], &end, 0);
        const long limit = (width == 1) ? 255 : 65535;
        if (end == args[2] || *end != '\0' || raw < 0 || raw > limit) {
            std::cout << "'" << args[2] << "' is not a valid " << width << "-byte value (0-"
                      << limit << ")\n";
            return 1;
        }

        if (reg < kFirstSramRegister && !force) {
            std::cout << "register " << static_cast<int>(reg)
                      << " is EEPROM: the write persists across power cycles, and register 6\n"
                         "(baud rate) or 5 (ID) can make the servo unreachable. Re-run with\n"
                         "--force if that is really what you want, or use `setid` to change an ID.\n";
            return 1;
        }

        const bool ok = (width == 1)
                            ? bus.write_u8(id, reg, static_cast<uint8_t>(raw))
                            : bus.write_u16(id, reg, static_cast<uint16_t>(raw));
        if (!ok) {
            std::cout << "failed: " << bus.last_error() << "\n";
            return 1;
        }
        std::cout << "servo " << static_cast<int>(id) << " reg " << static_cast<int>(reg)
                  << " <- " << raw << "\n";
        return 0;
    }

    if (command == "torque-off") {
        // Recovery after `replay` holds. Carries on past a failed joint so the rest let go.
        bool ok = true;
        for (uint8_t id = 1; id <= 4; ++id) {
            if (bus.write_u8(id, sts3215::kRegTorqueEnable, 0)) {
                std::cout << "servo " << static_cast<int>(id) << " torque off\n";
            } else {
                std::cout << "servo " << static_cast<int>(id) << " failed: " << bus.last_error() << "\n";
                ok = false;
            }
        }
        return ok ? 0 : 1;
    }

    if (command == "assign") {
        int count = 4;
        if (remaining >= 1) {
            count = std::atoi(argv[arg]);
        }
        if (count < 1 || count > 253) {
            std::cout << "count must be between 1 and 253\n";
            return 1;
        }
        return run_assign(bus, count);
    }

    std::cout << "unknown command: " << command << "\n\n";
    print_usage();
    return 1;
}

int main(int argc, char** argv) {
#ifdef WITH_MUJOCO
    const std::string first = argc > 1 ? argv[1] : "";
    if (first == "--sim" || first == "--sim-headless") {
        argv[1] = argv[0];   // drop the flag, so the command sees its usual arguments
        return sim::run(SIM_SCENE, first == "--sim-headless", [&] { return run_tool(argc - 1, argv + 1); });
    }
#endif
    return run_tool(argc, argv);
}
