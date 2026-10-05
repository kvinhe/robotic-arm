#include "sim/sim.h"

#include "bus/connection.h"
#include "kinematics/kinematics.h"
#include "motion/arm.h"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

namespace sim {
namespace {

using Clock = std::chrono::steady_clock;

constexpr const char* kJoints[4] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"};
constexpr double kBaseAxisX = 0.0388353;   // m: where the robot frame's origin sits in the model
constexpr double kMaxSpeed = 3000;         // steps/s: the servo's own limit when goal speed is 0

uint16_t u16(const uint8_t* reg) { return reg[0] | (reg[1] << 8); }
int signed_offset(uint16_t v) { return (v & 0x800) ? -static_cast<int>(v & 0x7FF) : v & 0x7FF; }

std::vector<uint8_t> status(int id, const std::vector<uint8_t>& params) {
    std::vector<uint8_t> r{0xFF, 0xFF, static_cast<uint8_t>(id), static_cast<uint8_t>(params.size() + 2), 0};
    r.insert(r.end(), params.begin(), params.end());
    unsigned sum = 0;
    for (size_t i = 2; i < r.size(); ++i) sum += r[i];
    r.push_back(static_cast<uint8_t>(~sum & 0xFF));
    return r;
}

// The arm and phone in MuJoCo, with four simulated STS3215 servos answering bus packets.
class World {
public:
    ~World() {
        mj_deleteData(data_);
        mj_deleteModel(model_);
    }

    bool load(const std::string& scene, std::string* error);
    std::vector<uint8_t> handle(const std::vector<uint8_t>& packet);

    // Steps the physics up to the wall clock (at most half a second at a time).
    void advance() {
        std::lock_guard<std::mutex> lock(mutex_);
        step_until_now();
    }

    // Ideal servos: count 2048 at every joint's zero, positive the model's way.
    arm::Calibration calibration() const {
        arm::Calibration cal;
        for (int j = 0; j < 4; ++j) {
            const double* range = model_->jnt_range + 2 * mj_name2id(model_, mjOBJ_JOINT, kJoints[j]);
            cal.joint[j] = {2048, 1, static_cast<int>(std::lround(2048 + range[0] * arm::kCountsPerRad)),
                            static_cast<int>(std::lround(2048 + range[1] * arm::kCountsPerRad))};
        }
        return cal;
    }

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    std::mutex mutex_;

private:
    struct Servo {
        uint8_t reg[70] = {};
        double setpoint = 0;   // rad: moves toward the goal within the servo's limits
        double velocity = 0;
        bool torque = false;
    };

    void step_until_now();
    void step_servos(double dt);
    void watch_contacts();

    std::array<Servo, 4> servos_;
    std::array<int, 4> qpos_{};
    std::array<double, 4> max_force_{};
    int stylus_ = -1, phone_ = -1, tip_ = -1, screen_ = -1;
    Clock::time_point clock_;

    bool armed_ = false;   // touches count once the stylus is clear of the glass under power
    bool touching_ = false;
    double touch_start_ = 0, last_contact_ = 0;
    std::array<double, 2> touch_from_{}, touch_to_{};
    int touches_ = 0;
};

bool World::load(const std::string& scene, std::string* error) {
    char message[1000] = "";
    model_ = mj_loadXML(scene.c_str(), nullptr, message, sizeof(message));
    if (!model_) {
        *error = "cannot load " + scene + ": " + message;
        return false;
    }
    data_ = mj_makeData(model_);
    for (int j = 0; j < 4; ++j) {
        qpos_[j] = model_->jnt_qposadr[mj_name2id(model_, mjOBJ_JOINT, kJoints[j])];
        max_force_[j] = model_->actuator_forcerange[2 * j + 1];
    }
    stylus_ = mj_name2id(model_, mjOBJ_GEOM, "stylus");
    phone_ = mj_name2id(model_, mjOBJ_GEOM, "phone_body");
    tip_ = mj_name2id(model_, mjOBJ_SITE, "stylus_tip");
    screen_ = mj_name2id(model_, mjOBJ_SITE, "screen_centre");

    // Start with the stylus resting on the middle of the screen, as if placed there by hand.
    mj_forward(model_, data_);
    const mjtNum* c = data_->site_xpos + 3 * screen_;
    arm::Joints q;
    std::string why;
    if (!arm::inverse({(c[0] - kBaseAxisX) * 1000, c[1] * 1000, c[2] * 1000, -M_PI / 2}, &q, &why)) {
        *error = "the arm cannot reach the phone in the scene: " + why;
        return false;
    }
    const double angles[4] = {q.pan, q.shoulder, q.elbow, q.wrist};
    for (int j = 0; j < 4; ++j) {
        data_->qpos[qpos_[j]] = data_->ctrl[j] = servos_[j].setpoint = angles[j];
        uint8_t* reg = servos_[j].reg;
        reg[5] = static_cast<uint8_t>(j + 1);   // ID
        reg[55] = 1;                            // EEPROM locked
        reg[62] = 121;                          // 12.1 V
        reg[63] = 30;                           // 30 C
        reg[48] = 1000 & 0xFF;                  // torque limit 100 %
        reg[49] = 1000 >> 8;
    }
    mj_forward(model_, data_);
    clock_ = Clock::now();
    return true;
}

std::vector<uint8_t> World::handle(const std::vector<uint8_t>& p) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (p.size() < 6 || p[0] != 0xFF || p[1] != 0xFF || p[2] < 1 || p[2] > 4) return {};
    const int id = p[2], instruction = p[4];
    Servo& s = servos_[id - 1];
    step_until_now();   // so the reply reflects "now"
    if (instruction == 0x01) return status(id, {});   // ping

    const int reg = p[5];
    const int count = instruction == 0x02 ? p[6] : static_cast<int>(p.size()) - 7;
    if (reg + count > static_cast<int>(sizeof(s.reg))) return {};
    if (instruction == 0x02) {   // read
        if (reg == 56) {   // present position, after the servo's position offset
            const int raw = static_cast<int>(std::lround(2048 + data_->qpos[qpos_[id - 1]] * arm::kCountsPerRad));
            const int present = ((raw - signed_offset(u16(s.reg + 31))) % 4096 + 4096) % 4096;
            s.reg[56] = present & 0xFF;
            s.reg[57] = static_cast<uint8_t>(present >> 8);
        }
        return status(id, std::vector<uint8_t>(s.reg + reg, s.reg + reg + count));
    }
    if (instruction == 0x03) {   // write
        std::copy(p.begin() + 6, p.end() - 1, s.reg + reg);
        if (reg == 40) {
            s.torque = s.reg[40] != 0;
            s.setpoint = data_->qpos[qpos_[id - 1]];   // the goal starts where the joint is
            s.velocity = 0;
        }
        if (reg == 48) {
            const double limit = max_force_[id - 1] * u16(s.reg + 48) / 1000.0;
            model_->actuator_forcerange[2 * (id - 1)] = -limit;
            model_->actuator_forcerange[2 * (id - 1) + 1] = limit;
        }
        return status(id, {});
    }
    return {};
}

void World::step_until_now() {
    const auto now = Clock::now();
    double behind = std::min(0.5, std::chrono::duration<double>(now - clock_).count());
    clock_ = now;
    for (; behind > 0; behind -= model_->opt.timestep) {
        step_servos(model_->opt.timestep);
        mj_step(model_, data_);
        watch_contacts();
    }
}

// Each servo moves its setpoint toward the goal within its speed and acceleration limits,
// and the position actuator follows the setpoint, so lag and gravity sag show up as on the
// real arm. With torque off the joint stays where it was, as if held by hand.
void World::step_servos(double dt) {
    for (int j = 0; j < 4; ++j) {
        Servo& s = servos_[j];
        if (s.torque) {
            const double goal = (u16(s.reg + 42) + signed_offset(u16(s.reg + 31)) - 2048) / arm::kCountsPerRad;
            const double speed = (u16(s.reg + 46) ? u16(s.reg + 46) : kMaxSpeed) / arm::kCountsPerRad;
            const double accel = (s.reg[41] ? s.reg[41] * 100.0 : 1e6) / arm::kCountsPerRad;
            const double error = goal - s.setpoint;
            const double wanted = std::copysign(std::min(speed, std::sqrt(2 * accel * std::abs(error))), error);
            s.velocity += std::clamp(wanted - s.velocity, -accel * dt, accel * dt);
            if (std::abs(s.velocity * dt) >= std::abs(error)) {
                s.setpoint = goal;
                s.velocity = 0;
            } else {
                s.setpoint += s.velocity * dt;
            }
        }
        data_->ctrl[j] = s.setpoint;
    }
}

// Logs each touch of the stylus on the phone, in mm left of and above the middle of the
// screen ("up" is toward the Dynamic Island).
void World::watch_contacts() {
    bool contact = false;
    for (int i = 0; i < data_->ncon; ++i) {
        const mjContact& c = data_->contact[i];
        contact |= (c.geom1 == stylus_ && c.geom2 == phone_) || (c.geom1 == phone_ && c.geom2 == stylus_);
    }
    const mjtNum* tip = data_->site_xpos + 3 * tip_;
    const mjtNum* centre = data_->site_xpos + 3 * screen_;
    if (!armed_) {
        armed_ = tip[2] - centre[2] > 0.002 &&
                 std::any_of(servos_.begin(), servos_.end(), [](const Servo& s) { return s.torque; });
        return;
    }
    const std::array<double, 2> at = {(tip[0] - centre[0]) * 1000, -(tip[1] - centre[1]) * 1000};

    // A touch ends after 30 ms without contact, so a bounce does not count as a new touch.
    if (contact) last_contact_ = data_->time;
    const bool touching = contact || (touching_ && data_->time - last_contact_ < 0.03);
    if (touching && !touching_) {
        touch_start_ = data_->time;
        touch_from_ = at;
    }
    if (contact) touch_to_ = at;
    if (!touching && touching_) {
        const double seconds = last_contact_ - touch_start_;
        if (std::hypot(touch_to_[0] - touch_from_[0], touch_to_[1] - touch_from_[1]) < 3.0) {
            std::printf("  [sim] touch %d: tap at %.1f left, %.1f up (%.2f s)\n", ++touches_,
                        touch_from_[0], touch_from_[1], seconds);
        } else {
            std::printf("  [sim] touch %d: drag from %.1f left, %.1f up to %.1f left, %.1f up (%.2f s)\n",
                        ++touches_, touch_from_[0], touch_from_[1], touch_to_[0], touch_to_[1], seconds);
        }
        std::fflush(stdout);
    }
    touching_ = touching;
}

// What the commands get instead of a serial port: packets go to the world, replies come back.
class SimulatedBus : public Transport {
public:
    explicit SimulatedBus(World& world) : world_(world) {}

    bool write(const uint8_t* data, size_t length) override {
        rx_ = world_.handle(std::vector<uint8_t>(data, data + length));
        return true;
    }
    size_t read(uint8_t* buffer, size_t length) override {
        const size_t n = std::min(length, rx_.size());
        std::copy(rx_.begin(), rx_.begin() + static_cast<long>(n), buffer);
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(n));
        return n;
    }
    void flush() override { rx_.clear(); }
    const std::string& last_error() const override { return error_; }

private:
    World& world_;
    std::vector<uint8_t> rx_;
    std::string error_;
};

// The window. GLFW callbacks are plain functions, so the camera they move lives here.
mjModel* g_model = nullptr;
mjvCamera g_camera;
double g_last_x = 0, g_last_y = 0;

void on_cursor(GLFWwindow* window, double x, double y) {
    const double dx = x - g_last_x, dy = y - g_last_y;
    g_last_x = x;
    g_last_y = y;
    int width = 0, height = 0;
    glfwGetWindowSize(window, &width, &height);
    const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (left || right) {
        mjv_moveCamera(g_model, right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V, dx / height, dy / height, &g_camera);
    }
}

void on_scroll(GLFWwindow*, double, double y) { mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0, -0.05 * y, &g_camera); }

// Draws the world until the window is closed. macOS only lets the main thread do this.
void view(World& world, const std::atomic<bool>& finished) {
    if (!glfwInit()) {
        std::printf("cannot open a window; running without one\n");
        while (!finished) world.advance();
        return;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "servo_tool --sim", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    g_model = world.model_;
    mjvOption options;
    mjvScene scene;
    mjrContext context;
    mjv_defaultFreeCamera(g_model, &g_camera);
    mjv_defaultOption(&options);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(g_model, &scene, 2000);
    mjr_makeContext(g_model, &context, mjFONTSCALE_150);
    glfwSetCursorPosCallback(window, on_cursor);
    glfwSetScrollCallback(window, on_scroll);

    while (!glfwWindowShouldClose(window)) {
        world.advance();   // keeps the physics running while the command waits
        {
            std::lock_guard<std::mutex> lock(world.mutex_);
            mjv_updateScene(g_model, world.data_, &options, nullptr, &g_camera, mjCAT_ALL, &scene);
        }
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjr_render(viewport, &scene, &context);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                    finished ? "Done -- close the window to exit" : "Drag: rotate   Right-drag: move   Scroll: zoom",
                    nullptr, &context);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    mjv_freeScene(&scene);
    mjr_freeContext(&context);
    glfwDestroyWindow(window);
    glfwTerminate();
}

}  // namespace

int run(const std::string& scene, bool headless, const std::function<int()>& command) {
    World world;
    std::string error;
    if (!world.load(scene, &error)) {
        std::printf("%s\n", error.c_str());
        return 1;
    }
    use_simulated_bus([&world] { return std::make_unique<SimulatedBus>(world); });
    arm::use_calibration(world.calibration());
    std::printf("Simulation: stylus resting on the middle of the phone screen.\n");
    if (headless) return command();

    // The command runs on its own thread while the main thread draws.
    int result = 0;
    std::atomic<bool> finished{false};
    std::thread worker([&] {
        result = command();
        finished = true;
    });
    view(world, finished);
    if (!finished) arm::g_stop = 1;   // window closed mid-motion: stop and hold, like Ctrl+C
    worker.join();
    return result;
}

}  // namespace sim
