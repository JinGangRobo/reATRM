// P0 simulation prototype: run rmcs_core hardware models against a fake
// "C board" transport (no MuJoCo yet) to validate the librmcs::client::CBoard
// transport seam. MiniInfantry only.
//
// The transport mimics the CAN feedback frames + IMU that a real ATRM C board
// would stream to the model, and consumes the CAN command frames the model
// pushes down. Hardware models are not modified at all.
//
// NOTE: this file is a prototype. The robot<->CAN topology below is derived from
// rmcs_core/src/hardware/model/mini_infantry.cpp + bringup config. In later
// phases it will be replaced by a MuJoCo-backed transport driven by mapping
// config files.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <librmcs/client/cboard.hpp>
#include <librmcs/client/cboard_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "sim_common.hpp"

#if defined(RMCS_SIM_HAS_MUJOCO)
namespace rmcs_core::simulation {
librmcs::client::CBoardTransport* create_mujoco_transport(int32_t usb_pid);
}
#endif

namespace rmcs_core::simulation {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr int kAngleMax = 8192; // DJI encoder counts per revolution

// Physically-plausible (but rough) DJI feedback encoder for a simulated load.
struct DjiSimMotor {
    std::string name;

    // Device configuration mirror (matches librmcs::device::DjiMotor::Config)
    double reduction = 1.0; // load-side reduction (motor turns per load turn)
    double sign = 1.0;      // -1 when reversed
    int zero = 0;           // encoder raw counts at load angle 0

    // DJI type constants (Nm per raw current unit == decode coefficient)
    double torque_per_raw = 0.0;

    // Simple load-side rigid-body dynamics
    double inertia = 0.01; // kg*m^2
    double damping = 0.05; // viscous N*m*s/rad

    // Simulated load-side state
    double angle = 0.0;    // rad (unwrapped)
    double velocity = 0.0; // rad/s

    int can_id = 0;        // feedback CAN id (e.g. 0x205)

    // Commanded raw current, written by the executor thread, read by sim thread
    std::atomic<int> command_raw{0};

    DjiSimMotor() = default;
    DjiSimMotor(const DjiSimMotor&) = delete;
    DjiSimMotor& operator=(const DjiSimMotor&) = delete;
    DjiSimMotor(DjiSimMotor&& other) noexcept
        : name(std::move(other.name))
        , reduction(other.reduction)
        , sign(other.sign)
        , zero(other.zero)
        , torque_per_raw(other.torque_per_raw)
        , inertia(other.inertia)
        , damping(other.damping)
        , angle(other.angle)
        , velocity(other.velocity)
        , can_id(other.can_id)
        , command_raw(other.command_raw.load()) {}
    DjiSimMotor& operator=(DjiSimMotor&& other) noexcept {
        if (this != &other) {
            name = std::move(other.name);
            reduction = other.reduction;
            sign = other.sign;
            zero = other.zero;
            torque_per_raw = other.torque_per_raw;
            inertia = other.inertia;
            damping = other.damping;
            angle = other.angle;
            velocity = other.velocity;
            can_id = other.can_id;
            command_raw.store(other.command_raw.load());
        }
        return *this;
    }
};

// Encode one DJI feedback frame (same byte layout as
// librmcs::device::DjiMotor::DjiMotorFeedback) so the device decodes exactly the
// simulated load angle/velocity.
uint64_t encode_dji_feedback(const DjiSimMotor& motor, int raw_current, int16_t velocity_raw) {
    const double counts = motor.angle * motor.reduction * kAngleMax / (2.0 * kPi) * motor.sign;
    double cmod = std::fmod(counts, kAngleMax);
    if (cmod < 0)
        cmod += kAngleMax;
    double raw_d = std::fmod(static_cast<double>(motor.zero) + cmod, kAngleMax);
    if (raw_d < 0)
        raw_d += kAngleMax;

    int angle_raw = static_cast<int>(std::lround(raw_d)) % kAngleMax;

    uint8_t bytes[8];
    bytes[0] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
    bytes[1] = static_cast<uint8_t>(angle_raw & 0xFF);
    bytes[2] = static_cast<uint8_t>((velocity_raw >> 8) & 0xFF);
    bytes[3] = static_cast<uint8_t>(velocity_raw & 0xFF);
    bytes[4] = static_cast<uint8_t>((raw_current >> 8) & 0xFF);
    bytes[5] = static_cast<uint8_t>(raw_current & 0xFF);
    bytes[6] = 30; // temperature ~30C
    bytes[7] = 0;  // unused

    uint64_t data = 0;
    std::memcpy(&data, bytes, sizeof(bytes));
    return data;
}

inline int16_t sign_extend_16(uint16_t v) { return static_cast<int16_t>(v); }

class FakeMiniTransport final : public librmcs::client::CBoardTransport {
public:
    explicit FakeMiniTransport(int32_t usb_pid)
        : pid_(usb_pid) {
        if (pid_ == 1) {
            // ---- Top board (gimbal / friction wheels) ----
            // M3508 (reduction 1): tc=0.3*187/3591, raw_max 16384, current_max 20
            add_motor("gimbal/left_friction", 0x201, 1.0, -1.0, 0, 0.002, 0.01, 1.9064e-5);
            add_motor("gimbal/right_friction", 0x202, 1.0, 1.0, 0, 0.002, 0.01, 1.9064e-5);
            // GM6020 (reduction 1): tc=0.741, raw_max 16384, current_max 3
            add_motor("gimbal/pitch", 0x205, 1.0, 1.0, 7556, 0.02, 0.05, 1.356e-4);
        } else if (pid_ == 2) {
            // ---- Bottom board (chassis / yaw / bullet feeder) ----
            // M3508 (reduction 268/17): tc=0.3*187/3591, raw_max 16384, current_max 20
            add_motor(
                "chassis/right_front_wheel", 0x201, 268.0 / 17.0, -1.0, 0, 0.3, 0.05, 3.005e-4);
            add_motor(
                "chassis/left_front_wheel", 0x202, 268.0 / 17.0, -1.0, 0, 0.3, 0.05, 3.005e-4);
            add_motor("chassis/left_back_wheel", 0x203, 268.0 / 17.0, -1.0, 0, 0.3, 0.05, 3.005e-4);
            add_motor(
                "chassis/right_back_wheel", 0x204, 268.0 / 17.0, -1.0, 0, 0.3, 0.05, 3.005e-4);
            // GM6020 (reduction 1)
            add_motor("gimbal/yaw", 0x206, 1.0, 1.0, 3606, 0.05, 0.1, 1.356e-4);
            // M2006 (reduction (33/27)*36): tc=0.18/36, raw_max 16384, current_max 10
            add_motor(
                "gimbal/bullet_feeder", 0x207, (33.0 / 27.0) * 36.0, -1.0, 0, 0.002, 0.01,
                1.3428e-4);
        } else {
            RCLCPP_ERROR(
                rclcpp::get_logger("Sim"), "FakeMiniTransport: unsupported board pid %d", usb_pid);
        }
    }

    void attach(librmcs::client::CBoard& board) override { board_ = &board; }

    void run() override {
        if (board_ == nullptr)
            return;

        RCLCPP_INFO(
            rclcpp::get_logger("Sim"), "FakeMiniTransport(board pid=%d): sim loop started", pid_);

        using namespace std::chrono_literals;
        constexpr double dt = 0.001; // 1 kHz, aligned with rmcs_executor control rate
        auto next = std::chrono::steady_clock::now();

        size_t tick = 0;
        while (!stop_.load(std::memory_order::relaxed)) {
            step(dt);

            if (++tick % 1000 == 0)
                log_status(tick);

            next += std::chrono::nanoseconds(static_cast<long>(1'000'000.0));
            std::this_thread::sleep_until(next);
        }

        RCLCPP_INFO(
            rclcpp::get_logger("Sim"), "FakeMiniTransport(board pid=%d): sim loop stopped", pid_);
    }

    void stop() override { stop_.store(true, std::memory_order::relaxed); }

    void can_transmission(
        uint8_t can_bus, uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
        bool is_remote_transmission, uint8_t can_data_length) override {
        (void)can_bus;
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
            return;

        // Command broadcast frames (see mini_infantry.cpp command_update):
        //   0x200 -> CAN ids 0x201..0x204,  0x1FF -> CAN ids 0x205..0x208
        uint32_t base = 0;
        if (can_id == 0x200)
            base = 0x201;
        else if (can_id == 0x1FF)
            base = 0x205;
        else
            return;

        uint8_t bytes[8];
        std::memcpy(bytes, &can_data, sizeof(bytes));
        for (uint32_t slot = 0; slot < 4; slot++) {
            uint16_t raw = static_cast<uint16_t>(
                (static_cast<uint16_t>(bytes[2 * slot]) << 8)
                | static_cast<uint16_t>(bytes[2 * slot + 1]));
            const uint32_t id = base + slot;
            for (auto& motor : motors_) {
                if (motor.can_id == static_cast<int>(id)) {
                    motor.command_raw.store(sign_extend_16(raw), std::memory_order::relaxed);
                    break;
                }
            }
        }
    }

    void uart1_transmission(const std::byte*, uint8_t) override {}
    void uart2_transmission(const std::byte*, uint8_t) override {}
    void dbus_transmission(const std::byte*, uint8_t) override {}
    void buzzer_transmission(uint8_t) override {}

private:
    void add_motor(
        const char* name, int can_id, double reduction, double sign, int zero, double inertia,
        double damping, double torque_per_raw) {
        DjiSimMotor motor;
        motor.name = name;
        motor.can_id = can_id;
        motor.reduction = reduction;
        motor.sign = sign;
        motor.zero = zero;
        motor.inertia = inertia;
        motor.damping = damping;
        motor.torque_per_raw = torque_per_raw;
        motors_.push_back(std::move(motor));
    }

    void step(double dt) {
        // Advance load-side dynamics from the last commanded current.
        for (auto& motor : motors_) {
            const int cmd = motor.command_raw.load(std::memory_order::relaxed);
            const double torque = static_cast<double>(cmd) * motor.torque_per_raw;
            const double accel = (torque - motor.damping * motor.velocity) / motor.inertia;
            motor.velocity += accel * dt;
            motor.angle += motor.velocity * dt;
        }

        // Stream feedback frames (keeps device watchdogs alive) + IMU.
        for (const auto& motor : motors_) {
            const int cmd = motor.command_raw.load(std::memory_order::relaxed);
            const double vel_raw_d =
                motor.sign * motor.velocity * motor.reduction * 60.0 / (2.0 * kPi);
            const int16_t vel_raw = static_cast<int16_t>(std::clamp(vel_raw_d, -32767.0, 32767.0));
            const uint64_t frame = encode_dji_feedback(motor, cmd, vel_raw);
            board_->can1_receive_callback(
                static_cast<uint32_t>(motor.can_id), frame, false, false, 8);
        }

        // IMU: level + stationary -> accelerometer +1g on Z, gyroscope ~0.
        board_->accelerometer_receive_callback(0, 0, 5461); // 32767/6 * 1g
        board_->gyroscope_receive_callback(0, 0, 0);

        // Inject the simulated DBUS (remote controller) into the top board.
        if (pid_ == 1) {
            std::byte dbus[18];
            make_dbus_frame(sim_global_config().remote, dbus);
            board_->dbus_receive_callback(dbus, 18);
        }
    }

    void log_status(size_t tick) {
        std::string parts;
        for (const auto& motor : motors_) {
            const int cmd = motor.command_raw.load(std::memory_order::relaxed);
            parts += " [";
            parts += motor.name;
            parts += "] ang=" + std::to_string(motor.angle)
                   + " vel=" + std::to_string(motor.velocity) + " cur=" + std::to_string(cmd);
        }
        RCLCPP_INFO(
            rclcpp::get_logger("Sim"), "tick %zu (board pid=%d):%s", tick, pid_, parts.c_str());
    }

    int32_t pid_ = 0;
    librmcs::client::CBoard* board_ = nullptr;
    std::atomic<bool> stop_{false};
    std::vector<DjiSimMotor> motors_;
};

librmcs::client::CBoardTransport* make_fake_mini_transport(int32_t usb_pid) {
    return new FakeMiniTransport(usb_pid);
}

librmcs::client::CBoardTransport* dispatch_transport_factory(int32_t usb_pid) {
    const auto& config = sim_global_config();
#if defined(RMCS_SIM_HAS_MUJOCO)
    if (config.backend == "mujoco") {
        auto* transport = create_mujoco_transport(usb_pid);
        if (transport != nullptr)
            return transport;
        RCLCPP_ERROR(
            rclcpp::get_logger("Sim"),
            "dispatch_transport_factory: MuJoCo transport creation failed for pid %d, "
            "falling back to fake",
            usb_pid);
    }
#endif
    return make_fake_mini_transport(usb_pid);
}

} // namespace

// Simulation bootstrap: when listed FIRST in the bringup config `components`,
// it is constructed before any hardware model and installs the CBoard transport
// factory so that every subsequent CBoard construction runs against the sim.
// Parameters (under the component instance name):
//   backend:     "fake" (default, dependency-free) | "mujoco"
//   model_file:  MJCF path used by the MuJoCo backend
class SimulationBootstrap final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimulationBootstrap()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        auto& config = sim_global_config();
        if (!has_parameter("backend"))
            declare_parameter("backend", "fake");
        get_parameter("backend", config.backend);
        if (!has_parameter("model_file"))
            declare_parameter("model_file", "");
        get_parameter("model_file", config.model_path);
        if (config.backend == "mujoco" && config.model_path.empty()) {
            // Default to the MJCF installed with this package:
            //   <share>/rmcs_core/mjcf/<robot>.xml
            config.model_path = ament_index_cpp::get_package_share_directory("rmcs_core")
                              + "/mjcf/mini_infantry.xml";
        }

        // Optional native MuJoCo viewer window (MuJoCo backend only; ignored
        // unless built with RMCS_SIM_HAS_GUI).
        if (!has_parameter("gui"))
            declare_parameter("gui", false);
        get_parameter("gui", config.gui);

        // Simulated remote control (DBUS) state. Switches are strings so the
        // config stays readable: up / middle / down.
        const auto parse_switch = [this](const std::string& name, const std::string& def) {
            if (!has_parameter(name))
                declare_parameter(name, def);
            std::string value;
            get_parameter(name, value);
            if (value == "up")
                return 1u;
            if (value == "middle")
                return 3u;
            if (value == "down")
                return 2u;
            RCLCPP_WARN(
                rclcpp::get_logger("Sim"), "SimulationBootstrap: bad switch value '%s' for '%s'",
                value.c_str(), name.c_str());
            return 0u; // UNKNOWN -> controllers stay disabled
        };
        config.remote.switch_right = parse_switch("remote_switch_right", "middle");
        config.remote.switch_left = parse_switch("remote_switch_left", "middle");
        const auto parse_channel = [this](const std::string& name, int def) {
            if (!has_parameter(name))
                declare_parameter(name, def);
            int value = def;
            get_parameter(name, value);
            return value;
        };
        config.remote.channel0 = parse_channel("remote_channel0", 1024);
        config.remote.channel1 = parse_channel("remote_channel1", 1024);
        config.remote.channel2 = parse_channel("remote_channel2", 1024);
        config.remote.channel3 = parse_channel("remote_channel3", 1024);

        librmcs::client::CBoard::set_transport_factory(&dispatch_transport_factory);
        RCLCPP_INFO(
            rclcpp::get_logger("Sim"),
            "SimulationBootstrap: transport factory installed (backend=%s model=%s "
            "dbus_sw=(%d,%d) gui=%d)",
            config.backend.c_str(), config.model_path.c_str(),
            config.remote.switch_right.load(std::memory_order::relaxed),
            config.remote.switch_left.load(std::memory_order::relaxed), config.gui ? 1 : 0);
    }

    void update() override {}
};

} // namespace rmcs_core::simulation

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::simulation::SimulationBootstrap, rmcs_executor::Component)
