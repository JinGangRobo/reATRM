// Simulation bootstrap (MuJoCo-only backend) for the rmcs_core hardware models.
//
// The simulation subsystem is designed to support exactly one backend: MuJoCo
// (RMCS_SIM_HAS_MUJOCO). When rmcs_core is built with MuJoCo available, this
// component installs a CBoard transport factory so that every hardware-model
// CBoard constructed afterwards (see bringup config) attaches to a MuJoCo
// transport that streams DJI feedback + IMU from the physics engine. Hardware
// models are not modified at all - the seam is
// librmcs::client::CBoard / CBoardTransport (see cboard_transport.hpp).

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
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
// Planar pose (x, y, yaw) of the mobile base's free joint, if the MuJoCo model
// has one (mobile base). Returns false when no free joint / engine is present.
bool engine_get_base_pose(double& x, double& y, double& yaw);
} // namespace rmcs_core::simulation
#endif

namespace rmcs_core::simulation {

namespace {

// Single transport-factory entry point: the simulation supports only the MuJoCo
// backend, so every board key (usb_pid) is handed straight to
// create_mujoco_transport. When rmcs_core was built without MuJoCo there is no
// backend at all; the factory returns nullptr and the CBoard constructor throws
// a clear error instead of silently touching real USB hardware.
librmcs::client::CBoardTransport* dispatch_transport_factory(int32_t usb_pid) {
#if defined(RMCS_SIM_HAS_MUJOCO)
    auto* transport = create_mujoco_transport(usb_pid);
    if (transport == nullptr) {
        RCLCPP_FATAL(
            rclcpp::get_logger("Sim"),
            "dispatch_transport_factory: MuJoCo transport creation failed for pid %d", usb_pid);
    }
    return transport;
#else
    RCLCPP_FATAL(
        rclcpp::get_logger("Sim"),
        "SimulationBootstrap: simulation is MuJoCo-only, but rmcs_core was built without "
        "MuJoCo (mujoco_vendor not found)");
    return nullptr;
#endif
}

// ---- binding-table parsing helpers ----------------------------------------
// The board/motor topology is config-driven (see sim_common.hpp):
//   boards: "pid|dbus|gyro_sensor|acc_sensor"
//   motors: "pid|can_bus|fb_can_id|kind|reduction|sign|zero|command_id|actuator"
std::vector<std::string> split_pipe(const std::string& s) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const size_t end = s.find('|', start);
        out.push_back(s.substr(start, end == std::string::npos ? std::string::npos : end - start));
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return out;
}

std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
        ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
        --e;
    return s.substr(b, e - b);
}

// "123", "-5", "0x1FF" -> int (base detected from the 0x prefix).
int parse_int(const std::string& raw) {
    std::string s = trim(raw);
    if (s.empty())
        return 0;
    int base = 10;
    size_t pos = 0;
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        pos = 2;
    }
    return static_cast<int>(std::strtol(s.c_str() + pos, nullptr, base));
}

double parse_double(const std::string& raw) {
    const std::string s = trim(raw);
    return s.empty() ? 0.0 : std::strtod(s.c_str(), nullptr);
}

MotorKind parse_motor_kind(const std::string& raw) {
    const std::string s = trim(raw);
    if (s == "GM6020_VOLTAGE" || s == "GM6020V")
        return MotorKind::GM6020_VOLTAGE;
    if (s == "M3508")
        return MotorKind::M3508;
    if (s == "M2006")
        return MotorKind::M2006;
    if (s == "J4310")
        return MotorKind::J4310;
    return MotorKind::GM6020;
}

} // namespace

// Simulation bootstrap: when listed FIRST in the bringup config `components`,
// it is constructed before any hardware model and installs the MuJoCo CBoard
// transport factory so that every subsequent CBoard construction runs against
// the physics engine.
//
// Parameters (under the component instance name):
//   model_file:    MJCF for the MuJoCo backend; "" or a bare name like
//                  "pid_tune_tool.xml" auto-resolves to
//                  <share>/rmcs_core/mjcf/<name>.xml (default mini_infantry.xml)
//   gui:           open the native MuJoCo (GLFW) viewer (MuJoCo builds only)
//   remote_switch_left/right, remote_channel0..3: simulated DR16 remote state
//   boards/motors: config-driven board/motor topology (see sim_common.hpp)
class SimulationBootstrap final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimulationBootstrap()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        auto& config = sim_global_config();
        if (!has_parameter("model_file"))
            declare_parameter("model_file", "");
        get_parameter("model_file", config.model_path);
        if (config.model_path.empty())
            config.model_path = "mini_infantry.xml"; // per-robot default
        if (config.model_path.find('/') == std::string::npos) {
            // Bare MJCF name -> the MJCF installed with this package.
            config.model_path = ament_index_cpp::get_package_share_directory("rmcs_core") + "/mjcf/"
                              + config.model_path;
        }

        // Optional native MuJoCo viewer window (MuJoCo backend only; ignored
        // unless built with RMCS_SIM_HAS_GUI).
        if (!has_parameter("gui"))
            declare_parameter("gui", false);
        get_parameter("gui", config.gui);

        // Engine-level kinematic ground geometry. Keep equal to the
        // omni_wheel_controller chassis params in the robot sim config.
        if (!has_parameter("ground_wheel_radius"))
            declare_parameter("ground_wheel_radius", config.ground_wheel_radius);
        get_parameter("ground_wheel_radius", config.ground_wheel_radius);
        if (!has_parameter("ground_radius_x"))
            declare_parameter("ground_radius_x", config.ground_radius_x);
        get_parameter("ground_radius_x", config.ground_radius_x);
        if (!has_parameter("ground_radius_y"))
            declare_parameter("ground_radius_y", config.ground_radius_y);
        get_parameter("ground_radius_y", config.ground_radius_y);

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

        // Config-driven board topology binding (see sim_common.hpp SimBoardConfig):
        //   boards: "pid|dbus|gyro_sensor|acc_sensor"
        //   motors: "pid|can_bus|fb_can_id|kind|reduction|sign|zero|command_id|actuator"
        if (!has_parameter("boards"))
            declare_parameter("boards", std::vector<std::string>{});
        std::vector<std::string> board_rows;
        get_parameter("boards", board_rows);
        if (!has_parameter("motors"))
            declare_parameter("motors", std::vector<std::string>{});
        std::vector<std::string> motor_rows;
        get_parameter("motors", motor_rows);

        for (const auto& row : board_rows) {
            const auto t = split_pipe(row);
            if (t.size() < 4)
                continue;
            SimBoardConfig b;
            b.pid = parse_int(t[0]);
            b.dbus = parse_int(t[1]) != 0;
            b.gyro_sensor = trim(t[2]);
            b.acc_sensor = trim(t[3]);
            config.boards.push_back(std::move(b));
        }
        for (const auto& row : motor_rows) {
            const auto t = split_pipe(row);
            if (t.size() < 9)
                continue;
            SimMotorSpec m;
            const int pid = parse_int(t[0]);
            m.can_bus = parse_int(t[1]);
            m.can_id = parse_int(t[2]);
            m.kind = parse_motor_kind(t[3]);
            m.reduction = parse_double(t[4]);
            m.sign = parse_double(t[5]);
            m.zero = parse_int(t[6]);
            m.command_id = parse_int(t[7]);
            m.actuator = trim(t[8]);
            for (auto& b : config.boards) {
                if (b.pid == pid) {
                    b.motors.push_back(std::move(m));
                    break;
                }
            }
        }
        for (const auto& b : config.boards) {
            RCLCPP_INFO(
                rclcpp::get_logger("Sim"),
                "SimulationBootstrap: board pid=%d dbus=%d imu=%s/%s motors=%zu", b.pid,
                b.dbus ? 1 : 0, b.gyro_sensor.c_str(), b.acc_sensor.c_str(), b.motors.size());
        }

        // Stand-in for the (non-simulated) chassis power controller: a generous,
        // constant power budget so OmniWheelController's torque constraint never
        // throttles the simulated wheels.
        register_output("/chassis/control_power_limit", chassis_control_power_limit_, 120.0);

        // Optional mobile-base planar pose observability (NaN when the MJCF has
        // no free joint, e.g. the fixed bench model).
        register_output("/sim/base/pose/x", sim_base_x_, nan_);
        register_output("/sim/base/pose/y", sim_base_y_, nan_);
        register_output("/sim/base/pose/yaw", sim_base_yaw_, nan_);

        librmcs::client::CBoard::set_transport_factory(&dispatch_transport_factory);
        RCLCPP_INFO(
            rclcpp::get_logger("Sim"),
            "SimulationBootstrap: MuJoCo transport factory installed (model=%s "
            "dbus_sw=(%d,%d) gui=%d)",
            config.model_path.c_str(), config.remote.switch_right.load(std::memory_order::relaxed),
            config.remote.switch_left.load(std::memory_order::relaxed), config.gui ? 1 : 0);
    }

    void update() override {
        *chassis_control_power_limit_ = 120.0;
#if defined(RMCS_SIM_HAS_MUJOCO)
        double x = nan_, y = nan_, yaw = nan_;
        if (engine_get_base_pose(x, y, yaw)) {
            *sim_base_x_ = x;
            *sim_base_y_ = y;
            *sim_base_yaw_ = yaw;
        }
#endif
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    OutputInterface<double> chassis_control_power_limit_;
    OutputInterface<double> sim_base_x_;
    OutputInterface<double> sim_base_y_;
    OutputInterface<double> sim_base_yaw_;
};

} // namespace rmcs_core::simulation

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::simulation::SimulationBootstrap, rmcs_executor::Component)
