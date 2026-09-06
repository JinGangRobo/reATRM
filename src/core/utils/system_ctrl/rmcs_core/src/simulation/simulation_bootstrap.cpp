// Simulation bootstrap (MuJoCo-only backend) for the rmcs_core hardware models.
//
// The simulation subsystem is designed to support exactly one backend: MuJoCo
// (RMCS_SIM_HAS_MUJOCO). When rmcs_core is built with MuJoCo available, this
// component installs a CBoard transport factory so that every hardware-model
// CBoard constructed afterwards (see bringup config) attaches to a MuJoCo
// transport that streams DJI feedback + IMU from the physics engine. Hardware
// models are not modified at all - the seam is
// librmcs::client::CBoard / CBoardTransport (see cboard_transport.hpp).

#include <cstdint>
#include <string>

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

} // namespace

// Simulation bootstrap: when listed FIRST in the bringup config `components`,
// it is constructed before any hardware model and installs the MuJoCo CBoard
// transport factory so that every subsequent CBoard construction runs against
// the physics engine.
//
// Parameters (under the component instance name):
//   model_file:    MJCF path for the MuJoCo backend ("" -> auto-resolve to
//                  <share>/rmcs_core/mjcf/<robot>.xml)
//   gui:           open the native MuJoCo (GLFW) viewer (MuJoCo builds only)
//   remote_switch_left/right, remote_channel0..3: simulated DR16 remote state
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
        if (config.model_path.empty()) {
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
            "SimulationBootstrap: MuJoCo transport factory installed (model=%s "
            "dbus_sw=(%d,%d) gui=%d)",
            config.model_path.c_str(), config.remote.switch_right.load(std::memory_order::relaxed),
            config.remote.switch_left.load(std::memory_order::relaxed), config.gui ? 1 : 0);
    }

    void update() override {}
};

} // namespace rmcs_core::simulation

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::simulation::SimulationBootstrap, rmcs_executor::Component)
