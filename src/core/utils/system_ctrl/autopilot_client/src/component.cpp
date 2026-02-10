#include "rclcpp/node_options.hpp"
#include <cstdint>
#include <rclcpp/node.hpp>

#include "rmcs_executor/component.hpp"
#include <rmcs_msgs/switch.hpp>

#include "communication.hpp"

namespace autopilot {
class AutoPilotComponent
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit AutoPilotComponent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , communication_() {
        RCLCPP_INFO(get_logger(), "AutoPilotComponent has been initialized.");

        state_data_update_rate_ = get_parameter("state_data_update_rate").as_int();
        state_data_sending_host_ = get_parameter("state_data_sending_host").as_string();
        state_data_sending_port_ = get_parameter("state_data_sending_port").as_int();
        pilot_data_receiving_host_ = get_parameter("pilot_data_receiving_host").as_string();
        pilot_data_receiving_port_ = get_parameter("pilot_data_receiving_port").as_int();

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        // May cause error here, need handle it in the future
        communication_.startReceiving(
            pilot_data_receiving_host_, pilot_data_receiving_port_,
            [this](const PilotData& data) { pilot_data_callback(data); });
    }

    ~AutoPilotComponent() { communication_.stop(); }

    void before_updating() override {}

    void update() override {
        // RCLCPP_INFO(get_logger(), "Update AutoPilotComponent");
        StateData state_data;
        state_data.gimbal_faceing[0] = 1.0;
        state_data.gimbal_faceing[1] = 0.0;
        state_data.gimbal_faceing[2] = 0.0;
        communication_.sendStateData(
            state_data, state_data_sending_host_, state_data_sending_port_);
    }

    void pilot_data_callback(const PilotData& pilotData) {
        RCLCPP_INFO(
            get_logger(), "Received pilot data: %f,%f,%f", pilotData.chassis_vel[0],
            pilotData.chassis_vel[1], pilotData.chassis_vel[2]);
    }

private:
    Communication communication_;

    uint16_t state_data_update_rate_;
    std::string state_data_sending_host_;
    uint16_t state_data_sending_port_;
    std::string pilot_data_receiving_host_;
    uint16_t pilot_data_receiving_port_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
};
} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(autopilot::AutoPilotComponent, rmcs_executor::Component)
