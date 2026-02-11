#include "rclcpp/logging.hpp"
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

        if (!communication_.startReceiving(
                pilot_data_receiving_host_, pilot_data_receiving_port_,
                [this](const PilotData& data) { pilot_data_callback(data); })) {
            RCLCPP_FATAL(
                get_logger(), "Init pilot data receiving failed. reason: %s",
                communication_.getLastReceivingError().c_str());
        }
    }

    ~AutoPilotComponent() { communication_.stop(); }

    void before_updating() override {}

    void update() override {
        StateData state_data;
        state_data.gimbal_faceing[0] = 1.0;
        state_data.gimbal_faceing[1] = 0.0;
        state_data.gimbal_faceing[2] = 0.0;

        if (!communication_.sendStateData(
                state_data, state_data_sending_host_, state_data_sending_port_)) {
            RCLCPP_ERROR(
                get_logger(), "Failed to send state data. reason: %s",
                communication_.getLastSendingError().c_str());
        }
    }

    void pilot_data_callback(const PilotData& pilotData) {
        latest_pilot_data_ = pilotData;
        RCLCPP_INFO(
            get_logger(), "Received pilot data: %f,%f,%f", pilotData.chassis_vel[0],
            pilotData.chassis_vel[1], pilotData.chassis_vel[2]);
    }

private:
    Communication communication_;
    PilotData latest_pilot_data_;

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
