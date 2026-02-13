#include "rmcs_executor/component.hpp"
#include <chrono>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

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

        register_output("/autopilot/chassis/velocity", auto_pilot_velocity_);

        std::string send_init_errmsg;
        if (!communication_.startSending(&send_init_errmsg)) {
            RCLCPP_FATAL(
                get_logger(), "Init pilot data sending failed. reason: %s",
                send_init_errmsg.c_str());
        }
        std::string recv_init_errmsg;
        if (!communication_.startReceiving(
                pilot_data_receiving_host_, pilot_data_receiving_port_,
                [this](const PilotData& data, const std::string err_msg) {
                    pilot_data_callback(data, err_msg);
                },
                &recv_init_errmsg)) {
            RCLCPP_FATAL(
                get_logger(), "Init pilot data receiving failed. reason: %s",
                recv_init_errmsg.c_str());
        }
    }

    ~AutoPilotComponent() { communication_.stop(); }

    void before_updating() override {}

    void update() override {
        // StateData state_data;
        // state_data.gimbal_faceing[0] = 1.0;
        // state_data.gimbal_faceing[1] = 0.0;
        // state_data.gimbal_faceing[2] = 0.0;

        // std::string sending_errmsg;
        // if (!communication_.sendStateData(
        //         state_data, state_data_sending_host_, state_data_sending_port_, &sending_errmsg))
        //         {
        //     RCLCPP_ERROR(
        //         get_logger(), "Failed to send state data. reason: %s", sending_errmsg.c_str());
        // }

        if (Clock::now() - last_valid_pilot_time_ > std::chrono::milliseconds(100)
            && (*auto_pilot_velocity_)[3] != 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "No valid pilot data received for 100ms, resetting velocity to zero.");
            *auto_pilot_velocity_ << 0.0, 0.0, 0.0, 0.0;
        }
    }

    void pilot_data_callback(const PilotData& pilotData, const std::string error_msg = "") {
        if (!error_msg.empty()) {
            RCLCPP_ERROR(get_logger(), "Error receiving pilot data: %s", error_msg.c_str());
            return;
        }

        *auto_pilot_velocity_ << pilotData.chassis_vel[0], pilotData.chassis_vel[1],
            pilotData.chassis_vel[2], 1.0;

        RCLCPP_INFO(get_logger(), "got autopilot data");

        last_valid_pilot_time_ = Clock::now();
    }

private:
    Communication communication_;
    Clock::time_point last_valid_pilot_time_;

    uint16_t state_data_update_rate_;
    std::string state_data_sending_host_;
    uint16_t state_data_sending_port_;
    std::string pilot_data_receiving_host_;
    uint16_t pilot_data_receiving_port_;

    OutputInterface<Eigen::Vector4d> auto_pilot_velocity_;
};
} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(autopilot::AutoPilotComponent, rmcs_executor::Component)
