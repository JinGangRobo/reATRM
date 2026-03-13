#include <chrono>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "communication.hpp"
#include "fields.hpp"
#include "rmcs_msgs/game_stage.hpp"

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

        state_data_update_cycles_ = get_parameter("state_data_update_cycles").as_int();
        state_data_sending_host_ = get_parameter("state_data_sending_host").as_string();
        state_data_sending_port_ = get_parameter("state_data_sending_port").as_int();
        pilot_data_receiving_host_ = get_parameter("pilot_data_receiving_host").as_string();
        pilot_data_receiving_port_ = get_parameter("pilot_data_receiving_port").as_int();

        state_data_limiter_.reset(state_data_update_cycles_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_);
        register_input(
            "/gimbal/control_bullet_allowance/limited_by_heat", control_bullet_allowance_);
        register_input("/referee/chassis/current_hp", current_hp_);
        register_input("/referee/game/stage", game_stage_);
        register_input("/referee/game/remain_time", stage_remain_time_);
        register_input("/referee/robots/rmul_rfid", rfid_state_);
        register_input("/referee/game/center_area", center_area_status_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_tracking_);
        register_input("/gimbal/auto_aim/target_position", auto_aim_target_position_);

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
        if (state_data_limiter_.tick()) {
            StateData state_data;
            state_data.gimbal_faceing[0] = 1.0; // Dummy data.
            state_data.gimbal_faceing[1] = 0.0;
            state_data.gimbal_faceing[2] = 0.0;
            state_data.autopilot_enabled = (*switch_left_ != rmcs_msgs::Switch::UNKNOWN
                                            && *switch_left_ != rmcs_msgs::Switch::DOWN)
                                        && *switch_right_ == rmcs_msgs::Switch::UP;

            state_data.current_hp = *current_hp_;
            state_data.game_state = static_cast<uint8_t>(*game_stage_);
            state_data.state_remain_time = *stage_remain_time_;
            state_data.rfid_state = *rfid_state_;
            state_data.center_area_state = *center_area_status_;
            state_data.projectile_allowance = *control_bullet_allowance_;

            state_data.auto_aim_tracking = !auto_aim_tracking_->isZero();
            state_data.target_position[0] = auto_aim_target_position_->x();
            state_data.target_position[1] = auto_aim_target_position_->y();
            state_data.target_position[2] = auto_aim_target_position_->z();

            switch (*rotary_knob_switch_) {
            case rmcs_msgs::Switch::UP: state_data.desired_nav_mode = NavMode::SLAM; break;
            case rmcs_msgs::Switch::MIDDLE: state_data.desired_nav_mode = NavMode::UNKNOWN; break;
            case rmcs_msgs::Switch::DOWN: state_data.desired_nav_mode = NavMode::RELOCATION; break;
            default: state_data.desired_nav_mode = NavMode::UNKNOWN; break;
            }

            std::string sending_errmsg;
            if (!communication_.sendStateData(
                    state_data, state_data_sending_host_, state_data_sending_port_,
                    &sending_errmsg)) {
                RCLCPP_ERROR(
                    get_logger(), "Failed to send state data. reason: %s", sending_errmsg.c_str());
            }
            state_data_limiter_.reset(state_data_update_cycles_);
        }

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
        if (pilotData.pilot_valid) {
            *auto_pilot_velocity_ << pilotData.chassis_vel[0], pilotData.chassis_vel[1],
                pilotData.chassis_vel[2], 1.0;

            last_valid_pilot_time_ = Clock::now();
        }
    }

private:
    Communication communication_;
    Clock::time_point last_valid_pilot_time_;

    rmcs_utility::TickTimer state_data_limiter_;

    uint16_t state_data_update_cycles_;
    std::string state_data_sending_host_;
    uint16_t state_data_sending_port_;
    std::string pilot_data_receiving_host_;
    uint16_t pilot_data_receiving_port_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;
    InputInterface<int64_t> control_bullet_allowance_;
    InputInterface<double> current_hp_;
    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<double> stage_remain_time_;
    InputInterface<uint8_t> rfid_state_;
    InputInterface<uint8_t> center_area_status_;
    InputInterface<Eigen::Vector3d> auto_aim_tracking_;
    InputInterface<Eigen::Vector3d> auto_aim_target_position_;

    OutputInterface<Eigen::Vector4d> auto_pilot_velocity_;
};
} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(autopilot::AutoPilotComponent, rmcs_executor::Component)
