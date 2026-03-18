#include <cstdint>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/operate_mode.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

#include "referee/status/field.hpp"

namespace rmcs_core::controller::chassis {

class OperateLogic
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OperateLogic()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        get_parameter("comfort_translational_velocity", comfort_translational_velocity);
        get_parameter("comfort_angular_velocity", comfort_angular_velocity);

        register_input("/autopilot/chassis/velocity", auto_pilot_velocity_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/remote/mouse", mouse_);

        register_input("/referee/robots/rmul_rfid", robots_rfid_);
        register_input("/referee/robot/hurt", hurt_data_);
        register_input("/referee/game/stage", game_stage_);

        register_output("/chassis/translational_vmax", translational_velocity_max_);
        register_output("/chassis/angular_vmax", angular_velocity_max_);
        register_output("/chassis/operate_mode", mode_);
        register_output("/chassis/boost", boost_);
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto keyboard = *keyboard_;

        bool is_switch_invalid = (switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                              || (switch_left == Switch::DOWN && switch_right == Switch::DOWN);
        bool is_auto_pilot_active = auto_pilot_velocity_.ready();
        bool is_game_not_started =
            !game_stage_.ready()
            || (game_stage_.ready()
                && (*game_stage_ == GameStage::NOT_START || *game_stage_ == GameStage::UNKNOWN));

        if (is_switch_invalid || is_auto_pilot_active || is_game_not_started) {
            *mode_ = OperateMode::MANUAL;
            reset_all_controls();
            return;
        }
        if (!last_keyboard_.e && keyboard.e) {
            *mode_ = *mode_ == OperateMode::MANUAL ? OperateMode::ASSIST : OperateMode::MANUAL;
        }
        if (*mode_ == OperateMode::ASSIST)
            update_status();
        else
            reset_all_controls();

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;

        *translational_velocity_max_ = translational_velocity_max;
        *angular_velocity_max_ = angular_velocity_max;
    }

    void reset_all_controls() {
        translational_velocity_max = comfort_translational_velocity;
        angular_velocity_max = comfort_angular_velocity;
        *boost_ = false;
    }

private:
    void update_status() {
        auto mouse = *mouse_;
        if (mouse.left) {                                                            // 发射弹丸
            translational_velocity_max = comfort_translational_velocity * 0.8;
            angular_velocity_max = comfort_angular_velocity * 2.0;
            *boost_ = true;

        } else if (mouse.right || (hurt_data_.ready() && hurt_data_->reason == 0)) { // 按下右键||
                                                                                     // 受击
            translational_velocity_max = comfort_translational_velocity * 1.5;
            angular_velocity_max = comfort_angular_velocity;
            *boost_ = true;

        } else if (*robots_rfid_ == 1) {                                             // 回家
            translational_velocity_max = comfort_translational_velocity;
            angular_velocity_max = comfort_angular_velocity * 0.1;
            *boost_ = false;
        } else {                                                                     // 一般模式
            translational_velocity_max = comfort_translational_velocity;
            angular_velocity_max = comfort_angular_velocity * 4.0;
            *boost_ = false;
        }
        last_mouse_ = mouse;
        return;
    }

    InputInterface<Eigen::Vector4d> auto_pilot_velocity_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<referee::status::HurtData> hurt_data_;
    InputInterface<uint8_t> robots_rfid_;

    InputInterface<rmcs_msgs::Mouse> mouse_;
    rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    double comfort_translational_velocity = 10.0;
    double comfort_angular_velocity = 15.0;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

    OutputInterface<double> translational_velocity_max_;
    OutputInterface<double> angular_velocity_max_;
    OutputInterface<rmcs_msgs::OperateMode> mode_;
    OutputInterface<bool> boost_;

    double translational_velocity_max;
    double angular_velocity_max;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::OperateLogic, rmcs_executor::Component)