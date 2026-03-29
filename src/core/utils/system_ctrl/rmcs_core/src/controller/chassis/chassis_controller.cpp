#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(6.0, 0.0, 0.0) {
        get_parameter("translational_velocity_limit", translational_velocity_limit_);
        get_parameter("angular_velocity_limit", angular_velocity_limit_);
        get_parameter("autopilot_spin_velocity_default", autopilot_spin_velocity_default);

        following_velocity_controller_.output_max = angular_velocity_limit_;
        following_velocity_controller_.output_min = -angular_velocity_limit_;
        translational_velocity_max = translational_velocity_limit_;
        angular_velocity_max = angular_velocity_limit_;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);

        register_input("/chassis/translational_vmax", translational_velocity_max_, false);
        register_input("/chassis/angular_vmax", angular_velocity_max_, false);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/autopilot/chassis/velocity", auto_pilot_velocity_, false);

        register_input("/chassis/left_front_wheel/alive", left_front_alive_);
        register_input("/chassis/left_back_wheel/alive", left_back_alive_);
        register_input("/chassis/right_front_wheel/alive", right_front_alive_);
        register_input("/chassis/right_back_wheel/alive", right_back_alive_);

        register_output("/chassis/angle", chassis_angle_, nan);
        register_output("/chassis/control_angle", chassis_control_angle_, nan);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_output("/chassis/is_spinning_forward", is_spinning_forward_);
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto keyboard = *keyboard_;

        // Update maximum velocities
        if (translational_velocity_max_.ready()) {
            if (*translational_velocity_max_ != nan)
                translational_velocity_max =
                    std::clamp(*translational_velocity_max_, 0.0, translational_velocity_limit_);
        }
        if (angular_velocity_max_.ready()) {
            if (*angular_velocity_max_ != nan) {
                angular_velocity_max =
                    std::clamp(*angular_velocity_max_, 0.0, angular_velocity_limit_);
                following_velocity_controller_.output_max = angular_velocity_max;
                following_velocity_controller_.output_min = -angular_velocity_max;
            }
        }

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            auto mode = *mode_;
            if (switch_left != Switch::DOWN) {
                if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        mode = rmcs_msgs::ChassisMode::STEP_DOWN;
                    } else {
                        mode = rmcs_msgs::ChassisMode::SPIN;
                        spinning_forward_ = !spinning_forward_;
                    }
                } else if (!last_keyboard_.c && keyboard.c) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        mode = rmcs_msgs::ChassisMode::AUTO;
                    } else {
                        mode = rmcs_msgs::ChassisMode::SPIN;
                        spinning_forward_ = !spinning_forward_;
                    }
                } else if (!last_keyboard_.x && keyboard.x) {
                    mode = mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                             ? rmcs_msgs::ChassisMode::AUTO
                             : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
                } else if (!last_keyboard_.z && keyboard.z) {
                    mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN
                             ? rmcs_msgs::ChassisMode::AUTO
                             : rmcs_msgs::ChassisMode::STEP_DOWN;
                }
                if (switch_right == Switch::UP) {
                    mode = rmcs_msgs::ChassisMode::AUTO_PILOT;
                } else {
                    if (mode == rmcs_msgs::ChassisMode::AUTO_PILOT)
                        mode = rmcs_msgs::ChassisMode::AUTO;
                }

                if (!(*left_back_alive_ && *right_back_alive_ && *left_front_alive_
                      && *right_front_alive_)) {
                    mode = rmcs_msgs::ChassisMode::AUTO;
                }
                *mode_ = mode;
                *is_spinning_forward_ = spinning_forward_;
            }

            update_velocity_control();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;

        *chassis_control_velocity_ = {nan, nan, nan};
    }

    void update_velocity_control() {
        if (*mode_ == rmcs_msgs::ChassisMode::AUTO_PILOT && auto_pilot_velocity_.ready()
            && (*auto_pilot_velocity_)[3] != 0) {
            Eigen::Vector2d autopilot_velocity_xy = auto_pilot_velocity_->head<2>();

            // do velocity limiting for autopilot here.
            if (autopilot_velocity_xy.norm() > translational_velocity_max) {
                autopilot_velocity_xy.normalize();
                autopilot_velocity_xy *= translational_velocity_max;
            }
            double angular_velocity_clamped =
                std::clamp((*auto_pilot_velocity_)[2], -angular_velocity_max, angular_velocity_max);

            chassis_control_velocity_->vector << autopilot_velocity_xy, angular_velocity_clamped;
            return;
        }

        auto translational_velocity = update_translational_velocity_control();
        auto angular_velocity = *mode_ == rmcs_msgs::ChassisMode::AUTO_PILOT
                                  ? autopilot_spin_velocity_default
                                  : update_angular_velocity_control();

        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        auto keyboard = *keyboard_;
        Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ + keyboard_move);

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max;

        return translational_velocity;
    }

    double update_angular_velocity_control() {
        double angular_velocity = 0.0;
        double chassis_control_angle = nan;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;
        case rmcs_msgs::ChassisMode::SPIN: {
            angular_velocity =
                0.6 * (spinning_forward_ ? angular_velocity_max : -angular_velocity_max);
        } break;
        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> [0, alignment) -> signed.
            // In step-down mode, two sides of the chassis can be used for alignment.
            // TODO: Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(err);
        } break;
        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> signed
            // In launch ramp mode, only one direction can be used for alignment.
            // TODO: Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);
        } break;
        case rmcs_msgs::ChassisMode::AUTO_PILOT: break;
        }
        *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;

        return angular_velocity;
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;
        // chassis_control_angle: [0, 2pi).

        // err = setpoint         -       measurement
        //          ^                          ^
        //          |gimbal_yaw_angle_error    |chassis_angle
        //                                            ^
        //                                            |(2pi - gimbal_yaw_angle)
        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;
        // err: [0, 2pi).

        return err;
    }

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double translational_velocity_limit_ = 15.0;
    double angular_velocity_limit_ = 15.0;

    // Maximum control velocities
    double translational_velocity_max = 10.0;
    double angular_velocity_max = 14.0;
    double autopilot_spin_velocity_default = 2.0;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<double> translational_velocity_max_;
    InputInterface<double> angular_velocity_max_;

    InputInterface<bool> left_front_alive_, left_back_alive_, right_front_alive_, right_back_alive_;

    InputInterface<Eigen::Vector4d> auto_pilot_velocity_; // [x, y, w, isActive(1/0)]

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<bool> is_spinning_forward_;
    bool spinning_forward_ = true;
    pid::PidCalculator following_velocity_controller_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)