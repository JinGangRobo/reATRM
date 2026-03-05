#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/gimbal/two_axis_gimbal_solver.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class SimpleGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimpleGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , two_axis_gimbal_solver(
              *this, get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double()) {
        get_parameter("shift_control_clamp", shift_control_clamp_);
        get_parameter("joystick_left_bias_y", joystick_left_bias_y_);
        get_parameter("joystick_left_bias_x", joystick_left_bias_x_);
        get_parameter("depond_motors", depond_motors_);

        for (const auto& motor_ : depond_motors_) {
            auto timestamp_input = std::make_unique<InputInterface<int64_t>>();
            register_input(motor_ + "/last_update_time", *timestamp_input);
            depond_motor_timestamp_inputs_.push_back(std::move(timestamp_input));
        }

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);
        register_input("/gimbal/auto_aim/scan_direction", auto_aim_scan_direction_, false);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);
    }

    void update() override {
        auto angle_error = calculate_angle_error();
        *yaw_angle_error_ = angle_error.yaw_angle_error;
        *pitch_angle_error_ = angle_error.pitch_angle_error;
    }

    double value_abs_clamp(double value) {
        return std::copysign(std::min(std::abs(value), shift_control_clamp_), value);
    }

    TwoAxisGimbalSolver::AngleError calculate_angle_error() {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto mouse = *mouse_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN))
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled());

        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count();
        for (const auto& timestamp_input : depond_motor_timestamp_inputs_) {
            if (now - **timestamp_input > timeout_ms_) {
                return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled());
            }
        }

        bool autopilot_active = (switch_left != Switch::DOWN && switch_right == Switch::UP);

        if (auto_aim_control_direction_.ready() && (mouse.right || autopilot_active)
            && !auto_aim_control_direction_->isZero()) {
            return two_axis_gimbal_solver.update(
                TwoAxisGimbalSolver::SetControlDirection(
                    OdomImu::DirectionVector(*auto_aim_control_direction_)));
        } else {
            if (autopilot_active) {
                Eigen::Vector3d current_direction =
                    two_axis_gimbal_solver.current_control_direction();
                if (current_direction.norm() > 1e-9) {
                    double scan_direction;
                    if (auto_aim_scan_direction_.ready())
                        scan_direction = *auto_aim_scan_direction_ > 0 ? 1 : -1;
                    else {
                        scan_direction = 1;
                    }

                    // TODO: use parameter instead of hardcoded speed.
                    Eigen::Vector3d new_direction =
                        Eigen::AngleAxis(scan_direction * 0.0013, Eigen::Vector3d::UnitZ())
                        * current_direction;

                    new_direction.z() =
                        sin((now & 0xFFF) * 0.001534 * 2) * 0.2 - 0.1; // 0.001534=(1/4096)*2*pi
                    new_direction.normalize();

                    return two_axis_gimbal_solver.update(
                        TwoAxisGimbalSolver::SetControlDirection(
                            OdomImu::DirectionVector(new_direction)));
                }
            }
        }

        if (!two_axis_gimbal_solver.enabled())
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetToLevel());

        constexpr double joystick_sensitivity = 0.004;
        constexpr double mouse_sensitivity = 0.5;
        
        double yaw_shift = value_abs_clamp(
            joystick_sensitivity * (joystick_left_->y() - joystick_left_bias_y_)
            + mouse_sensitivity * mouse_velocity_->y());
        double pitch_shift = value_abs_clamp(
            -joystick_sensitivity * (joystick_left_->x() - joystick_left_bias_x_)
            + mouse_sensitivity * mouse_velocity_->x());

        return two_axis_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr int64_t timeout_ms_ = 200;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;
    InputInterface<int8_t> auto_aim_scan_direction_;

    TwoAxisGimbalSolver two_axis_gimbal_solver;

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;

    double joystick_left_bias_y_ = 0.0;
    double joystick_left_bias_x_ = 0.0;
    double shift_control_clamp_ = 0.0045;
    std::vector<std::string> depond_motors_ = {};
    std::vector<std::unique_ptr<InputInterface<int64_t>>> depond_motor_timestamp_inputs_ = {};
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SimpleGimbalController, rmcs_executor::Component)