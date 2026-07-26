#include <cmath>

#include <algorithm>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "controller/chassis/qcp_solver.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "utility/low_pass_filter.hpp"

namespace rmcs_core::controller::chassis {

class OmniWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OmniWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , translational_velocity_pid_calculator_(5.0, 0.0, 0.0)
        , angular_velocity_pid_calculator_(5.0, 0.0, 0.0)
        , direction_pid_calculator_(0.005, 0.0, 0.0)
        , wheel_velocity_pid_(0.6, 0.0, 0.0) {
        get_parameter("mass", mess_);
        get_parameter("moment_of_inertia", moment_of_inertia_);
        get_parameter("chassis_radius_x", chassis_radius_x_);
        get_parameter("chassis_radius_y", chassis_radius_y_);
        get_parameter("wheel_radius", wheel_radius_);

        register_input("/chassis/left_front_wheel/max_torque", wheel_motor_max_control_torque_);

        register_input("/chassis/control_mode", mode_);
        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        register_input("/chassis/left_front_wheel/alive", left_front_alive_);
        register_input("/chassis/left_back_wheel/alive", left_back_alive_);
        register_input("/chassis/right_front_wheel/alive", right_front_alive_);
        register_input("/chassis/right_back_wheel/alive", right_back_alive_);

        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/control_power_limit", power_limit_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);

        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_control_torque_, nan_);
        register_output("/chassis/left_back_wheel/control_torque", left_back_control_torque_, nan_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_control_torque_, nan_);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_control_torque_, nan_);
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(), "Max control torque of wheel motor: %.f",
            *wheel_motor_max_control_torque_);
    }

    void update() override {
        if (!(*left_back_alive_ && *right_back_alive_ && *left_front_alive_
              && *right_front_alive_)) {
            wheel_alive_protecter_.reset(1'000);
            wheel_enable_ = false;
        }
        if (wheel_alive_protecter_.tick())
            wheel_enable_ = true;

        if (std::isnan(chassis_control_velocity_->vector[0]) || !wheel_enable_) {
            reset_all_controls();
            return;
        }

        Eigen::Vector4d wheel_velocities = {
            *left_front_velocity_, *left_back_velocity_,  //
            *right_back_velocity_, *right_front_velocity_ //
        };

        const auto chassis_velocity = calculate_chassis_velocity(wheel_velocities);
        auto chassis_control_torque = calculate_chassis_control_torque(chassis_velocity);
        const auto wheel_pid_torques =
            calculate_wheel_pid_torques(wheel_velocities, chassis_velocity);
        chassis_control_torque.torque = constrain_chassis_control_torque(
            wheel_velocities, chassis_control_torque, wheel_pid_torques);
        const auto wheel_control_torques =
            calculate_wheel_control_torques(chassis_control_torque, wheel_pid_torques);

        *left_front_control_torque_ = wheel_control_torques[0];
        *left_back_control_torque_ = wheel_control_torques[1];
        *right_back_control_torque_ = wheel_control_torques[2];
        *right_front_control_torque_ = wheel_control_torques[3];
    }

private:
    struct ChassisControlTorque {
        Eigen::Vector2d torque;
        Eigen::Vector2d lambda;
    };

    void reset_all_controls() {
        *left_front_control_torque_ = 0.0;
        *left_back_control_torque_ = 0.0;
        *right_back_control_torque_ = 0.0;
        *right_front_control_torque_ = 0.0;
    }

    Eigen::Vector3d calculate_chassis_velocity(const Eigen::Vector4d& wheel_velocities) {
        const auto& [w1, w2, w3, w4] = wheel_velocities;
        Eigen::Vector3d velocity;
        velocity.x() = -w1 - w2 + w3 + w4;
        velocity.y() = w1 - w2 - w3 + w4;
        velocity.z() = (w1 + w2 + w3 + w4) / (chassis_radius_x_ + chassis_radius_y_);
        velocity *= (-std::numbers::sqrt2 / 4 * wheel_radius_);
        return velocity;
    }

    ChassisControlTorque calculate_chassis_control_torque(const Eigen::Vector3d& chassis_velocity) {
        ChassisControlTorque result;

        Eigen::Vector3d control_velocity_rotated = chassis_control_velocity_->vector;

        if (*mode_ == rmcs_msgs::ChassisMode::SPIN) {     // Fix spinning bais
            double direction_err =
                (atan2(
                    control_velocity_rotated.x() * chassis_velocity.y()
                        - control_velocity_rotated.y() * chassis_velocity.x(),
                    control_velocity_rotated.head<2>().dot(chassis_velocity.head<2>())))
                * (chassis_control_velocity_->vector.z() > 0 ? -1 : 1);

            if (abs(direction_err) < direction_fix_range_) {
                direction_pid_output_ += direction_pid_calculator_.update(
                    direction_err  // Need more case to prove number 100 is ok
                    * std::clamp(-(abs(translational_torque_diff_) * 100) + 1, 0.0, 1.0));
                direction_pid_output_ = std::clamp(
                    fmod(direction_pid_output_, direction_fix_range_), -direction_fix_range_, 0.0);
            }

            control_velocity_rotated.head<2>() =
                Eigen::Rotation2Dd{
                    (chassis_control_velocity_->vector.z() > 0 ? 1 : -1) * direction_pid_output_}
                * chassis_control_velocity_->vector.head<2>();
        }

        Eigen::Vector3d err = control_velocity_rotated - chassis_velocity;
        err.head<2>() = Eigen::Rotation2Dd{-*gimbal_yaw_angle_} * err.head<2>();

        Eigen::Vector2d translational_torque =
            (-std::numbers::sqrt2 / 4 * wheel_radius_) * mess_
            * (Eigen::Rotation2Dd{*gimbal_yaw_angle_}
               * translational_velocity_pid_calculator_.update(err.head<2>()));

        result.torque.x() = translational_torque.norm();
        double translational_torque_filtered =
            translational_torque_filter_.update(translational_torque.norm());

        translational_torque_diff_ = translational_torque_diff_filter_.update(
            translational_torque_filtered - last_translational_torque_);
        last_translational_torque_ = translational_torque_filtered;

        result.torque.y() = (-std::numbers::sqrt2 / 4 * wheel_radius_)
                          * (moment_of_inertia_ / (chassis_radius_x_ + chassis_radius_y_))
                          * angular_velocity_pid_calculator_.update(err[2]);

        Eigen::Vector2d translational_torque_direction;
        if (result.torque.x() > 0)
            translational_torque_direction = translational_torque / result.torque.x();
        else
            translational_torque_direction = Eigen::Vector2d::UnitX();
        auto& [x, y] = translational_torque_direction;
        result.lambda = {-x + y, -x - y};

        return result;
    }

    Eigen::Vector4d calculate_wheel_pid_torques(
        const Eigen::Vector4d& wheel_velocities, const Eigen::Vector3d& chassis_velocity) {
        const auto& [x, y, z] = chassis_velocity;
        double a_plus_b = chassis_radius_x_ + chassis_radius_y_;
        Eigen::Vector4d wheel_control_velocity = {
            -x + y + a_plus_b * z,
            -x - y + a_plus_b * z,                                   //
            +x - y + a_plus_b * z,
            +x + y + a_plus_b * z,                                   //
        };
        wheel_control_velocity *= -1 / (std::numbers::sqrt2 * wheel_radius_);
        return wheel_velocity_pid_.update(wheel_control_velocity - wheel_velocities);
    }

    Eigen::Vector2d constrain_chassis_control_torque(
        const Eigen::Vector4d& wheel_velocities, const ChassisControlTorque& chassis_control_torque,
        const Eigen::Vector4d& wheel_pid_torques) const {
        const auto& [w1, w2, w3, w4] = wheel_velocities;

        const auto& [x_max, y_max] = chassis_control_torque.torque;
        const double y_sign = y_max > 0 ? 1.0 : -1.0;
        const auto& [lambda_1, lambda_2] = chassis_control_torque.lambda;

        const auto& [t1, t2, t3, t4] = wheel_pid_torques;

        const double rhombus_top = (friction_coefficient_ * mess_ * g_ * wheel_radius_) / 4;
        const double rhombus_right = rhombus_top / std::max(std::abs(lambda_1), std::abs(lambda_2));

        const double a = 4 * k1_;
        const double b = 0;
        const double c = 4 * k1_;
        const double d = lambda_1 * (w1 - w3 + 2 * k1_ * (t1 - t3))  //
                       + lambda_2 * (w2 - w4 + 2 * k1_ * (t2 - t4));
        const double e = y_sign * (2 * k1_ * wheel_pid_torques.sum() + wheel_velocities.sum());
        const double f = k1_ * wheel_pid_torques.array().pow(2).sum()
                       + (wheel_pid_torques.array() * wheel_velocities.array()).sum()
                       + k2_ * wheel_velocities.array().pow(2).sum() //
                       - no_load_power_ - *power_limit_;

        auto result = qcp_solver_.solve(
            {1.0, 1.0}, {x_max, std::abs(y_max)}, {rhombus_right, rhombus_top}, {a, b, c, d, e, f});
        result.y() *= y_sign;
        return result;
    }

    static Eigen::Vector4d calculate_wheel_control_torques(
        ChassisControlTorque chassis_control_torque, Eigen::Vector4d wheel_pid_torques) {
        const auto& [lambda_1, lambda_2] = chassis_control_torque.lambda;
        Eigen::Vector4d wheel_torques = {
            +lambda_1 * chassis_control_torque.torque.x(),
            +lambda_2 * chassis_control_torque.torque.x(),
            -lambda_1 * chassis_control_torque.torque.x(),
            -lambda_2 * chassis_control_torque.torque.x(),
        };
        wheel_torques.array() += chassis_control_torque.torque.y();
        wheel_torques.array() += wheel_pid_torques.array();
        return wheel_torques;
    }

    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double k1_ = 2.958580e+00, k2_ = 3.082190e-03, no_load_power_ = 2.7;

    static constexpr double g_ = 9.81;

    double mess_ = 22.0;
    double moment_of_inertia_ = 4.08;
    double chassis_radius_x_ = 0.25, chassis_radius_y_ = 0.25;
    double wheel_radius_ = 0.07;
    double friction_coefficient_ = 0.6;

    double direction_fix_range_ = std::numbers::pi / 3;
    double direction_pid_output_ = -0.2;
    double last_translational_torque_;
    double translational_torque_diff_;

    InputInterface<double> wheel_motor_max_control_torque_;

    InputInterface<rmcs_msgs::ChassisMode> mode_;
    InputInterface<double> left_front_velocity_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;
    InputInterface<double> right_front_velocity_;

    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<bool> left_front_alive_, left_back_alive_, right_front_alive_, right_back_alive_;
    rmcs_utility::TickTimer wheel_alive_protecter_;
    bool wheel_enable_ = true;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> power_limit_;

    pid::MatrixPidCalculator<2> translational_velocity_pid_calculator_;
    pid::PidCalculator angular_velocity_pid_calculator_;
    pid::PidCalculator direction_pid_calculator_;

    pid::MatrixPidCalculator<4> wheel_velocity_pid_;

    QcpSolver qcp_solver_;

    utility::LowPassFilter<> translational_torque_diff_filter_{5, 1000};
    utility::LowPassFilter<> translational_torque_filter_{5, 1000};

    OutputInterface<double> left_front_control_torque_;
    OutputInterface<double> left_back_control_torque_;
    OutputInterface<double> right_back_control_torque_;
    OutputInterface<double> right_front_control_torque_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::OmniWheelController, rmcs_executor::Component)