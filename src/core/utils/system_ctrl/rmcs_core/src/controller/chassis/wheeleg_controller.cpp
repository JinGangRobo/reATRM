
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/wheel_leg_mode.hpp>
#include <rmcs_msgs/wheel_leg_state.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "utility/kalman_filter.hpp"
#include "utility/low_pass_filter.hpp"
#include <rmcs_description/tf_description.hpp>

#include "desire_state_solver.hpp"
#include "vmc_solver.hpp"
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class WheelegController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelegController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , body_mass_(6.3)
        , leg_mass_(0.629)
        , wheel_mass_(0.459)
        , wheel_radius_(0.058)
        , wheel_distance_(0.214)
        , gas_spring(150)
        , centroid_position_coefficient_(0.0)
        , left_leg_vmc_solver_(0.21, 0.25, 0.0)
        , right_leg_vmc_solver_(0.21, 0.25, 0.0)
        // , velocity_kalman_filter_(A_, H_, Q_, R_)
        , roll_angle_pid_calculator_(70.0, 0.0, 1.0)
        , leg_length_pid_calculator_(190.0, 0.0, 20.0)
        , anti_crash_pid_(60.0, 0.0, 0.2)
        , yaw_speed_pid_(1.2, 0.0, 5.0)
        , rescue_velocity_pid_calculator_(5.0, 0.0, 0.0)
        , rescue_angle_pid_calculator_(8.0, 0.0, 0.0)
        , rescue_length_pid_calculator_(150.0, 1.0, 10.0)

    // , leg_Left_L0_pid_(500.0, 0.0, 100.0)
    // , leg_Right_L0_pid_(500.0, 0.0, 100.0)

    // , leg_Left_Tp_pid_(15.0, 0.0, 5.0)
    // , leg_Right_Tp_pid_(15.0, 0.0, 5.0)

    // , yaw_speed_pid_(0.2, 0.0, 5.0)
    {

        // roll_k_ = 1000.0;

        // left_phi1_offset_ = std::numbers::pi - 0.1;
        // left_phi4_offset_ = std::numbers::pi - 0.1;
        // right_phi1_offset_ = std::numbers::pi - 0.1;
        // right_phi4_offset_ = std::numbers::pi - 0.1;

        // last_time_ = this->get_clock()->now();
        // robot_weight_per_leg_ = 40.0; // 重力补偿
        // gas_spring_ = -38.0;          // 气弹簧

        // wheel_weight_ = 0.5;          // 轮子重量

        register_input("/chassis/left_back_hip/angle", left_front_hip_angle_);
        register_input("/chassis/left_front_hip/angle", left_back_hip_angle_);
        register_input("/chassis/right_back_hip/angle", right_front_hip_angle_);
        register_input("/chassis/right_front_hip/angle", right_back_hip_angle_);

        register_input("/chassis/left_back_hip/velocity", left_front_hip_velocity_);
        register_input("/chassis/left_front_hip/velocity", left_back_hip_velocity_);
        register_input("/chassis/right_back_hip/velocity", right_front_hip_velocity_);
        register_input("/chassis/right_front_hip/velocity", right_back_hip_velocity_);

        register_input("/chassis/imu/pitch", imu_pitch_, false);
        register_input("/chassis/imu/pitch_velocity", imu_d_pitch_, false);
        register_input("/chassis/imu/yaw", imu_yaw_, false);
        register_input("/chassis/imu/yaw_velocity", imu_d_yaw_, false);
        register_input("/chassis/imu/roll", imu_roll_);
        register_input("/debug/imu/ax", imu_ax_);
        register_input("/debug/imu/az", imu_az_);
        register_input("/debug/imu/ddx", imu_ddx_, false);
        register_input("/debug/imu/ddz", imu_ddz_);

        register_input("/chassis/control_velocity", chassis_control_velocity_, false);
        register_input("/remote/rotary_knob", rotary_knob_, false);

        register_input("/chassis/left_wheel/velocity", left_wheel_velocity_, false);
        register_input("/chassis/right_wheel/velocity", right_wheel_velocity_, false);
        register_output("/chassis/left_wheel/control_torque", left_wheel_control_torque_);
        register_output("/chassis/right_wheel/control_torque", right_wheel_control_torque_);

        register_input("/chassis/left_back_hip/torque", left_front_hip_torque_);
        register_input("/chassis/left_front_hip/torque", left_back_hip_torque_);
        register_input("/chassis/right_back_hip/torque", right_front_hip_torque_);
        register_input("/chassis/right_front_hip/torque", right_back_hip_torque_);

        register_output("/chassis/left_back_hip/control_torque", left_front_hip_control_torque_);
        register_output("/chassis/left_front_hip/control_torque", left_back_hip_control_torque_);
        register_output("/chassis/right_back_hip/control_torque", right_front_hip_control_torque_);
        register_output("/chassis/right_front_hip/control_torque", right_back_hip_control_torque_);

        register_output("/debug/left/L0", dbg_L0_[LEFT]);
        register_output("/debug/right/L0", dbg_L0_[RIGHT]);

        register_output("/debug/left/dL0", dbg_dL0_[LEFT]);
        register_output("/debug/right/dL0", dbg_dL0_[RIGHT]);

        register_output("/debug/left/ddL0", dbg_ddL0_[LEFT]);
        register_output("/debug/right/ddL0", dbg_ddL0_[RIGHT]);

        register_output("/debug/left/Theta", dbg_theta_[LEFT]);
        register_output("/debug/right/Theta", dbg_theta_[RIGHT]);
        register_output("/debug/left/dTheta", dbg_dtheta_[LEFT]);
        register_output("/debug/right/dTheta", dbg_dtheta_[RIGHT]);
        register_output("/debug/left/sportFN", dbg_sportFN[LEFT]);
        register_output("/debug/right/sportFN", dbg_sportFN[RIGHT]);
        register_output("/debug/vel", dbg_vel);
        register_output("/debug/vel_filterd", dbg_filtered_vel);
        register_input("/remote/switch/left", switch_left_);
    }

    void update() override {

        // // 遥控器部分

        // double rc_cmd_v = 0.0;
        // double rc_cmd_w = 0.0;
        using namespace rmcs_msgs;
        auto switch_left = *switch_left_;

        if (switch_left == Switch::MIDDLE) {
            desire_speed = 1.0;
        }

        if (std::isnan(chassis_control_velocity_->vector[0])) {
            reset_all_controls();
            return;
        }
        auto vel_vec = (*chassis_control_velocity_).vector;

        // Observer
        auto hip_angles = calculate_hips_angles();
        auto wheel_velocities = calculate_wheel_velocities();

        auto leg_posture = calculate_leg_posture(hip_angles);
        kf_init_();
        auto distance = calculate_translational_distance(leg_posture, wheel_velocities);

        // State: [s ds phi d_phi theta_l d_theta_l theta_r d_theta_r theta_b d_theta_b]
        auto measure_state = calculate_measure_state(distance, leg_posture);
        auto sport_force = calculate_support_force(leg_posture);
        // Controller
        auto chassis_control_velocity = calculate_chassis_control_velocity();
        calculate_leg_length();
        auto leg_forces = calculate_leg_force(leg_posture, measure_state);

        // auto desire_state = calculate_desire_state(chassis_control_velocity, measure_state);
        // auto control_torques = calculate_control_torques(desire_state, measure_state,
        // leg_posture);

        // if (*is_balanceless_) {
        // update_balanceless_control_torques(chassis_control_velocity, wheel_velocities);
        //     return;
        // }

        // if (!stand_active_) {
        //     update_hip_stand_control_torques(leg_posture);
        //     return;
        // }

        auto left_state = calculate_lqr_gains(
            leg_posture.leg_length(0), leg_posture.tilt_angle(0), leg_posture.diff_tilt_angle(0),
            distance);
        auto right_state = calculate_lqr_gains(
            leg_posture.leg_length(1), leg_posture.tilt_angle(1), leg_posture.diff_tilt_angle(1),
            distance);

        auto yaw_control_torque = yaw_control(vel_vec[1]);

        *dbg_L0_[LEFT] = leg_posture.leg_length(0);
        *dbg_L0_[RIGHT] = leg_posture.leg_length(1);

        *dbg_dL0_[LEFT] = leg_posture.diff_leg_length(0);
        *dbg_dL0_[RIGHT] = leg_posture.diff_leg_length(1);

        *dbg_ddL0_[LEFT] = leg_posture.second_order_diff_leg_length(0);
        *dbg_ddL0_[RIGHT] = leg_posture.second_order_diff_leg_length(1);

        *dbg_theta_[LEFT] = leg_posture.tilt_angle(0);
        *dbg_theta_[RIGHT] = leg_posture.tilt_angle(1);

        *dbg_dtheta_[LEFT] = leg_posture.diff_tilt_angle_cal(1);

        auto left_vel = left_leg_vmc_solver_.update_velocity(
            *right_front_hip_velocity_, *right_back_hip_velocity_);

        *dbg_dtheta_[RIGHT] = leg_posture.diff_tilt_angle(1);
        *dbg_sportFN[LEFT] = sport_force(0);
        *dbg_sportFN[RIGHT] = desire_speed;
        *dbg_vel = distance(1);

        // is_rescue_tip_over_ = true;

        if (is_rescue_tip_over_) {
            
            if (std::abs(leg_posture.tilt_angle(0)) < 1.5
                && std::abs(leg_posture.tilt_angle(1)) < 1.5) {
                update_leg_rescue_tip_over_control_torques(leg_posture, 0.18);
                if (leg_posture.leg_length(0) < 0.21 && leg_posture.leg_length(1) < 0.21) {
                    auto leg_velocity = update_hip_rescue_tip_over_control_velocitys(leg_posture);
                    update_hip_rescue_tip_over_control_torques(leg_velocity);
                    if (std::abs(leg_posture.tilt_angle(0)) < 0.5
                        && std::abs(leg_posture.tilt_angle(1)) < 0.5) {
                        is_rescue_tip_over_ = false;
                    }
                }

            } else {
                auto leg_velocity = update_hip_rescue_tip_over_control_velocitys(leg_posture);
                update_hip_rescue_tip_over_control_torques(leg_velocity);
            }
            reset_distance();
            return;
            // if ( ) {
            //     is_rescue_tip_over_ = false;
            // }
        }


        update_hip_and_wheel_torques(
            leg_forces, left_state, right_state, yaw_control_torque, leg_posture);

        // *left_front_hip_control_torque_ = 0.0;
        // *left_back_hip_control_torque_ = 0.0;
        // *right_front_hip_control_torque_ = 0.0;
        // *right_back_hip_control_torque_ = 0.0;

        // *left_wheel_control_torque_ = 0.0;
        // *right_wheel_control_torque_ = 0.0;

        // *left_wheel_control_torque_ = 0.0;
        // *right_wheel_control_torque_ = 0.0;

        //  RCLCPP_INFO(
        //         get_logger(), "Chassis is levitating! Average support force: %f",
        //         leg_posture(1));
        // auto distance = calculate_translational_distance(leg_posture, wheel_velocities);

        desire_speed = vel_vec[0] * 0.1;
        // if (fabs(vel_vec[0]) > 0.1) {
        //     desire_distance += vel_vec[0] * 0.1 * dt_;
        // }
    }

private:
    enum Side { LEFT = 0, RIGHT = 1 };
    enum Joint { BACK = 0, FRONT = 1 };
    struct LegStatus {
        bool solved = false;
        double phi1, phi4;
        double L0, phi0, theta;
        double dL0, d_theta;
        double last_L0;
        double J11, J12, J21, J22;
    };

    struct LegPosture {
        Eigen::Vector2d leg_length;
        Eigen::Vector2d tilt_angle;

        Eigen::Vector2d diff_leg_length;
        Eigen::Vector2d diff_tilt_angle;
        Eigen::Vector2d diff_tilt_angle_cal;

        Eigen::Vector2d second_order_diff_leg_length;
        Eigen::Vector2d second_order_diff_tilt_angle;
    };
    void reset_distance() {
        distance = 0;
        desire_distance = 0;
    }
    void reset_all_controls() {

        left_leg_vmc_solver_.reset();
        right_leg_vmc_solver_.reset();
        is_rescue_tip_over_ = true;

        // current_target_vel = 0.0;

        *left_front_hip_control_torque_ = nan_;
        *left_back_hip_control_torque_ = nan_;
        *right_front_hip_control_torque_ = nan_;
        *right_back_hip_control_torque_ = nan_;

        *left_wheel_control_torque_ = 0.0;
        *right_wheel_control_torque_ = 0.0;
        distance = 0;
        desire_distance = 0;
        desire_leg_length_ = 0.20;
        current_target_L0 = 0.20;

        // current_target_L0 = 0.33;
    }
    Eigen::Vector2d calculate_wheel_velocities() {
        return {
            *left_wheel_velocity_ * wheel_radius_, //
            *right_wheel_velocity_ * wheel_radius_};
    }

    Eigen::Vector4d calculate_hips_angles() {
        return {
            *left_front_hip_angle_,                //
            *left_back_hip_angle_,                 //
            *right_back_hip_angle_,                //
            *right_front_hip_angle_,               //
        };
    }

    LegPosture calculate_leg_posture(const Eigen::Vector4d& hip_angles) {
        const double lf = hip_angles(0);
        const double lb = hip_angles(1);
        const double rb = hip_angles(2);
        const double rf = hip_angles(3);
        LegPosture result;

        Eigen::Vector2d left_leg_state = left_leg_vmc_solver_.update(lf, lb);
        Eigen::Vector2d right_leg_state = right_leg_vmc_solver_.update(rf, rb);
        Eigen::Vector2d left_leg_velocity = left_leg_vmc_solver_.update_velocity(
            *left_front_hip_velocity_, *left_back_hip_velocity_);
        Eigen::Vector2d right_leg_velocity = right_leg_vmc_solver_.update_velocity(
            *right_front_hip_velocity_, *right_back_hip_velocity_);
        Eigen::Vector2d imu_state = {*imu_d_pitch_, *imu_d_pitch_};

        const double left_leg_length = left_leg_state(0);
        const double left_tilt_angle = left_leg_state(1);
        const double right_leg_length = right_leg_state(0);
        const double right_tilt_angle = right_leg_state(1);

        result.leg_length = Eigen::Vector2d{left_leg_length, right_leg_length};
        result.tilt_angle = Eigen::Vector2d{left_tilt_angle, right_tilt_angle};

        result.tilt_angle.array() += *imu_pitch_;

        result.diff_leg_length.x() = left_leg_velocity.x();
        result.diff_leg_length.x() = right_leg_velocity.x();
        // result.diff_leg_length.y() = right_leg_vmc_solver_.update_velocity((result.leg_length.y()
        // - last_leg_length_.y()) / dt_);

        last_leg_length_ = result.leg_length;

        result.diff_tilt_angle = (result.tilt_angle - last_tilt_angle_) / dt_;
        result.diff_tilt_angle_cal = {left_leg_velocity.y(), -right_leg_velocity.y()};
        last_tilt_angle_ = result.tilt_angle;

        result.second_order_diff_leg_length =
            leg_ddl0_filter.update((result.diff_leg_length - last_dot_leg_length_) / dt_);
        last_dot_leg_length_ = result.diff_leg_length;

        result.second_order_diff_tilt_angle =
            leg_ddphi0_filter.update((result.diff_tilt_angle_cal - last_dot_tilt_angle_) / dt_);
        last_dot_tilt_angle_ = result.diff_tilt_angle_cal;

        return result;
    }

    Eigen::Vector2d calculate_support_force(LegPosture leg_posture) {
        Eigen::Vector2d result = Eigen::Vector2d::Zero();

        Eigen::Vector2d left_virtual_torque = left_leg_vmc_solver_.update_virtual_torque(
            *left_front_hip_torque_, *left_back_hip_torque_);

        const double left_leg_force = left_virtual_torque(0);
        const double left_leg_torque = left_virtual_torque(1);

        Eigen::Vector2d right_virtual_torque = right_leg_vmc_solver_.update_virtual_torque(
            *right_front_hip_torque_, *right_back_hip_torque_);

        const double right_leg_force = right_virtual_torque(0);
        const double right_leg_torque = right_virtual_torque(1);

        auto left_leg_to_wheel_force =
            left_leg_force * std::cos(leg_posture.tilt_angle.x())
            + left_leg_torque * std::sin(leg_posture.tilt_angle.x()) / leg_posture.leg_length.x();
        auto right_leg_to_wheel_force =
            right_leg_force * std::cos(leg_posture.tilt_angle.y())
            + right_leg_torque * std::sin(leg_posture.tilt_angle.y()) / leg_posture.leg_length.y();

        // todo: test
        auto left_wheel_vertical_accel =
            *imu_az_
            - leg_posture.second_order_diff_leg_length.x() * std::cos(leg_posture.tilt_angle.x())
            + 2 * leg_posture.diff_leg_length.x() * leg_posture.diff_tilt_angle_cal.x()
                  * std::sin(leg_posture.tilt_angle.x())
            + leg_posture.leg_length.x() * leg_posture.second_order_diff_tilt_angle.x()
                  * std::sin(leg_posture.tilt_angle.x())
            + leg_posture.leg_length.x() * leg_posture.diff_tilt_angle_cal.x()
                  * leg_posture.diff_tilt_angle_cal.x() * std::cos(leg_posture.tilt_angle.x());

        auto right_wheel_vertical_accel =
            *imu_az_
            - leg_posture.second_order_diff_leg_length.y() * std::cos(leg_posture.tilt_angle.y())
            + 2 * leg_posture.diff_leg_length.y() * leg_posture.diff_tilt_angle_cal.y()
                  * std::sin(leg_posture.tilt_angle.y())
            + leg_posture.leg_length.y() * leg_posture.second_order_diff_tilt_angle.y()
                  * std::sin(leg_posture.tilt_angle.y())
            + leg_posture.leg_length.y() * leg_posture.diff_tilt_angle_cal.y()
                  * leg_posture.diff_tilt_angle_cal.y() * std::cos(leg_posture.tilt_angle.y());

        result(0) = left_leg_to_wheel_force + wheel_mass_ * (g_ + left_wheel_vertical_accel);
        result(1) = right_leg_to_wheel_force + wheel_mass_ * (g_ + right_wheel_vertical_accel);

        return result;
    }
    Eigen::Vector2d
        calculate_translational_distance(LegPosture leg_posture, Eigen::Vector2d wheel_velocities) {
        Eigen::Vector2d result;
        // auto& [distance, velocity] = result;

        // double world_left_wheel_velocity =
        //     (-wheel_velocities.x()) - (-leg_posture.diff_tilt_angle_cal(0)) - *imu_d_pitch_;

        // double world_right_wheel_velocity =
        //     -(wheel_velocities.y()) - (-leg_posture.diff_tilt_angle_cal(1)) - *imu_d_pitch_;

        // double left_body_velocity = world_left_wheel_velocity * wheel_radius_
        //                           + (-leg_posture.diff_tilt_angle_cal.x()) *
        //                           std::sin(-leg_posture.tilt_angle.x())
        //                           + leg_posture.leg_length.x()
        //                                 * std::cos(-leg_posture.tilt_angle.x())
        //                                 * (-leg_posture.diff_tilt_angle_cal.x());

        // double right_body_velocity = world_right_wheel_velocity * wheel_radius_
        //                           + (-leg_posture.diff_tilt_angle_cal.y()) *
        //                           std::sin(-leg_posture.tilt_angle.y())
        //                           + leg_posture.leg_length.y()
        //                                 * std::cos(-leg_posture.tilt_angle.y())
        //                                 * (-leg_posture.diff_tilt_angle_cal.y());

        auto wheel_velocity = -(wheel_velocities.x() + wheel_velocities.y()) / 2.0;

        auto left_leg_velocity =
            leg_posture.leg_length.x() * std::cos(leg_posture.tilt_angle.x())
                * leg_posture.diff_tilt_angle_cal.x()
            + leg_posture.diff_leg_length.x() * std::sin(leg_posture.tilt_angle.x());

        auto right_leg_velocity =
            leg_posture.leg_length.y() * std::cos(leg_posture.tilt_angle.y())
                * leg_posture.diff_tilt_angle_cal.y()
            + leg_posture.diff_leg_length.y() * std::sin(leg_posture.tilt_angle.y());

        auto calculate_velocity = wheel_velocity + (left_leg_velocity + right_leg_velocity) / 2.0;
        // double calculate_velocity = world_right_wheel_velocity;

        Eigen::Vector2d measurement;
        measurement << static_cast<double>(calculate_velocity), *imu_ddx_;

        Eigen::Vector2d optimal_state = kf_->update(measurement);

        // 提取融合后的最优结果
        filtered_velocity = optimal_state(0);            // 融合后的干净速度
        double filtered_acceleration = optimal_state(1); // 融合后的干净加速度

        // double filtered_vel = OnSensorDataReceived(calculate_velocity,  *imu_ddx_);
        *dbg_filtered_vel = optimal_state(0);
        double velocity = calculate_velocity;

        // auto is_parked = std::abs(desire_speed) < 0.1;
        // // When the vehicle stops the position control is activated.
        // last_desire_speed = desire_speed;
        double park;
        double step = 0.02;
        double vel_diff = desire_speed - park;
        if (vel_diff > step) {
            park += step;        // 加速（或正向减速）
        } else if (vel_diff < -step) {
            park -= step;        // 减速（或负向加速）
        } else {
            park = desire_speed; // 差值在步长内，直接对齐目标
        }

        if (std::abs(desire_speed) < 0.1) {
            distance = last_distance_ + velocity * dt_;
        } else {
            distance = 0.0;
        }

        // distance = last_distance_ + velocity * dt_;
        last_distance_ = distance;
        //  RCLCPP_INFO(get_logger(), "dist %f, vel %f", distance, velocity);

        return Eigen::Vector2d{distance, velocity};
    }

    rmcs_msgs::WheelLegState
        calculate_measure_state(Eigen::Vector2d distance, LegPosture leg_posture) {
        rmcs_msgs::WheelLegState measure_state;

        measure_state.distance = distance.x();
        measure_state.velocity = distance.y();

        measure_state.yaw_angle = *imu_yaw_;
        measure_state.yaw_velocity = *imu_d_yaw_;

        measure_state.left_tilt_angle = leg_posture.tilt_angle.x();
        measure_state.left_tilt_velocity = leg_posture.diff_tilt_angle.x();

        measure_state.right_tilt_angle = leg_posture.tilt_angle.y();
        measure_state.right_tilt_velocity = leg_posture.diff_tilt_angle.y();

        measure_state.body_pitch_angle = *imu_pitch_;
        measure_state.body_pitch_velocity = *imu_d_pitch_;

        // measure_state.distance = 0.0;
        // measure_state.velocity = 0.0;

        // measure_state.yaw_angle = 0.0;
        // measure_state.yaw_velocity = 0.0;

        // measure_state.left_tilt_angle = 0.0;
        // measure_state.left_tilt_velocity = 0.0;

        // measure_state.right_tilt_angle = 0.0;
        // measure_state.right_tilt_velocity = 0.0;

        // measure_state.body_pitch_angle = 0.0;
        // measure_state.body_pitch_velocity = 0.0;

        return measure_state;
    }
    void calculate_leg_length() {

        double knob_val = 0.0;
        if (rotary_knob_.ready()) {
            knob_val = -*rotary_knob_;
        }
        if (std::abs(knob_val) < 0.05) {
            knob_val = 0.0;
        }

        double min_L0 = 0.16;             // 最矮状态
        double max_L0 = 0.30;             // 最高状态
        double L0_speed_multiplier = 1.0; // 伸缩速度倍率

        // 默认高度

        current_target_L0 += knob_val * L0_speed_multiplier * 0.001;

        current_target_L0 = std::clamp(current_target_L0, min_L0, max_L0);

        desire_leg_length_ = current_target_L0;
        // RCLCPP_INFO(get_logger(), "yawspeed: %f", current_target_L0);
    }
    Eigen::Vector2d
        calculate_leg_force(LegPosture leg_posture, rmcs_msgs::WheelLegState measure_state) {
        Eigen::Vector2d result;

        auto leg_length = (leg_posture.leg_length.x() + leg_posture.leg_length.y()) / 2.0;

        auto roll_control_force =
            roll_angle_pid_calculator_.update(desire_roll_angle_ - *imu_roll_);
        auto leg_length_control_force =
            leg_length_pid_calculator_.update(desire_leg_length_ - leg_length);

        auto calculate_compensation_feedforward_force = [this](double coefficient) {
            return (body_mass_ / 2.0 + centroid_position_coefficient_ * leg_mass_) * coefficient;
        };

        auto gravity_feedforward_control_force = calculate_compensation_feedforward_force(g_);
        auto inertial_feedforward_control_force =
            calculate_compensation_feedforward_force(
                leg_length / (2 * wheel_distance_) * measure_state.yaw_velocity
                * measure_state.velocity)
            ;

        //                       F_ψ
        // F_bl_l =  1 1 1 -1 *  F_l
        // F_bl_r   -1 1 1  1    F_g
        //                       F_i
        result = Eigen::Vector2d{
            -roll_control_force + leg_length_control_force + gravity_feedforward_control_force
                - inertial_feedforward_control_force, // F_bl_l
            roll_control_force + leg_length_control_force + gravity_feedforward_control_force
                + inertial_feedforward_control_force  // F_bl_r
        };

        return result;
    }

    Eigen::Vector3d calculate_chassis_control_velocity() {
        Eigen::Vector3d chassis_control_velocity;
        chassis_control_velocity = chassis_control_velocity_->vector;
        return chassis_control_velocity;
    }
    Eigen::Vector2d
        calculate_lqr_gains(double L0, double theta, double d_theta, Eigen::Vector2d body_state) {
        Eigen::Vector2d result;
        double dis = body_state(0);
        double vel = body_state(1);

        static const Eigen::Matrix<double, 12, 3> K_poly{
            {    39.9272,   -36.1729,    -6.4157},
            {    -0.1937,    -0.8776,    -0.4573},
            {    13.9181,   -10.4709,    -4.0458},
            {     4.9432,    -4.4477,    -2.5769},
            {   111.0756,  -104.1270,    32.3540},
            {    15.5322,   -13.7168,     4.1889},
            {    13.2803,   -20.1727,     9.4797},
            {     2.4843,    -2.1723,     0.6808},
            {    39.4106,   -37.0871,    11.5822},
            {    22.9763,   -20.8764,     6.3076},
            {  -191.4673,   141.5164,    56.9088},
            {   -23.6379,    17.3923,     3.3896}
        };

        // RCLCPP_INFO(get_logger(), "dist %f, vel %f", dist_, vel_);

        Eigen::Vector3d L_vec(L0 * L0, L0, 1.0);
        Eigen::Matrix<double, 12, 1> K_flat = K_poly * L_vec;

        Eigen::Matrix<double, 2, 6> K =
            Eigen::Map<const Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>(K_flat.data());

        Eigen::Matrix<double, 6, 1> x = {theta,          d_theta,      (dis), (vel - desire_speed),
                                         (-*imu_pitch_), *imu_d_pitch_};
        // double FN = sportFN[side];
        // if (FN < 20.0) {
        //     K.row(0).setZero();
        //     K.row(1).tail<4>().setZero();
        // }

        Eigen::Vector2d u = K * x;
        result(0) = u(0);
        result(1) = -u(1);
        return result;
    }

    Eigen::Vector2d update_hip_rescue_tip_over_control_velocitys(LegPosture leg_posture) {
        double control_angle = -0.3;

        auto left_control_velocitys =
            rescue_angle_pid_calculator_.update(control_angle + leg_posture.tilt_angle(0));
        auto right_control_velocitys =
            rescue_angle_pid_calculator_.update(control_angle + leg_posture.tilt_angle(1));

        double clamp_left_control_velocitys = clamp_leg_control_velocity(left_control_velocitys);
        double clamp_right_control_velocitys = clamp_leg_control_velocity(right_control_velocitys);
        Eigen::Vector2d result = {clamp_left_control_velocitys, clamp_right_control_velocitys};
        return result;
    }
    void update_hip_rescue_tip_over_control_torques(Eigen::Vector2d control_velocity) {

        // stand_active_ = false;
        auto left_front_hip_control_torque = rescue_velocity_pid_calculator_.update(
            control_velocity(0) - (-*left_front_hip_velocity_));
        auto left_back_hip_control_torque = rescue_velocity_pid_calculator_.update(
            control_velocity(0) - (-*left_back_hip_velocity_));
        auto right_front_hip_control_torque = rescue_velocity_pid_calculator_.update(
            control_velocity(1) - *right_front_hip_velocity_);
        auto right_back_hip_control_torque =
            rescue_velocity_pid_calculator_.update(control_velocity(1) - *right_back_hip_velocity_);

        *left_front_hip_control_torque_ = left_front_hip_control_torque;
        *left_back_hip_control_torque_ = left_back_hip_control_torque;
        *right_front_hip_control_torque_ = right_front_hip_control_torque;
        *right_back_hip_control_torque_ = right_back_hip_control_torque;

        // reset_control_angles();
    }
    void update_leg_rescue_tip_over_control_torques(LegPosture leg_posture, double leg_length) {

        auto left_leg_force =
            rescue_length_pid_calculator_.update(leg_length - leg_posture.leg_length.x());
        auto right_leg_force =
            rescue_length_pid_calculator_.update(leg_length - leg_posture.leg_length.y());

        auto left_hip_control_torque =
            left_leg_vmc_solver_.update_joint_torque(left_leg_force, 0.0);
        auto right_hip_control_torque =
            right_leg_vmc_solver_.update_joint_torque(right_leg_force, 0.0);

        *left_front_hip_control_torque_ = clamp_hip_control_torque(-left_hip_control_torque.x());
        *left_back_hip_control_torque_ = clamp_hip_control_torque(left_hip_control_torque.y());
        *right_front_hip_control_torque_ = clamp_hip_control_torque(-right_hip_control_torque.x());
        *right_back_hip_control_torque_ = clamp_hip_control_torque(right_hip_control_torque.y());
    }

    Eigen::Vector2d yaw_control(double ctrl_vel) {
        Eigen::Vector2d ctrl_torque;
        double u_turn = yaw_speed_pid_.update(ctrl_vel - *imu_d_yaw_);
        ctrl_torque(0) = u_turn;
        ctrl_torque(1) = -u_turn;
        return ctrl_torque;
    }
    void update_hip_and_wheel_torques(
        Eigen::Vector2d leg_forces, Eigen::Vector2d left_control_torques,
        Eigen::Vector2d right_control_torques, Eigen::Vector2d yaw_ctrl_torques,
        LegPosture leg_posture) {
        double left_leg_force = leg_forces(0);
        double right_leg_force = leg_forces(1);

        double left_wheel_control_torque = left_control_torques(0);
        double left_leg_control_torque = left_control_torques(1);
        double right_wheel_control_torque = right_control_torques(0);
        double right_leg_control_torque = right_control_torques(1);
        double anti_crash_ =
            anti_crash_pid_.update(leg_posture.tilt_angle(0) - leg_posture.tilt_angle(1));
        auto left_hip_control_torque = left_leg_vmc_solver_.update_joint_torque(
            left_leg_force, left_leg_control_torque - anti_crash_);
        auto right_hip_control_torque = right_leg_vmc_solver_.update_joint_torque(
            right_leg_force, right_leg_control_torque + anti_crash_);

        // RCLCPP_INFO(
        //     get_logger(), "lf: %f, lb: %f, rf: %f, rb: %f,control_torque: %f",
        //     left_hip_control_torque.x(), left_hip_control_torque.y(),
        //     right_hip_control_torque.x(), right_hip_control_torque.y(), torque);

        // if (levitate_active_) {
        //     *left_wheel_control_torque_ = 0.0;
        //     *right_wheel_control_torque_ = 0.0;
        // }

        *left_wheel_control_torque_ =
            clamp_wheel_control_torque(left_wheel_control_torque) + yaw_ctrl_torques(0);
        *right_wheel_control_torque_ =
            clamp_wheel_control_torque(right_wheel_control_torque) + yaw_ctrl_torques(1);

        *left_front_hip_control_torque_ = clamp_hip_control_torque(-left_hip_control_torque.x());
        *left_back_hip_control_torque_ = clamp_hip_control_torque(left_hip_control_torque.y());
        *right_front_hip_control_torque_ = clamp_hip_control_torque(-right_hip_control_torque.x());
        *right_back_hip_control_torque_ = clamp_hip_control_torque(right_hip_control_torque.y());

        // RCLCPP_INFO(
        //     get_logger(), "lf: %f, lb: %f, rb: %f, rf: %f", *left_front_hip_control_torque_,
        //     *left_back_hip_control_torque_, *right_back_hip_control_torque_,
        //     *right_front_hip_control_torque_);
    }

    static double clamp_wheel_control_torque(const double& torque) {
        return std::clamp(torque, -5.4, 5.4);
    }

    static double clamp_hip_control_torque(const double& torque) {
        return std::clamp(torque, -20.0, 20.0);
    }
    static double clamp_leg_control_velocity(const double& velocity) {
        return std::clamp(velocity, -pi_, pi_);
    }

    void reset_control_torques() {
        *left_front_hip_control_torque_ = nan_;
        *left_back_hip_control_torque_ = nan_;
        *right_front_hip_control_torque_ = nan_;
        *right_back_hip_control_torque_ = nan_;
    }

    InputInterface<double> left_wheel_velocity_;
    InputInterface<double> right_wheel_velocity_;

    InputInterface<double> left_front_hip_angle_;
    InputInterface<double> left_back_hip_angle_;
    InputInterface<double> right_front_hip_angle_;
    InputInterface<double> right_back_hip_angle_;

    InputInterface<double> left_front_hip_velocity_;
    InputInterface<double> left_back_hip_velocity_;
    InputInterface<double> right_front_hip_velocity_;
    InputInterface<double> right_back_hip_velocity_;

    InputInterface<double> left_front_hip_torque_;
    InputInterface<double> left_back_hip_torque_;
    InputInterface<double> right_front_hip_torque_;
    InputInterface<double> right_back_hip_torque_;

    OutputInterface<double> left_front_hip_control_torque_;
    OutputInterface<double> left_back_hip_control_torque_;
    OutputInterface<double> right_front_hip_control_torque_;
    OutputInterface<double> right_back_hip_control_torque_;

    OutputInterface<double> left_wheel_control_torque_;
    OutputInterface<double> right_wheel_control_torque_;

    OutputInterface<double> dbg_L0_[2], dbg_dL0_[2], dbg_ddL0_[2], dbg_theta_[2], dbg_dtheta_[2],
        dbg_sportFN[2], dbg_vel, dbg_filtered_vel;

    InputInterface<rmcs_msgs::Switch> switch_left_;

    double left_phi1_offset_, left_phi4_offset_, right_phi1_offset_, right_phi4_offset_;
    double robot_weight_per_leg_;
    double gas_spring_;

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double pi_ = std::numbers::pi;
    static constexpr double dt_ = 1e-3;
    static constexpr double g_ = 9.80665;
    static constexpr double sigma_q_ = 1.0;
    static constexpr double sigma_v_ = 0.75;
    static constexpr double sigma_a_ = 1.0;

    bool is_rescue_tip_over_ = true;

    const Eigen::Matrix2d A_ = (Eigen::Matrix2d() << 1, dt_, 1, 1).finished();
    const Eigen::Matrix2d H_ = Eigen::Vector2d::Identity().asDiagonal();
    const Eigen::Matrix2d W_ = Eigen::Vector2d{0.5 * dt_ * dt_, dt_}.asDiagonal();
    const Eigen::Matrix2d Q_ =
        Eigen::Vector2d{sigma_q_ * sigma_q_, sigma_q_ * sigma_q_}.asDiagonal();
    const Eigen::Matrix2d R_ =
        Eigen::Vector2d{sigma_v_ * sigma_v_, sigma_a_ * sigma_a_}.asDiagonal();

    utility::KalmanFilter<2, 2> velocity_kalman_filter_;

    InputInterface<double> joint_pos_[2][2];
    InputInterface<double> joint_vel_[2][2];
    InputInterface<double> imu_pitch_, imu_d_pitch_, imu_yaw_, imu_d_yaw_, imu_roll_, imu_ax_,
        imu_az_, imu_ddz_, imu_ddx_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    InputInterface<double> rotary_knob_;
    InputInterface<double> t_joint_[2][2];
    OutputInterface<double> ctl_t_joint_[2][2];

    LegStatus legs_[2];

    const double body_mass_;
    const double leg_mass_;
    const double wheel_mass_;
    const double wheel_radius_;
    const double wheel_distance_;
    const double gas_spring;
    const double centroid_position_coefficient_;

    double current_target_L0 = 0.20;
    double desire_leg_length_ = 0.2;
    double desire_roll_angle_ = 0.0;
    double desire_speed = 0.0;
    double last_desire_speed = 0.0;
    double desire_distance = 0.0;

    double distance;
    double last_distance_;

    bool last_is_parked_{false};

    VmcSolver left_leg_vmc_solver_, right_leg_vmc_solver_;
    Eigen::Vector2d last_leg_length_, last_tilt_angle_;
    Eigen::Vector2d last_dot_leg_length_, last_dot_tilt_angle_;

    pid::PidCalculator roll_angle_pid_calculator_, leg_length_pid_calculator_;

    // rmcs_core::controller::pid::PidCalculator leg_Left_L0_pid_;
    // rmcs_core::controller::pid::PidCalculator leg_Right_L0_pid_;
    // rmcs_core::controller::pid::PidCalculator leg_Left_Tp_pid_;
    // rmcs_core::controller::pid::PidCalculator leg_Right_Tp_pid_;
    rmcs_core::controller::pid::PidCalculator anti_crash_pid_;
    // rmcs_core::controller::pid::PidCalculator roll_pid_;
    rmcs_core::controller::pid::PidCalculator yaw_speed_pid_;
    rmcs_core::controller::pid::PidCalculator rescue_velocity_pid_calculator_;
    rmcs_core::controller::pid::PidCalculator rescue_angle_pid_calculator_;
    rmcs_core::controller::pid::PidCalculator rescue_length_pid_calculator_;
    rmcs_core::utility::LowPassFilter<> imu_pitch_filter{20.0f, 1000.0f};
    rmcs_core::utility::LowPassFilter<> ddot_z_w_filter{37.3f, 1000.0f};
    rmcs_core::utility::LowPassFilter<2> leg_ddphi0_filter{10.0f, 1000.0f};
    rmcs_core::utility::LowPassFilter<2> leg_ddl0_filter{6.0f, 1000.0f};

    // double pitch_filter = 0.0;
    // double body_velocity = 0.0;
    // double current_target_vel = 0.0;
    double filtered_velocity = 0.0;
    // double dist_ = 0.0;

    // double target_L0;
    // double wheel_radius_ = 0.058;           // 轮子半径
    // double reduction_ratio_ = 268.0 / 17.0; // 减速比
    // double wheel_weight_ = 0.5;             // 轮子重量
    // double roll_k_;

    // rclcpp::Time last_time_;

    double normalize_angle(double angle) {
        double a = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
        if (a < 0)
            a += 2.0 * std::numbers::pi;
        return a - std::numbers::pi;
    }

    using KF = rmcs_core::utility::KalmanFilter<2, 2, 0>;

    void kf_init_() {
        // 状态量：[电机角度速度, 机体前进加速度] (State=2)
        // 观测量：[电机角度速度, 机体前进加速度] (Measure=2)
        // 控制量：[无] (Control=0)

        // 2. 采样时间设置 (例如 1ms 采样周期，根据你的实际定时器调整)
        double dt = 0.001;

        // 3. 初始化状态转移矩阵 A (v_k = v_{k-1} + a_{k-1} * dt)
        Eigen::Matrix2d A;
        A << 1.0, dt, 0.0, 1.0;

        // 4. 初始化观测矩阵 H (直接观测到速度和加速度)
        Eigen::Matrix2d H;
        H << 1.0, 0.0, 0.0, 1.0;

        // 5. 配置过程噪声 Q (物理模型的不确定性)
        double vel_process_noise = 15.0;
        double acc_process_noise = 1000.0;

        Eigen::Matrix2d Q;
        Q << vel_process_noise, 0.0, 0.0, acc_process_noise;

        // 6. 配置测量噪声 R (传感器的固有白噪声)
        double vel_measure_noise = 1000.0;
        double acc_measure_noise = 0.01;

        Eigen::Matrix2d R;
        R << vel_measure_noise, 0.0, 0.0, acc_measure_noise;

        // 7. 实例化卡尔曼滤波器
        // 使用模板参数指定维度：state=2, measure=2, control=0
        kf_ = std::make_unique<KF>(A, H, Q, R);
    }

    std::unique_ptr<KF> kf_;
    double OnSensorDataReceived(double raw_wheel_speed, double raw_imu_accel) {
        // 创建测量向量 z = [v_m, a_m]^T
        Eigen::Vector2d measurement;
        measurement << static_cast<double>(raw_wheel_speed), raw_imu_accel;

        // 调用更新方程，自动完成【预测】与【矫正】
        // 由于 CONTROL_DIM 为 0，不需要传第二个参数
        Eigen::Vector2d optimal_state = kf_->update(measurement);

        // 提取融合后的最优结果
        filtered_velocity = optimal_state(0);            // 融合后的干净速度
        double filtered_acceleration = optimal_state(1); // 融合后的干净加速度
        // RCLCPP_INFO(get_logger(), "vel %f, velb %f", filtered_velocity, body_velocity);

        // double moto_vel = *wheel_vel_[side];
        // double vel_ = moto_vel * (17.0 / 268.0) * 0.058  ;

        return filtered_velocity;
    }
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelegController, rmcs_executor::Component)

/*body_fusion = 1;    %机体速度
g = 9.8;            %重力加速度
R = 0.058;          %轮半径
m_w = 0.24; %轮质量
m _p = 0.845;        %摆杆质量
M = 10.5;           %机体质量
I_w = 0.00037840; %轮转动惯量
I_M =  0.277;                 %机体转动惯量
l = 0.0;                  %机体质心离转轴距离

Q_cost=diag([4000 1 1500 1 20000 1]);
R_cost=diag([15, 1]);*/
