
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "utility/kalman_filter.hpp"
#include "utility/low_pass_filter.hpp"
#include <rmcs_description/tf_description.hpp>

namespace rmcs_core::controller::chassis {

class WheelegController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelegController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , leg_Left_L0_pid_(60.0, 0.0, 50.0)
        , leg_Right_L0_pid_(60.0, 0.0, 50.0)

        , leg_Left_Tp_pid_(15.0, 0.0, 5.0)
        , leg_Right_Tp_pid_(15.0, 0.0, 5.0)

        , anti_crash_pid_(40.0, 0.0, 0.2)
        , yaw_speed_pid_(0.2, 0.0, 5.0) {

        l1_ = 0.210;
        l2_ = 0.250;
        l3_ = 0.250;
        l4_ = 0.210;
        l5_ = 0.0;

        roll_k_ = 1000.0;

        left_phi1_offset_ = std::numbers::pi - 0.1;
        left_phi4_offset_ = std::numbers::pi - 0.1;
        right_phi1_offset_ = std::numbers::pi - 0.1;
        right_phi4_offset_ = std::numbers::pi - 0.1;

        last_time_ = this->get_clock()->now();
        robot_weight_per_leg_ = -50.0; // 重力补偿

        wheel_weight_ = 0.5;               // 轮子重量

        register_input("/chassis/left_back_hip/angle", joint_pos_[LEFT][FRONT]);
        register_input("/chassis/left_front_hip/angle", joint_pos_[LEFT][BACK]);
        register_input("/chassis/right_back_hip/angle", joint_pos_[RIGHT][FRONT]);
        register_input("/chassis/right_front_hip/angle", joint_pos_[RIGHT][BACK]);

        register_input("/chassis/left_back_hip/velocity", joint_vel_[LEFT][FRONT]);
        register_input("/chassis/left_front_hip/velocity", joint_vel_[LEFT][BACK]);
        register_input("/chassis/right_back_hip/velocity", joint_vel_[RIGHT][FRONT]);
        register_input("/chassis/right_front_hip/velocity", joint_vel_[RIGHT][BACK]);

        register_input("/chassis/imu/pitch", imu_pitch_, false);
        register_input("/chassis/imu/pitch_velocity", imu_d_pitch_, false);
        register_input("/chassis/imu/yaw_velocity", imu_d_yaw_, false);
        register_input("/chassis/imu/roll", imu_roll_);
        register_input("/debug/imu/ddz", imu_ddx_);
        register_input("/debug/imu/ddz", imu_ddz_);

        register_input("/chassis/control_velocity", chassis_control_velocity_, false);
        register_input("/remote/rotary_knob", rotary_knob_, false);

        register_input("/chassis/left_wheel/velocity", wheel_vel_[LEFT], false);
        register_input("/chassis/right_wheel/velocity", wheel_vel_[RIGHT], false);
        register_output("/chassis/left_wheel/control_torque", t_wheel_[LEFT]);
        register_output("/chassis/right_wheel/control_torque", t_wheel_[RIGHT]);

        register_output("/chassis/left_back_hip/control_torque", t_joint_[LEFT][FRONT]);
        register_output("/chassis/left_front_hip/control_torque", t_joint_[LEFT][BACK]);
        register_output("/chassis/right_back_hip/control_torque", t_joint_[RIGHT][FRONT]);
        register_output("/chassis/right_front_hip/control_torque", t_joint_[RIGHT][BACK]);

        register_output("/debug/left/L0", dbg_L0_[LEFT]);
        register_output("/debug/right/L0", dbg_L0_[RIGHT]);
        register_output("/debug/left/Theta", dbg_theta_[LEFT]);
        register_output("/debug/right/Theta", dbg_theta_[RIGHT]);
        register_output("/debug/left/dTheta", dbg_dtheta_[LEFT]);
        register_output("/debug/right/dTheta", dbg_dtheta_[RIGHT]);
        register_output("/debug/left/sportFN", dbg_sportFN[LEFT]);
        register_output("/debug/right/sportFN", dbg_sportFN[RIGHT]);
        register_output("/debug/vel", dbg_vel);
        register_output("/debug/vel_filterd", dbg_vel_filtered);
    }

    void update() override {

        // 遥控器部分

        double rc_cmd_v = 0.0;
        double rc_cmd_w = 0.0;

        if (std::isnan(chassis_control_velocity_->vector[0])) {
            reset_all_controls();
            return;
        }

        auto vel_vec = (*chassis_control_velocity_).vector;
        rc_cmd_v = -vel_vec[0];
        rc_cmd_w = -vel_vec[1];

        rc_cmd_v = std::clamp(rc_cmd_v, -1.5, 1.5);
        rc_cmd_w = std::clamp(rc_cmd_w, -20.0, 20.0);

        double knob_val = 0.0;
        if (rotary_knob_.ready()) {
            knob_val = -*rotary_knob_;
        }
        if (std::abs(knob_val) < 0.05) {
            knob_val = 0.0;
        }

        double min_L0 = 0.16;             // 最矮状态
        double max_L0 = 0.32;             // 最高状态
        double L0_speed_multiplier = 0.5; // 伸缩速度倍率

        // 默认高度

        current_target_L0 += knob_val * L0_speed_multiplier * 0.001;

        current_target_L0 = std::clamp(current_target_L0, min_L0, max_L0);

        target_L0 = current_target_L0;

        double target_roll = 0.0;

        // 高度调节结束

        if (std::abs(rc_cmd_v) < 0.05)
            rc_cmd_v = 0.0;
        if (std::abs(rc_cmd_w) < 0.05)
            rc_cmd_w = 0.0;

        // 防止突变
        bool is_braking = (std::abs(rc_cmd_v) < std::abs(current_target_vel)) || (rc_cmd_v == 0.0);

        double accel_step = 3.0 * 0.001;
        double brake_step = 10.0 * 0.001;

        double step = is_braking ? brake_step : accel_step;

        if (rc_cmd_v > current_target_vel + step) {
            current_target_vel += step;
        } else if (rc_cmd_v < current_target_vel - step) {
            current_target_vel -= step;
        } else {
            current_target_vel = rc_cmd_v;
        }

        double target_vel = current_target_vel;

        pitch_filter = imu_pitch_filter.update(*imu_pitch_);

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0.0 || dt > 0.1)
            dt = 0.001;

        process_kinematics(LEFT);
        process_kinematics(RIGHT);


        velocity_calculate();
        kf_init_();
        OnSensorDataReceived(body_velocity, -*imu_ddx_);
        calculate_lqr_gains(LEFT, L0[LEFT], target_vel, filtered_velocity);
        calculate_lqr_gains(RIGHT, L0[RIGHT], target_vel, filtered_velocity);

        // 腿部PID计算
        double F_L = 0, Tp_L = 0;
        double F_R = 0, Tp_R = 0;

        double anti_crash_ = anti_crash_pid_.update(legs_[LEFT].theta - legs_[RIGHT].theta);
        // double anti_crash_ = 0.0;

        double right_leg_L0_err_;
        double left_leg_L0_err_;
        double roll_err_force_;
        double robot_weight_per_leg;

        if (sportFN[LEFT] < 20.0 || sportFN[RIGHT] < 20.0) {
            roll_err_force_ = 0.0;
            robot_weight_per_leg = 0.0;

        } else {
            roll_err_force_ = (target_roll - *imu_roll_) * roll_k_;
            robot_weight_per_leg = robot_weight_per_leg_;
        }

        if (legs_[LEFT].solved) {
            left_leg_L0_err_ = target_L0 - legs_[LEFT].L0;

            F_L =
                leg_Left_L0_pid_.update(left_leg_L0_err_) + robot_weight_per_leg_ - roll_err_force_;
            Tp_L = Tp[LEFT] + anti_crash_;
        }

        if (legs_[RIGHT].solved) {

            right_leg_L0_err_ = target_L0 - legs_[RIGHT].L0;

            F_R = leg_Right_L0_pid_.update(right_leg_L0_err_) + robot_weight_per_leg_
                + roll_err_force_;
            Tp_R = Tp[RIGHT] - anti_crash_;
        }

        double T_L_Back, T_L_Front, T_R_Back, T_R_Front;
        map_vmc_force(LEFT, F_L, Tp_L, T_L_Back, T_L_Front);
        map_vmc_force(RIGHT, F_R, Tp_R, T_R_Back, T_R_Front);

        *t_joint_[LEFT][FRONT] = T_L_Back;
        *t_joint_[LEFT][BACK] = T_L_Front;
        *t_joint_[RIGHT][FRONT] = T_R_Back;
        *t_joint_[RIGHT][BACK] = T_R_Front;

        *dbg_L0_[LEFT] = legs_[LEFT].L0;
        *dbg_L0_[RIGHT] = legs_[RIGHT].L0;
        *dbg_theta_[LEFT] = legs_[LEFT].theta;
        *dbg_theta_[RIGHT] = legs_[RIGHT].theta;
        *dbg_dtheta_[LEFT] = kinematic_w_theta[LEFT];
        *dbg_dtheta_[RIGHT] = kinematic_w_theta[RIGHT];
        *dbg_sportFN[LEFT] = sportFN[LEFT];
        *dbg_sportFN[RIGHT] = sportFN[RIGHT];
        *dbg_vel = body_velocity;
        *dbg_vel_filtered = filtered_velocity;

        // 差动转向控制

        // double K_turn = 0.0;
        // double u_turn = rc_cmd_w * K_turn;

        double u_turn = yaw_speed_pid_.update((-rc_cmd_w) * 1.0 - *imu_d_yaw_);

        if (sportFN[LEFT] < 20) {
            *t_wheel_[LEFT] = (wheelT[LEFT] * 17.0 / 268.0);
        } else {
            *t_wheel_[LEFT] = (wheelT[LEFT] * 17.0 / 268.0) + u_turn;
        }

        if (sportFN[RIGHT] < 20) {
            *t_wheel_[RIGHT] = (wheelT[RIGHT] * 17.0 / 268.0);
        } else {
            *t_wheel_[RIGHT] = (wheelT[RIGHT] * 17.0 / 268.0) - u_turn;
        }
    }

private:
    struct LegStatus {
        bool solved = false;
        double phi1, phi4;
        double L0, phi0, theta;
        double dL0, d_theta;
        double last_L0;
        double J11, J12, J21, J22;
    };

    enum Side { LEFT = 0, RIGHT = 1 };
    enum Joint { BACK = 0, FRONT = 1 };

    double l1_, l2_, l3_, l4_, l5_;
    double left_phi1_offset_, left_phi4_offset_, right_phi1_offset_, right_phi4_offset_;
    double robot_weight_per_leg_;

    InputInterface<double> joint_pos_[2][2];
    InputInterface<double> joint_vel_[2][2];
    InputInterface<double> imu_pitch_, imu_d_pitch_, imu_d_yaw_, imu_roll_, imu_ddz_, imu_ddx_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    InputInterface<double> rotary_knob_;
    OutputInterface<double> t_joint_[2][2];
    OutputInterface<double> dbg_L0_[2], dbg_theta_[2], dbg_dtheta_[2], dbg_sportFN[2], dbg_vel,
        dbg_vel_filtered;

    LegStatus legs_[2];
    rmcs_core::controller::pid::PidCalculator leg_Left_L0_pid_;
    rmcs_core::controller::pid::PidCalculator leg_Right_L0_pid_;
    rmcs_core::controller::pid::PidCalculator leg_Left_Tp_pid_;
    rmcs_core::controller::pid::PidCalculator leg_Right_Tp_pid_;
    rmcs_core::controller::pid::PidCalculator anti_crash_pid_;
    rmcs_core::controller::pid::PidCalculator roll_pid_;
    rmcs_core::controller::pid::PidCalculator yaw_speed_pid_;
    rmcs_core::utility::LowPassFilter<> imu_pitch_filter{20.0f, 1000.0f};
    rmcs_core::utility::LowPassFilter<> ddot_z_w_filter{37.3f, 1000.0f};

    double pitch_filter = 0.0;
    double body_velocity = 0.0;
    double current_target_vel = 0.0;
    double filtered_velocity = 0.0;
    double dist_ = 0.0;
    double current_target_L0 = 0.13;
    double target_L0;
    double wheel_radius_ = 0.058;           // 轮子半径
    double reduction_ratio_ = 268.0 / 17.0; // 减速比
    double wheel_weight_ = 0.5;             // 轮子重量
    double roll_k_;
    double kinematic_theta[2];
    double kinematic_w_theta[2];
    double D_L0[2];
    double D_phi2[2];
    double L0[2];
    double Tp[2];
    double sportFN[2];
    double wheelT[2];

    InputInterface<double> wheel_vel_[2];
    OutputInterface<double> t_wheel_[2];

    rclcpp::Time last_time_;
    void reset_all_controls() {
        current_target_vel = 0.0;
        *t_wheel_[LEFT] = 0.0;
        *t_wheel_[RIGHT] = 0.0;
        *t_joint_[LEFT][FRONT] = 0.0;
        *t_joint_[LEFT][BACK] = 0.0;
        *t_joint_[RIGHT][FRONT] = 0.0;
        *t_joint_[RIGHT][BACK] = 0.0;
        current_target_L0 = 0.13;
    }
    void velocity_calculate() {
        double left_w_wheel_ =
            (*wheel_vel_[LEFT] / reduction_ratio_) + D_phi2[LEFT] - *imu_d_pitch_;
        double right_w_wheel_ =
            (*wheel_vel_[RIGHT] / reduction_ratio_) + D_phi2[RIGHT] - *imu_d_pitch_;

        double left_v_body_ = left_w_wheel_ * wheel_radius_ + L0[LEFT] * kinematic_theta[LEFT] * std::cos(kinematic_theta[LEFT])
                            + D_L0[LEFT] * std::sin(kinematic_theta[LEFT]);

        double right_v_body_ = right_w_wheel_ * wheel_radius_ + L0[RIGHT] * kinematic_theta[RIGHT] * std::cos(kinematic_theta[RIGHT])
                             + D_L0[RIGHT] * std::sin(kinematic_theta[RIGHT]);
        double vel_m = (left_v_body_ + right_v_body_) / 2.0;
        if (sportFN[LEFT] < 20.0 && sportFN[RIGHT] < 20.0f) {
            vel_m = 0;
        }
        body_velocity = vel_m;
    }

    void calculate_lqr_gains(int side, double L0, double vel, double vel_) {

        double w_theta = kinematic_w_theta[side];
        double theta = kinematic_theta[side];
        static const Eigen::Matrix<double, 12, 3> K_poly{
            {    60.4638,   -70.3176,    -6.5438},
            {    -3.6340,    -7.0609,    -0.1953},
            {    36.5273,   -29.7571,    -2.9203},
            {    14.0516,   -14.4658,    -2.9994},
            {    68.4233,   -84.0850,    34.8581},
            {    10.8650,   -10.7467,     4.0707},
            {    24.8887,   -73.1983,    52.5905},
            {   -16.0217,    10.5177,     2.7448},
            {    70.8964,   -92.2227,    39.4185},
            {    75.8473,   -76.9852,    28.0531},
            {  -492.2891,   393.2205,    32.6987},
            {   -58.2475,    45.4204,    -1.9708}
        };

        double d_theta;

        d_theta = w_theta;

        // RCLCPP_INFO(get_logger(), "dist %f, vel %f", dist_, vel_);

        Eigen::Vector3d L_vec(L0 * L0, L0, 1.0);
        Eigen::Matrix<double, 12, 1> K_flat = K_poly * L_vec;

        Eigen::Matrix<double, 2, 6> K =
            Eigen::Map<const Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>(K_flat.data());

        Eigen::Matrix<double, 6, 1> x = {theta,       d_theta,        -0.0,
                                         (vel - vel_), -(*imu_pitch_), *imu_d_pitch_};
        // double FN = sportFN[side];
        // if (FN < 20.0) {
        //     K.row(0).setZero();
        //     K.row(1).tail<4>().setZero();
        // }
        Eigen::Vector2d u = K * x;
        wheelT[side] = u(0);
        Tp[side] = u(1);
    }

    double normalize_angle(double angle) {
        double a = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
        if (a < 0)
            a += 2.0 * std::numbers::pi;
        return a - std::numbers::pi;
    }

    void process_kinematics(int side) {
        LegStatus& leg = legs_[side];
        double raw_back = *joint_pos_[side][BACK];
        double raw_front = *joint_pos_[side][FRONT];

        double w_phi1_ = *joint_vel_[side][BACK];
        double w_phi4_ = *joint_vel_[side][FRONT];

        double last_w_theta_;
        double last_v_l0_;
        double ddot_z_M_ = *imu_ddz_;
        double ddot_z_w_;

        if (side == LEFT) {
            leg.phi1 = normalize_angle(left_phi1_offset_ - raw_back);
            leg.phi4 = normalize_angle(left_phi4_offset_ - raw_front);
        } else {
            leg.phi1 = normalize_angle(right_phi1_offset_ + raw_back);
            leg.phi4 = normalize_angle(right_phi4_offset_ + raw_front);
        }

        double xb = l1_ * std::cos(leg.phi1);
        double yb = l1_ * std::sin(leg.phi1);
        double xd = l5_ + l4_ * std::cos(leg.phi4);
        double yd = l4_ * std::sin(leg.phi4);

        double bd = std::pow(xd - xb, 2) + std::pow(yd - yb, 2);
        double a0 = 2 * l2_ * (xd - xb);
        double b0 = 2 * l2_ * (yd - yb);

        leg.solved = true;

        double phi2 =
            2
            * std::atan2(
                b0 + std::sqrt(std::pow(a0, 2) + std::pow(b0, 2) - std::pow(bd, 2)), a0 + bd);
        double xc = l1_ * std::cos(leg.phi1) + l2_ * std::cos(phi2);
        double yc = l1_ * std::sin(leg.phi1) + l2_ * std::sin(phi2);

        leg.L0 = std::sqrt(std::pow(xc - l5_ / 2.0, 2) + std::pow(yc, 2));
        double phi0 = std::atan2(yc, xc - l5_ / 2.0);
        leg.theta = phi0 - std::numbers::pi / 2.0 + *imu_pitch_;
        kinematic_theta[side] = leg.theta;
        L0[side] = leg.L0;
        double phi3 = std::atan2(yb - yd + l2_ * std::sin(phi2), xb - xd + l2_ * std::cos(phi2));

        double predict_dt = 0.001;
        float phi1_pred = leg.phi1 + w_phi1_ * predict_dt; // 预测下一时刻的关节角度(利用关节角速度)
        float phi4_pred = leg.phi4 + w_phi4_ * predict_dt;
        // 重新计算腿长和腿角度
        double xb_ = l1_ * std::cos(phi1_pred);
        double yb_ = l1_ * std::sin(phi1_pred);
        double xd_ = l5_ + l1_ * std::cos(phi4_pred);
        double yd_ = l1_ * std::sin(phi4_pred);
        double bd_ = std::pow(xd_ - xb_, 2) + std::pow(yd_ - yb_, 2);
        double a0_ = 2 * l2_ * (xd_ - xb_);
        double b0_ = 2 * l2_ * (yd_ - yb_);

        double phi2_pred =
            2
            * std::atan2(
                b0_ + std::sqrt(std::pow(a0_, 2) + std::pow(b0_, 2) - std::pow(bd_, 2)), a0_ + bd_);
        double xc_ = xb_ + l2_ * std::cos(phi2_pred);
        double yc_ = yb_ + l2_ * std::sin(phi2_pred);
        float phi0_pred = std::atan2(yc_, xc_ - l5_ / 2);
        // 差分计算腿长变化率和腿角速度
        double w_phi2_ = (phi2_pred - phi2) / predict_dt;
        double w_phi0_ = (phi0_pred - phi0) / predict_dt;
        double v_l0_ =
            ((std::sqrt(std::pow(xc_ - l5_ / 2, 2) + std::pow(yc_, 2))) - leg.L0) / predict_dt;
        double w_theta_;
        if (side == LEFT) {
            w_theta_ = (phi0_pred - 0.5 * std::numbers::pi + *imu_pitch_ - leg.theta) / predict_dt;
        } else {
            w_theta_ = -(phi0_pred - 0.5 * std::numbers::pi + *imu_pitch_ - leg.theta) / predict_dt;
        }

        kinematic_w_theta[side] = w_theta_;

        double v_height_ = v_l0_ * std::cos(leg.theta) - leg.L0 * std::sin(leg.theta) * w_theta_;
        double dot_v_l0_ =
            (v_l0_ - last_v_l0_) / (0.002 + 0.008) + dot_v_l0_ * 0.008 / (0.002 + 0.008);
        double dotw_theta_ =
            (w_theta_ - last_w_theta_) / (0.002 + 0.008) + dotw_theta_ * 0.008 / (0.002 + 0.008);
        ddot_z_w_ = ddot_z_M_ - dot_v_l0_ * std::cos(leg.theta)
                  + 2.0f * v_l0_ * w_theta_ * std::sin(leg.theta)
                  + leg.L0 * dotw_theta_ * std::cos(leg.theta)
                  + leg.L0 * powf(w_theta_, 2) * std::sin(leg.theta);
        last_w_theta_ = w_theta_;
        last_v_l0_ = v_l0_;

        D_L0[side] = v_l0_;
        D_phi2[side] = w_phi2_;

        // 雅可比

        double phi32 = std::sin(phi3 - phi2);
        double phi03 = phi0 - phi3;
        double phi02 = phi0 - phi2;
        // if (std::abs(phi32) < 1e-4)
        //     phi32 = 1e-4;

        leg.J11 = (l1_ * std::sin(phi03) * std::sin(leg.phi1 - phi2)) / phi32;
        leg.J21 = (l1_ * std::cos(phi03) * std::sin(leg.phi1 - phi2)) / (leg.L0 * phi32);
        leg.J12 = (l4_ * std::sin(phi02) * std::sin(phi3 - leg.phi4)) / phi32;
        leg.J22 = (l4_ * std::cos(phi02) * std::sin(phi3 - leg.phi4)) / (leg.L0 * phi32);

        double mea_t1_ = *t_joint_[side][FRONT];
        double mea_t2_ = *t_joint_[side][BACK];
        float det = leg.J11 * leg.J22 - leg.J21 * leg.J12;
        double J11 = leg.J22 / det;
        double J21 = -leg.J21 / det;
        double J12 = -leg.J12 / det;
        double J22 = leg.J11 / det;

        double mea_F_ = J11 * mea_t1_ + J21 * mea_t2_;
        double mea_Tp_ = J12 * mea_t1_ + J22 * mea_t2_;
        double p_ = mea_F_ * std::cos(leg.theta) + mea_Tp_ * std::sin(leg.theta) / leg.L0;
        if (side == LEFT) {
            sportFN[side] = p_ + wheel_weight_ * (9.8f + ddot_z_w_);
        } else {
            sportFN[side] = -p_ + wheel_weight_ * (9.8f + ddot_z_w_);
        }
    }

    void map_vmc_force(int side, double F, double Tp, double& T_Back, double& T_Front) {
        LegStatus& leg = legs_[side];

        double sign_F_Back, sign_F_Front;
        double sign_Tp_Back, sign_Tp_Front;
        // 电机转向调节
        if (side == LEFT) {
            sign_F_Back = 1.0;
            sign_F_Front = 1.0;
            sign_Tp_Back = 1.0;
            sign_Tp_Front = 1.0;
        } else {
            sign_F_Back = -1.0;
            sign_F_Front = -1.0;
            sign_Tp_Back = -1.0;
            sign_Tp_Front = -1.0;
        }

        T_Back = (sign_F_Back * leg.J11 * F) + (sign_Tp_Back * leg.J21 * Tp);
        T_Front = (sign_F_Front * leg.J12 * F) + (sign_Tp_Front * leg.J22 * Tp);
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
        double vel_process_noise = 25.0;
        double acc_process_noise = 2000.0;

        Eigen::Matrix2d Q;
        Q << vel_process_noise, 0.0, 0.0, acc_process_noise;

        // 6. 配置测量噪声 R (传感器的固有白噪声)
        double vel_measure_noise = 800.0;
        double acc_measure_noise = 0.01;

        Eigen::Matrix2d R;
        R << vel_measure_noise, 0.0, 0.0, acc_measure_noise;

        // 7. 实例化卡尔曼滤波器
        // 使用模板参数指定维度：state=2, measure=2, control=0
        kf_ = std::make_unique<KF>(A, H, Q, R);
    }

    std::unique_ptr<KF> kf_;
    void OnSensorDataReceived(double raw_wheel_speed, double raw_imu_accel) {
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

        if (fabs(filtered_velocity) < 0.1) {
            dist_ += filtered_velocity * 0.001;
        } else {
            dist_ = 0.0f;
        }
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
