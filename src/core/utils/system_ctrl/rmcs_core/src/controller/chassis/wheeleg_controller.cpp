
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
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
        , leg_L0_pid_(800.0, 0.0, 15.0)
        , leg_Right_L0_pid_(800.0, 0.0, 15.0)
        , leg_Left_Tp_pid_(15.0, 0.0, 5.0)
        , leg_Right_Tp_pid_(15.0, 0.0, 5.0)
        , anti_crash_pid_(30.0, 0.0, 5.0) {

        RCLCPP_INFO(get_logger(), "=== Step 3.5: VMC with High-Precision Gyro ===");

        l1_ = 0.210;
        l2_ = 0.250;
        l3_ = 0.250;
        l4_ = 0.210;
        l5_ = 0.0;

        left_phi1_offset_ = std::numbers::pi;
        left_phi4_offset_ = std::numbers::pi;
        right_phi1_offset_ = std::numbers::pi;
        right_phi4_offset_ = std::numbers::pi;

        odom_.track_width = 0.36;    // 轮距
        last_time_ = this->get_clock()->now();
        robot_weight_per_leg_ = 0.0; // 重力补偿

        wheel_weight_ = 0.5;         // 轮子重量

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

        // register_output("/chassis/Right_leg_L0_err", right_leg_L0_err_,0.0);
        // register_output("/chassis/Left_leg_L0_err", left_leg_L0_err_,0.0);

        // register_input("/chassis/Right_leg_F0", right_F0_, false);
        // register_input("/chassis/Left_leg_F0", left_F0_, false);

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
        register_output("/debug/left/Theta", dbg_theta_[LEFT]);
        register_output("/debug/right/Theta", dbg_theta_[RIGHT]);
    }

    void update() override {

        double raw_pitch = *imu_pitch_;
        double raw_d_pitch = *imu_d_pitch_;

        // 遥控器部分

        double rc_cmd_v = 0.0;
        double rc_cmd_w = 0.0;
        bool is_killed = false;

        if (chassis_control_velocity_.ready()) {
            auto vel_vec = (*chassis_control_velocity_).vector;
            if (std::isnan(vel_vec[0]) || std::isnan(vel_vec[2])) {
                is_killed = true;
            } else {
                rc_cmd_v = -vel_vec[0];
                rc_cmd_w = -vel_vec[1];

                rc_cmd_v = std::clamp(rc_cmd_v, -0.5, 0.5);
                rc_cmd_w = std::clamp(rc_cmd_w, -20.0, 20.0);
            }
        }

        static double current_target_vel = 0.0;
        static double vel_integral = 0.0;

        if (is_killed) {
            current_target_vel = 0.0;
            vel_integral = 0.0;
            *t_wheel_[LEFT] = 0.0;
            *t_wheel_[RIGHT] = 0.0;
            return;
        }
        // 高度调节
        double knob_val = 0.0;
        if (rotary_knob_.ready()) {
            knob_val = -*rotary_knob_;
        }
        if (std::abs(knob_val) < 0.05) {
            knob_val = 0.0;
        }

        double min_L0 = 0.16;                   // 最矮状态
        double max_L0 = 0.38;                   // 最高状态
        double L0_speed_multiplier = 0.5;       // 伸缩速度倍率

        static double current_target_L0 = 0.30; // 默认高度

        current_target_L0 += knob_val * L0_speed_multiplier * 0.001;

        current_target_L0 = std::clamp(current_target_L0, min_L0, max_L0);

        double target_L0 = current_target_L0;
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

        double pitch = -raw_pitch;
        double d_pitch = raw_d_pitch;

        double target_vel = current_target_vel;

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0.0 || dt > 0.1)
            dt = 0.001;

        double v_L = 0, v_R_raw = 0;

        if (wheel_vel_[LEFT].ready())
            v_L = (*wheel_vel_[LEFT] / reduction_ratio_) * wheel_radius_;
        if (wheel_vel_[RIGHT].ready())
            v_R_raw = (*wheel_vel_[RIGHT] / reduction_ratio_) * wheel_radius_;

        double v_R = -v_R_raw;
        odom_.update(v_L, v_R, dt); // 里程计计算

        // 目前速度环使用的外环积分控制 LQR没调通 先用这个简单的 PI 来防止溜车，等 LQR 调通了再换
        // double vel_err = target_vel - odom_.v;

        // double Kp_v = 1.0;
        // double Ki_v = 0.02;
        // double max_pitch_tilt = 0.26;    // 最大倾斜角度

        // if (rc_cmd_v == 0.0) {
        //     vel_integral *= 0.2;
        // } else if (std::abs(odom_.v) < 1.0) {
        //     vel_integral = vel_integral * 0.999 + vel_err * dt;
        // }
        // if (-0.05 < std::abs(odom_.v) && std::abs(odom_.v) < 0.05) {
        //     vel_integral = vel_integral * 0.999 + vel_err * dt;
        // }
        // vel_integral = std::clamp(vel_integral, -max_pitch_tilt / Ki_v, max_pitch_tilt / Ki_v);

        // double pi_adjust_pitch = -(Kp_v * vel_err + Ki_v * vel_integral);

        // pi_adjust_pitch = std::clamp(pi_adjust_pitch, -max_pitch_tilt, max_pitch_tilt);

        // double base_pitch_offset = 0.04; // 0.16高度下的静态补偿

        // double dynamic_target_pitch = base_pitch_offset + pi_adjust_pitch;

        // LQR计算部分
        // double avg_L0 = std::clamp((legs_[LEFT].L0 + legs_[RIGHT].L0) / 2.0, 0.22, 0.40);
        // double K[12];
        // calculate_lqr_gains(avg_L0, K);
        // double x_err[6] = {0};

        // x_err[0] = pitch - dynamic_target_pitch;
        // x_err[1] = d_pitch - 0.0;
        // x_err[2] = 0.0;
        // x_err[3] = 0.0;
        // x_err[4] = 0.0;
        // x_err[5] = 0.0;

        // double u_wheel = 0.0;
        // for (int i = 0; i < 6; i++) {
        //     u_wheel += -K[i] * x_err[i];
        // }

        // double lqr_scale = 1.0; // 因为腿部并没有在和轮子有联动 所以LQR乘了一个比较大的倍率
        // double lqr_direction = -1.0;
        // u_wheel = u_wheel * lqr_scale * lqr_direction;'

        double u_wheel = 0.0;

        process_kinematics(LEFT);
        process_kinematics(RIGHT);
        calculate_lqr_gains(LEFT, L0[LEFT]);
        calculate_lqr_gains(RIGHT, L0[RIGHT]);
        // LegForceCalc(LEFT);
        // LegForceCalc(RIGHT);

        // 腿部摆角软限位
        double mech_limit = 0.0;                       // 限位值

        double center_phi = 0.0;                       // 机身坐标系下的垂直中点 (1.57)
        double min_phi_body = center_phi - mech_limit; // 机身系下的最小角度 (最前)
        double max_phi_body = center_phi + mech_limit; // 机身系下的最大角度 (最后)

        double target_theta_world = 0.0;

        double target_phi_body = center_phi;

        double safe_phi_body = std::clamp(target_phi_body, min_phi_body, max_phi_body);

        double safe_target_theta = 0.0;

        // 腿部PID计算
        double F_L = 0, Tp_L = 0;
        double F_R = 0, Tp_R = 0;
        double right_F0_;
        double left_F0_;

        // double anti_crash_ = anti_crash_pid_.update(legs_[LEFT].theta - legs_[RIGHT].theta);
        double anti_crash_ = 0.0;
        double right_leg_L0_err_;
        double left_leg_L0_err_;
        if (legs_[LEFT].solved) {
            left_leg_L0_err_ = target_L0 - legs_[LEFT].L0;

            F_L = leg_L0_pid_.update(left_leg_L0_err_) + robot_weight_per_leg_;
            // F_L = 15.0;

            // double left_angle_err = safe_target_theta - legs_[LEFT].theta;
            // Tp_L = leg_Left_Tp_pid_.update(left_angle_err);
            // Tp_L = 0.0;
            Tp_L = Tp[LEFT] * 1.5 - anti_crash_;
            // Tp_L = -anti_crash_;
        }

        if (legs_[RIGHT].solved) {

            right_leg_L0_err_ = target_L0 - legs_[RIGHT].L0;

            F_R = leg_Right_L0_pid_.update(right_leg_L0_err_) + robot_weight_per_leg_;
            // F_R =15.0;
            // double right_angle_err = safe_target_theta - legs_[RIGHT].theta;
            // Tp_R = leg_Right_Tp_pid_.update(right_angle_err);
            // Tp_R = 0.0;
            Tp_R = Tp[RIGHT] * 1.5 + anti_crash_;
            // Tp_R = anti_crash_;
        }

        double T_L_Back, T_L_Front, T_R_Back, T_R_Front;
        map_vmc_force(LEFT, F_L, Tp_L, T_L_Back, T_L_Front);
        map_vmc_force(RIGHT, F_R, Tp_R, T_R_Back, T_R_Front);

        // 关节电机扭矩限幅
        auto safe_clamp = [&](double v) {
            if (std::isnan(v))
                return 0.0;
            return std::clamp(v, -25.0, 25.0);
        };

        *t_joint_[LEFT][FRONT] = safe_clamp(T_L_Back);
        *t_joint_[LEFT][BACK] = safe_clamp(T_L_Front);
        *t_joint_[RIGHT][FRONT] = safe_clamp(T_R_Back);
        *t_joint_[RIGHT][BACK] = safe_clamp(T_R_Front);

        *dbg_L0_[LEFT] = legs_[LEFT].L0;
        *dbg_theta_[LEFT] = legs_[LEFT].theta;
        *dbg_theta_[RIGHT] = legs_[RIGHT].theta;

        // 差动转向控制
        // double K_turn = 1.0;
        // double u_turn = rc_cmd_w * K_turn;

        // double final_u_L = wheelT[LEFT] - u_turn;
        // double final_u_R = wheelT[RIGHT] + u_turn;

        // double out_wheel_L = final_u_L / reduction_ratio_;
        // double out_wheel_R = (final_u_R * -1.0) / reduction_ratio_;

        // auto clamp_wheel = [&](double v) {
        //     if (std::isnan(v))
        //         return 0.0;
        //     return std::clamp(v, -1.0, 1.0);
        // };

        *t_wheel_[LEFT] = (wheelT[LEFT] * 17.0 / 268.0) * 4.0;
        *t_wheel_[RIGHT] = (wheelT[RIGHT] * 17.0 / 268.0) * 4.0;
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
    double DEGREE_2_RAD = std::numbers::pi / 180.0;
    InputInterface<double> joint_pos_[2][2];
    InputInterface<double> joint_vel_[2][2];
    InputInterface<double> imu_pitch_, imu_d_pitch_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    InputInterface<double> rotary_knob_;
    OutputInterface<double> t_joint_[2][2];
    OutputInterface<double> dbg_L0_[2], dbg_theta_[2];

    LegStatus legs_[2];
    rmcs_core::controller::pid::PidCalculator leg_L0_pid_;
    rmcs_core::controller::pid::PidCalculator leg_Right_L0_pid_;
    rmcs_core::controller::pid::PidCalculator leg_Left_Tp_pid_;
    rmcs_core::controller::pid::PidCalculator leg_Right_Tp_pid_;
    rmcs_core::controller::pid::PidCalculator anti_crash_pid_;

    double wheel_radius_ = 0.058;           // 轮子半径
    double reduction_ratio_ = 268.0 / 17.0; // 减速比
    double wheel_weight_ = 0.5;             // 轮子重量

    double kinematic_theta[2];
    double kinematic_w_theta[2];
    double L0[2];
    double Tp[2];
    double wheelT[2];

    InputInterface<double> wheel_vel_[2];
    OutputInterface<double> t_wheel_[2];

    struct OdomCalculator {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;

        double s = 0.0;
        double v = 0.0;
        double w = 0.0;

        double track_width = 0.4;           // 轮距

        void update(double v_L, double v_R, double dt) {

            v = (v_L + v_R) / 2.0;
            w = (v_R - v_L) / track_width;
            s += v * dt;
            yaw += w * dt;
            yaw = std::atan2(std::sin(yaw), std::cos(yaw));
            x += v * std::cos(yaw) * dt;
            y += v * std::sin(yaw) * dt;
        }
        void reset() {
            x = 0.0;
            y = 0.0;
            yaw = 0.0;
            s = 0.0;
            v = 0.0;
            w = 0.0;
        }
    };

    OdomCalculator odom_;
    rclcpp::Time last_time_;

    // void calculate_lqr_gains(double L, double* K_out) {
    //     const double P[6][3] = {
    //         // 我们只提取前6行(轮子控制)，彻底丢掉髋关节的干扰
    //         { 99.3792, -118.6757, -15.2298},
    //         { 17.1818,  -20.3025,  -3.8227},
    //         {-21.5961,    8.7075, -11.1011},
    //         {-34.7038,   17.7707, -16.8081},
    //         {179.9289, -162.0310,  61.2403},
    //         { 29.8472,  -25.1950,  11.2166}, // K[5]: Leg dAng
    //     };
    //     double L2 = L * L;
    //     for (int i = 0; i < 6; i++) {
    //         K_out[i] = P[i][0] * L2 + P[i][1] * L + P[i][2];
    //     }
    // }

    void calculate_lqr_gains(int side, double L0) {

        double w_theta = kinematic_w_theta[side];
        double theta = kinematic_theta[side];
        double k[12][3] = {
            {  67.9359,  -63.8194,  -7.6284},
            {   0.8628,   -2.0442,  -0.3059},
            {  -4.7515,    0.9133,  -9.1717},
            { -23.3841,   16.9861, -10.8968},
            { 102.8670, -114.6216,  47.8767},
            {  12.8094,  -11.7436,   5.5136},
            {  54.1380,  -91.7153,  62.9892},
            {   2.4768,   -3.5791,   1.7836},
            { -63.4130,   16.8473,  11.1278},
            { -25.5687,   -5.6609,  10.6532},
            {-472.2591,  400.3138,  56.2421},
            { -19.1531,   23.5305,   0.3676}
        };
        /* External variables --------------------------------------------------------*/
        /* Private function prototypes -----------------------------------------------*/
        double leg_len_ = L0;
        float lsqr = leg_len_ * leg_len_;
        double d_theta;
        if (side == LEFT) {
            d_theta = w_theta;
        } else {
            d_theta = -w_theta;
        }
        double dist_ = 0.0;
        double vel_ = *wheel_vel_[side] * (17.0 / 268.0) * 0.058;
        
        dist_ += vel_ * 0.001;
       

        double T_[2], T_K_[2][6];
        for (uint8_t i = 0; i < 2; ++i) {
            uint8_t j = i * 6;
            T_K_[i][0] = (k[j + 0][0] * lsqr + k[j + 0][1] * leg_len_ + k[j + 0][2]) * -theta;
            T_K_[i][1] = (k[j + 1][0] * lsqr + k[j + 1][1] * leg_len_ + k[j + 1][2]) * -d_theta;
            T_K_[i][2] = (k[j + 2][0] * lsqr + k[j + 2][1] * leg_len_ + k[j + 2][2]) * -dist_;
            T_K_[i][3] = (k[j + 3][0] * lsqr + k[j + 3][1] * leg_len_ + k[j + 3][2]) * -vel_;
            T_K_[i][4] = (k[j + 4][0] * lsqr + k[j + 4][1] * leg_len_ + k[j + 4][2]) * -*imu_pitch_;
            T_K_[i][5] =
                (k[j + 5][0] * lsqr + k[j + 5][1] * leg_len_ + k[j + 5][2]) * *imu_d_pitch_;
        }

        // // if (F_N_ < 20.0f) {
        //     for (uint8_t i = 0; i < 6; ++i) {
        //         T_K_[0][i] = 0.0f;
        //     };
        //     T_K_[1][2] = T_K_[1][3] = T_K_[1][4] = T_K_[1][5] = 0.0f;
        // // }

        for (uint8_t i = 0; i < 2; ++i) {
            T_[i] = T_K_[i][0] + T_K_[i][1] + T_K_[i][2] + T_K_[i][3] + T_K_[i][4] + T_K_[i][5];
        }
        wheelT[side] = T_[0];
        RCLCPP_INFO(get_logger(), "LQR T_%s: %f", side == LEFT ? "LEFT" : "RIGHT", wheelT[side]);
        Tp[side] = T_[1];
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
        double ddot_z_M_ = 9.8;
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

        // double c0 = l2_ * l2_ + std::pow(xd - xb, 2) + std::pow(yd - yb, 2) - l3_ * l3_;
        // double delta = a0 * a0 + b0 * b0 - c0 * c0;

        // if (delta < 0) {
        //     leg.solved = false;
        //     return;
        // }
        leg.solved = true;

        double phi2 =
            2
            * std::atan2(
                b0 + std::sqrt(std::pow(a0, 2) + std::pow(b0, 2) - std::pow(bd, 2)), a0 + bd);
        double xc = l1_ * std::cos(leg.phi1) + l2_ * std::cos(phi2);
        double yc = l1_ * std::sin(leg.phi1) + l2_ * std::sin(phi2);

        leg.L0 = std::sqrt(std::pow(xc - l5_ / 2.0, 2) + std::pow(yc, 2));
        double phi0 = std::atan2(yc, xc - l5_ / 2.0);
        leg.theta = std::numbers::pi / 2.0 - phi0 - *imu_pitch_;
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
        double w_theta_ =
            (0.5 * std::numbers::pi - phi0_pred - *imu_pitch_ - leg.theta) / predict_dt;

        kinematic_w_theta[side] = w_theta_;

        // RCLCPP_INFO(get_logger(), "Predicted theta velocity: %f", kinematic_w_theta[RIGHT]);
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
        double F_N_ = p_ + wheel_weight_ * (9.8f + ddot_z_w_);
        // RCLCPP_INFO(get_logger(), "sport force: %f", F_N_);
    }
    // void LegForceCalc(int side) {
    //     LegStatus& leg = legs_[side];

    // }

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
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelegController, rmcs_executor::Component)