
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
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
        , leg_L0_pid_(600.0, 0.0, 15.0)
        , leg_angle_pid_(3.0, 0.0, 0.0) {

        RCLCPP_INFO(get_logger(), "=== Step 3.5: VMC with High-Precision Gyro ===");

        l1_ = 0.11;
        l2_ = 0.132;
        l3_ = 0.132;
        l4_ = 0.11;
        l5_ = 0.0;

        left_phi1_offset_ = std::numbers::pi - 0.1;
        left_phi4_offset_ = std::numbers::pi - 0.1;
        right_phi1_offset_ = std::numbers::pi - 0.1;
        right_phi4_offset_ = std::numbers::pi - 0.1;

        odom_.track_width = 0.36; //轮距
        last_time_ = this->get_clock()->now();
        robot_weight_per_leg_ = 5.8 * 9.8 / 2.0;//重力补偿

        register_input("/chassis/left_back_hip/angle", joint_pos_[LEFT][FRONT]);
        register_input("/chassis/left_front_hip/angle", joint_pos_[LEFT][BACK]);
        register_input("/chassis/right_back_hip/angle", joint_pos_[RIGHT][FRONT]);
        register_input("/chassis/right_front_hip/angle", joint_pos_[RIGHT][BACK]);
        register_input("/chassis/imu/pitch", imu_pitch_, false);
        register_input("/chassis/imu/pitch_velocity", imu_d_pitch_, false);

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
    }

    void update() override {


        double raw_pitch = *imu_pitch_;
        double raw_d_pitch = *imu_d_pitch_;

        //遥控器部分

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
        //高度调节
        double knob_val = 0.0;
        if (rotary_knob_.ready()) {
            knob_val = -*rotary_knob_; 
        }
        if (std::abs(knob_val) < 0.05) {
            knob_val = 0.0;
        }

        double min_L0 = 0.12; // 最矮状态
        double max_L0 = 0.19; // 最高状态

        double L0_speed_multiplier = 0.15; //伸缩速度倍率

        static double current_target_L0 = 0.16; //默认高度

        current_target_L0 += knob_val * L0_speed_multiplier * 0.001;

        current_target_L0 = std::clamp(current_target_L0, min_L0, max_L0);

        double target_L0 = current_target_L0;
        //高度调节结束

        if (std::abs(rc_cmd_v) < 0.05)
            rc_cmd_v = 0.0;
        if (std::abs(rc_cmd_w) < 0.05)
            rc_cmd_w = 0.0;

        //防止突变
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
        odom_.update(v_L, v_R, dt); //里程计计算

       //目前速度环使用的外环积分控制 LQR没调通 先用这个简单的 PI 来防止溜车，等 LQR 调通了再换
        double vel_err = target_vel - odom_.v;


        double Kp_v = 1.0;
        double Ki_v = 0.02;
        double max_pitch_tilt = 0.26; //最大倾斜角度

        if (rc_cmd_v == 0.0) {
            vel_integral *= 0.2;
        } else if (std::abs(odom_.v) < 1.0) {
            vel_integral = vel_integral * 0.999 + vel_err * dt;
        }
        if (-0.05 < std::abs(odom_.v) < 0.05) {
            vel_integral = vel_integral * 0.999 + vel_err * dt;
        }
        vel_integral = std::clamp(vel_integral, -max_pitch_tilt / Ki_v, max_pitch_tilt / Ki_v);

        double pi_adjust_pitch = -(Kp_v * vel_err + Ki_v * vel_integral);

        pi_adjust_pitch = std::clamp(pi_adjust_pitch, -max_pitch_tilt, max_pitch_tilt);

        double base_pitch_offset = 0.12;//0.16高度下的静态补偿


        double dynamic_target_pitch = base_pitch_offset + pi_adjust_pitch;
        //LQR计算部分
        double avg_L0 = std::clamp((legs_[LEFT].L0 + legs_[RIGHT].L0) / 2.0, 0.10, 0.25);
        double K[12];
        calculate_lqr_gains(avg_L0, K);
        double x_err[6] = {0};

        x_err[0] = pitch - dynamic_target_pitch;
        x_err[1] = d_pitch - 0.0;
        x_err[2] = 0.0;
        x_err[3] = 0.0;
        x_err[4] = 0.0;
        x_err[5] = 0.0;


        double u_wheel = 0.0;
        for (int i = 0; i < 6; i++) {
            u_wheel += -K[i] * x_err[i];
        }

        double lqr_scale = 4.5; //因为腿部并没有在和轮子有联动 所以LQR乘了一个比较大的倍率
        double lqr_direction = -1.0;
        u_wheel = u_wheel * lqr_scale * lqr_direction;
     
        process_kinematics(LEFT, pitch, d_pitch);
        process_kinematics(RIGHT, pitch, d_pitch);


        //腿部摆角软限位
        double mech_limit = 0.0;   //限位值

        double center_phi = 0.0;                       // 机身坐标系下的垂直中点 (1.57)
        double min_phi_body = center_phi - mech_limit; // 机身系下的最小角度 (最前)
        double max_phi_body = center_phi + mech_limit; // 机身系下的最大角度 (最后)

        double target_theta_world = 0.0;

        double target_phi_body = center_phi - (target_theta_world + pitch);


        double safe_phi_body = std::clamp(target_phi_body, min_phi_body, max_phi_body);

        double safe_target_theta = center_phi - (safe_phi_body + pitch);

        //腿部PID计算
        double F_L = 0, Tp_L = 0;
        double F_R = 0, Tp_R = 0;

        if (legs_[LEFT].solved) {
            double pid_F = 600.0 * (target_L0 - legs_[LEFT].L0);
            double damping_F = -15.0 * legs_[LEFT].dL0;
            F_L = pid_F + damping_F + robot_weight_per_leg_;


            double pid_Tp = 20.0 * (safe_target_theta - legs_[LEFT].theta);
            double damping_Tp = -0.8 * legs_[LEFT].d_theta;
            Tp_L = pid_Tp + damping_Tp;
        }

        if (legs_[RIGHT].solved) {

            double pid_F = 600.0 * (target_L0 - legs_[RIGHT].L0);
            double damping_F = -15.0 * legs_[RIGHT].dL0;
            F_R = pid_F + damping_F + robot_weight_per_leg_;


            double pid_Tp = 20.0 * (safe_target_theta - legs_[RIGHT].theta);
            double damping_Tp = -0.8 * legs_[RIGHT].d_theta;
            Tp_R = pid_Tp + damping_Tp;
        }




        double T_L_Back, T_L_Front, T_R_Back, T_R_Front;
        map_vmc_force(LEFT, F_L, Tp_L, T_L_Back, T_L_Front);
        map_vmc_force(RIGHT, F_R, Tp_R, T_R_Back, T_R_Front);

        //关节电机扭矩限幅
        auto safe_clamp = [&](double v) {
            if (std::isnan(v))
                return 0.0;
            return std::clamp(v, -10.0, 10.0);
        };

        *t_joint_[LEFT][FRONT] = safe_clamp(T_L_Back);
        *t_joint_[LEFT][BACK] = safe_clamp(T_L_Front);
        *t_joint_[RIGHT][FRONT] = safe_clamp(T_R_Back);
        *t_joint_[RIGHT][BACK] = safe_clamp(T_R_Front);

        *dbg_L0_[LEFT] = legs_[LEFT].L0;
        *dbg_theta_[LEFT] = legs_[LEFT].theta;

        //差动转向控制
        double K_turn = 1.0; 
        double u_turn = rc_cmd_w * K_turn;

        double final_u_L = u_wheel - u_turn;
        double final_u_R = u_wheel + u_turn;

        double out_wheel_L = final_u_L / reduction_ratio_;
        double out_wheel_R = (final_u_R * -1.0) / reduction_ratio_;

        auto clamp_wheel = [&](double v) {
            if (std::isnan(v))
                return 0.0;
            return std::clamp(v, -20.0, 20.0);
        };

        *t_wheel_[LEFT] = clamp_wheel(out_wheel_L);
        *t_wheel_[RIGHT] = clamp_wheel(out_wheel_R);
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
    InputInterface<double> imu_pitch_, imu_d_pitch_; 
    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> rotary_knob_;
    OutputInterface<double> t_joint_[2][2];
    OutputInterface<double> dbg_L0_[2], dbg_theta_[2];

    LegStatus legs_[2];
    rmcs_core::controller::pid::PidCalculator leg_L0_pid_;
    rmcs_core::controller::pid::PidCalculator leg_angle_pid_;

    double wheel_radius_ = 0.075; //轮子半径
    double reduction_ratio_ = 19.0; // 减速比

    InputInterface<double> wheel_vel_[2];
    OutputInterface<double> t_wheel_[2];

    struct OdomCalculator {
        double x = 0.0;   
        double y = 0.0;   
        double yaw = 0.0; 

       
        double s = 0.0; 
        double v = 0.0; 
        double w = 0.0; 

        double track_width = 0.4; //轮距

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

    void calculate_lqr_gains(double L, double* K_out) {
        const double P[6][3] = {
            // 我们只提取前6行(轮子控制)，彻底丢掉髋关节的干扰
            {104.3384, -118.0367, -19.0764}, // K[0]: Pitch
            { 16.5697,  -19.2057,  -4.8828}, // K[1]: dPitch
            { -9.2461,    0.2136, -10.3235}, // K[2]: Pos
            {-23.7808,   10.2178, -16.2167}, // K[3]: Vel
            { 77.6070,  -89.3704,  43.6870}, // K[4]: Leg Ang
            {  9.4764,  -11.0691,   6.9514}  // K[5]: Leg dAng
        };
        double L2 = L * L;
        for (int i = 0; i < 6; i++) {
            K_out[i] = P[i][0] * L2 + P[i][1] * L + P[i][2];
        }
    }

    double normalize_angle(double angle) {
        double a = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
        if (a < 0)
            a += 2.0 * std::numbers::pi;
        return a - std::numbers::pi;
    }

    void process_kinematics(int side, double pitch, double d_pitch) {
        LegStatus& leg = legs_[side];
        double raw_back = *joint_pos_[side][BACK];
        double raw_front = *joint_pos_[side][FRONT];

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

        double lbd2 = std::pow(xd - xb, 2) + std::pow(yd - yb, 2);
        double a0 = 2 * l2_ * (xd - xb);
        double b0 = 2 * l2_ * (yd - yb);
        double c0 = l2_ * l2_ + std::pow(xd - xb, 2) + std::pow(yd - yb, 2) - l3_ * l3_;
        double delta = a0 * a0 + b0 * b0 - c0 * c0;

        if (delta < 0) {
            leg.solved = false;
            return;
        }
        leg.solved = true;

        double phi2 = 2 * std::atan2(b0 + std::sqrt(delta), a0 + c0);
        double xc = l1_ * std::cos(leg.phi1) + l2_ * std::cos(phi2);
        double yc = l1_ * std::sin(leg.phi1) + l2_ * std::sin(phi2);

        leg.L0 = std::sqrt(std::pow(xc - l5_ / 2.0, 2) + std::pow(yc, 2));
        double phi0 = std::atan2(yc, xc - l5_ / 2.0);
        leg.theta = std::numbers::pi / 2.0 - (phi0 + pitch);

        // 雅可比
        double phi3 = std::atan2(yb - yd + l2_ * std::sin(phi2), xb - xd + l2_ * std::cos(phi2));
        double s1 = std::sin(phi3 - phi2);
        double s2 = std::sin(phi3 - leg.phi4);
        double s3 = std::sin(leg.phi1 - phi2);
        double s4 = std::sin(leg.phi0 - phi3);
        double s5 = std::cos(leg.phi0 - phi3);
        double s6 = std::sin(leg.phi0 - phi2);
        double s7 = std::cos(leg.phi0 - phi2);

        if (std::abs(s1) < 1e-4)
            s1 = 1e-4;

        leg.J11 = (l1_ * s4 * s3) / s1;
        leg.J12 = (l4_ * s6 * s2) / s1;
        leg.J21 = (l1_ * s5 * s3) / (leg.L0 * s1);
        leg.J22 = (l4_ * s7 * s2) / (leg.L0 * s1);

        //腿部角速度计算
        double curr_dL0 = (leg.L0 - leg.last_L0) / 0.001;
        leg.dL0 = std::clamp(curr_dL0, -5.0, 5.0);
        leg.last_L0 = leg.L0;

        // 2. dTheta (摆动速度) - 融合公式
        // dTheta = - (dPhi0 + dPitch)
        // dPhi0 我们很难算准（需要电机速度），但我们可以近似认为腿相对于机身摆动很慢
        // 主要的高频分量来自 dPitch (机身晃动)
        // 所以直接用 dPitch 作为 dTheta 的主要成分，效果极佳且无延迟！

        // 近似公式：dTheta ≈ -dPitch (假设腿关节不动，机身晃动)
        // 进阶公式：dTheta = (theta - last_theta)/dt * 0.1 + (-dPitch) * 0.9 (互补滤波)
        // 这里我们直接信任陀螺仪：
        leg.d_theta = d_pitch;
    }

    void map_vmc_force(int side, double F, double Tp, double& T_Back, double& T_Front) {
        LegStatus& leg = legs_[side];

        double sign_F_Back, sign_F_Front;
        double sign_Tp_Back, sign_Tp_Front;
        //电机转向调节
        if (side == LEFT) {
            sign_F_Back = 1.0;
            sign_F_Front = -1.0;
            sign_Tp_Back = -1.0;
            sign_Tp_Front = 1.0;
        } else {
            sign_F_Back = -1.0;
            sign_F_Front = 1.0;
            sign_Tp_Back = 1.0;
            sign_Tp_Front = -1.0;
        }

        T_Back = (sign_F_Back * leg.J11 * F) + (sign_Tp_Back * leg.J21 * Tp);
        T_Front = (sign_F_Front * leg.J12 * F) + (sign_Tp_Front * leg.J22 * Tp);
    }
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelegController, rmcs_executor::Component)