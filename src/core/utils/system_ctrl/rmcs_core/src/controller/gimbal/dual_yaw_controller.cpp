#include <chrono>
#include <cstdlib>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "utility/kalman_filter.hpp"

namespace rmcs_core::controller::gimbal {

class DualYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , bottom_yaw_velocity_estimator_(0.001, 0.0001, 0.5, 0.01) {
        auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
            pid.kp = get_parameter(name + "_kp").as_double();
            pid.ki = get_parameter(name + "_ki").as_double();
            pid.kd = get_parameter(name + "_kd").as_double();
            get_parameter(name + "_integral_min", pid.integral_min);
            get_parameter(name + "_integral_max", pid.integral_max);
            get_parameter(name + "_output_min", pid.output_min);
            get_parameter(name + "_output_max", pid.output_max);
        };
        set_pid_parameter(top_yaw_angle_pid_, "top_yaw_angle");
        set_pid_parameter(top_yaw_velocity_pid_, "top_yaw_velocity");
        set_pid_parameter(bottom_yaw_angle_pid_, "bottom_yaw_angle");
        set_pid_parameter(bottom_yaw_velocity_pid_, "bottom_yaw_velocity");

        get_parameter("top_yaw_angle_limit", top_yaw_angle_limit_);

        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);
        register_input("/gimbal/top_yaw/last_update_time", top_yaw_timestamp_);
        register_input("/gimbal/bottom_yaw/last_update_time", bottom_yaw_timestamp_);

        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);

        register_input("/gimbal/yaw/control_angle_error", control_angle_error_);
        register_input("/gimbal/yaw/control_angle_shift", control_angle_shift_, false);

        register_output("/gimbal/top_yaw/control_torque", top_yaw_control_torque_, 0.0);
        register_output("/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque_, 0.0);

        register_output("/debug/estimated_bottom_yaw_vel", debug_estimated_vel, 1.0);

        status_component_ =
            create_partner_component<DualYawStatus>(get_component_name() + "_status");
    }

    void before_updating() override {
        if (!control_angle_shift_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_shift\", set to NaN.");
            control_angle_shift_.bind_directly(nan_);
        }
    }

    void update() override {
        double bottom_yaw_velocity_imu =
            bottom_yaw_velocity_estimator_.update(*bottom_yaw_angle_, *chassis_yaw_velocity_imu_);
        *debug_estimated_vel = bottom_yaw_velocity_imu;

        if (std::isnan(*control_angle_error_)) {
            *top_yaw_control_torque_ = nan_;
            *bottom_yaw_control_torque_ = nan_;
        } else {
            auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
            if (now - *top_yaw_timestamp_ > motor_timeout_ms_
                || now - *bottom_yaw_timestamp_ > motor_timeout_ms_) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Dual Yaw Executor timestamp is too old");

                top_yaw_velocity_pid_.reset();
                bottom_yaw_velocity_pid_.reset();
                top_yaw_angle_pid_.reset();
                bottom_yaw_angle_pid_.reset();

                *top_yaw_control_torque_ = nan_;
                *bottom_yaw_control_torque_ = nan_;

                return;
            }

            if (*control_angle_error_ * top_yaw_angle_pid_.integral() < 0) {
                // If the error sign changes, reset the PID to prevent overshoot
                top_yaw_angle_pid_.clear_integral();
                top_yaw_velocity_pid_.clear_integral();
            }

            double top_yaw_vel_error =
                top_yaw_angle_pid_.update(*control_angle_error_) - *gimbal_yaw_velocity_imu_;
            double bottom_yaw_angle_error = bottom_yaw_control_error();

            if ((abs(bottom_yaw_angle_error) > top_yaw_angle_limit_)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Top yaw angle is near limit, resetting PID.");
                top_yaw_angle_pid_.clear_integral();
                top_yaw_velocity_pid_.clear_integral();
            }

            *top_yaw_control_torque_ = top_yaw_velocity_pid_.update(top_yaw_vel_error);

            *bottom_yaw_control_torque_ = bottom_yaw_velocity_pid_.update(
                bottom_yaw_angle_pid_.update(bottom_yaw_angle_error) - bottom_yaw_velocity_imu);
        }

        if (std::isnan(*control_angle_shift_)) {
            *top_yaw_control_angle_ = nan_;
            *bottom_yaw_control_angle_shift_ = nan_;
        } else {
            *top_yaw_control_angle_ = 0.0;
            *bottom_yaw_control_angle_shift_ = *control_angle_shift_;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr int16_t motor_timeout_ms_ = 100;
    double top_yaw_angle_limit_ = 0.71;

    double bottom_yaw_control_error() {
        double err = *top_yaw_angle_ + *control_angle_error_;
        if (err > std::numbers::pi)
            err -= 2 * std::numbers::pi;
        return err;
    }

    InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
    InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;
    InputInterface<int64_t> top_yaw_timestamp_, bottom_yaw_timestamp_;

    InputInterface<double> gimbal_yaw_velocity_imu_, chassis_yaw_velocity_imu_;

    InputInterface<double> control_angle_error_, control_angle_shift_;

    pid::PidCalculator top_yaw_angle_pid_, top_yaw_velocity_pid_;
    pid::PidCalculator bottom_yaw_angle_pid_, bottom_yaw_velocity_pid_;

    OutputInterface<double> top_yaw_control_torque_;
    OutputInterface<double> bottom_yaw_control_torque_;

    OutputInterface<double> top_yaw_control_angle_;
    OutputInterface<double> bottom_yaw_control_angle_shift_;

    OutputInterface<double> debug_estimated_vel;

    class DualYawStatus : public rmcs_executor::Component {
    public:
        explicit DualYawStatus() {
            register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
            register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
            register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
            register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);

            register_output("/gimbal/yaw/angle", yaw_angle_, 0.0);
            register_output("/gimbal/yaw/velocity", yaw_velocity_, 0.0);
        }

        void update() override {
            double yaw_angle = *top_yaw_angle_ + *bottom_yaw_angle_;
            if (yaw_angle < 0)
                yaw_angle += 2 * std::numbers::pi;
            else if (yaw_angle > 2 * std::numbers::pi)
                yaw_angle -= 2 * std::numbers::pi;
            *yaw_angle_ = yaw_angle;

            *yaw_velocity_ = *top_yaw_velocity_ + *bottom_yaw_velocity_;
        }

    private:
        InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
        InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;

        OutputInterface<double> yaw_angle_, yaw_velocity_;
    };
    std::shared_ptr<DualYawStatus> status_component_;

    class BottomYawVelocityEstimator {
    public:
        // 状态量：[电机角度, 云台绝对角速度] (State=2)
        // 观测量：[电机测量角度] (Measure=1)
        // 控制量：[圆盘IMU gz] (Control=1)
        using EKF = rmcs_core::utility::ExtendedKalmanFilter<2, 1, 1>;

        using State = Eigen::Vector2d;
        using Measure = Eigen::Matrix<double, 1, 1>;
        using Control = Eigen::Matrix<double, 1, 1>;

        BottomYawVelocityEstimator(
            double dt, double process_noise_angle, double process_noise_velocity,
            double measurement_noise)
            : dt_(dt) {

            // 1. 过程模型 f(x, u)：u(0) 就是 gz
            // 下一时刻电机角度 = 当前电机角度 + (云台绝对速度 - 底盘圆盘速度) * dt
            auto process_model = [this](const State& x, const Control& u) -> State {
                State next_state;
                next_state(0) =
                    x(0) + (x(1) - u(0)) * dt_; // θ_m(k+1) = θ_m(k) + (ω_g(k) - gz) * dt
                next_state(1) = x(1);           // ω_g(k+1) = ω_g(k)
                return next_state;
            };

            // 2. 测量模型 h(x)：我们只测得了电机的相对角度
            auto measurement_model = [](const State& x) -> Measure {
                Measure z;
                z(0) = x(0);
                return z;
            };

            // 3. 过程雅可比矩阵 F = ∂f/∂x
            auto process_jacobian = [this](const State& x, const Control& u) -> Eigen::Matrix2d {
                (void)x;
                (void)u;

                Eigen::Matrix2d F;
                F << 1, dt_, // ∂θ_{m,k+1}/∂θ_{m,k} = 1, ∂θ_{m,k+1}/∂ω_{g,k} = dt
                    0, 1;    // ∂ω_{g,k+1}/∂θ_{m,k} = 0, ∂ω_{g,k+1}/∂ω_{g,k} = 1
                return F;
            };

            // 4. 测量雅可比矩阵 H = ∂h/∂x
            auto measurement_jacobian = [](const State& x) -> Eigen::Matrix<double, 1, 2> {
                (void)x;

                Eigen::Matrix<double, 1, 2> H;
                H << 1, 0; // 仅对位置项有偏导
                return H;
            };

            // 5. 过程噪声 Q 与 测量噪声 R
            Eigen::Matrix2d Q;
            Q << process_noise_angle, 0, 0, process_noise_velocity;

            Eigen::Matrix<double, 1, 1> R;
            R(0, 0) = measurement_noise;

            ekf_ = std::make_unique<EKF>(
                process_model, measurement_model, process_jacobian, measurement_jacobian, Q, R);
        }

        double normalize_angle(double angle) const {
            while (angle > M_PI)
                angle -= 2 * M_PI;
            while (angle < -M_PI)
                angle += 2 * M_PI;
            return angle;
        }

        double angle_difference(double target, double current) const {
            return normalize_angle(target - current);
        }

        /**
         * 更新状态并获取云台绝对速度评估
         * @param encoder_angle 编码器测出的0~2pi角度
         * @param disk_gz       圆盘独立IMU的测出的Z轴角速度
         * @return 预测出的云台平滑绝对角速度 (rad/s)
         */
        double update(double encoder_angle, double disk_gz) {
            // ---- 连续性角度处理 (Wrap-around 处理) ----
            double current_angle = ekf_->posterior_estimate()(0);
            double angle_diff = angle_difference(encoder_angle, current_angle);
            double unwrapped_angle = current_angle + angle_diff;

            // 构造测量和控制向量
            Measure measurement;
            measurement(0) = unwrapped_angle;

            Control control;
            control(0) = disk_gz;

            // EKF预测与更新
            State estimated_state = ekf_->update(measurement, control);

            // 返回平滑处理后的云台绝对速度
            return estimated_state(1) / 4.0; // TODO: 统一量纲
        }

        double get_gimbal_absolute_velocity() const { return ekf_->posterior_estimate()(1); }
        double get_motor_relative_angle() const {
            return normalize_angle(ekf_->posterior_estimate()(0));
        }

        void reset() { ekf_->reset(); }

    private:
        std::unique_ptr<EKF> ekf_;
        double dt_;
    };

    BottomYawVelocityEstimator bottom_yaw_velocity_estimator_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawController, rmcs_executor::Component)