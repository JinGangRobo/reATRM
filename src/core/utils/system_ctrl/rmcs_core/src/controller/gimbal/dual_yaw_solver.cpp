#include "utility/angle_warper.hpp"
#include "utility/kalman_filter.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {

class DualYawSolver
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawSolver()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , bottom_yaw_velocity_estimator_(0.001, 0.0001, 0.5, 0.01) {

        register_input("/gimbal/yaw/control_angle_error", control_angle_error_);
        register_input("/gimbal/yaw/control_angle_shift", control_angle_shift_, false);
        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_velocity", control_angle_velocity_, false);

        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);

        register_output(
            "/gimbal/bottom_yaw/estimated_velocity", estimated_bottom_yaw_velocity_, 0.0);

        register_output("/gimbal/top_yaw/target_angle_error", top_yaw_target_error_, 0.0);
        register_output("/gimbal/bottom_yaw/target_angle_error", bottom_yaw_target_error_, 0.0);
        register_output("/gimbal/top_yaw/target_angle_velocity", top_yaw_target_velocity_, 0.0);
        register_output(
            "/gimbal/bottom_yaw/target_angle_velocity", bottom_yaw_target_velocity_, 0.0);

        register_output(
            "/gimbal/bottom_yaw/control_angle_shift", bottom_yaw_control_angle_shift_, 0.0);

        status_component_ =
            create_partner_component<DualYawStatus>(get_component_name() + "_status");
    }

    void before_updating() override {
        if (!control_angle_shift_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_shift\", set to NaN.");
            control_angle_shift_.bind_directly(nan_);
        }
        if (!control_angle_velocity_.ready()) {
            RCLCPP_INFO(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_velocity\", set to 0.");
            control_angle_velocity_.bind_directly(0.0);
        }
    }

    void update() override {
        constexpr double TOP_YAW_LIMIT = std::numbers::pi / 3.0;
        *estimated_bottom_yaw_velocity_ =
            bottom_yaw_velocity_estimator_.update(*bottom_yaw_angle_, *chassis_yaw_velocity_imu_);

        if (std::isnan(*control_angle_error_)) {
            *top_yaw_target_error_ = nan_;
            *bottom_yaw_target_error_ = nan_;
            *top_yaw_target_velocity_ = nan_;
            *bottom_yaw_target_velocity_ = nan_;

        } else {
            double e_total = utility::angle_wraper::wrap_to_pi(*control_angle_error_);
            double e_bot = utility::angle_wraper::wrap_to_pi(e_total + *top_yaw_angle_);

            double top_target = std::clamp(e_bot, -TOP_YAW_LIMIT, TOP_YAW_LIMIT);

            *top_yaw_target_error_ =
                utility::angle_wraper::wrap_to_pi(top_target - *top_yaw_angle_);
            *bottom_yaw_target_error_ = e_bot;

            *bottom_yaw_target_velocity_ = *control_angle_velocity_;
            *top_yaw_target_velocity_ = *control_angle_velocity_ - *estimated_bottom_yaw_velocity_;
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

    InputInterface<double> control_angle_error_, control_angle_shift_;
    InputInterface<double> top_yaw_angle_;
    InputInterface<double> control_angle_velocity_;
    InputInterface<double> bottom_yaw_angle_;
    InputInterface<double> chassis_yaw_velocity_imu_;

    OutputInterface<double> estimated_bottom_yaw_velocity_;
    OutputInterface<double> top_yaw_target_error_, bottom_yaw_target_error_;
    OutputInterface<double> top_yaw_target_velocity_, bottom_yaw_target_velocity_;
    OutputInterface<double> top_yaw_control_angle_, bottom_yaw_control_angle_shift_;

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
            *yaw_angle_ = utility::angle_wraper::wrap_to_2pi(*top_yaw_angle_ + *bottom_yaw_angle_);

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
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawSolver, rmcs_executor::Component)