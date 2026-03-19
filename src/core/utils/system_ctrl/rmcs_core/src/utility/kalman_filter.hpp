#pragma once

#include <cstddef>
#include <functional>
#include <memory>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::utility {

template <size_t m, size_t n = m>
using Matrix = Eigen::Matrix<double, m, n>;

template <size_t state, size_t measure, size_t control = 0>
class KalmanFilter {
    using State = Eigen::Vector<double, state>;
    using Measure = Eigen::Vector<double, measure>;
    using Control = Eigen::Vector<double, control>;

public:
    KalmanFilter() {}

    KalmanFilter(
        const Matrix<state>& A, const Matrix<measure, state>& H, const Matrix<state>& Q,
        const Matrix<measure>& R, const Matrix<state>& W = Matrix<state>::Identity(),
        const Matrix<measure>& V = Matrix<measure>::Identity(),
        const Matrix<state, control>& B = Matrix<state, control>::Zero())
        : measurement_noise_transition_(V)
        , measurement_noise_covariance_(R)
        , state_transition_(A)
        , process_noise_transition_(W)
        , process_noise_covariance_(Q)
        , control_input_(B)
        , measurement_transition_(H) {
        reset();
    }

    void reset() {
        posterior_estimate_ = State::Zero();
        prior_estimate_ = State::Zero();

        posterior_error_covariance_ = 1000.0 * Matrix<state>::Identity();
        prior_error_covariance_ = posterior_error_covariance_;

        kalman_gain_ = Matrix<state, measure>::Zero();
    }

    State update(const Measure& measurement, const Control& control_vector = Control::Zero()) {
        // Prediction
        prior_estimate_ = state_transition_ * posterior_estimate_ + control_input_ * control_vector;
        prior_error_covariance_ =
            state_transition_ * posterior_error_covariance_ * state_transition_.transpose()
            + process_noise_transition_ * process_noise_covariance_
                  * process_noise_transition_.transpose();

        // Correction
        kalman_gain_ = prior_error_covariance_ * measurement_transition_.transpose()
                     * (measurement_transition_ * prior_error_covariance_
                            * measurement_transition_.transpose()
                        + measurement_noise_transition_ * measurement_noise_covariance_
                              * measurement_noise_transition_.transpose())
                           .inverse();
        posterior_estimate_ +=
            kalman_gain_ * (measurement - measurement_transition_ * prior_estimate_);

        // Update
        posterior_error_covariance_ =
            (Matrix<state>::Identity() - kalman_gain_ * measurement_transition_)
            * prior_error_covariance_;

        return posterior_estimate_;
    }

private:
    State prior_estimate_, posterior_estimate_;

    Matrix<measure> measurement_noise_transition_, measurement_noise_covariance_;
    Matrix<state> state_transition_, process_noise_transition_, process_noise_covariance_,
        prior_error_covariance_, posterior_error_covariance_;

    Matrix<state, measure> kalman_gain_;
    Matrix<state, control> control_input_;

    Matrix<measure, state> measurement_transition_;
};

template <size_t state, size_t measure, size_t control = 0>
class ExtendedKalmanFilter {
    using State = Eigen::Vector<double, state>;
    using Measure = Eigen::Vector<double, measure>;
    using Control = Eigen::Vector<double, control>;
    using StateJacobian = Eigen::Matrix<double, state, state>;
    using MeasureJacobian = Eigen::Matrix<double, measure, state>;

    // 函数类型：f(x, u) 和 h(x) 以及它们的雅可比矩阵
    using ProcessModel = std::function<State(const State&, const Control&)>;
    using MeasurementModel = std::function<Measure(const State&)>;
    using ProcessJacobian = std::function<StateJacobian(const State&, const Control&)>;
    using MeasurementJacobian = std::function<MeasureJacobian(const State&)>;

public:
    ExtendedKalmanFilter(
        ProcessModel f, MeasurementModel h, ProcessJacobian F, MeasurementJacobian H,
        const Matrix<state>& Q, const Matrix<measure>& R)
        : process_model_(f)
        , measurement_model_(h)
        , process_jacobian_(F)
        , measurement_jacobian_(H)
        , process_noise_covariance_(Q)
        , measurement_noise_covariance_(R) {
        reset();
    }

    void reset() {
        posterior_estimate_ = State::Zero();
        posterior_error_covariance_ = 1000.0 * Matrix<state>::Identity();
    }

    State update(const Measure& measurement, const Control& control_vector = Control::Zero()) {
        // 预测步骤（非线性，使用 f 函数）
        State prior_estimate = process_model_(posterior_estimate_, control_vector);
        StateJacobian F = process_jacobian_(posterior_estimate_, control_vector);
        Matrix<state> prior_error_covariance =
            F * posterior_error_covariance_ * F.transpose() + process_noise_covariance_;

        // 更新步骤（非线性，使用 h 函数）
        Measure predicted_measurement = measurement_model_(prior_estimate);
        MeasureJacobian H = measurement_jacobian_(prior_estimate);

        Matrix<measure> innovation_covariance =
            H * prior_error_covariance * H.transpose() + measurement_noise_covariance_;
        Matrix<state, measure> kalman_gain =
            prior_error_covariance * H.transpose() * innovation_covariance.inverse();

        posterior_estimate_ = prior_estimate + kalman_gain * (measurement - predicted_measurement);
        posterior_error_covariance_ =
            (Matrix<state>::Identity() - kalman_gain * H) * prior_error_covariance;

        return posterior_estimate_;
    }

    const State& posterior_estimate() const { return posterior_estimate_; }

private:
    ProcessModel process_model_;
    MeasurementModel measurement_model_;
    ProcessJacobian process_jacobian_;
    MeasurementJacobian measurement_jacobian_;

    State posterior_estimate_;
    Matrix<state> process_noise_covariance_, posterior_error_covariance_;
    Matrix<measure> measurement_noise_covariance_;
};

class EncoderVelocityEstimator {
public:
    using EKF =
        rmcs_core::utility::ExtendedKalmanFilter<2, 1>; // 状态：[角度, 角速度], 测量：[角度]
    using State = Eigen::Vector2d;
    using Measure = Eigen::Matrix<double, 1, 1>;

    EncoderVelocityEstimator(
        double dt, double process_noise_angle, double process_noise_velocity,
        double measurement_noise)
        : dt_(dt) {
        // 过程模型：f(x, u) = [θ + ω*dt, ω]（常速度模型）
        auto process_model = [this](const State& x, const Eigen::Vector<double, 0>&) -> State {
            State next_state;
            next_state(0) = x(0) + x(1) * dt_; // θ_{k+1} = θ_k + ω_k * dt
            next_state(1) = x(1);              // ω_{k+1} = ω_k
            return next_state;
        };

        // 测量模型：h(x) = θ（只测量角度）
        auto measurement_model = [](const State& x) -> Measure {
            Measure z;
            z(0) = x(0);
            return z;
        };

        // 过程雅可比矩阵 F = ∂f/∂x
        auto process_jacobian =
            [this](const State& x, const Eigen::Vector<double, 0>&) -> Eigen::Matrix2d {
            Eigen::Matrix2d F;
            F << 1, dt_, // ∂θ_{k+1}/∂θ_k = 1, ∂θ_{k+1}/∂ω_k = dt
                0, 1;    // ∂ω_{k+1}/∂θ_k = 0, ∂ω_{k+1}/∂ω_k = 1
            return F;
        };

        // 测量雅可比矩阵 H = ∂h/∂x
        auto measurement_jacobian = [](const State& x) -> Eigen::Matrix<double, 1, 2> {
            Eigen::Matrix<double, 1, 2> H;
            H << 1, 0; // ∂θ/∂θ = 1, ∂θ/∂ω = 0
            return H;
        };

        // 过程噪声协方差 Q：对角矩阵
        Eigen::Matrix2d Q;
        Q << process_noise_angle, 0, 0, process_noise_velocity;

        // 测量噪声协方差 R：标量
        Matrix<1> R;
        R(0, 0) = measurement_noise;

        ekf_ = std::make_unique<EKF>(
            process_model, measurement_model, process_jacobian, measurement_jacobian, Q, R);
    }

    // 工具函数：处理角度wrap-around（归一化到 -π 到 π）
    double normalize_angle(double angle) const {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    // 工具函数：计算两个角度的最短差值
    double angle_difference(double target, double current) const {
        double diff = target - current;
        return normalize_angle(diff);
    }

    /**
     * 更新EKF并获取速度估计
     * @param encoder_angle 编码器原始角度 [0, 2π)
     * @return 估计的角速度 (rad/s)
     */
    double update(double encoder_angle) {
        // 处理angle wrap-around：计算到当前估计的最短路径
        double current_angle = ekf_->posterior_estimate()(0);
        double angle_diff = angle_difference(encoder_angle, current_angle);

        // 为了正确处理wrap-around，我们需要"展开"角度
        // 即使编码器值在[0, 2π)，内部状态可以超出这个范围
        double unwrapped_angle = current_angle + angle_diff;

        // 创建测量向量
        Measure measurement;
        measurement(0) = unwrapped_angle;

        // 执行EKF更新
        State estimated_state = ekf_->update(measurement);

        // 返回估计的角速度
        return estimated_state(1);
    }

    /**
     * 获取估计的角度（在 -π 到 π 范围内）
     */
    double get_angle() const { return normalize_angle(ekf_->posterior_estimate()(0)); }

    /**
     * 获取估计的角速度
     */
    double get_velocity() const { return ekf_->posterior_estimate()(1); }

    /**
     * 重置EKF状态
     */
    void reset() { ekf_->reset(); }

private:
    std::unique_ptr<EKF> ekf_;
    double dt_; // 时间步长 (秒)
};

} // namespace rmcs_core::utility