#pragma once

#include <cstddef>
#include <functional>

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

} // namespace rmcs_core::utility