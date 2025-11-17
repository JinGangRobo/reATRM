#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

namespace librmcs::utility {

namespace internal {

// A minimal matrix library to replace ARM CMSIS-DSP
struct Matrix {
    int rows, cols;
    std::vector<double> data;

    explicit Matrix(int r = 0, int c = 0)
        : rows(r)
        , cols(c)
        , data(r * c, 0.0f) {}

    double& at(int r, int c) { return data[r * cols + c]; }
    const double& at(int r, int c) const { return data[r * cols + c]; }

    static Matrix identity(int size) {
        Matrix m(size, size);
        for (int i = 0; i < size; ++i) {
            m.at(i, i) = 1.0f;
        }
        return m;
    }
};

inline Matrix multiply(const Matrix& a, const Matrix& b) {
    if (a.cols != b.rows)
        return Matrix();
    Matrix result(a.rows, b.cols);
    for (int i = 0; i < a.rows; ++i) {
        for (int j = 0; j < b.cols; ++j) {
            double sum = 0.0f;
            for (int k = 0; k < a.cols; ++k) {
                sum += a.at(i, k) * b.at(k, j);
            }
            result.at(i, j) = sum;
        }
    }
    return result;
}

inline Matrix add(const Matrix& a, const Matrix& b) {
    if (a.rows != b.rows || a.cols != b.cols)
        return Matrix();
    Matrix result(a.rows, a.cols);
    for (int i = 0; i < a.rows * a.cols; ++i) {
        result.data[i] = a.data[i] + b.data[i];
    }
    return result;
}

inline Matrix subtract(const Matrix& a, const Matrix& b) {
    if (a.rows != b.rows || a.cols != b.cols)
        return Matrix();
    Matrix result(a.rows, a.cols);
    for (int i = 0; i < a.rows * a.cols; ++i) {
        result.data[i] = a.data[i] - b.data[i];
    }
    return result;
}

inline Matrix transpose(const Matrix& a) {
    Matrix result(a.cols, a.rows);
    for (int i = 0; i < a.rows; ++i) {
        for (int j = 0; j < a.cols; ++j) {
            result.at(j, i) = a.at(i, j);
        }
    }
    return result;
}

inline Matrix inverse(const Matrix& m) {
    if (m.rows != m.cols)
        return Matrix();
    int n = m.rows;
    Matrix temp(n, 2 * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            temp.at(i, j) = m.at(i, j);
        }
        temp.at(i, i + n) = 1.0f;
    }

    for (int i = 0; i < n; ++i) {
        int pivot = i;
        for (int j = i + 1; j < n; ++j) {
            if (std::abs(temp.at(j, i)) > std::abs(temp.at(pivot, i))) {
                pivot = j;
            }
        }
        if (pivot != i) {
            for (int j = 0; j < 2 * n; ++j) {
                std::swap(temp.at(i, j), temp.at(pivot, j));
            }
        }

        double div = temp.at(i, i);
        if (std::abs(div) < 1e-9)
            return Matrix(); // No inverse
        for (int j = 0; j < 2 * n; ++j) {
            temp.at(i, j) /= div;
        }

        for (int j = 0; j < n; ++j) {
            if (i != j) {
                double mult = temp.at(j, i);
                for (int k = 0; k < 2 * n; ++k) {
                    temp.at(j, k) -= mult * temp.at(i, k);
                }
            }
        }
    }

    Matrix result(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            result.at(i, j) = temp.at(i, j + n);
        }
    }
    return result;
}

class KalmanFilter {
public:
    int xhat_size, u_size, z_size;
    bool use_auto_adjustment;
    int measurement_valid_num;

    std::vector<uint8_t> measurement_map;
    std::vector<double> measurement_degree;
    std::vector<double> mat_r_diagonal_elements;
    std::vector<double> state_min_variance;
    std::vector<uint8_t> temp_indices;

    Matrix xhat, xhatminus, u, z, p, pminus, f, ft, b, h, ht, q, r, k;

    std::function<void(KalmanFilter*)> user_func0_f, user_func1_f, user_func2_f, user_func3_f,
        user_func4_f, user_func5_f, user_func6_f;
    bool skip_eq1 = false, skip_eq2 = false, skip_eq3 = false, skip_eq4 = false, skip_eq5 = false;

    KalmanFilter(int xs, int us, int zs)
        : xhat_size(xs)
        , u_size(us)
        , z_size(zs)
        , use_auto_adjustment(false)
        , measurement_valid_num(0) {
        xhat = Matrix(xs, 1);
        xhatminus = Matrix(xs, 1);
        p = Matrix(xs, xs);
        pminus = Matrix(xs, xs);
        f = Matrix(xs, xs);
        ft = Matrix(xs, xs);
        q = Matrix(xs, xs);

        if (us > 0) {
            u = Matrix(us, 1);
            b = Matrix(xs, us);
        }

        z = Matrix(zs, 1);
        h = Matrix(zs, xs);
        ht = Matrix(xs, zs);
        r = Matrix(zs, zs);
        k = Matrix(xs, zs);

        if (use_auto_adjustment) {
            measurement_map.resize(zs);
            measurement_degree.resize(zs);
            mat_r_diagonal_elements.resize(zs);
            temp_indices.resize(zs);
        }
        state_min_variance.resize(xs, 0.0f);
    }

    void update(
        const std::vector<double>& measured_vector, const std::vector<double>& control_vector = {}) {
        if (use_auto_adjustment) {
            h_k_r_adjustment(measured_vector);
        } else {
            z.data = measured_vector;
        }
        if (u_size > 0) {
            u.data = control_vector;
        }

        if (user_func0_f)
            user_func0_f(this);

        // 1. xhat'(k) = F·xhat(k-1) + B·u
        if (!skip_eq1) {
            if (u_size > 0) {
                xhatminus = add(multiply(f, xhat), multiply(b, u));
            } else {
                xhatminus = multiply(f, xhat);
            }
        }

        if (user_func1_f)
            user_func1_f(this);

        // 2. P'(k) = F·P(k-1)·FT + Q
        if (!skip_eq2) {
            ft = transpose(f);
            pminus = add(multiply(multiply(f, p), ft), q);
        }

        if (user_func2_f)
            user_func2_f(this);

        if (measurement_valid_num > 0 || !use_auto_adjustment) {
            // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
            if (!skip_eq3) {
                ht = transpose(h);
                Matrix s = add(multiply(multiply(h, pminus), ht), r);
                Matrix s_inv = inverse(s);
                k = multiply(multiply(pminus, ht), s_inv);
            }

            if (user_func3_f)
                user_func3_f(this);

            // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
            if (!skip_eq4) {
                Matrix innovation = subtract(z, multiply(h, xhatminus));
                xhat = add(xhatminus, multiply(k, innovation));
            }

            if (user_func4_f)
                user_func4_f(this);

            // 5. P(k) = (I - K(k)·H)·P'(k)
            if (!skip_eq5) {
                Matrix i_kh = subtract(Matrix::identity(xhat_size), multiply(k, h));
                p = multiply(i_kh, pminus);
            }
        } else {
            xhat = xhatminus;
            p = pminus;
        }

        if (user_func5_f)
            user_func5_f(this);

        for (int i = 0; i < xhat_size; ++i) {
            if (p.at(i, i) < state_min_variance[i]) {
                p.at(i, i) = state_min_variance[i];
            }
        }

        if (user_func6_f)
            user_func6_f(this);
    }

private:
    void h_k_r_adjustment(const std::vector<double>& measured_vector) {
        measurement_valid_num = 0;
        std::vector<double> temp_z_data;

        for (int i = 0; i < z_size; i++) {
            if (measured_vector[i] != 0) {
                temp_z_data.push_back(measured_vector[i]);
                temp_indices[measurement_valid_num] = i;
                measurement_valid_num++;
            }
        }

        z = Matrix(measurement_valid_num, 1);
        z.data = temp_z_data;

        h = Matrix(measurement_valid_num, xhat_size);
        for (int i = 0; i < measurement_valid_num; ++i) {
            h.at(i, measurement_map[temp_indices[i]] - 1) = measurement_degree[temp_indices[i]];
        }

        r = Matrix(measurement_valid_num, measurement_valid_num);
        for (int i = 0; i < measurement_valid_num; i++) {
            r.at(i, i) = mat_r_diagonal_elements[temp_indices[i]];
        }

        k = Matrix(xhat_size, measurement_valid_num);
    }
};

} // namespace internal

class EkfImu {
public:
    struct State {
        double q[4] = {1, 0, 0, 0};
        double gyro_bias[3] = {0, 0, 0};
        double roll = 0, pitch = 0, yaw = 0;
        double yaw_total_angle = 0;
    };

    EkfImu(
        double process_noise1, double process_noise2, double measure_noise, double lambda = 0.9996f,
        double lpf = 0.0f)
        : kf_(6, 0, 3)
        , q1_(process_noise1)
        , q2_(process_noise2)
        , r_(measure_noise)
        , lambda_(lambda)
        , acc_lpf_coef_(lpf) {

        state_.q[0] = 1.0f;
        std::copy(std::begin(state_.q), std::end(state_.q), kf_.xhat.data.begin());

        kf_.f.data = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
        kf_.p.data = {100000, 0.1, 0.1,    0.1, 0.1, 0.1, 0.1, 100000, 0.1, 0.1,    0.1, 0.1,
                      0.1,    0.1, 100000, 0.1, 0.1, 0.1, 0.1, 0.1,    0.1, 100000, 0.1, 0.1,
                      0.1,    0.1, 0.1,    0.1, 100, 0.1, 0.1, 0.1,    0.1, 0.1,    0.1, 100};

        kf_.user_func1_f =
            std::bind(&EkfImu::f_linearization_p_fading, this, std::placeholders::_1);
        kf_.user_func2_f = std::bind(&EkfImu::set_h, this, std::placeholders::_1);
        kf_.user_func3_f = std::bind(&EkfImu::xhat_update, this, std::placeholders::_1);

        kf_.skip_eq3 = true;
        kf_.skip_eq4 = true;
    }

    void set_initial_gyro_bias(double bias_x, double bias_y, double bias_z) {
        state_.gyro_bias[0] = bias_x;
        state_.gyro_bias[1] = bias_y;
        state_.gyro_bias[2] = bias_z; // 虽然z轴不被EKF更新，但仍可设置
        // 将初始值写入EKF状态向量
        kf_.xhat.at(4, 0) = bias_x;
        kf_.xhat.at(5, 0) = bias_y;
    }

    void update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
        dt_ = dt;
        double gyro[3] = {
            gx - state_.gyro_bias[0], gy - state_.gyro_bias[1], gz - state_.gyro_bias[2]};

        double halfgxdt = 0.5f * gyro[0] * dt;
        double halfgydt = 0.5f * gyro[1] * dt;
        double halfgzdt = 0.5f * gyro[2] * dt;

        kf_.f.data = {
            1,         -halfgxdt, -halfgydt, -halfgzdt, 0,         0, halfgxdt, 1, halfgzdt,
            -halfgydt, 0,         0,         halfgydt,  -halfgzdt, 1, halfgxdt, 0, 0,
            halfgzdt,  halfgydt,  -halfgxdt, 1,         0,         0, 0,        0, 0,
            0,         1,         0,         0,         0,         0, 0,        0, 1};

        if (update_count_ == 0) {
            accel_[0] = ax;
            accel_[1] = ay;
            accel_[2] = az;
        }
        accel_[0] =
            accel_[0] * acc_lpf_coef_ / (dt + acc_lpf_coef_) + ax * dt / (dt + acc_lpf_coef_);
        accel_[1] =
            accel_[1] * acc_lpf_coef_ / (dt + acc_lpf_coef_) + ay * dt / (dt + acc_lpf_coef_);
        accel_[2] =
            accel_[2] * acc_lpf_coef_ / (dt + acc_lpf_coef_) + az * dt / (dt + acc_lpf_coef_);

        double accel_norm =
            std::sqrt(accel_[0] * accel_[0] + accel_[1] * accel_[1] + accel_[2] * accel_[2]);
        std::vector<double> measured_vector(3);
        if (accel_norm > 1e-6) {
            measured_vector[0] = accel_[0] / accel_norm;
            measured_vector[1] = accel_[1] / accel_norm;
            measured_vector[2] = accel_[2] / accel_norm;
        }

        double gyro_norm = std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
        stable_flag_ = (gyro_norm < 0.3f && accel_norm > 9.3f && accel_norm < 10.3f);

        kf_.q = internal::Matrix(6, 6);
        kf_.q.at(0, 0) = kf_.q.at(1, 1) = kf_.q.at(2, 2) = kf_.q.at(3, 3) = q1_ * dt;
        kf_.q.at(4, 4) = kf_.q.at(5, 5) = q2_ * dt;

        kf_.r = internal::Matrix(3, 3);
        kf_.r.at(0, 0) = kf_.r.at(1, 1) = kf_.r.at(2, 2) = r_;

        kf_.update(measured_vector);

        std::copy(kf_.xhat.data.begin(), kf_.xhat.data.begin() + 4, state_.q);
        std::copy(kf_.xhat.data.begin() + 4, kf_.xhat.data.begin() + 6, state_.gyro_bias);
        state_.gyro_bias[2] = 0;

        calculate_euler_angles();
        update_count_++;
    }

    const State& get_state() const { return state_; }

private:
    internal::KalmanFilter kf_;
    State state_;
    double q1_, q2_, r_, lambda_, acc_lpf_coef_;
    double dt_;
    double accel_[3];
    bool stable_flag_ = false;
    bool converge_flag_ = false;
    int error_count_ = 0;
    uint64_t update_count_ = 0;
    double yaw_angle_last_ = 0;
    int yaw_round_count_ = 0;
    double chi_square_test_threshold_ = 1e-8;

    void f_linearization_p_fading(internal::KalmanFilter* kf) const {
        double q0 = kf->xhatminus.at(0, 0), q1 = kf->xhatminus.at(1, 0), q2 = kf->xhatminus.at(2, 0),
              q3 = kf->xhatminus.at(3, 0);
        double norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        if (norm > 1e-6) {
            kf->xhatminus.at(0, 0) /= norm;
            kf->xhatminus.at(1, 0) /= norm;
            kf->xhatminus.at(2, 0) /= norm;
            kf->xhatminus.at(3, 0) /= norm;
        }
        q0 = kf->xhatminus.at(0, 0);
        q1 = kf->xhatminus.at(1, 0);
        q2 = kf->xhatminus.at(2, 0);
        q3 = kf->xhatminus.at(3, 0);

        kf->f.at(0, 4) = q1 * dt_ / 2;
        kf->f.at(0, 5) = q2 * dt_ / 2;
        kf->f.at(1, 4) = -q0 * dt_ / 2;
        kf->f.at(1, 5) = q3 * dt_ / 2;
        kf->f.at(2, 4) = -q3 * dt_ / 2;
        kf->f.at(2, 5) = -q0 * dt_ / 2;
        kf->f.at(3, 4) = q2 * dt_ / 2;
        kf->f.at(3, 5) = -q1 * dt_ / 2;

        kf->p.at(4, 4) /= lambda_;
        kf->p.at(5, 5) /= lambda_;

        if (kf->p.at(4, 4) > 10000)
            kf->p.at(4, 4) = 10000;
        if (kf->p.at(5, 5) > 10000)
            kf->p.at(5, 5) = 10000;
    }

    void set_h(internal::KalmanFilter* kf) {
        double q0 = kf->xhatminus.at(0, 0), q1 = kf->xhatminus.at(1, 0), q2 = kf->xhatminus.at(2, 0),
              q3 = kf->xhatminus.at(3, 0);
        double dq0 = 2 * q0, dq1 = 2 * q1, dq2 = 2 * q2, dq3 = 2 * q3;

        kf->h = internal::Matrix(3, 6);
        kf->h.at(0, 0) = -dq2;
        kf->h.at(0, 1) = dq3;
        kf->h.at(0, 2) = -dq0;
        kf->h.at(0, 3) = dq1;
        kf->h.at(1, 0) = dq1;
        kf->h.at(1, 1) = dq0;
        kf->h.at(1, 2) = dq3;
        kf->h.at(1, 3) = dq2;
        kf->h.at(2, 0) = dq0;
        kf->h.at(2, 1) = -dq1;
        kf->h.at(2, 2) = -dq2;
        kf->h.at(2, 3) = dq3;
    }

    void xhat_update(internal::KalmanFilter* kf) {
        kf->ht = internal::transpose(kf->h);
        internal::Matrix s =
            internal::add(internal::multiply(internal::multiply(kf->h, kf->pminus), kf->ht), kf->r);
        internal::Matrix s_inv = internal::inverse(s);

        double q0 = kf->xhatminus.at(0, 0), q1 = kf->xhatminus.at(1, 0), q2 = kf->xhatminus.at(2, 0),
              q3 = kf->xhatminus.at(3, 0);
        internal::Matrix h_xhatminus(3, 1);
        h_xhatminus.at(0, 0) = 2 * (q1 * q3 - q0 * q2);
        h_xhatminus.at(1, 0) = 2 * (q0 * q1 + q2 * q3);
        h_xhatminus.at(2, 0) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        internal::Matrix innovation = internal::subtract(kf->z, h_xhatminus);

        internal::Matrix chi_square_matrix = internal::multiply(
            internal::multiply(internal::transpose(innovation), s_inv), innovation);
        double chi_square = chi_square_matrix.at(0, 0);

        double adaptive_gain_scale = 1.0f;
        if (chi_square < 0.5f * chi_square_test_threshold_) {
            converge_flag_ = true;
        }

        if (chi_square > chi_square_test_threshold_ && converge_flag_) {
            if (stable_flag_)
                error_count_++;
            else
                error_count_ = 0;
            if (error_count_ > 50) {
                converge_flag_ = false;
                kf->skip_eq5 = false;
            } else {
                kf->xhat = kf->xhatminus;
                kf->p = kf->pminus;
                kf->skip_eq5 = true;
                return;
            }
        } else {
            if (chi_square > 0.1f * chi_square_test_threshold_ && converge_flag_) {
                adaptive_gain_scale =
                    (chi_square_test_threshold_ - chi_square) / (0.9f * chi_square_test_threshold_);
            }
            error_count_ = 0;
            kf->skip_eq5 = false;
        }

        kf->k = internal::multiply(internal::multiply(kf->pminus, kf->ht), s_inv);
        for (size_t i = 0; i < kf->k.data.size(); ++i) {
            kf->k.data[i] *= adaptive_gain_scale;
        }

        internal::Matrix correction = internal::multiply(kf->k, innovation);

        if (converge_flag_) {
            for (int i = 4; i < 6; ++i) {
                if (correction.at(i, 0) > 1e-2f * dt_)
                    correction.at(i, 0) = 1e-2f * dt_;
                if (correction.at(i, 0) < -1e-2f * dt_)
                    correction.at(i, 0) = -1e-2f * dt_;
            }
        }
        correction.at(3, 0) = 0; // No yaw correction from accelerometer

        kf->xhat = internal::add(kf->xhatminus, correction);
    }

    void calculate_euler_angles() {
        double q0 = state_.q[0], q1 = state_.q[1], q2 = state_.q[2], q3 = state_.q[3];
        state_.roll = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
        double sinp = 2 * (q0 * q2 - q3 * q1);
        if (std::abs(sinp) >= 1)
            state_.pitch = std::copysign(M_PI / 2, sinp);
        else
            state_.pitch = std::asin(sinp);
        state_.yaw = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

        // to degrees
        state_.roll *= 180.0 / M_PI;
        state_.pitch *= 180.0 / M_PI;
        state_.yaw *= 180.0 / M_PI;

        if (state_.yaw - yaw_angle_last_ > 180.0f) {
            yaw_round_count_--;
        } else if (state_.yaw - yaw_angle_last_ < -180.0f) {
            yaw_round_count_++;
        }
        state_.yaw_total_angle = 360.0f * yaw_round_count_ + state_.yaw;
        yaw_angle_last_ = state_.yaw;
    }
};

} // namespace librmcs::device
