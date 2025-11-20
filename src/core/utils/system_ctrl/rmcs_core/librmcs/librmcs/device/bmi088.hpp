#pragma once

#include "../utility/ekf_imu.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <numbers>
#include <utility>

namespace librmcs::device {

class Bmi088 {
public:
    explicit Bmi088(
        double process_noise1 = 10.0f, double process_noise2 = 0.001f,
        double measurement_noise = 1000000.0f, double q0 = 1, double q1 = 0, double q2 = 0,
        double q3 = 0)
        : imu_filter_(process_noise1, process_noise2, measurement_noise)
        , q0_(q0)
        , q1_(q1)
        , q2_(q2)
        , q3_(q3) {
        last_update_time_ = std::chrono::steady_clock::now();
    };

    void set_coordinate_mapping(
        std::function<std::tuple<double, double, double>(double, double, double)>
            mapping_function) {
        coordinate_mapping_function_ = std::move(mapping_function);
    }

    void store_accelerometer_status(int16_t x, int16_t y, int16_t z) {
        accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void store_gyroscope_status(int16_t x, int16_t y, int16_t z, bool cali_mode = false) {
        if (cali_mode && cali_cnt < 65535) {
            cali_cnt++;
            cali_gx = (cali_gx * (cali_cnt - 1) + x) / cali_cnt;
            cali_gy = (cali_gy * (cali_cnt - 1) + y) / cali_cnt;
            cali_gz = (cali_gz * (cali_cnt - 1) + z) / cali_cnt;

            return;
        }
        gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void update_status() {
        auto acc = accelerometer_data_.load(std::memory_order::relaxed);
        auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

        auto solve_acc = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };

        gx_ = solve_gyro(gyro.x), gy_ = solve_gyro(gyro.y), gz_ = solve_gyro(gyro.z);
        ax_ = solve_acc(acc.x), ay_ = solve_acc(acc.y), az_ = solve_acc(acc.z);

        if (coordinate_mapping_function_) {
            std::tie(gx_, gy_, gz_) = coordinate_mapping_function_(gx_, gy_, gz_);
            std::tie(ax_, ay_, az_) = coordinate_mapping_function_(ax_, ay_, az_);
        }

        ekf_ahrs_update_imu(ax_, ay_, az_, gx_, gy_, gz_);
    }

    void reset_calibration() {
        cali_cnt = 0;
        cali_gx = 0.0f;
        cali_gy = 0.0f;
        cali_gz = 0.0f;
    }

    double ax() const { return ax_; }
    double ay() const { return ay_; }
    double az() const { return az_; }

    double gx() const { return gx_; }
    double gy() const { return gy_; }
    double gz() const { return gz_; }

    double& q0() { return q0_; }
    double& q1() { return q1_; }
    double& q2() { return q2_; }
    double& q3() { return q3_; }

    double& cali_gx_ref() { return cali_gx; }
    double& cali_gy_ref() { return cali_gy; }
    double& cali_gz_ref() { return cali_gz; }

private:
    void ekf_ahrs_update_imu(double ax, double ay, double az, double gx, double gy, double gz) {
        auto now = std::chrono::steady_clock::now();
        float dt = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(
                                          now - last_update_time_)
                                          .count())
                 / 1000000.0f;
        imu_filter_.update(gx, gy, gz, ax, ay, az, dt);
        last_update_time_ = now;

        const auto& state = imu_filter_.get_state();
        q0_ = state.q[0];
        q1_ = state.q[1];
        q2_ = state.q[2];
        q3_ = state.q[3];
    }

    utility::EkfImu imu_filter_;
    std::chrono::steady_clock::time_point last_update_time_;

    struct alignas(8) ImuData {
        int16_t x, y, z;
    };
    std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
    static_assert(std::atomic<ImuData>::is_always_lock_free);

    double ax_, ay_, az_, gx_, gy_, gz_;

    int32_t cali_cnt = 0;
    double cali_gx, cali_gy, cali_gz = 0.0f;

    std::function<std::tuple<double, double, double>(double, double, double)>
        coordinate_mapping_function_;

    // Quaternion of sensor frame relative to auxiliary frame
    double q0_, q1_, q2_, q3_;
};

} // namespace librmcs::device