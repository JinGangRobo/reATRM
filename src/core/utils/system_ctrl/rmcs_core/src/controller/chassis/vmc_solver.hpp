#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace rmcs_core::controller::chassis {
class VmcSolver {
public:
    VmcSolver(double l1, double l2, double l5, double gas_spring)
        : l1_(l1)
        , l2_(l2)
        , l3_(l2)
        , l4_(l1)
        , l5_(l5)
        , gas_spring_(gas_spring) {
        reset();
    }

    void reset() {
        tilt_angle_ = nan_;
        leg_length_ = nan_;
    }

    Eigen::Vector2d update(double phi1, double phi4) {
        if (std::isnan(phi1) || std::isnan(phi4)) {
            reset();
            return Eigen::Vector2d{leg_length_, tilt_angle_};
        }

        calculate_five_link_solution(phi1, phi4);

        return get_leg_posture();
    }
    Eigen::Vector2d update_velocity(double vel1, double vel4) {
        if (std::isnan(vel1) || std::isnan(vel4)) {
            reset();
            return Eigen::Vector2d{leg_length_, tilt_angle_};
        }
        Eigen::Vector2d Dot_phi = {vel1, vel4};

        return jacobian_matrix * Dot_phi;
    }
    Eigen::Vector2d update_joint_torque(double F, double Tp) {
        return jacobian_matrix.transpose() * Eigen::Vector2d{F, Tp};
    }

    Eigen::Vector2d update_virtual_torque(double T1, double T2) {
        return -(jacobian_matrix.transpose().inverse() * Eigen::Vector2d{T1, T2}); // F , Tp
    }
    double update_gas_spring() {
        double gas_spring_force = std::fabs(
            (0.5f * (gas_spring_ * std::sin(alpha[0])))
            * std::sin(pi_ - 0.5f * alpha[1] - alpha[0]));
        return  gas_spring_force;
    }

private:
    void calculate_five_link_solution(double phi1, double phi4) {
        auto xb = l1_ * std::cos(phi1), yb = l1_ * std::sin(phi1);
        auto xd = l5_ + l4_ * std::cos(phi4), yd = l4_ * std::sin(phi4);

        auto lbd = std::sqrt((xd - xb) * (xd - xb) + (yd - yb) * (yd - yb));

        auto a = 2 * l2_ * (xd - xb), b = 2 * l2_ * (yd - yb),
             c = l2_ * l2_ + lbd * lbd - l3_ * l3_;

        auto phi2 = 2 * std::atan2(b + std::sqrt(a * a + b * b - c * c), (a + c)),
             phi3 = std::atan2((yb - yd) + l2_ * std::sin(phi2), (xb - xd) + l2_ * std::cos(phi2));

        auto xc = l1_ * std::cos(phi1) + l2_ * std::cos(phi2),
             yc = l1_ * std::sin(phi1) + l2_ * std::sin(phi2);

        auto phi0 = std::atan2(yc, xc - l5_ / 2.0);

        tilt_angle_ = pi_ / 2.0 - phi0;
        leg_length_ = std::sqrt((xc - l5_ / 2.0) * (xc - l5_ / 2.0) + yc * yc);

        leg_posture_ = {leg_length_, tilt_angle_};

        auto j11 = l1_ * std::sin(phi0 - phi3) * std::sin(phi1 - phi2) / std::sin(phi3 - phi2),
             j12 = l4_ * std::sin(phi0 - phi2) * std::sin(phi3 - phi4) / std::sin(phi3 - phi2),
             j21 = l1_ * std::cos(phi0 - phi3) * std::sin(phi1 - phi2)
                 / (leg_length_ * std::sin(phi3 - phi2)),
             j22 = l4_ * std::cos(phi0 - phi2) * std::sin(phi3 - phi4)
                 / (leg_length_ * std::sin(phi3 - phi2));

        alpha[0] = acos((l1_ * l1_ + l2_ * l2_ - leg_length_ * leg_length_) / (2.0f * l1_ * l2_));
        alpha[1] = phi1 - phi4;
        if (alpha[1] > 2 * std::numbers::pi) {
            alpha[1] -= 2 * std::numbers::pi;
        }

        jacobian_matrix = Eigen::Matrix2d{
            {j11, j12},
            {j21, j22}
        };
    }

    Eigen::Vector2d get_leg_posture() const { return leg_posture_; }

    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double pi_ = std::numbers::pi;

    const double l1_, l2_, l3_, l4_, l5_, gas_spring_;

    double leg_length_, tilt_angle_, alpha[2];

    Eigen::Vector2d leg_posture_;
    Eigen::Matrix2d joint_torque_matrix_;
    Eigen::Matrix2d jacobian_matrix;
};
} // namespace rmcs_core::controller::chassis