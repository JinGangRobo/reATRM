#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

namespace rmcs_core::controller::chassis {

class LqrController {
public:
    LqrController() { K_.setZero(); }

    /**
     * @brief 设置K矩阵的多项式拟合系数
     * @param coeffs 包含12个向量的向量，对应K矩阵的2x6个元素。
     *               顺序为行优先：K(0,0), K(0,1), ..., K(0,5), K(1,0), ..., K(1,5)。
     *               每个内部向量包含多项式系数，从高阶到低阶（MATLAB polyfit 格式: an*x^n + ... +
     * a0）。
     */
    void set_k_poly_coeffs(const std::vector<std::vector<double>>& coeffs) {
        if (coeffs.size() != 12) {
            // 可以在这里添加日志报错，系数数量必须对应矩阵元素数量(2*6=12)
            return;
        }
        int idx = 0;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 6; ++j) {
                k_poly_coeffs_[i][j] = coeffs[idx++];
            }
        }
    }

    /**
     * @brief 计算LQR控制输出 u = -K * x
     * @param state 当前状态向量 [theta, d_theta, x, d_x, phi, d_phi]
     * @param leg_length 当前腿长 (用于计算K矩阵)
     * @return 控制量 u = [T, Tp] (驱动轮力矩, 髋关节力矩)
     */
    Eigen::Vector2d compute(const Eigen::Matrix<double, 6, 1>& state, double leg_length) {
        update_k(leg_length);
        // 标准LQR控制律: u = -K * x
        return -K_ * state;
    }

    /**
     * @brief 计算带参考状态的LQR控制输出 u = -K * (x - x_ref)
     * @param state 当前状态向量
     * @param state_ref 期望状态向量
     * @param leg_length 当前腿长
     * @return 控制量 u
     */
    Eigen::Vector2d compute(
        const Eigen::Matrix<double, 6, 1>& state, const Eigen::Matrix<double, 6, 1>& state_ref,
        double leg_length) {
        update_k(leg_length);
        return -K_ * (state - state_ref);
    }

    // 获取当前计算出的K矩阵 (用于调试)
    const Eigen::Matrix<double, 2, 6>& get_K() const { return K_; }

private:
    // LQR 反馈增益矩阵 K (2x6)
    Eigen::Matrix<double, 2, 6> K_;

    // 存储多项式系数，2行6列，每个元素是一个系数向量
    std::vector<double> k_poly_coeffs_[2][6];

    // 根据腿长更新K矩阵
    void update_k(double leg_length) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 6; ++j) {
                K_(i, j) = poly_val(k_poly_coeffs_[i][j], leg_length);
            }
        }
    }

    // 多项式求值 (Horner算法 / 秦九韶算法)
    // coeffs: [an, an-1, ..., a1, a0]
    double poly_val(const std::vector<double>& coeffs, double x) {
        double result = 0.0;
        for (double c : coeffs) {
            result = result * x + c;
        }
        return result;
    }
};

} // namespace rmcs_core::controller::chassis