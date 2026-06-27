#pragma once
#include <vector>
#define IKFAST_HAS_LIBRARY
// #include <algorithm>
#include <array>
#include <cmath>
#include <limits>

// #include <limits>
// #include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
// #include <cstdint>
// #include <rmcs_utility/eigen_structured_bindings.hpp>
#include "ikfast.h"

namespace rmcs_core::controller::arm {
using namespace rmcs_description;

class ArmKinematicsSolver {
    class Operation {
        friend class ArmKinematicsSolver;
        // 返回 true 表示需要控制，并将期望的目标位姿写入 target_pose
        // 返回 false 表示解除控制（禁用）
        virtual bool update(ArmKinematicsSolver& super, Eigen::Isometry3d& target_pose) const = 0;
    };
public:
    struct JointAngle {
        double q0 , q1 , q2 , q3 , q4 , q5;
        bool Vaild = false;
    };
    
    ArmKinematicsSolver(rmcs_executor::Component& component) {
        component.register_input("/arm_tf", tf_);
        component.register_input("/arm/joint0/angle", joint0_angle_);
        component.register_input("/arm/joint1/angle", joint1_angle_);
        component.register_input("/arm/joint2/angle", joint2_angle_);
        component.register_input("/arm/joint3/angle", joint3_angle_);
        component.register_input("/arm/joint4/angle", joint4_angle_);
        component.register_input("/arm/joint5/angle", joint5_angle_);
    }

    class SetDisabled : public Operation {
        bool update(ArmKinematicsSolver& super, Eigen::Isometry3d& /*target_pose*/) const override {
            super.control_enabled_ = false;
            return false;
        }
    };

    class SetTargetPose : public Operation {
    public:
        explicit SetTargetPose(Eigen::Isometry3d target)
            : target_(std::move(target)) {}
    private:
        bool update (ArmKinematicsSolver& super, Eigen::Isometry3d& target_pose) const override {
            super.control_enabled_ = true;
            target_pose = target_;
            return true;
        }
        Eigen::Isometry3d target_;
    };

    JointAngle update(const Operation& operation) {
        Eigen::Isometry3d target_pose;
        if(!operation.update(*this, target_pose)){
            return {};
        }
        std::array<double, 6>now_joint_angle = {
            *joint0_angle_ , *joint1_angle_ , *joint2_angle_,
            *joint3_angle_ , *joint4_angle_ , *joint5_angle_
        };
        JointAngle result = inverseKinematics(target_pose, now_joint_angle);
        return result;
    }
private:
    bool isValidJoint(const std::vector<IkReal>& /*solutions*/) {
        return true;
    }

    double NormAngle(double angle){
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += 2.0 * M_PI;
        }
        return a - M_PI;
    }

    JointAngle inverseKinematics(const Eigen::Isometry3d& target_pose, const std::array<double, 6>& now_joint_angle) {
        const Eigen::Matrix3d& R = target_pose.rotation();
        ikfast::IkSolutionList<IkReal> solutions;

        IkReal eerot[9] = {
            static_cast<IkReal>(R(0, 0)), static_cast<IkReal>(R(0, 1)), static_cast<IkReal>(R(0, 2)),
            static_cast<IkReal>(R(1, 0)), static_cast<IkReal>(R(1, 1)), static_cast<IkReal>(R(1, 2)),
            static_cast<IkReal>(R(2, 0)), static_cast<IkReal>(R(2, 1)), static_cast<IkReal>(R(2, 2))
        };
        IkReal eetrans[3] {
            static_cast<IkReal>(target_pose.translation().x()),
            static_cast<IkReal>(target_pose.translation().y()),
            static_cast<IkReal>(target_pose.translation().z())
        };

        for(auto& v:eetrans){
            if(v==0.0)
                v = 0.01;
        }
        IkReal pfree[1] = {static_cast<IkReal>(now_joint_angle[4])};
        // RCLCPP_INFO(rclcpp::get_logger("jointIK"),"NumJoints=%d, NumFreeParams=%d", GetNumJoints(), GetNumFreeParameters());
        bool bSuccess = ComputeIk(eetrans, eerot, pfree, solutions);
        if(!bSuccess){
            return {};
        }else{
            std::vector<IkReal> sol_values(GetNumJoints());
            std::vector<IkReal> best_sol;
            double min_error = std::numeric_limits<double>::max();
            for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i){
                const auto& sol = solutions.GetSolution(i);
                std::vector<IkReal> free(sol.GetFree().size());
                sol.GetSolution(sol_values.data(), free.empty() ? nullptr : free.data());
                double error = 0.0;
                for (std::size_t j = 0; j < sol_values.size(); ++j) {
                    double e = NormAngle(sol_values[j] - now_joint_angle[j]);
                    error += e * e;
                }
                if (error < min_error && isValidJoint(sol_values)) {
                    min_error = error;
                    best_sol = sol_values;
                }
            }
            if(!best_sol.empty()){
                return {best_sol[0],best_sol[1],best_sol[2],
                        best_sol[3],best_sol[4],best_sol[5],
                        true};
            }
        }
        return {};
    }
    rmcs_executor::Component::InputInterface<ArmTf> tf_;
    rmcs_executor::Component::InputInterface<double> joint0_angle_;
    rmcs_executor::Component::InputInterface<double> joint1_angle_;
    rmcs_executor::Component::InputInterface<double> joint2_angle_;
    rmcs_executor::Component::InputInterface<double> joint3_angle_;
    rmcs_executor::Component::InputInterface<double> joint4_angle_;
    rmcs_executor::Component::InputInterface<double> joint5_angle_;
    
    bool control_enabled_ = false;
    };

} // namespace rmcs_core::controller::arm