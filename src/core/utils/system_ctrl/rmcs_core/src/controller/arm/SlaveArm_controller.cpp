#include "controller/arm/kinematics_solver.hpp"
#include <cmath>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::arm {

class SlaveArmController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SlaveArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , armKinematicsSolver(*this) {
        register_input("/arm/tcp/target_x", target_x_);
        register_input("/arm/tcp/target_y", target_y_);
        register_input("/arm/tcp/target_z", target_z_);
        register_input("/arm/tcp/target_roll",  target_roll_);
        register_input("/arm/tcp/target_pitch", target_pitch_);
        register_input("/arm/tcp/target_yaw",   target_yaw_);

        register_output("/arm/joint0/control_angle", joint0_target_angle_);
        register_output("/arm/joint1/control_angle_", joint1_target_angle_);
        register_output("/arm/joint2/control_angle_", joint2_target_angle_);
        register_output("/arm/joint3/control_angle", joint3_target_angle_);
        register_output("/arm/joint4/control_angle", joint4_target_angle_);
        register_output("/arm/joint5/control_angle", joint5_target_angle_);
    }

    void update() override {
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        target_pose.translation() = Eigen::Vector3d(*target_x_, *target_y_, *target_z_);
        target_pose.linear() =
            (Eigen::AngleAxisd(*target_yaw_,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(*target_pitch_, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(*target_roll_,  Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
        // TEST：
        // target_pose.translation() = Eigen::Vector3d{0.3, 0.0, 0.2}; // x, y, z
        // target_pose.linear() = (Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ())
        //                         * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY())
        //                         * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()))
        //                            .toRotationMatrix();

        auto result = armKinematicsSolver.update(ArmKinematicsSolver::SetTargetPose(target_pose));
        if (!result.Vaild) {
            // TODO:现阶段保持上一次数据
            return;
        } else {
            *joint0_target_angle_ = result.q0;
            *joint1_target_angle_ = result.q1;
            *joint2_target_angle_ = result.q2;
            *joint3_target_angle_ = result.q3;
            *joint4_target_angle_ = result.q4;
            *joint5_target_angle_ = result.q5;
        }
    }

private:
    InputInterface<double> target_x_, target_y_, target_z_;
    InputInterface<double> target_roll_, target_pitch_, target_yaw_;

    OutputInterface<double> joint0_target_angle_;
    OutputInterface<double> joint1_target_angle_;
    OutputInterface<double> joint2_target_angle_;
    OutputInterface<double> joint3_target_angle_;
    OutputInterface<double> joint4_target_angle_;
    OutputInterface<double> joint5_target_angle_;
    ArmKinematicsSolver armKinematicsSolver;
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::SlaveArmController, rmcs_executor::Component)