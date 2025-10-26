#ifndef RMOSS_PROJECTILE_MOTION__PROJECTILE_SOLVER_NODE_HPP_
#define RMOSS_PROJECTILE_MOTION__PROJECTILE_SOLVER_NODE_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rmoss_projectile_motion/projectile_solver_interface.hpp"
#include "rmoss_projectile_motion/iterative_projectile_tool.hpp"
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"
#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"
#include "rmoss_projectile_motion/gaf_projectile_solver.hpp"

#include "auto_aim_interfaces/msg/target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "atrm_interfaces/msg/cmd_gimbal.hpp"

namespace rmoss_projectile_motion
{
    class ProjectileSolverNode : public rclcpp::Node
    {
    public:
        ProjectileSolverNode(const rclcpp::NodeOptions &options);

    private:
        void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);

        rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

        rclcpp::Publisher<atrm_interfaces::msg::CmdGimbal>::SharedPtr cmd_gimbal_pub_;

        std::shared_ptr<sensor_msgs::msg::JointState> joint_state_;

        atrm_interfaces::msg::CmdGimbal cmd_gimbal_;

        double target_yaw_, target_pitch_;
    };

} // namespace rmoss_projectile_motion

#endif // RMOSS_PROJECTILE_MOTION__PROJECTILE_SOLVER_NODE_HPP_