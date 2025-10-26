#include "rmoss_projectile_motion/projectile_solver_node.hpp"

namespace rmoss_projectile_motion
{
    ProjectileSolverNode::ProjectileSolverNode(const rclcpp::NodeOptions &options)
        : Node("armor_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "projectile_solver_node start");
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "gimbal_state", 3, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)
            { joint_state_ = std::make_shared<sensor_msgs::msg::JointState>(*msg); });
        target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
            "tracker/target", rclcpp::SensorDataQoS(), std::bind(&ProjectileSolverNode::targetCallback, this, std::placeholders::_1));
        cmd_gimbal_pub_ = this->create_publisher<atrm_interfaces::msg::CmdGimbal>(
            "cmd_gimbal", 3);
    }

    void ProjectileSolverNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
    {
        // 创建考虑重力和空气阻力的求解器，GafProjectileSolver，参数代表子弹速度为25m/s，空气阻力系数为0.1
        if (!msg->tracking)
        {
            return;
        }

        auto gaf_solver = std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(2500, 0.1);

        // 创建gimbal_transform_tool，传入graviry_solver求解器
        auto projectile_tansformoss_tool =
            std::make_shared<rmoss_projectile_motion::GimbalTransformTool>(gaf_solver);

        // 求解例子

        auto distance = msg->position.x * msg->position.x + msg->position.y * msg->position.y;
        distance = sqrt(distance);
        Eigen::Vector3d position(distance, 0, msg->position.z);
        bool solve_status = projectile_tansformoss_tool->solve(position, target_pitch_, target_yaw_);
        target_yaw_ = atan2(msg->position.y, msg->position.x);

        if (!solve_status)
        {
            RCLCPP_WARN(this->get_logger(), "Solve failed: %s", projectile_tansformoss_tool->error_message().c_str());
            return;
        }

        cmd_gimbal_.pitch = -target_pitch_ + 0.72; // TODO: WTF??
        cmd_gimbal_.yaw = target_yaw_ + 1.55;
        cmd_gimbal_.fire = false;

        cmd_gimbal_pub_->publish(cmd_gimbal_);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_projectile_motion::ProjectileSolverNode)