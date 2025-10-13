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

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <message_filters/subscriber.h>

namespace rmoss_projectile_motion
{
    class ProjectileSolverNode : public rclcpp::Node
    {
    public:
        ProjectileSolverNode(const rclcpp::NodeOptions &options);

    private:
        void targetCallback(const auto_aim_interfaces::msg::Target::ConstSharedPtr msg);
        void publishVisualization(const geometry_msgs::msg::PointStamped &point);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

        message_filters::Subscriber<auto_aim_interfaces::msg::Target> target_subscriber_;
        std::shared_ptr<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>> target_filter_;

        rclcpp::Publisher<atrm_interfaces::msg::CmdGimbal>::SharedPtr cmd_gimbal_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualization_pub_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::shared_ptr<sensor_msgs::msg::JointState> joint_state_;
        atrm_interfaces::msg::CmdGimbal cmd_gimbal_;
        double target_yaw_, target_pitch_;
    };

} // namespace rmoss_projectile_motion

#endif // RMOSS_PROJECTILE_MOTION__PROJECTILE_SOLVER_NODE_HPP_