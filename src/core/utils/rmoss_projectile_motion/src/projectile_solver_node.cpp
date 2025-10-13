#include "rmoss_projectile_motion/projectile_solver_node.hpp"

namespace rmoss_projectile_motion
{
    ProjectileSolverNode::ProjectileSolverNode(const rclcpp::NodeOptions &options)
        : Node("armor_detector", options),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "projectile_solver_node start");

        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer_.setCreateTimerInterface(timer_interface);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "gimbal_state", 3, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)
            { joint_state_ = std::make_shared<sensor_msgs::msg::JointState>(*msg); });

        // 创建 message_filters::Subscriber
        target_subscriber_.subscribe(this, "tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());

        target_filter_ = std::make_shared<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>>(
            target_subscriber_,
            tf_buffer_,
            "gimbal_sub_yaw_odom",
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface(),
            std::chrono::duration<int>(1)); // 修改为 double 类型的 duration

        target_filter_->registerCallback(
            std::bind(&ProjectileSolverNode::targetCallback, this, std::placeholders::_1));

        cmd_gimbal_pub_ = this->create_publisher<atrm_interfaces::msg::CmdGimbal>(
            "cmd_gimbal", 3);
        visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "tracker/marker", 10);
    }

    void ProjectileSolverNode::targetCallback(const auto_aim_interfaces::msg::Target::ConstSharedPtr msg)
    {
        // 创建考虑重力和空气阻力的求解器，GafProjectileSolver，参数代表子弹速度为25m/s，空气阻力系数为0.1
        if (!msg->tracking)
        {
            return;
        }

        try
        {
            geometry_msgs::msg::PointStamped point_origin;
            point_origin.header = msg->header;
            point_origin.header.frame_id = "industrial_camera";
            point_origin.point.x = msg->position.x;
            point_origin.point.y = msg->position.y;
            point_origin.point.z = msg->position.z;

            geometry_msgs::msg::PointStamped point_target;
            point_target = tf_buffer_.transform(point_origin, "gimbal_sub_yaw_odom", tf2::durationFromSec(0.1));

            // 绘制变换后的位置
            publishVisualization(point_target);

            // 使用变换后的坐标进行弹道求解
            auto gaf_solver = std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(2500, 0.1);
            auto projectile_tansformoss_tool =
                std::make_shared<rmoss_projectile_motion::GimbalTransformTool>(gaf_solver);

            // 计算距离（使用变换后的坐标）
            auto distance = point_target.point.x * point_target.point.x +
                            point_target.point.y * point_target.point.y;
            distance = sqrt(distance);

            Eigen::Vector3d position(distance, 0, point_target.point.z);
            bool solve_status = projectile_tansformoss_tool->solve(position, target_pitch_, target_yaw_);
            target_yaw_ = atan2(point_target.point.y, point_target.point.x);

            if (!solve_status)
            {
                RCLCPP_WARN(this->get_logger(), "Solve failed: %s",
                            projectile_tansformoss_tool->error_message().c_str());
                return;
            }

            cmd_gimbal_.pitch = -target_pitch_ + 0.72; // TODO: WTF??
            cmd_gimbal_.yaw = target_yaw_;
            cmd_gimbal_.fire = false;

            cmd_gimbal_pub_->publish(cmd_gimbal_);

            RCLCPP_DEBUG(this->get_logger(),
                         "Transformed target from camo (%.3f, %.3f, %.3f) to gimy (%.3f, %.3f, %.3f)",
                         msg->position.x, msg->position.y, msg->position.z,
                         point_target.point.x, point_target.point.y, point_target.point.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from camo to gimy: %s", ex.what());
            return;
        }
    }

    void ProjectileSolverNode::publishVisualization(const geometry_msgs::msg::PointStamped &point)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "gimbal_sub_yaw_odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "target_position";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = point.point.x;
        marker.pose.position.y = point.point.y;
        marker.pose.position.z = point.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(100000000); // 0.1秒

        visualization_pub_->publish(marker);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_projectile_motion::ProjectileSolverNode)