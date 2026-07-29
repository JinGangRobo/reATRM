#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <limits>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rmcs_executor/component.hpp>
#include <string>
#include <urdf/model.h>

namespace rmcs_core::controller::arm {

class ArmConfig
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmConfig()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , link{
            Link(*this,"/arm/link_1"), Link(*this,"/arm/link_2"), Link(*this,"/arm/link_3"), 
            Link(*this,"/arm/link_4"), Link(*this,"/arm/link_5"), Link(*this,"/arm/link_6"), 
        }
        , joint{
            Joint(*this,"/arm/joint_1"), Joint(*this,"/arm/joint_2"), Joint(*this,"/arm/joint_3"), 
            Joint(*this,"/arm/joint_4"), Joint(*this,"/arm/joint_5"), Joint(*this,"/arm/joint_6"), 
        } {
        arm_urdf_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(0).transient_local().reliable(),
            [this](const std_msgs::msg::String::ConstSharedPtr& msg) { this->load_urdf(msg); });
        joint_states_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        for (std::size_t i = 0; i < num_joints_; ++i) {
            register_input("/arm/joint_" + std::to_string(i + 1) + "/motor/angle", joint_angle_[i]);
            register_input("/arm/joint_" + std::to_string(i + 1) + "/motor/velocity", joint_velocity_[i]);
            register_input("/arm/joint_" + std::to_string(i + 1) + "/motor/torque", joint_torque_[i]);
        }
        register_output("urdf_loaded", is_load, false);
    };

    static inline double normalize_angle(double angle) {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0) {
            a += 2.0 * M_PI;
        }
        return a - M_PI;
        }

    void update() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::array<double, num_joints_> calibrated_angles;
        std::array<double, num_joints_> calibrated_velocities;
        for (std::size_t i = 0; i < num_joints_; ++i) {
            double raw_angle = *joint_angle_[i];
            double raw_vel   = *joint_velocity_[i]; 
            double gear_ratio = (i == 1 || i == 2) ? 50.0 : 1.0;
            double act_angle = (raw_angle / gear_ratio - joint_offsets_[i]) * joint_directions_[i];
            calibrated_angles[i]     = act_angle;
            calibrated_velocities[i] = (raw_vel / gear_ratio) * joint_directions_[i];

            joint[i].update(calibrated_angles[i], calibrated_velocities[i], *joint_torque_[i]);
        }

        static std::size_t count{0};
        if (++count >= 9) {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp    = this->now();
            msg.header.frame_id = "base_link";
            msg.name            = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            // msg.position = {*joint_angle_[0], *joint_angle_[1], *joint_angle_[2],
            //             *joint_angle_[3], *joint_angle_[4], *joint_angle_[5]};
            msg.position = {calibrated_angles[0],calibrated_angles[1],calibrated_angles[2],
                            calibrated_angles[3],calibrated_angles[4],calibrated_angles[5]};
            msg.velocity = {calibrated_velocities[0], calibrated_velocities[1], calibrated_velocities[2],
                        calibrated_velocities[3], calibrated_velocities[4], calibrated_velocities[5]};
        
            joint_states_pub_->publish(msg);
            count = 0;
        }
    }

private:
    static constexpr std::size_t num_joints_ = 6;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    const std::array<double, num_joints_> joint_offsets_ = {
    3.775127,   // J1
    1.226224,   // J2 
   -1.249272,   // J3 
    3.626331,   // J4
    6.125952,   // J5
    1.460350    // J6
    };
    const std::array<double, num_joints_> joint_directions_ = {
        1.0,   // joint_1
        1.0,   // joint_2
        1.0,   // joint_3 
        1.0,   // joint_4
        1.0,   // joint_5 
        1.0    // joint_6
    };
    void modify_link_length(const urdf::Model& model) {
        link[0].load_length(model.getJoint("joint_2")->parent_to_joint_origin_transform.position.z);
        link[1].load_length(model.getJoint("joint_3")->parent_to_joint_origin_transform.position.y);
        link[2].load_length(model.getJoint("joint_4")->parent_to_joint_origin_transform.position.x);
        link[3].load_length(
            model.getJoint("joint_4")->parent_to_joint_origin_transform.position.y
            + model.getJoint("joint_5")->parent_to_joint_origin_transform.position.z);
        link[4].load_length(0.0);
        link[5].load_length(model.getJoint("joint_6")->parent_to_joint_origin_transform.position.y);
    }
    class Link {
    public:
        explicit Link(rmcs_executor::Component& status_component, const std::string& name_prefix) {
            status_component.register_output(
                name_prefix + "/com", com_,
                Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
            status_component.register_output(
                name_prefix + "/inertia", inertia_,
                Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN()));
            status_component.register_output(name_prefix + "/mass", mass_, NAN);
            status_component.register_output(name_prefix + "/length", length_, NAN);
        }
        void load_parameter(const double& m, const Eigen::Vector3d& c, const Eigen::Matrix3d& I) {
            *mass_ = m;
            *com_ = c;
            *inertia_ = I;
        }
        void load_length(const double& l) { *length_ = l; }
        double get_mass() const { return *mass_; }
        double get_length() const { return *length_; }
        Eigen::Vector3d get_com() const { return *com_; }
        Eigen::Matrix3d get_inertia() const { return *inertia_; }

    private:
        OutputInterface<Eigen::Vector3d> com_;
        OutputInterface<Eigen::Matrix3d> inertia_;
        OutputInterface<double> mass_;
        OutputInterface<double> length_;
    };
    class Joint {
    public:
        explicit Joint(rmcs_executor::Component& status_component, const std::string& name_prefix) {
            status_component.register_output(name_prefix + "/lower_limit", lower_limit_, NAN);
            status_component.register_output(name_prefix + "/upper_limit", upper_limit_, NAN);
            status_component.register_output(name_prefix + "/velocity_limit", velocity_limit_, NAN);
            status_component.register_output(name_prefix + "/torque_limit", torque_limit_, NAN);
            status_component.register_output(name_prefix + "/theta", theta_, NAN);
            status_component.register_output(name_prefix + "/velocity", velocity_, NAN);
            status_component.register_output(name_prefix + "/torque", torque_, NAN);
            status_component.register_output(
                name_prefix + "/position", pos_,
                Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
            status_component.register_output(name_prefix + "/friction", friction_, NAN);
        }
        void load_parameter(
            const double& upper, const double& lower, const double& vel, const double& torque,
            const Eigen::Vector3d& pos, const double& f) {
            *lower_limit_ = lower;
            *upper_limit_ = upper;
            *velocity_limit_ = vel;
            *torque_limit_ = torque;
            *pos_ = pos;
            *friction_ = f;
        }
        void update(const double& angle, const double& vel, const double& tor) {
            *theta_ = angle;
            *velocity_ = vel;
            *torque_ = tor;
        }
        double get_upper_limit() const { return *upper_limit_; }
        double get_lower_limit() const { return *lower_limit_; }
        double get_velocity_limit() const { return *velocity_limit_; }
        double get_torque_limit() const { return *torque_limit_; }
        double get_angle() const { return *theta_; }

    private:
        OutputInterface<double> lower_limit_;
        OutputInterface<double> upper_limit_;
        OutputInterface<double> velocity_limit_;
        OutputInterface<double> torque_limit_;

        OutputInterface<double> theta_;
        OutputInterface<double> velocity_;
        OutputInterface<double> torque_;

        OutputInterface<Eigen::Vector3d> pos_;
        OutputInterface<double> friction_;
    };
    void load_urdf(const std_msgs::msg::String::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (*is_load)
            return;
        const std::string& urdf_xml = msg->data;
        urdf::Model model;
        if (!model.initString(urdf_xml)) {
            RCLCPP_ERROR(rclcpp::get_logger("ArmConfig"), "Failed to parse URDF");
            return;
        }
        for (std::size_t i = 0; i < num_joints_; ++i) {
            const std::string link_name = "link_" + std::to_string(i + 1);
            auto link_urdf = model.getLink(link_name);
            if (!link_urdf) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("ArmConfig"), "Failed to find link %s in URDF",
                    link_name.c_str());
                return;
            }
            if (!link_urdf->inertial) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("ArmConfig"), "Link %s has no inertial information in URDF",
                    link_name.c_str());
                return;
            }
            const double mass = link_urdf->inertial->mass;

            const auto& p = link_urdf->inertial->origin.position;
            Eigen::Vector3d com(p.x, p.y, p.z);

            const auto& I = link_urdf->inertial;
            Eigen::Matrix3d inertia;
            inertia << I->ixx, I->ixy, I->ixz, I->ixy, I->iyy, I->iyz, I->ixz, I->iyz, I->izz;
            link[i].load_parameter(mass, com, inertia);
            const std::string joint_name = "joint_" + std::to_string(i + 1);
            auto joint_urdf = model.getJoint(joint_name);
            if (!joint_urdf) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("ArmConfig"), "Failed to find joint %s in URDF",
                    joint_name.c_str());
                return;
            }
            if (!joint_urdf->limits) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("ArmConfig"), "Joint %s has no limits information in URDF",
                    joint_name.c_str());
                return;
            }
            const auto& jp = joint_urdf->parent_to_joint_origin_transform.position;
            Eigen::Vector3d joint_pos(jp.x, jp.y, jp.z);
            joint[i].load_parameter(
                joint_urdf->limits->upper, joint_urdf->limits->lower, joint_urdf->limits->velocity,
                joint_urdf->limits->effort, joint_pos, joint_urdf->dynamics->friction);
        }
        modify_link_length(model);
        *is_load = true;
        arm_urdf_.reset();
    };

    std::array<Link, num_joints_> link;
    std::array<Joint, num_joints_> joint;
    OutputInterface<bool> is_load;
    std::mutex data_mutex_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_urdf_;
    std::array<InputInterface<double>, num_joints_> joint_angle_;
    std::array<InputInterface<double>, num_joints_> joint_velocity_;
    std::array<InputInterface<double>, num_joints_> joint_torque_;
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmConfig, rmcs_executor::Component)
