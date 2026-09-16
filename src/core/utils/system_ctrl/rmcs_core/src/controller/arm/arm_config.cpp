#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <cmath>
#include <cstdint>
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
#include <rmcs_msgs/vtswitch.hpp>
#include <string>
#include <urdf/model.h>
#include <vector>

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
            "/robot_description", rclcpp::QoS(10).transient_local().reliable(),
            [this](const std_msgs::msg::String::ConstSharedPtr& msg) { this->load_urdf(msg); });
        joint_states_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        for (std::size_t i = 0; i < num_joints_; ++i) {
            register_input("/arm/joint_" + std::to_string(i + 1) + "/motor/angle", joint_angle_[i]);
            register_input("/arm/joint_" + std::to_string(i + 1) + "/motor/velocity", joint_velocity_[i]);
            register_input("/arm/joint_" + std::to_string(i + 1) + "/motor/torque", joint_torque_[i]);
            register_input("/arm/joint_" + std::to_string(i + 1) + "/motor/alive", joint_alive_[i], false);
        }
        register_input("/vt03/mode_switch", mode_switch_);

        register_output("urdf_loaded", is_load, false);

        if (!this->has_parameter("offsets_verified"))
            this->declare_parameter("offsets_verified", false);
        offsets_verified_parameter_ = this->get_parameter("offsets_verified").as_bool();
        register_output(
            "/arm/config/offsets_verified", offsets_verified_, offsets_verified_parameter_);
        register_output("/arm/config/reference_latched", reference_latched_, 0.0);

        load_default_joint_offsets();
        load_gear_ratios();
        load_latch_parameters();

        offset_set_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm/config/set_offsets", rclcpp::QoS{1},[this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (msg->data.size() != num_joints_) {
                RCLCPP_WARN(get_logger(), "set_offsets: expected %zu values, got %zu",num_joints_, msg->data.size());
                return;
            }
            for (std::size_t i = 0; i < num_joints_; ++i) {
                const double theta_measured = joint[i].get_angle();
                if (!std::isfinite(theta_measured)) {
                    RCLCPP_WARN(get_logger(), "set_offsets: joint_%zu no feedback (NAN), skipped",i + 1);
                        continue;
                }
                const double desired = msg->data[i];
                if (!std::isfinite(desired)) continue;
                default_joint_offsets_[i] += theta_measured - desired;
                angle_baseline_ready_[i] = true;
            }
            RCLCPP_INFO(get_logger(),"ArmCalib: offsets updated -> [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                default_joint_offsets_[0], default_joint_offsets_[1], default_joint_offsets_[2],
                default_joint_offsets_[3], default_joint_offsets_[4], default_joint_offsets_[5]);
            });

        latch_reference_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/arm/config/latch_reference", rclcpp::QoS{1},
            [this](const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latch_reference(*msg);
            });
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
            calibrated_angles[i] = normalize_angle(joint_source_angle(i) - default_joint_offsets_[i]);
            calibrated_velocities[i] = *joint_velocity_[i];

            joint[i].update(calibrated_angles[i], calibrated_velocities[i], *joint_torque_[i]);
        }

        *reference_latched_ = angle_baseline_ok() ? 1.0 : 0.0;
        *offsets_verified_ = offsets_verified_parameter_ && latch_interlock_ok();
        report_calibration_state();

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
    void load_default_joint_offsets() {
        if (!this->has_parameter("default_joint_offsets"))
            this->declare_parameter(
                "default_joint_offsets",
                std::vector<double>{3.775127, 1.226224, -1.249272, 3.626331, 6.125952, 1.702718});
        const auto defaults = this->get_parameter("default_joint_offsets").as_double_array();
        for (std::size_t i = 0; i < num_joints_ && i < defaults.size(); ++i) {
            default_joint_offsets_[i] = defaults[i];
        }
    }

    /// 电机轴到输出轴的减速比。编码器在电机侧时需要除;直驱填 1。
    /// 待 librmcs 侧按"编码器在哪一侧"统一换算后,本参数与下面的除法一起删除。
    void load_gear_ratios() {
        if (!this->has_parameter("joint_gear_ratio"))
            this->declare_parameter(
                "joint_gear_ratio", std::vector<double>{1.0, 50.0, 50.0, 1.0, 1.0, 1.0});
        const auto ratios = this->get_parameter("joint_gear_ratio").as_double_array();
        for (std::size_t i = 0; i < num_joints_ && i < ratios.size(); ++i) {
            if (ratios[i] > 0.0)
                joint_gear_ratio_[i] = ratios[i];
        }
    }

    void load_latch_parameters() {
        auto get_double = [this](const std::string& name, double fallback) {
            if (!this->has_parameter(name))
                this->declare_parameter(name, fallback);
            return this->get_parameter(name).as_double();
        };

        if (!this->has_parameter("require_latch"))
            this->declare_parameter("require_latch", true);
        require_latch_ = this->get_parameter("require_latch").as_bool();

        if (!this->has_parameter("latch_velocity_threshold"))
            this->declare_parameter("latch_velocity_threshold", 0.1);
        latch_velocity_threshold_ = this->get_parameter("latch_velocity_threshold").as_double();

        if (!this->has_parameter("latch_joints"))
            this->declare_parameter("latch_joints", std::vector<int64_t>{2, 3});
        latch_joints_.clear();
        const auto joint_ids = this->get_parameter("latch_joints").as_integer_array();
        for (const int64_t joint_id : joint_ids) {
            if (joint_id < 1 || joint_id > static_cast<int64_t>(num_joints_)) {
                RCLCPP_WARN(
                    get_logger(), "latch_joints: invalid joint id %ld, ignored",
                    static_cast<long>(joint_id));
                continue;
            }
            latch_joints_.push_back(static_cast<std::size_t>(joint_id - 1));
        }

        for (std::size_t i = 0; i < num_joints_; ++i) {
            latch_reference_angle_[i] = get_double(
                "latch_reference_angle_j" + std::to_string(i + 1),
                std::numeric_limits<double>::quiet_NaN());
        }
    }

    /// 关节角来源:电机多圈角除以减速比,得到输出轴角度
    /// 减速比换算收敛到 librmcs 之后,这里可以直接返回 *joint_angle_[index]
    double joint_source_angle(std::size_t index) const {
        return *joint_angle_[index] / joint_gear_ratio_[index];
    }

    bool angle_baseline_ok() const {
        for (const std::size_t index : latch_joints_) {
            if (!angle_baseline_ready_[index])
                return false;
        }
        return true;
    }

    bool latch_interlock_ok() const {
        return !require_latch_ || angle_baseline_ok();
    }

    void report_calibration_state() {
        const bool baseline_ok = angle_baseline_ok();
        if (baseline_ok != last_baseline_ok_) {
            last_baseline_ok_ = baseline_ok;
            if (baseline_ok) {
                RCLCPP_INFO(get_logger(), "ArmCalib: angle baseline ready");
            } else {
                RCLCPP_WARN(
                    get_logger(),
                    "ArmCalib: angle baseline missing; move the joints listed in latch_joints to "
                    "their reference pose, then publish /arm/config/latch_reference");
            }
        }
        if (baseline_ok && !offsets_verified_parameter_ && !offset_parameter_warned_) {
            offset_parameter_warned_ = true;
            RCLCPP_WARN(
                get_logger(),
                "ArmCalib: baseline ready but offsets_verified is false; set it true to release the "
                "torque interlock");
        }
    }

    void latch_reference(const std_msgs::msg::Int32MultiArray& msg) {
        if (msg.data.empty()) {
            RCLCPP_WARN(get_logger(), "ArmLatch: empty joint list, nothing to do");
            return;
        }
        const auto switch_position = *mode_switch_;
        if (switch_position != rmcs_msgs::VtSwitch::LEFT
            && switch_position != rmcs_msgs::VtSwitch::UNKNOWN) {
            RCLCPP_WARN(
                get_logger(), "ArmLatch: arm is enabled, refuse; put the mode switch to the left");
            return;
        }
        for (const int32_t joint_id : msg.data) {
            if (joint_id < 1 || joint_id > static_cast<int32_t>(num_joints_)) {
                RCLCPP_WARN(get_logger(), "ArmLatch: invalid joint id %d, skipped", joint_id);
                continue;
            }
            const std::size_t index = static_cast<std::size_t>(joint_id) - 1;
            if (!std::isfinite(latch_reference_angle_[index])) {
                RCLCPP_WARN(
                    get_logger(), "ArmLatch: joint_%zu has no latch_reference_angle_j%zu, skipped",
                    index + 1, index + 1);
                continue;
            }
            if (!joint_alive_[index].ready() || !*joint_alive_[index]) {
                RCLCPP_WARN(get_logger(), "ArmLatch: joint_%zu motor offline, skipped", index + 1);
                continue;
            }
            const double velocity = *joint_velocity_[index];
            if (!std::isfinite(velocity) || std::abs(velocity) > latch_velocity_threshold_) {
                RCLCPP_WARN(
                    get_logger(), "ArmLatch: joint_%zu moving at %.3f rad/s, hold it still",
                    index + 1, velocity);
                continue;
            }
            const double source_angle = joint_source_angle(index);
            if (!std::isfinite(source_angle)) {
                RCLCPP_WARN(
                    get_logger(), "ArmLatch: joint_%zu source angle invalid, skipped", index + 1);
                continue;
            }
            const double previous_offset = default_joint_offsets_[index];
            default_joint_offsets_[index] = source_angle - latch_reference_angle_[index];
            angle_baseline_ready_[index] = true;
            RCLCPP_INFO(
                get_logger(), "ArmLatch: joint_%zu latched at %.4f rad, offset %.6f -> %.6f",
                index + 1, latch_reference_angle_[index], previous_offset,
                default_joint_offsets_[index]);
        }
    }

    double default_joint_offsets_[num_joints_]{};
    std::array<double, num_joints_> joint_gear_ratio_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double latch_reference_angle_[num_joints_]{};
    bool angle_baseline_ready_[num_joints_]{};
    bool offsets_verified_parameter_{false};
    bool require_latch_{true};
    bool last_baseline_ok_{true};
    bool offset_parameter_warned_{false};
    double latch_velocity_threshold_{0.1};
    std::vector<std::size_t> latch_joints_;

    OutputInterface<bool> offsets_verified_;
    OutputInterface<double> reference_latched_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr offset_set_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr latch_reference_sub_;
    std::array<InputInterface<bool>, num_joints_> joint_alive_;
    InputInterface<rmcs_msgs::VtSwitch> mode_switch_;

    void modify_link_length(const urdf::Model& model) {
        link[0].load_length(model.getJoint("joint_2")->parent_to_joint_origin_transform.position.z);
        link[1].load_length(model.getJoint("joint_3")->parent_to_joint_origin_transform.position.y);
        // 小臂长度 = 沿 link_3 的 y 轴到肘的距离;原来的 .x 是横向偏置(-0.005),取错了字段
        link[2].load_length(model.getJoint("joint_4")->parent_to_joint_origin_transform.position.y);
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
