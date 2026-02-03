#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Ref.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class OperateLogic
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OperateLogic()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        get_parameter("comfort_translational_velocity", comfort_translational_velocity);
        get_parameter("comfort_angular_velocity", comfort_angular_velocity);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/gimbal/bullet_fired", bullet_fired_);
        register_input("/chassis/supercap/voltage", supercap_voltage_);
        register_input("/referee/id", robot_id_);
        register_input("/remote/mouse", mouse_);

        register_output("/chassis/translational_vmax", translational_velocity_max_);
        register_output("/chassis/angular_vmax", angular_velocity_max_);
        register_output("/chassis/boost_mode", boost_mode_);
        register_output("/gimbal/friction_wheel", friction_wheel_mode_, 0.0);
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }
        } while (false);
        update_status();
        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;

        *translational_velocity_max_ = translational_velocity_max;
        *angular_velocity_max_ = angular_velocity_max;
    }

    void reset_all_controls() {
        translational_velocity_max = comfort_translational_velocity;
        angular_velocity_max = comfort_angular_velocity;
        *boost_mode_ = 0.0;
        *friction_wheel_mode_ = 0.0;
    }

private:
    void update_status() {
        // auto bullet_fired = *bullet_fired_;
        auto mouse = *mouse_;
        last_translational_velocity_max = translational_velocity_max;
        last_angular_velocity_max = angular_velocity_max;
        if (mouse.left) {          // 发射弹丸
            translational_velocity_max = comfort_translational_velocity * 0.1;
            angular_velocity_max = comfort_angular_velocity * 2.0;
            *boost_mode_ = 1.0;

        } else if (mouse.right) { // 按下右键||  受击（没写）
            translational_velocity_max = comfort_translational_velocity * 1.2;
            angular_velocity_max = comfort_angular_velocity;
            *boost_mode_ = 1.0;

        } else if (rfid == 1.0) {                       // rfid识别 没找到
            translational_velocity_max = comfort_translational_velocity;
            angular_velocity_max = comfort_angular_velocity * 0.0;
            *boost_mode_ = 0.0;
        } else {                                        // 一般模式
            translational_velocity_max = comfort_translational_velocity;
            angular_velocity_max = comfort_angular_velocity * 20.0;
            *friction_wheel_mode_ = 1.0;
            *boost_mode_ = 0.0;
        }
        last_mouse_ = mouse;
        return;
    }
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<bool> bullet_fired_;
    InputInterface<double> supercap_voltage_;
    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();

    double comfort_translational_velocity = 10.0;
    double comfort_angular_velocity = 15.0;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

    OutputInterface<double> translational_velocity_max_;
    OutputInterface<double> angular_velocity_max_;
    OutputInterface<double> boost_mode_;
    OutputInterface<double> friction_wheel_mode_;
    double translational_velocity_max;
    double angular_velocity_max;
    double rfid = 0.0;
    double last_translational_velocity_max = 0.0;
    double last_angular_velocity_max = 0.0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::OperateLogic, rmcs_executor::Component)