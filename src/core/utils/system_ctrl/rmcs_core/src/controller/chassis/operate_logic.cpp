#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Ref.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

#include "referee/status/field.hpp"

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
        register_input("/referee/chassis/current_hp", robots_hp_);
        register_input("/remote/mouse", mouse_);
        register_input("/referee/robots/rmul_rfid", robots_rfid_);

        register_output("/chassis/translational_vmax", translational_velocity_max_);
        register_output("/chassis/angular_vmax", angular_velocity_max_);
        register_output("/chassis/boost_mode", boost_mode_);
        register_output("/gimbal/friction_wheel", friction_wheel_mode_, 0.0);
        register_output("/robot/hurt", hurt_, 0.0);
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
        // 血量检测
        double current_hp = *robots_hp_;
        auto now = this->get_clock()->now();

        if (current_hp < last_robotHP_ && !timer_running_) {
            is_hurt_state_ = true;
            timer_running_ = true;
            hurt_start_time_ = now;
        }

        if (timer_running_) {
            auto elapsed = (now - hurt_start_time_).seconds();
            if (elapsed >= 2.0) {
                is_hurt_state_ = false;
                timer_running_ = false;
            }
        }
        last_robotHP_ = current_hp;
        double hurt_signal = is_hurt_state_ ? 1.0 : 0.0;
        

        // rfid检测
         
        // uint32_t bit_0 = robots_rfid_ -> rfid_status & 0x01;
        // uint32_t status = robots_rfid_->rfid_status;
        // bool bit_0 = (status & 0x01);

        // if (bit_0)
        //     rfid_status = 1.0;
        // else
        //     rfid_status = 0.0;
        // *hurt_ = rfid_status;
        // RCLCPP_INFO(this->get_logger(), "RAW RFID Status: 0x%08X", robots_rfid_ -> rfid_status);
        // RCLCPP_INFO(this->get_logger(), "RFID Status: %d", bit_0);
        

        auto mouse = *mouse_;
        last_translational_velocity_max = translational_velocity_max;
        last_angular_velocity_max = angular_velocity_max;
        if (mouse.left) {                               // 发射弹丸
            translational_velocity_max = comfort_translational_velocity * 0.1;
            angular_velocity_max = comfort_angular_velocity * 2.0;
            *boost_mode_ = 1.0;

        } else if (mouse.right || hurt_signal == 1.0) { // 按下右键||  受击
            translational_velocity_max = comfort_translational_velocity * 1.2;
            angular_velocity_max = comfort_angular_velocity;
            *boost_mode_ = 1.0;

        } else if (rfid_status == 1.0) {                // rfid识别 没找到
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
    InputInterface<double> robots_hp_;
    InputInterface<uint8_t> robots_rfid_;

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
    OutputInterface<double> robotId_;
    OutputInterface<double> hurt_;



    double translational_velocity_max;
    double angular_velocity_max;
    double rfid_status = 0.0;
    double last_translational_velocity_max = 0.0;
    double last_angular_velocity_max = 0.0;
    rclcpp::Time delay_start_time_;

    double last_robotHP_ = 0.0;                         // 记录上一帧的血量
    bool is_hurt_state_ = false;                        // 掉血状态位（输出给外部）
    bool timer_running_ = false;                        // 计时器是否在运行
    rclcpp::Time hurt_start_time_;                      // 记录掉血发生的瞬间时间
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::OperateLogic, rmcs_executor::Component)