#include <cmath>
#include <cstdint>
#include <numbers>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/operate_mode.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "referee/app/ui/shape/shape.hpp"
#include "referee/app/ui/widget/status_bar.hpp"
#include "referee/app/ui/widget/status_ring.hpp"
#include "referee/status/field.hpp"

namespace rmcs_core::referee::app::ui {
using namespace std::chrono_literals;

class MiniInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MiniInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , status_ring_(100, 220, 600, 30)
        , status_bar_(100, 99)
        , time_reminder_(Shape::Color::PINK, 50, 5, x_center + 150, y_center + 65, 0, false) {

        for (int i = 0; i < 4; ++i) {
            chassis_indicator_[i].set_x(x_center);
            chassis_indicator_[i].set_y(y_center);
            chassis_indicator_[i].set_r(84);
            chassis_indicator_[i].set_width(8);
            chassis_indicator_[i].set_color(Shape::Color::WHITE);
        }
        hurt_indicator_.set_x(x_center);
        hurt_indicator_.set_y(y_center);
        hurt_indicator_.set_r(120);
        hurt_indicator_.set_width(9);
        hurt_indicator_.set_color(Shape::Color::PINK);
        hurt_indicator_.set_visible(false);

        hurt_indicator_clean_timer_.reset(300);

        register_input("/tf", rmcs_tf);

        register_input("/chassis/supercap/enabled", supercap_enabled_);
        register_input("/chassis/supercap/energy_percentage", supercap_energy_percentage_);
        register_input("/chassis/power", chassis_power_);

        register_input("/gimbal/left_friction/control_velocity", first_friction_control_velocity_);
        register_input("/gimbal/left_friction/velocity", first_friction_velocity_);
        register_input("/gimbal/auto_aim/available", auto_aim_ready_);

        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);
        register_input("/referee/shooter/initial_speed", robot_initial_speed_);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_);
        register_input("/referee/chassis/power_limit", referee_chassis_power_limit_);

        register_input("/chassis/control_mode", chassis_mode_);
        register_input("/chassis/angle", chassis_angle_);
        register_input("/chassis/control_angle", chassis_control_angle_);
        register_input("/chassis/operate_mode", operate_mode_, false);

        register_input("/chassis/control_power_limit", chassis_control_power_limit_);

        register_input("/gimbal/shooter/mode", shoot_mode_);
        register_input("/gimbal/shooter/heat", shooter_heat_);

        register_input("/remote/mouse", mouse_);

        register_input("/referee/game/stage", game_stage_);
        register_input("/referee/robot/hurt", hurt_data_);
        register_input("/referee/game/center_area", center_area_status_);
    }

    void update() override {
        update_normal_ui();

        set_normal_ui_visible(true);
    }

private:
    void set_normal_ui_visible(bool value) {
        status_ring_.set_visible(value);
        status_bar_.set_visible(value);

        hurt_indicator_visible_ = value;
        for (int i = 0; i < 4; ++i) {
            chassis_indicator_[i].set_visible(value);
        }
    }

    void update_normal_ui() {
        update_chassis_direction_indicator();

        status_ring_.set_power_safe(*referee_chassis_power_limit_);
        status_ring_.update_bullet_speed(*robot_initial_speed_);
        status_ring_.update_friction_wheel_speed(
            *first_friction_velocity_,
            *first_friction_control_velocity_ > 0 && *robot_bullet_allowance_ > 0);
        status_ring_.update_supercap(
            *supercap_energy_percentage_, supercap_enabled_.ready() && *supercap_enabled_);
        status_ring_.update_power(*chassis_power_, supercap_enabled_.ready() && *supercap_enabled_);
        update_static_status_ring();

        status_bar_.set_cool_limit(*shooter_heat_limit_);
        status_bar_.update_power_part(
            *chassis_power_, *chassis_control_power_limit_,
            operate_mode_.ready() ? *operate_mode_ == rmcs_msgs::OperateMode::ASSIST : false);
        status_bar_.update_dynamic_part(
            *shooter_heat_limit_ - *shooter_heat_, *robot_bullet_allowance_, *center_area_status_);
    }

    void update_time_reminder() {
        if (!game_stage_.ready())
            return;
    }

    void update_static_status_ring() {
        auto auto_aim_enable = mouse_->right == 1;
        auto precise_enable = *shoot_mode_ == rmcs_msgs::ShootMode::PRECISE;
        auto auto_aim_ready = auto_aim_ready_.ready() ? *auto_aim_ready_ : false;

        status_ring_.update_static_parts({auto_aim_enable, precise_enable, auto_aim_ready});
    }

    void update_chassis_direction_indicator() {
        auto chassis_mode = *chassis_mode_;

        if (!(chassis_mode_.ready() && chassis_angle_.ready())) {
            return;
        }

        auto to_referee_angle = [](double angle) {
            return static_cast<int>(
                std::round((2 * std::numbers::pi - angle) / std::numbers::pi * 180));
        };
        auto unify_angle = [](int angle) constexpr noexcept {
            angle %= 360;
            return (angle < 0) ? (angle + 360) : angle;
        };

        for (int i = 0; i < 4; ++i) {
            switch (chassis_mode) {
            case rmcs_msgs::ChassisMode::SPIN:
                chassis_indicator_[i].set_color(Shape::Color::GREEN);
                break;
            case rmcs_msgs::ChassisMode::STEP_DOWN:
                chassis_indicator_[i].set_color(i == 0 ? Shape::Color::YELLOW : Shape::Color::CYAN);
                break;
            case rmcs_msgs::ChassisMode::LAUNCH_RAMP:
                chassis_indicator_[i].set_color(i == 0 ? Shape::Color::YELLOW : Shape::Color::CYAN);
                break;
            default:
                chassis_indicator_[i].set_color(
                    i == 0 ? Shape::Color::YELLOW : Shape::Color::WHITE);
                break;
            }

            chassis_indicator_[i].set_angle(
                unify_angle(to_referee_angle(*chassis_angle_) + i * 90), 20);
        }
        if (hurt_data_.ready()) {
            if (hurt_data_->reason == 0) {
                hurt_indicator_clean_timer_.reset(3'500);
                uint8_t armor = armor_index[hurt_data_->armor_id];

                Eigen::Quaterniond gimbal_center_to_odom_imu_transform = fast_tf::lookup_transform<
                    rmcs_description::GimbalCenterLink, rmcs_description::OdomImu>(*rmcs_tf);

                Eigen::Vector3d hurt_direction_in_gimbal_center =
                    Eigen::AngleAxisd(-armor * std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ())
                    * Eigen::Vector3d::UnitX();

                hurt_direction_in_odom_imu_ =
                    gimbal_center_to_odom_imu_transform * hurt_direction_in_gimbal_center;

                if (hurt_indicator_visible_)
                    hurt_indicator_.set_visible(true);
            }
        }
        if (hurt_indicator_.visible()) {
            Eigen::Quaterniond odom_imu_to_yaw_transform =
                fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::YawLink>(
                    *rmcs_tf);

            Eigen::Vector3d hurt_direction_in_yaw =
                odom_imu_to_yaw_transform * hurt_direction_in_odom_imu_;

            double hurt_direction_angle_in_yaw =
                std::atan2(hurt_direction_in_yaw.y(), hurt_direction_in_yaw.x());

            hurt_indicator_.set_angle(
                unify_angle(hurt_direction_angle_in_yaw * 180 / std::numbers::pi), 20);
        }
        if (hurt_indicator_clean_timer_.tick()) {
            hurt_indicator_.set_visible(false);
        }
    }

    static constexpr uint16_t screen_width = 1920, screen_height = 1080;
    static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;

    static constexpr uint16_t height_min = 0, height_max = 500;

    static constexpr uint8_t armor_index[4] = {0, 1, 2, 3};

    InputInterface<rmcs_description::Tf> rmcs_tf;

    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<double> chassis_angle_, chassis_control_angle_;
    InputInterface<rmcs_msgs::OperateMode> operate_mode_;

    InputInterface<bool> supercap_enabled_;
    InputInterface<double> supercap_energy_percentage_;
    InputInterface<double> chassis_power_;

    InputInterface<double> referee_chassis_power_limit_;
    InputInterface<double> chassis_control_power_limit_;

    InputInterface<uint16_t> robot_bullet_allowance_;
    InputInterface<int64_t> shooter_heat_;
    InputInterface<int64_t> shooter_heat_limit_;

    InputInterface<double> first_friction_control_velocity_;
    InputInterface<double> first_friction_velocity_;
    InputInterface<double> robot_initial_speed_;
    InputInterface<bool> auto_aim_ready_;

    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<status::HurtData> hurt_data_;
    InputInterface<uint8_t> center_area_status_;

    InputInterface<rmcs_msgs::ShootMode> shoot_mode_;

    StatusRing status_ring_;
    StatusBar status_bar_;

    Arc chassis_indicator_[4];

    Arc hurt_indicator_;
    bool hurt_indicator_visible_ = false;
    Eigen::Vector3d hurt_direction_in_odom_imu_;
    rmcs_utility::TickTimer hurt_indicator_clean_timer_;

    Integer time_reminder_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::MiniInfantry, rmcs_executor::Component)