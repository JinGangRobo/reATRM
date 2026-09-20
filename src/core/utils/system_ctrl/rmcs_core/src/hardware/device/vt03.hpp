#pragma once

#include <eigen3/Eigen/Dense>
#include <librmcs/device/vt03.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/vtswitch.hpp>

namespace rmcs_core::hardware::device {

class Vt03 : public librmcs::device::Vt03 {
public:
    explicit Vt03(rmcs_executor::Component& component) {
        component.register_output(
            "/vt03/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        component.register_output(
            "/vt03/joystick/left", joystick_left_, Eigen::Vector2d::Zero());
        component.register_output(
            "/vt03/mode_switch", mode_switch_, rmcs_msgs::VtSwitch::UNKNOWN);
        component.register_output(
            "/vt03/switch/right", switch_right_, rmcs_msgs::VtSwitch::MIDDLE);
        component.register_output(
            "/vt03/switch/left", switch_left_, rmcs_msgs::VtSwitch::MIDDLE);
        component.register_output(
            "/vt03/mouse/velocity", mouse_velocity_, Eigen::Vector2d::Zero());
        component.register_output(
            "/vt03/mouse/mouse_wheel", mouse_wheel_);
        component.register_output(
            "/vt03/mouse", mouse_);
        std::memset(&*mouse_, 0, sizeof(*mouse_));
        component.register_output(
            "/vt03/keyboard", keyboard_);
        std::memset(&*keyboard_, 0, sizeof(*keyboard_));
        component.register_output(
            "/vt03/rotary_knob", rotary_knob_, 0.0);
        component.register_output(
            "/vt03/pause", pause_, false);
        component.register_output(
            "/vt03/fn_1", fn_1_);
        component.register_output(
            "/vt03/fn_2", fn_2_);
        component.register_output(
            "/vt03/trigger", trigger_, false);
        for (std::size_t i = 0; i < 6; ++i) {
            component.register_output(
                "/vt03/joint_expect/pos_" + std::to_string(i + 1),
                joint_expect_pos_[i], NAN);
        }
    }

    void update_status() {
        librmcs::device::Vt03::update_status();

        *joystick_right_ = joystick_right();
        *joystick_left_ = joystick_left();

        *mode_switch_ = mode_switch();

        *mouse_velocity_ = mouse_velocity();
        *mouse_wheel_ = mouse_wheel();

        *mouse_ = mouse();
        *keyboard_ = keyboard();

        *rotary_knob_ = rotary_knob();

        *pause_ = pause();
        *fn_1_ = fn_1();
        *fn_2_ = fn_2();
        *trigger_ = trigger();

        auto pos = joint_expect_pos();
        for (std::size_t i = 0; i < 6; ++i) {
            *joint_expect_pos_[i] = pos[i];
        }
    }

    Eigen::Vector2d joystick_right() const {
        return to_eigen_vector(librmcs::device::Vt03::joystick_right());
    }
    Eigen::Vector2d joystick_left() const {
        return to_eigen_vector(librmcs::device::Vt03::joystick_left());
    }

    rmcs_msgs::VtSwitch mode_switch() const {
        return std::bit_cast<rmcs_msgs::VtSwitch>(librmcs::device::Vt03::mode_switch());
    }

    Eigen::Vector2d mouse_velocity() const {
        return to_eigen_vector(librmcs::device::Vt03::mouse_velocity());
    }

    rmcs_msgs::Mouse mouse() const {
        return std::bit_cast<rmcs_msgs::Mouse>(librmcs::device::Vt03::mouse());
    }
    rmcs_msgs::Keyboard keyboard() const {
        return std::bit_cast<rmcs_msgs::Keyboard>(librmcs::device::Vt03::keyboard());
    }

private:
    static Eigen::Vector2d to_eigen_vector(librmcs::device::Vt03::VtVector vector) { return {vector.x, vector.y}; }

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_;
    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::VtSwitch> mode_switch_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::VtSwitch> switch_right_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::VtSwitch> switch_left_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_;
    rmcs_executor::Component::OutputInterface<double> mouse_wheel_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Mouse> mouse_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_executor::Component::OutputInterface<double> rotary_knob_;

    rmcs_executor::Component::OutputInterface<bool> pause_;
    rmcs_executor::Component::OutputInterface<bool> fn_1_;
    rmcs_executor::Component::OutputInterface<bool> fn_2_;
    rmcs_executor::Component::OutputInterface<bool> trigger_;

    rmcs_executor::Component::OutputInterface<double> joint_expect_pos_[6];
};

} // namespace rmcs_core::hardware::device