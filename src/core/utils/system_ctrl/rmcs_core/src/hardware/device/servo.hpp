#pragma once

#include "librmcs/device/bmi088.hpp"
#include "rmcs_utility/tick_timer.hpp"

#include <cstdint>
#include <librmcs/device/servo.hpp>
#include <rclcpp/logger.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class Servo : public librmcs::device::Servo {
public:
    Servo(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::Servo() {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/max_angle", max_angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        command_component.register_input(name_prefix + "/control_angle", control_angle_, 0.0);
        servo_name_ = name_prefix;
        alive_watchdog_.reset(50);
    }

    Servo(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Servo_enable& servo_enable)
        : Servo(status_component, command_component, name_prefix) {
        configure(servo_enable);
    }

    void configure(const Servo_enable& servo_enable) {
        librmcs::device::Servo::configure(servo_enable);
        *max_palus_ = max_palus();
    }

    template <typename Func>
    void send_data(double target_relative_angle, Func&& pwm_write_func) {
        librmcs::device::Servo::send_data(target_relative_angle, pwm_write_func);
        *alive_ = true;
        alive_watchdog_.reset(50);
    }

    template <typename Func>
    void handle_switch_cmd(uint8_t can_rx_byte, Func&& pwm_write_func) {
        librmcs::device::Servo::handle_switch_cmd(can_rx_byte, pwm_write_func);
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> max_angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> max_palus_;
    rmcs_executor::Component::OutputInterface<bool> alive_;
    rmcs_executor::Component::InputInterface<double> control_angle_;
    rmcs_utility::TickTimer alive_watchdog_;
    std::string servo_name_;
};

} // namespace rmcs_core::hardware::device
