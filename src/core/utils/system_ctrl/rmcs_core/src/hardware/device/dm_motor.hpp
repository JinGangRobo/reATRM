#pragma once

#include <chrono>

#include "rmcs_utility/tick_timer.hpp"
#include "utility/low_pass_filter.hpp"
#include <librmcs/device/dm_motor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <string>

namespace rmcs_core::hardware::device {

class DmMotor : public librmcs::device::DmMotor {
public:
    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::DmMotor() {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle", raw_angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);
        status_component.register_output(name_prefix + "/alive", alive_, false);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
        command_component.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);
        status_component.register_output(
            name_prefix + "/velocity_filtered", velocity_filtered_, 0.0);

        motor_name_ = name_prefix;
        alive_watchdog_.reset(50);
    }

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DmMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    void configure(const Config& config) {
        librmcs::device::DmMotor::configure(config);

        *max_torque_ = max_torque();
    }

    void update_status() {
        librmcs::device::DmMotor::update_status();

        if (alive_watchdog_.tick()) {
            *alive_ = false;
            RCLCPP_WARN(rclcpp::get_logger("HW_Diag"), "Dm Motor %s offline!", motor_name_.c_str());
        }

        *angle_ = angle();
        *raw_angle_ = last_raw_angle();
        *velocity_ = velocity();
        *torque_ = torque();
        *velocity_filtered_ = velocity_lpf_.update(velocity());
    }

    void store_status(uint64_t can_data) {
        librmcs::device::DmMotor::store_status(can_data);

        *alive_ = true;
        alive_watchdog_.reset(50);
    }

    double control_velocity() const {
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return 0.0;
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    using librmcs::device::DmMotor::generate_torque_command;
    uint64_t generate_torque_command() { return generate_torque_command(control_torque()); }

    using librmcs::device::DmMotor::generate_velocity_command;
    uint64_t generate_velocity_command() { return generate_velocity_command(control_velocity()); }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> raw_angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> velocity_filtered_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;
    rmcs_executor::Component::OutputInterface<bool> alive_;

    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_torque_;

    std::string motor_name_;
    rmcs_utility::TickTimer alive_watchdog_;
    rmcs_core::utility::LowPassFilter<> velocity_lpf_{4, 1000};
};

} // namespace rmcs_core::hardware::device