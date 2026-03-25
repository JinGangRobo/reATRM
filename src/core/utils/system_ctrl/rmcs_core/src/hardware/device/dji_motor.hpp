#pragma once

#include "rmcs_utility/tick_timer.hpp"
#include "utility/low_pass_filter.hpp"

#include <cstdint>
#include <librmcs/device/dji_motor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class DjiMotor : public librmcs::device::DjiMotor {
public:
    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::DjiMotor() {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle", raw_angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);
        status_component.register_output(
            name_prefix + "/velocity_filtered", velocity_filtered_, 0.0);
        status_component.register_output(name_prefix + "/alive", alive_, false);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);

        motor_name_ = name_prefix;
        alive_watchdog_.reset(50);
        throttle_clock_.reset(3000);
    }

    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DjiMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    void configure(const Config& config) {
        librmcs::device::DjiMotor::configure(config);

        *max_torque_ = max_torque();
    }

    void update_status() {
        librmcs::device::DjiMotor::update_status();

        if (alive_watchdog_.tick())
            *alive_ = false;

        if (!*alive_) [[unlikely]]
            if (throttle_clock_.tick()) {
                throttle_clock_.reset(3000);
                RCLCPP_WARN(
                    rclcpp::get_logger("HW_Diag"), "Dji Motor %s offline!", motor_name_.c_str());
            }

        *angle_ = angle();
        *raw_angle_ = last_raw_angle();
        *velocity_ = velocity();
        *torque_ = torque();
        *velocity_filtered_ = velocity_lpf_.update(velocity());
    }

    void store_status(uint64_t can_data) {
        librmcs::device::DjiMotor::store_status(can_data);

        *alive_ = true;
        alive_watchdog_.reset(50);
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    uint16_t generate_command() {
        return librmcs::device::DjiMotor::generate_command(control_torque());
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> raw_angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> velocity_filtered_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;
    rmcs_executor::Component::OutputInterface<bool> alive_;

    rmcs_executor::Component::InputInterface<double> control_torque_;

    std::string motor_name_;
    rmcs_utility::TickTimer alive_watchdog_;
    rmcs_utility::TickTimer throttle_clock_;
    rmcs_core::utility::LowPassFilter<> velocity_lpf_{4, 1000};
};

} // namespace rmcs_core::hardware::device