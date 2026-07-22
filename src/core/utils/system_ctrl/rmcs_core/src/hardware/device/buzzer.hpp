#pragma once

#include <librmcs/device/buzzer.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class Buzzer : librmcs::device::Buzzer {
public:
    Buzzer(rmcs_executor::Component& command_component)
        : librmcs::device::Buzzer() {
        command_component.register_input("/buzzer/score", buzzer_score_input_, false);

        set_score(
            {device::Buzzer::Tone::LOW, device::Buzzer::Tone::MEDIUM, device::Buzzer::Tone::HIGH,
             0},
            device::Buzzer::Mode::ONCE);
    };

    void update_status() {
        if (buzzer_score_input_.ready()) {
            if (buzzer_score_input_->has_value())
                set_score(buzzer_score_input_->value(), Mode::CONTINUOUS);
            else
                set_score({Tone::OFF, Tone::OFF, Tone::OFF, 0}, Mode::CONTINUOUS);

            librmcs::device::Buzzer::update_status();
        }
    }

    uint8_t generate_command() { return librmcs::device::Buzzer::generate_command(); }

private:
    rmcs_executor::Component::InputInterface<std::optional<librmcs::device::Buzzer::BuzzerScore>>
        buzzer_score_input_;
};

} // namespace rmcs_core::hardware::device