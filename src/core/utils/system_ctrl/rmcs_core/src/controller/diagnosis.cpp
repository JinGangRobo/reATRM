#include "librmcs/device/buzzer.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/autopilot.hpp>
#include <sys/types.h>
#include <vector>

namespace rmcs_core::controller::diagnosis {
using Buzzer = librmcs::device::Buzzer;
using Tone = librmcs::device::Buzzer::Tone;

enum class DiagnosisMsg : uint8_t { // larger value means higher priority
    ALL_READY,
    CTR_MOTOR_OFFLINE,
    AIM_READY,
    CTR_SUPERCAP_OFFLINE,
    NAV_WARNING,
    NAV_RELOCATION_FAILED,
    NAV_FAILURE,
    NAV_OFFLINE
};
constexpr std::array<std::pair<DiagnosisMsg, Buzzer::BuzzerScore>, 7> DiagnosisScores = {
    {{DiagnosisMsg::CTR_MOTOR_OFFLINE, {Tone::LOW, Tone::OFF, Tone::OFF, 0}},
     {DiagnosisMsg::AIM_READY, {Tone::MEDIUM, Tone::MEDIUM, Tone::MEDIUM, 0}},
     {DiagnosisMsg::CTR_SUPERCAP_OFFLINE, {Tone::LOW, Tone::LOW, Tone::OFF, 0}},
     {DiagnosisMsg::NAV_WARNING, {Tone::HIGH, Tone::LOW, Tone::OFF, 0}},
     {DiagnosisMsg::NAV_RELOCATION_FAILED, {Tone::HIGH, Tone::LOW, Tone::LOW, 0}},
     {DiagnosisMsg::NAV_FAILURE, {Tone::HIGH, Tone::MEDIUM, Tone::OFF, 0}},
     {DiagnosisMsg::NAV_OFFLINE, {Tone::HIGH, Tone::OFF, Tone::OFF, 0}}}
};
constexpr Buzzer::BuzzerScore getScoreFromDiagMsg(DiagnosisMsg msg) {
    auto it = std::find_if(DiagnosisScores.begin(), DiagnosisScores.end(), [msg](const auto& pair) {
        return pair.first == msg;
    });

    if (it != DiagnosisScores.end()) {
        return it->second;
    }
    return {Tone::OFF, Tone::OFF, Tone::OFF, 0};
}

class Diagnosis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Diagnosis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        get_parameter("motors", motors_interface_name_);
        get_parameter("enable_motor_check", enable_motor_check_);
        get_parameter("enable_supercap_check", enable_supercap_check_);
        get_parameter("enable_aim_check", enable_aim_check_);
        get_parameter("enable_nav_check", enable_nav_check_);

        for (const auto& motor : motors_interface_name_) {
            auto motor_alive_input = std::make_unique<InputInterface<bool>>();
            register_input(motor + "/alive", *motor_alive_input);
            motor_alive_inputs_.push_back(std::move(motor_alive_input));
        }
        register_input("/chassis/supercap/enabled", supercap_alive_input_);
        register_input("/gimbal/auto_aim/available", aim_ready_input_);
        register_input("/autopilot/diagnosis", pilot_diag_input_);

        register_output("/buzzer/score", buzzer_score_output_);
    }

    void update() override {
        errors_.clear();

        // Check motor status
        if (enable_motor_check_)
            for (auto& motor_alive_input : motor_alive_inputs_) {
                if (!motor_alive_input->ready() || !**motor_alive_input) {
                    errors_.push_back(DiagnosisMsg::CTR_MOTOR_OFFLINE);
                }
            }

        // Check supercap status
        if (enable_supercap_check_)
            if (!supercap_alive_input_.ready() || !*supercap_alive_input_) {
                errors_.push_back(DiagnosisMsg::CTR_SUPERCAP_OFFLINE);
            }

        // Check aim status
        if (enable_aim_check_)
            if (aim_ready_input_.ready() && *aim_ready_input_) {
                errors_.push_back(DiagnosisMsg::AIM_READY);
            }

        // Check autopilot diagnosis
        if (enable_nav_check_)
            if (pilot_diag_input_.ready()) {
                rmcs_msgs::PilotDiag diag = *pilot_diag_input_;

                switch (diag) {
                case rmcs_msgs::PilotDiag::WARNING:
                    errors_.push_back(DiagnosisMsg::NAV_WARNING);
                    return;
                case rmcs_msgs::PilotDiag::BAD_RELOCATION:
                    *buzzer_score_output_ =
                        getScoreFromDiagMsg(DiagnosisMsg::NAV_RELOCATION_FAILED);
                    return;
                case rmcs_msgs::PilotDiag::FATAL:
                    errors_.push_back(DiagnosisMsg::NAV_FAILURE);
                    return;
                case rmcs_msgs::PilotDiag::OFFLINE:
                    errors_.push_back(DiagnosisMsg::NAV_OFFLINE);
                    return;
                case rmcs_msgs::PilotDiag::STARTING:
                    errors_.push_back(DiagnosisMsg::NAV_OFFLINE);
                    return;
                default: break;
                }
            }

        if (!errors_.empty()) {
            *buzzer_score_output_ = getScoreFromDiagMsg(
                std::max_element(
                    errors_.begin(), errors_.end(), [](DiagnosisMsg a, DiagnosisMsg b) {
                        return static_cast<uint8_t>(a) < static_cast<uint8_t>(b);
                    })[0]);
        } else
            *buzzer_score_output_ = std::nullopt;
    }

private:
    InputInterface<bool> enable_;
    OutputInterface<double> control_;

    std::vector<std::string> motors_interface_name_ = {};
    bool enable_motor_check_ = false;
    bool enable_supercap_check_ = false;
    bool enable_aim_check_ = false;
    bool enable_nav_check_ = false;

    std::vector<std::unique_ptr<InputInterface<bool>>> motor_alive_inputs_;
    InputInterface<bool> supercap_alive_input_;
    InputInterface<bool> aim_ready_input_;
    InputInterface<rmcs_msgs::PilotDiag> pilot_diag_input_;

    std::vector<DiagnosisMsg> errors_ = {};

    OutputInterface<std::optional<Buzzer::BuzzerScore>> buzzer_score_output_;
};

} // namespace rmcs_core::controller::diagnosis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::diagnosis::Diagnosis, rmcs_executor::Component)
