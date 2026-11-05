#pragma once

#include <cstddef>
#include <cstdint>

#include <librmcs/device/servo_protocol.hpp>

namespace rmcs_core::hardware::device {

// Hardware-layer adapter for the USB servo protocol. The USB field header is
// still added by CBoard::TransmitBuffer, just like the other device adapters.
class ServoProtocol {
public:
    using Protocol = librmcs::device::Servo;
    using Action = Protocol::Action;
    using Command = Protocol::Command;
    using Config = Protocol::Config;

    static constexpr std::size_t command_packet_size = Protocol::command_packet_size;
    static constexpr std::size_t set_angle_packet_size = Protocol::set_angle_packet_size;
    static constexpr std::size_t query_all_packet_size = Protocol::query_all_packet_size;
    static constexpr std::size_t query_servo_packet_size = Protocol::query_servo_packet_size;
    static constexpr std::size_t action_packet_size = Protocol::action_packet_size;
    static constexpr std::size_t response_packet_size = Protocol::response_packet_size;

    explicit ServoProtocol(std::uint8_t id)
        : protocol_{Config{id}} {}

    explicit ServoProtocol(const Config& config)
        : protocol_{config} {}

    std::size_t generate_command(
        double angle, std::uint8_t sequence, std::byte* output) const {
        return protocol_.generate_set_angle_command(angle, sequence, output);
    }

    std::size_t generate_set_angle_command(
        double angle, std::uint8_t sequence, std::byte* output) const {
        return protocol_.generate_set_angle_command(angle, sequence, output);
    }

    static std::size_t generate_query_command(
        std::uint8_t sequence, std::byte* output, std::uint8_t servo_id = 0) {
        return Protocol::generate_query_command(sequence, output, servo_id);
    }

    static std::size_t generate_query_all_command(std::uint8_t sequence, std::byte* output) {
        return Protocol::generate_query_all_command(sequence, output);
    }

    static std::size_t generate_action_command(
        Action action, std::uint8_t sequence, std::byte* output) {
        return Protocol::generate_action_command(action, sequence, output);
    }

    static bool validate_response(const std::byte* input, std::uint8_t length) {
        return Protocol::validate_response(input, length);
    }

    bool store_status(const std::byte* input, std::uint8_t length) {
        return protocol_.store_status(input, length);
    }

    bool response_matches(std::uint8_t sequence, Command command) const {
        return protocol_.response_matches(sequence, command);
    }

    std::uint8_t id() const { return protocol_.id(); }
    double angle() const { return protocol_.angle(); }
    std::uint8_t response_sequence() const { return protocol_.response_sequence(); }
    std::uint8_t response_command() const { return protocol_.response_command(); }
    std::uint8_t response_status() const { return protocol_.response_status(); }

private:
    Protocol protocol_;
};

} // namespace rmcs_core::hardware::device
