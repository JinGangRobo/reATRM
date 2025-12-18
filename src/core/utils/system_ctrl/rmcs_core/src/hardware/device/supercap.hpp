#pragma once

#include <cmath>

#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Supercap {
public:
    explicit Supercap(Component& status_component) {
        status_component.register_output("/chassis/power", chassis_power_, 0.0);
        status_component.register_output("/chassis/supercap/voltage", supercap_voltage_, 0.0);
        status_component.register_output("/chassis/supercap/enabled", supercap_enabled_, false);
    }

    void store_status(uint64_t can_data) {
        can_data_.store(std::bit_cast<SupercapStatus>(can_data), std::memory_order::relaxed);
    }

    void update_status() {
        auto status = can_data_.load(std::memory_order::relaxed);

        *chassis_power_ = std::bit_cast<float>(status.chassis_pow);
        *supercap_voltage_ = ((status.voltage_B1 << 8) | status.voltage_B2) / 100.0; 
        *supercap_enabled_ = status.supcap_status;
    }

    double chassis_power() { return *chassis_power_; }
    double supercap_voltage() { return *supercap_voltage_; }
    double supercap_enabled() { return *supercap_enabled_; }

private:
    static constexpr double
        uint_to_double(std::unsigned_integral auto value, double min, double max) {
        double span = max - min;
        double offset = min;
        return (double)value / (double)decltype(value)(-1) * span + offset;
    }

    struct __attribute__((packed, aligned(8))) SupercapStatus {
        uint8_t voltage_B1;
        uint8_t voltage_B2;
        uint8_t reserved;
        uint32_t chassis_pow;
        uint8_t supcap_status;
    };
    std::atomic<SupercapStatus> can_data_{};
    static_assert(decltype(can_data_)::is_always_lock_free);

    Component::OutputInterface<double> chassis_power_;
    Component::OutputInterface<double> supercap_voltage_;
    Component::OutputInterface<bool> supercap_enabled_;
};

} // namespace rmcs_core::hardware::device