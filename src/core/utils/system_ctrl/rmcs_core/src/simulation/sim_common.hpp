// Shared helpers for the rmcs_core simulation transports (fake + MuJoCo).
//
// Contains the DJI feedback-frame encoder (the exact inverse of
// librmcs::device::DjiMotor::update_status decoding), so that every simulation
// backend streams byte-identical frames to the unmodified hardware models.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string>
#include <vector>

namespace rmcs_core::simulation {

constexpr double kPi = std::numbers::pi;
constexpr int kAngleMax = 8192; // DJI encoder counts per revolution

// Device-side configuration of one simulated DJI motor (mirror of
// librmcs::device::DjiMotor::Config), plus its CAN identity.
struct DjiSimSpec {
    std::string name;       // human/log name
    int can_id = 0;         // feedback CAN id (e.g. 0x205)

    double reduction = 1.0; // load-side reduction (motor turns per load turn)
    double sign = 1.0;      // -1 when reversed
    int zero = 0;           // encoder raw counts at load angle 0

    // N*m of (load-frame) torque produced per unit raw current == the
    // librmcs decode coefficient sign*reduction*torque_constant/raw_max*cmax.
    double torque_per_raw = 0.0;

    // MuJoCo actuator (== joint) name this motor drives (MuJoCo backend only).
    std::string actuator;
};

// DJI motor type constants -> (reduction, torque_per_raw at given reduction).
// `reduction` is the load-side ratio; returned torque_per_raw already includes
// sign and reduction. Values match librmcs::device::DjiMotor exactly.
struct DjiTypeConsts {
    double torque_constant;
    double raw_current_max;
    double current_max;
};

enum class DjiType { GM6020, M3508, M2006 };

inline DjiTypeConsts dji_type_consts(DjiType type) {
    switch (type) {
    case DjiType::GM6020: return {0.741, 16384.0, 3.0};
    case DjiType::M3508: return {0.3 * 187.0 / 3591.0, 16384.0, 20.0};
    case DjiType::M2006: return {0.18 * 1.0 / 36.0, 16384.0, 10.0};
    }
    return {0.0, 1.0, 1.0};
}

// Build the full DJI per-raw-current coefficient: sign*reduction*tc/raw*cmax.
inline double dji_torque_per_raw(DjiType type, double reduction, double sign = 1.0) {
    const auto c = dji_type_consts(type);
    return sign * reduction * c.torque_constant / c.raw_current_max * c.current_max;
}

inline int16_t dji_sign_extend_16(uint16_t v) { return static_cast<int16_t>(v); }

// Decode the raw (big-endian int16) current of one motor from a 0x200/0x1FF
// command broadcast frame (slot = can_id - base, 0..3).
inline int decode_command_current(
    uint64_t can_data, uint32_t can_id, uint32_t frame_base, uint8_t* bytes_out = nullptr) {
    uint8_t bytes[8];
    std::memcpy(bytes, &can_data, sizeof(bytes));
    const uint32_t slot = can_id - frame_base;
    uint16_t raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(bytes[2 * slot]) << 8) | static_cast<uint16_t>(bytes[2 * slot + 1]));
    if (bytes_out != nullptr) {
        bytes_out[0] = bytes[2 * slot];
        bytes_out[1] = bytes[2 * slot + 1];
    }
    return dji_sign_extend_16(raw);
}

// Encode one DJI feedback frame (same byte layout as
// librmcs::device::DjiMotor::DjiMotorFeedback) so the device decodes exactly the
// simulated load angle (device-frame, rad) and velocity (rad/s).
inline uint64_t
    encode_dji_feedback(double angle, double velocity, const DjiSimSpec& spec, int raw_current) {
    const double counts = angle * spec.reduction * kAngleMax / (2.0 * kPi) * spec.sign;
    double cmod = std::fmod(counts, kAngleMax);
    if (cmod < 0)
        cmod += kAngleMax;
    double raw_d = std::fmod(static_cast<double>(spec.zero) + cmod, kAngleMax);
    if (raw_d < 0)
        raw_d += kAngleMax;
    int angle_raw = static_cast<int>(std::lround(raw_d)) % kAngleMax;

    const double vel_raw_d = spec.sign * velocity * spec.reduction * 60.0 / (2.0 * kPi);
    const int16_t vel_raw = static_cast<int16_t>(std::clamp(vel_raw_d, -32767.0, 32767.0));

    uint8_t bytes[8];
    bytes[0] = static_cast<uint8_t>((angle_raw >> 8) & 0xFF);
    bytes[1] = static_cast<uint8_t>(angle_raw & 0xFF);
    bytes[2] = static_cast<uint8_t>((vel_raw >> 8) & 0xFF);
    bytes[3] = static_cast<uint8_t>(vel_raw & 0xFF);
    bytes[4] = static_cast<uint8_t>((raw_current >> 8) & 0xFF);
    bytes[5] = static_cast<uint8_t>(raw_current & 0xFF);
    bytes[6] = 30; // temperature ~30C
    bytes[7] = 0;  // unused

    uint64_t data = 0;
    std::memcpy(&data, bytes, sizeof(bytes));
    return data;
}

// Runtime simulation configuration set by the bootstrap component and consumed
// by the transport factory / backends.
struct SimGlobalConfig {
    std::string backend = "fake"; // "fake" | "mujoco"
    std::string model_path;       // MJCF path (MuJoCo backend)
};

inline SimGlobalConfig& sim_global_config() {
    static SimGlobalConfig config;
    return config;
}

} // namespace rmcs_core::simulation
