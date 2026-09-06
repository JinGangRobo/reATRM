// Shared helpers for the rmcs_core simulation transports (fake + MuJoCo).
//
// Contains the DJI feedback-frame encoder (the exact inverse of
// librmcs::device::DjiMotor::update_status decoding), so that every simulation
// backend streams byte-identical frames to the unmodified hardware models.

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string>

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

// ---- Simulated DBUS (remote controller) ---------------------
// Switch codes mirror librmcs::device::Dr16::Switch: UNKNOWN=0 UP=1 DOWN=2
// MIDDLE=3. Channels are centered at 1024 (0..2047), identical to the DR16.
// Runtime-editable fields are atomics so the ~60 Hz GUI thread (keyboard) and the
// 1 kHz physics thread (which encodes them into DBUS frames every tick) can share
// the remote state lock-free. Switches mirror Dr16: UP=1 DOWN=2 MIDDLE=3.
struct SimRemote {
    std::atomic<int> switch_right{3}; // MIDDLE (Dr16: UP=1 DOWN=2 MIDDLE=3)
    std::atomic<int> switch_left{3};  // MIDDLE
    std::atomic<int> channel0{1024};  // -> joystick right y (GUI: I/K)
    std::atomic<int> channel1{1024};  // -> joystick right x (GUI: J/L)
    std::atomic<int> channel2{1024};  // -> joystick left y  (GUI: W/S or arrows)
    std::atomic<int> channel3{1024};  // -> joystick left x  (GUI: A/D or arrows)
    int16_t mouse_x = 0;
    int16_t mouse_y = 0;
    int16_t mouse_z = 0;
    bool mouse_left = false;
    bool mouse_right = false;
    uint16_t keyboard = 0;            // rmcs_msgs::Keyboard bitfield
    std::atomic<int> rotary{1024};    // rotary knob (GUI: [ / ])
};

// Build an 18-byte DBUS frame whose layout is the exact inverse of
// librmcs::device::Dr16::store_status/update_status.
inline void make_dbus_frame(const SimRemote& r, std::byte* out) {
    constexpr auto relaxed = std::memory_order::relaxed;
    auto clamp_ch = [](int v) { return std::clamp(v, 0, 2047); };
    const uint64_t ch0 = static_cast<uint64_t>(clamp_ch(r.channel0.load(relaxed)));
    const uint64_t ch1 = static_cast<uint64_t>(clamp_ch(r.channel1.load(relaxed)));
    const uint64_t ch2 = static_cast<uint64_t>(clamp_ch(r.channel2.load(relaxed)));
    const uint64_t ch3 = static_cast<uint64_t>(clamp_ch(r.channel3.load(relaxed)));
    const uint64_t part1 = ch0 | (ch1 << 11) | (ch2 << 22) | (ch3 << 33)
                         | (static_cast<uint64_t>(r.switch_right.load(relaxed) & 0x3) << 44)
                         | (static_cast<uint64_t>(r.switch_left.load(relaxed) & 0x3) << 46);

    uint8_t b[18] = {0};
    for (int i = 0; i < 6; i++)
        b[i] = static_cast<uint8_t>((part1 >> (8 * i)) & 0xFF);

    // part2: mouse x/y/z int16 + left/right bytes (little-endian)
    const auto put16 = [&b](int offset, uint16_t v) {
        b[offset] = static_cast<uint8_t>(v & 0xFF);
        b[offset + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    put16(6, static_cast<uint16_t>(r.mouse_x));
    put16(8, static_cast<uint16_t>(r.mouse_y));
    put16(10, static_cast<uint16_t>(r.mouse_z));
    b[12] = r.mouse_left ? 1 : 0;
    b[13] = r.mouse_right ? 1 : 0;

    // part3: keyboard u16 + rotary knob u16
    put16(14, r.keyboard);
    put16(16, static_cast<uint16_t>(r.rotary.load(relaxed)));

    std::memcpy(out, b, 18);
}

// Runtime simulation configuration set by the bootstrap component and consumed
// by the transport factory / backends.
struct SimGlobalConfig {
    std::string backend = "fake"; // "fake" | "mujoco"
    std::string model_path;       // MJCF path (MuJoCo backend)
    SimRemote remote;             // injected remote-control (DBUS) state
    bool gui = false;             // open the native MuJoCo (GLFW) viewer
};

inline SimGlobalConfig& sim_global_config() {
    static SimGlobalConfig config;
    return config;
}

} // namespace rmcs_core::simulation
