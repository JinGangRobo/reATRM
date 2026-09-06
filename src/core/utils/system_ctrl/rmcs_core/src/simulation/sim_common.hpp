// Shared helpers for the rmcs_core simulation transports (MuJoCo backend).
//
// Contains the DJI feedback-frame encoder (the exact inverse of
// librmcs::device::DjiMotor::update_status decoding), so the simulation backend
// streams byte-identical frames to the unmodified hardware models.

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string>
#include <vector>

namespace rmcs_core::simulation {

constexpr double kPi = std::numbers::pi;
constexpr int kAngleMax = 8192; // DJI encoder counts per revolution

// ---- Simulated motor kinds -------------------------------------------------
// Mirrors every motor type used by the unmodified hardware models:
//   * DJI current-broadcast motors (DjiMotor): GM6020 / GM6020_VOLTAGE / M3508 /
//     M2006. Commanded through the 0x200/0x1FF current broadcasts; feedback
//     can_ids 0x201..0x204 / 0x205..0x208.
//   * DM MIT direct-torque motors (DmMotor): J4310. Commanded through a per-motor
//     master id on CAN2; the 8-byte payload is an MIT torque command (N*m).
enum class MotorKind : uint8_t { GM6020, GM6020_VOLTAGE, M3508, M2006, J4310 };

inline bool motor_is_dm(MotorKind kind) { return kind == MotorKind::J4310; }

// (DJI) motor-frame constants; mirror librmcs::device::DjiMotor::configure.
struct MotorKindConsts {
    double torque_constant; // N*m per A (pre-reduction motor torque)
    double raw_current_max;
    double current_max;     // A
};

inline MotorKindConsts motor_kind_consts(MotorKind kind) {
    switch (kind) {
    case MotorKind::GM6020: return {0.741, 16384.0, 3.0};
    case MotorKind::GM6020_VOLTAGE: return {0.741, 25000.0, 3.0};
    case MotorKind::M3508: return {0.3 * 187.0 / 3591.0, 16384.0, 20.0};
    case MotorKind::M2006: return {0.18 * 1.0 / 36.0, 16384.0, 10.0};
    case MotorKind::J4310: return {0.0, 1.0, 1.0}; // DM: commanded in N*m directly
    }
    return {0.0, 1.0, 1.0};
}

// Load-frame N*m produced per unit raw current for a DJI motor (mirror of
// DjiMotor::raw_current_to_torque_coefficient_):
//   sign * reduction * torque_constant / raw_current_max * current_max
inline double dji_torque_per_raw(MotorKind kind, double reduction, double sign = 1.0) {
    const auto c = motor_kind_consts(kind);
    return sign * reduction * c.torque_constant / c.raw_current_max * c.current_max;
}

// One simulated motor: data-driven binding that mirrors the DjiMotor/DmMotor
// Config + CAN identity declared in the (unmodified) hardware-model .cpp. Every
// field comes from the robot's sim config, so adding a robot never touches the
// simulation backend code.
struct SimMotorSpec {
    MotorKind kind = MotorKind::GM6020;
    int can_bus = 1;        // 1 = CAN1, 2 = CAN2
    int can_id = 0;         // feedback CAN id (e.g. 0x206)
    double reduction = 1.0; // load-side reduction (DJI; DM drives the load directly)
    double sign = 1.0;      // -1 when reversed
    int zero = 0;           // encoder raw counts at load angle 0
    int command_id = 0;     // DJI: 0 => derive 0x200/0x1FF group from can_id;
                            // DM:  MIT master command id (e.g. 0x9)
    std::string actuator;   // MuJoCo actuator (== joint) name this motor drives

    // Derived (DJI): N*m of load-frame torque per unit raw current.
    double torque_per_raw = 0.0;

    // DJI feedback can_id -> first feedback id of its command group, i.e.
    // 0x201 (driven by the 0x200 broadcast) or 0x205 (driven by 0x1FF); 0 if
    // this is not a DJI feedback id. The byte slot inside the command frame is
    // can_id - this base (0..3), matching decode_command_current.
    int dji_feedback_base() const {
        if (can_id >= 0x201 && can_id <= 0x204)
            return 0x201;
        if (can_id >= 0x205 && can_id <= 0x208)
            return 0x205;
        return 0;
    }
    // Command broadcast frame id that drives this motor (0x200 / 0x1FF / 0).
    int dji_frame_id() const {
        const int b = dji_feedback_base();
        if (b == 0)
            return 0;
        return b == 0x201 ? 0x200 : 0x1FF;
    }
};

// One CBoard of the simulated robot. `pid` is the board key from the model
// config (usb_pid_top/bottom_board / etc.), matching the usb_pid handed to
// CBoard::set_transport_factory by the transport factory.
struct SimBoardConfig {
    int pid = 0;             // board key (== usb_pid_* value in the model config)
    bool dbus = false;       // board hosts the DR16 -> simulated remote injected
    std::string gyro_sensor; // MJCF <sensor> feeding this board's gyro ("" = none)
    std::string acc_sensor;  // MJCF <sensor> feeding this board's accelerometer
    std::vector<SimMotorSpec> motors;
};

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
    encode_dji_feedback(double angle, double velocity, const SimMotorSpec& spec, int raw_current) {
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

// ---- DM (MIT, J4310) helper encoders/decoders -----------------------------
// DM motors are direct-torque: the downlink MIT frame carries a signed torque
// (N*m, motor frame); the feedback 8 bytes mirror DmMotor::DmMotorFeedback.
// DmMotor applies no reduction anywhere, so `reduction` stays 1 and the load
// torque equals the MIT torque with `sign` folding the reversed mounting.

// Motor-frame torque (N*m) packed in an MIT command frame (inverse of
// DmMotor::to_dm_mit_control_command, torque field).
inline double dm_decode_command_torque(uint64_t can_data) {
    uint8_t bytes[8];
    std::memcpy(bytes, &can_data, sizeof(bytes));
    const uint16_t raw = (static_cast<uint16_t>(bytes[6] & 0x0F) << 8) | bytes[7];
    return static_cast<double>(raw) / 4095.0 * 20.0 - 10.0; // [T_MIN,T_MAX]=[-10,10]
}

// Encode one DM feedback frame so DmMotor::update_status reports exactly the
// simulated load angle (rad) / velocity (rad/s); `sign` mirrors reversed mount.
inline uint64_t
    encode_dm_feedback(double angle, double velocity, const SimMotorSpec& spec, double torque) {
    // 8192 counts per revolution (matches DmMotor's 0x1FFF mask). The 16-bit raw
    // value advances continuously so both single- and multi-turn decodes unwrap
    // to the true angle.
    const double counts = angle * 8192.0 / (2.0 * kPi);
    const int64_t c = static_cast<int64_t>(std::lround(counts));
    const int64_t cal_raw = (spec.sign < 0.0) ? -c : c;
    int64_t r = static_cast<int64_t>(spec.zero) + cal_raw;
    r %= 65536;
    if (r < 0)
        r += 65536;
    const int pos = static_cast<int>(r);

    // Velocity: DmMotor decodes ((raw - 2048)/4096)*60 rad/s (no sign/zero).
    const int vel_raw = static_cast<int>(
        std::clamp(static_cast<long>(std::lround(velocity / 60.0 * 4096.0 + 2048.0)), 0L, 4095L));
    // Torque readout: DmMotor decodes ((raw - 2048)/4096)*20 N*m.
    const int torque_raw = static_cast<int>(
        std::clamp(static_cast<long>(std::lround(torque / 20.0 * 4096.0 + 2048.0)), 0L, 4095L));

    uint8_t bytes[8];
    bytes[0] = 0;  // id | error<<4 (no error)
    bytes[1] = static_cast<uint8_t>((pos >> 8) & 0xFF);
    bytes[2] = static_cast<uint8_t>(pos & 0xFF);
    bytes[3] = static_cast<uint8_t>((vel_raw >> 4) & 0xFF);
    bytes[4] = static_cast<uint8_t>(
        ((vel_raw & 0x0F) << 4) | static_cast<uint8_t>((torque_raw >> 8) & 0x0F));
    bytes[5] = static_cast<uint8_t>(torque_raw & 0xFF);
    bytes[6] = 30; // T_MOS ~30C
    bytes[7] = 30; // T_Rotor ~30C

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
// by the transport factory / MuJoCo backend.
struct SimGlobalConfig {
    std::string model_path;             // MJCF path (MuJoCo backend)
    SimRemote remote;                   // injected remote-control (DBUS) state
    bool gui = false;                   // open the native MuJoCo (GLFW) viewer
    std::vector<SimBoardConfig> boards; // config-driven board topology

    // Kinematic ground parameters (engine-level omni drive). Keep these equal
    // to the omni_wheel_controller chassis geometry in the robot sim config so
    // the engine's forward kinematics matches what the real chassis controller
    // assumes.
    double ground_wheel_radius = 0.07;
    double ground_radius_x = 0.165;
    double ground_radius_y = 0.165;
};

inline SimGlobalConfig& sim_global_config() {
    static SimGlobalConfig config;
    return config;
}

} // namespace rmcs_core::simulation
