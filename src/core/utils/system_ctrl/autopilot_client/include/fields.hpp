#pragma once

#include <cstdint>

namespace autopilot {
enum class PilotDiag : uint8_t { BAD_RELOCATION, WARNING, FATAL, READY, STARTING };
enum class NavMode : uint8_t { SLAM, RELOCATION, UNKNOWN };
struct StateData {
    double gimbal_faceing[3];
    bool autopilot_enabled;

    uint16_t current_hp;
    uint8_t game_state;
    uint16_t state_remain_time;
    uint8_t rfid_state;
    uint8_t center_area_state;
    int64_t projectile_allowance;

    bool auto_aim_tracking;
    double target_position[3];

    NavMode desired_nav_mode;
};
struct PilotData {
    bool pilot_valid;
    double chassis_vel[3]; // x, y, w
    PilotDiag pilot_state;
    NavMode current_nav_mode;
};
} // namespace autopilot