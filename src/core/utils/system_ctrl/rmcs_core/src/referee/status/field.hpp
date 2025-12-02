#pragma once

#include <cstdint>

namespace rmcs_core::referee::status {

struct __attribute__((packed)) GameStatus {
    uint8_t game_type  : 4;
    uint8_t game_stage : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

struct __attribute__((packed)) GameRobotHp {
    uint16_t robot_1;
    uint16_t robot_2;
    uint16_t robot_3;
    uint16_t robot_4;
    uint16_t reserved;
    uint16_t robot_7;
    uint16_t robot_outpost;
    uint16_t robot_base;
};

struct __attribute__((packed)) RobotStatus {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
};

struct __attribute__((packed)) PowerHeatData {
    uint16_t reserved_0;
    uint16_t reserved_1;
    float reserved_2;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};

struct __attribute__((packed)) RobotPosition {
    float x;
    float y;
    float angle;
};

struct __attribute__((packed)) HurtData {
    uint8_t armor_id : 4;
    uint8_t reason   : 4;
};

struct __attribute__((packed)) ShootData {
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
};

struct __attribute__((packed)) BulletAllowance {
    uint16_t bullet_allowance_17mm;
    uint16_t bullet_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
};

struct __attribute__((packed)) GameRobotPosition {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float infantry_3_x;
    float infantry_3_y;
    float infantry_4_x;
    float infantry_4_y;
    float reserved_0;
    float reserved_1;
};

} // namespace rmcs_core::referee::status