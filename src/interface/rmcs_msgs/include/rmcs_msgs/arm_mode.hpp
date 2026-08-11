#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ArmMode : uint8_t {
    execute_vt03_position,
    execute_vt03_orientation,
    Custome,
    Gripper,
    None
};

} // namespace rmcs_msgs