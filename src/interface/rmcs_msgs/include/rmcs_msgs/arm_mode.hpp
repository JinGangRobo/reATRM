#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ArmMode : uint8_t {
    execute_dr16_position,
    execute_dr16_orientation,
    Custome,
    None
};

} // namespace rmcs_msgs