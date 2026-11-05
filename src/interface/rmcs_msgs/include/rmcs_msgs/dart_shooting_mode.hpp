#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class dart_shooting_mode : std::uint8_t {
    SHOOT = 0,
    RELOAD = 1,
    STEP_DOWN = 2,
    
};

} // namespace rmcs_msgs