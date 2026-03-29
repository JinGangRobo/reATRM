#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class PilotDiag : uint8_t { // numbers don't matter, just for readability
    BAD_RELOCATION = 0,
    WARNING = 1,
    FATAL = 2,
    READY = 3,
    STARTING = 4,
    OFFLINE = 5,
    SLAMING = 6,
};
enum class NavMode : uint8_t { SLAM = 0, RELOCATION = 1, UNKNOWN = 2 };

} // namespace rmcs_msgs