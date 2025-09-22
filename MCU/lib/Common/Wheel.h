#pragma once
#include <stdint.h>

// Single source of truth for wheel identifiers used across modules.
enum class Wheel : uint8_t
{
    FL = 0,
    RL = 1,
    FR = 2,
    RR = 3
};
