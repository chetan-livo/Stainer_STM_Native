#pragma once
#include <stdint.h>
namespace SensorFaultPolicy {
inline bool adcValid(long value) { return value >= 0 && value <= 4095; }
inline bool fresh(uint32_t before, uint32_t after, bool valid) { return valid && before != after; }
inline bool accelValid(int16_t x, int16_t y, int16_t z) {
    // Full-scale clipping and all-zero bus samples cannot support a level result.
    return (x || y || z) && x != INT16_MIN && x != INT16_MAX &&
        y != INT16_MIN && y != INT16_MAX && z != INT16_MIN && z != INT16_MAX;
}
}
