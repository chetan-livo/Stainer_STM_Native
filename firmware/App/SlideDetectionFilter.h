#pragma once
#include <stdint.h>

// Shared by IR4, IR6 and IR12. Require both a clear gap and a stable leading edge.
class SlideDetectionFilter {
public:
    uint32_t detected = 0;
    bool isArmed() const { return armed; }
    void reset() { *this = SlideDetectionFilter(); }
    bool sample(uint32_t now, int raw, int present, int clear,
                uint32_t presentMs, uint32_t clearMs) {
        const uint8_t level = raw < 0 || raw > 4095 ? 0 :
                              raw < present ? 1 : raw > clear ? 2 : 0;
        if (!level) { candidate = 0; samples = 0; return false; }
        if (level != candidate) {
            candidate = level; since = now; samples = 1; return false;
        }
        if (samples < 3) ++samples;
        if (samples < 3 || (uint32_t)(now - since) < (level == 1 ? presentMs : clearMs)) return false;
        if (level == 2) { armed = true; return false; }
        if (!armed) return false;
        armed = false;
        detected = since;
        return true;
    }
private:
    bool armed = false;
    uint8_t candidate = 0, samples = 0;
    uint32_t since = 0;
};
