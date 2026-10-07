#pragma once
#include <stdint.h>

// Display-only model. Never controls motion or dispensing. One display position
// is one bed revolution: at FEED 60 it takes one minute, with 15 positions total.
class SlidePositionTracker {
    struct Slide { uint32_t origin = 0; bool used = false; } slides_[64];
    uint32_t position_ = 0, lastSequence_ = 0;
    bool valid_ = false;
public:
    void reset(uint32_t position = 0) {
        for (auto& s : slides_) s.used = false;
        position_ = position; lastSequence_ = 0; valid_ = true;
    }
    void invalidate() { valid_ = false; }
    bool valid() const { return valid_; }
    void observe(uint32_t position, uint32_t pitch) {
        if (!valid_) return;
        if (!pitch || int32_t(position - position_) < 0) { invalidate(); return; }
        position_ = position;
        for (auto& s : slides_) if (s.used && uint32_t(position_ - s.origin) >= uint64_t(pitch)*15)
            s.used = false;
    }
    bool admit(uint32_t sequence, uint32_t origin, uint32_t pitch) {
        if (!valid_ || !sequence || !pitch) return false;
        if (lastSequence_ && int32_t(sequence-lastSequence_) <= 0) return true;
        if (int32_t(position_-origin) < 0) { invalidate(); return false; }
        lastSequence_ = sequence;
        if (uint32_t(position_-origin) >= uint64_t(pitch)*15) return true;
        for (auto& s : slides_) if (!s.used) { s.origin=origin; s.used=true; return true; }
        invalidate(); return false; // Never overwrite an earlier slide on overflow.
    }
    uint16_t snapshot(uint32_t pitch, uint16_t speed, uint8_t minutes[15]) const {
        for (unsigned i=0;i<15;++i) minutes[i]=0;
        if (!valid_ || !pitch || !speed) return 0;
        uint16_t mask=0x8000; // Validity bit distinguishes empty from unknown.
        for (const auto& s : slides_) if (s.used) {
            const uint32_t travelled=position_-s.origin;
            const unsigned slot=travelled/pitch;
            if (slot>=15) continue;
            const uint64_t numerator=(uint64_t(pitch)*15-travelled)*60;
            const uint64_t denominator=uint64_t(pitch)*speed;
            const uint64_t remaining=(numerator+denominator-1)/denominator;
            const uint8_t value=remaining>255 ? 255 : uint8_t(remaining);
            mask|=uint16_t(1U<<slot);
            // Multiple slides in a display bin retain independent origins;
            // show the longest remaining time until they separate or leave.
            if(value>minutes[slot]) minutes[slot]=value;
        }
        return mask;
    }
};
