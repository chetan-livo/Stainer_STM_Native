#pragma once
#include <stdint.h>
#include "SlideDetectionFilter.h"

// Independent IR4 slide deadlines. No heap allocation and rollover-safe clocks.
class NozzleEthanolSchedule {
public:
    static const uint8_t Capacity = 16;
    struct Event {
        uint32_t detected, pumpAt, fanOn, fanOff, sequence;
        bool used, pumpSent, fanLogged;
    };
    Event events[Capacity] = {};
    bool overflow = false;
    bool isArmed() const { return filter.isArmed(); }
    void resetDetection() { filter.reset(); }

    static bool due(uint32_t now, uint32_t at) {
        return (int32_t)(now - at) >= 0;
    }
    void reset() { *this = NozzleEthanolSchedule(); }
    uint8_t depth() const {
        uint8_t count = 0;
        for (const auto& e : events) if (e.used) ++count;
        return count;
    }
    // Accept every confirmed clear-to-present transition, even while earlier
    // slides still have pending pump or fan deadlines.
    bool sample(uint32_t now, int raw, int present, int clear,
                uint32_t presentMs, uint32_t clearMs,
                uint32_t pumpDelay, uint32_t fanDelay, uint32_t fanRun) {
        if (!filter.sample(now, raw, present, clear, presentMs, clearMs)) return false;
        const uint32_t since = filter.detected;
        for (auto& e : events) if (!e.used) {
            e = {since, since + pumpDelay, since + fanDelay,
                 since + fanDelay + fanRun, ++sequence, true, false, false};
            return true;
        }
        overflow = true; // Preserve all previously accepted deadlines.
        return false;
    }
    template<class Pump, class FanStart>
    bool service(uint32_t now, Pump pump, FanStart fanStart) {
        bool fanRequired = false;
        for (auto& e : events) if (e.used) {
            if (!e.pumpSent && due(now, e.pumpAt)) {
                pump(e); e.pumpSent = true;
            }
            if (due(now, e.fanOn) && !due(now, e.fanOff)) {
                fanRequired = true;
                if (!e.fanLogged) { fanStart(e); e.fanLogged = true; }
            }
            if (e.pumpSent && due(now, e.fanOff)) e.used = false;
        }
        // Overlapping fan windows form a union; one slide cannot stop another's fan.
        return fanRequired;
    }
private:
    SlideDetectionFilter filter;
    uint32_t sequence = 0;
};
