#pragma once
#include <stdint.h>
#include "MotionPlanner.h"

// Main-loop stepping, replacing AccelStepper::run(): at most one STEP pulse
// per run() call, issued once the planned interval has elapsed since the
// previous step (timing measured from the actual step, as AccelStepper did,
// so a late call delays the profile rather than bunching steps).
//
// Used where the caller depends on one step per call: the Master bed (T),
// whose BS2 barrier sends one frame per step, and the STEPISR 0 fallback.
class PolledStepper {
public:
    PolledStepper(uint8_t stepPin, uint8_t dirPin);
    void begin();                         // configure STEP/DIR as outputs
    void setPinsInverted(bool directionInvert, bool stepInvert);

    bool run();                           // step if due; returns isRunning()
    // One immediate pulse outside the planner (constant-speed callers);
    // the position follows, any planned move is cancelled.
    void stepNow(bool forward);
    MotionPlanner& planner() { return planner_; }
    const MotionPlanner& planner() const { return planner_; }

private:
    void pulse(bool forward);
    MotionPlanner planner_;
    uint8_t stepPin_, dirPin_;
    bool dirInverted_ = false, stepInverted_ = false;
    uint32_t lastStepUs_ = 0;
};
