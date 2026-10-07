#pragma once
#include <stdint.h>

// Trapezoidal step-timing planner, written from D. Austin, "Generate
// stepper-motor speed profiles in real time" (Embedded Systems Programming
// magazine). Pure arithmetic, no hardware: StepEngine (timer interrupt)
// and PolledStepper (main loop) both drive it.
//
//   first step delay   c0 = 0.676 * sqrt(2 / a)               (eq. 15, corrected)
//   ramp recurrence    cn = c(n-1) - 2 c(n-1) / (4 n + 1)       (eq. 13)
//   braking distance   n  = v^2 / (2 a)                         (eq. 16)
//
// n > 0 counts steps up the acceleration ramp (the delay stops shrinking at
// the max-speed limit); n < 0 counts the braking steps left; n == 0 is rest.
// Delays are in microseconds; the step interval handed to an engine is the
// whole-microsecond floor, which keeps timing comparable with the Arduino
// firmware. tests/host/test_planner.cpp checks step timing and end
// positions against the AccelStepper build the firmware used.
class MotionPlanner {
public:
    MotionPlanner();

    // Targets.
    void moveTo(long absolute);
    void move(long relative) { moveTo(position_ + relative); }
    void stop();                          // brake to rest as quickly as allowed
    void setCurrentPosition(long position); // also halts immediately

    // Limits. Changing them mid-move takes effect from the next step.
    void setMaxSpeed(float stepsPerSecond);
    void setAcceleration(float stepsPerSecond2);
    float maxSpeed() const { return maxSpeed_; }
    float acceleration() const { return accel_; }

    // State.
    long currentPosition() const { return position_; }
    long targetPosition() const { return target_; }
    long distanceToGo() const { return target_ - position_; }
    float speed() const { return velocity_; }       // signed steps/s
    bool isRunning() const { return velocity_ != 0.0f || target_ != position_; }
    bool forward() const { return heading_ > 0; }   // direction of the next step
    uint32_t stepIntervalUs() const { return intervalUs_; } // 0 = no step due

    // Engines call this right after issuing a STEP pulse.
    void stepTaken();

private:
    void plan();
    long brakingSteps() const;

    long position_ = 0, target_ = 0;
    long ramp_ = 0;               // Austin's n (signed, see above)
    float maxSpeed_ = 0.0f, accel_ = 0.0f;   // set to 1 by the constructor
    float firstDelayUs_ = 0.0f, minDelayUs_ = 1e6f, delayUs_ = 0.0f;
    float velocity_ = 0.0f;
    int8_t heading_ = -1;         // +1 forward, -1 reverse
    uint32_t intervalUs_ = 0;
};

// Step-time accumulator for a fixed-rate tick (the StepEngine interrupt).
// Step times accumulate exactly, so the average rate equals the planned rate;
// each step lands on the first tick at or after its due time. Time only
// advances while the motor may step, so a hold or a delayed interrupt pauses
// the profile instead of building a burst of catch-up steps.
struct StepCountdown {
    int32_t remainingUs = 0;
    // One tick elapsed; true when a step is due now.
    bool tick(int32_t tickUs) { remainingUs -= tickUs; return remainingUs <= 0; }
    // After the step: schedule the next one (0 = none pending).
    void scheduleNext(uint32_t intervalUs)
    {
        if (!intervalUs) { remainingUs = 0; return; }
        remainingUs += (int32_t)intervalUs;
        if (remainingUs < 0) remainingUs = 0; // interval shorter than a tick: no backlog
    }
    void reset() { remainingUs = 0; }
};

