#pragma once
#include "Print.h"
#include "Pins.h"

// Timer-driven STEP generation for TMC axes.
//
// A 50 kHz TIM7 interrupt issues STEP pulses, so step timing no longer depends
// on how often loop() reaches each motor. The planner is AccelStepper 1.64's
// trapezoid (same equations, same whole-microsecond intervals). Step times
// accumulate exactly, so the average rate equals the commanded rate up to one
// step per tick (50,000 steps/s); individual steps jitter by at most one tick.
//
// Freezes that the polled code produced by not servicing a motor are kept:
//  - holdSteps(): explicit freeze at the bed gates (immediate);
//  - service lease: a motor not serviced by TMCModule::loop() for 50 ms stops
//    stepping, so blocking routines that service other axes, or a stalled main
//    loop during an open-ended homing move, still freeze it.
// A frozen motor resumes at its previous speed, as before.
//
// Ported from Livo-Stainer perf/timer-step-engine (271a10b). Uses TIM7 with
// a native interrupt handler; parts without TIM7 (F401) build without it.
#if defined(TIM7)
#define LIVO_STEP_ENGINE 1
#endif

namespace StepEngine {
constexpr uint32_t TickHz = 50000;
constexpr int32_t TickUs = 1000000 / TickHz;
constexpr uint32_t LeaseTicks = TickHz / 20; // 50 ms
constexpr uint8_t MaxSteppers = 8;

uint32_t ticks();
// Masks the step interrupt while the main loop changes planner state. Nests.
class Guard {
public:
    Guard();
    ~Guard();
    Guard(const Guard&) = delete;
    Guard& operator=(const Guard&) = delete;
private:
    bool wasEnabled;
};
void reportStats(Print& out); // appended to LOOPSTAT
}

class IsrStepper {
public:
    IsrStepper();
    // Main context. Callers hold StepEngine::Guard around planner calls.
    bool attach(uint8_t stepPin, uint8_t dirPin);
    void setPinsInverted(bool directionInvert, bool stepInvert);
    void moveTo(long absolute);
    void move(long relative) { moveTo(current_ + relative); }
    void setMaxSpeed(float speed);
    float maxSpeed() const { return maxSpeed_; }
    void setAcceleration(float acceleration);
    float acceleration() const { return acceleration_; }
    void setCurrentPosition(long position);
    long currentPosition() const { return current_; }
    long distanceToGo() const { return target_ - current_; }
    bool isRunning() const { return !(speed_ == 0.0f && target_ == current_); }
    void stop();

    // Single-word flags; safe without the guard.
    void setActive(bool active) { active_ = active; }
    bool active() const { return active_; }
    void setDriverEnabled(bool enabled) { driverEnabled_ = enabled; }
    void serviced() { leaseTick_ = StepEngine::ticks(); leased_ = true; held_ = false; }
    void hold() { held_ = true; }

    // Step interrupt only.
    bool dueForStep(uint32_t tick);
    void raiseStep() { stepPort_->BSRR = stepHigh_; }
    void lowerStep() { stepPort_->BSRR = stepLow_; }
    void planNextStep();

private:
    void computeNewSpeed();

    GPIO_TypeDef* stepPort_ = nullptr;
    GPIO_TypeDef* dirPort_ = nullptr;
    uint32_t stepMask_ = 0, dirMask_ = 0;
    uint32_t stepHigh_ = 0, stepLow_ = 0;
    bool dirInverted_ = false, stepInverted_ = false;

    volatile long current_ = 0;
    long target_ = 0;
    float speed_ = 0.0f, maxSpeed_ = 0.0f, acceleration_ = 0.0f;
    long n_ = 0;
    float c0_ = 0.0f, cn_ = 0.0f, cmin_ = 1.0f;
    bool direction_ = false; // true = CW (+1), as AccelStepper
    uint32_t interval_ = 0;  // whole microseconds, 0 = no step pending
    int32_t remainingUs_ = 0;

    volatile bool active_ = false, driverEnabled_ = false, held_ = false, leased_ = false;
    volatile uint32_t leaseTick_ = 0;
};
