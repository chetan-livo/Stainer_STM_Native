#pragma once
#include "Print.h"
#include "Pins.h"
#include "MotionPlanner.h"

// Timer-driven STEP generation for TMC axes.
//
// A 50 kHz TIM7 interrupt issues STEP pulses, so step timing no longer depends
// on how often loop() reaches each motor. Step timing comes from
// MotionPlanner (clean-room trapezoid; matches AccelStepper 1.64 step for step
// in tests/host/test_planner.cpp). Step times accumulate exactly, so the
// average rate equals the commanded rate up to one step per tick (50,000
// steps/s); individual steps jitter by at most one tick.
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
    // Main context. Callers hold StepEngine::Guard around planner calls.
    bool attach(uint8_t stepPin, uint8_t dirPin);
    void setPinsInverted(bool directionInvert, bool stepInvert);
    void moveTo(long absolute) { planner_.moveTo(absolute); syncCountdown(); }
    void move(long relative) { planner_.move(relative); syncCountdown(); }
    void setMaxSpeed(float speed) { planner_.setMaxSpeed(speed); syncCountdown(); }
    float maxSpeed() const { return planner_.maxSpeed(); }
    void setAcceleration(float acceleration) { planner_.setAcceleration(acceleration); syncCountdown(); }
    float acceleration() const { return planner_.acceleration(); }
    void setCurrentPosition(long position) { planner_.setCurrentPosition(position); countdown_.reset(); }
    long currentPosition() const { return planner_.currentPosition(); }
    long distanceToGo() const { return planner_.distanceToGo(); }
    bool isRunning() const { return planner_.isRunning(); }
    float speed() const { return planner_.speed(); }
    void stop() { planner_.stop(); syncCountdown(); }

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
    // Coming to rest restarts the countdown, so the next move steps at once.
    void syncCountdown() { if (!planner_.stepIntervalUs()) countdown_.reset(); }

    MotionPlanner planner_;
    StepCountdown countdown_;
    GPIO_TypeDef* stepPort_ = nullptr;
    GPIO_TypeDef* dirPort_ = nullptr;
    uint32_t stepMask_ = 0, dirMask_ = 0;
    uint32_t stepHigh_ = 0, stepLow_ = 0;
    bool dirInverted_ = false, stepInverted_ = false;

    volatile bool active_ = false, driverEnabled_ = false, held_ = false, leased_ = false;
    volatile uint32_t leaseTick_ = 0;
};
