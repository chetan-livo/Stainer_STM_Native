#pragma once
// AccelStepper-compatible interface for StepperMotor's polled mode, backed by
// the clean-room MotionPlanner (PolledStepper). No AccelStepper code is used.
// Step timing equals AccelStepper 1.64 (tests/host/test_planner.cpp).
#include "Arduino.h"

class AccelStepper {
public:
    enum MotorInterfaceType { DRIVER = 1 };

    // Pins are configured by TMCModule::setup(); nothing touches hardware here.
    AccelStepper(uint8_t, uint8_t stepPin, uint8_t dirPin) : stepper_(stepPin, dirPin) {}

    void moveTo(long absolute) { constantSpeed_ = 0.0f; stepper_.planner().moveTo(absolute); }
    void move(long relative) { constantSpeed_ = 0.0f; stepper_.planner().move(relative); }
    bool run() { return stepper_.run(); }
    void stop() { stepper_.planner().stop(); }

    void setMaxSpeed(float speed) { stepper_.planner().setMaxSpeed(speed); }
    float maxSpeed() const { return stepper_.planner().maxSpeed(); }
    void setAcceleration(float acceleration) { stepper_.planner().setAcceleration(acceleration); }
    float acceleration() const { return stepper_.planner().acceleration(); }
    void setCurrentPosition(long position) { constantSpeed_ = 0.0f; stepper_.planner().setCurrentPosition(position); }
    long currentPosition() const { return stepper_.planner().currentPosition(); }
    long distanceToGo() const { return stepper_.planner().distanceToGo(); }
    float speed() const { return constantSpeed_ != 0.0f ? constantSpeed_ : stepper_.planner().speed(); }
    bool isRunning() const { return stepper_.planner().isRunning(); }
    void setPinsInverted(bool directionInvert, bool stepInvert, bool) { stepper_.setPinsInverted(directionInvert, stepInvert); }

    // Constant-speed mode (only StepperMotor's unused S-curve path calls these).
    void setSpeed(float speed);
    bool runSpeed();

private:
    PolledStepper stepper_;
    float constantSpeed_ = 0.0f;
    uint32_t lastConstantStepUs_ = 0;
};
