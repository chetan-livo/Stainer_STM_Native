#pragma once
// TMCStepper 0.7.3 interface used by TMCMotor, served by Devices/Tmc2209.
#include "Arduino.h"
#include "Tmc2209.h"

class TMC2209Stepper : public Tmc2209 {
public:
    TMC2209Stepper(Stream* port, float rSense, uint8_t address) : Tmc2209(*port, rSense, address) {}
};
