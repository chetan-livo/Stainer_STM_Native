#include "PolledStepper.h"
#include "Gpio.h"
#include "Timebase.h"

PolledStepper::PolledStepper(uint8_t stepPin, uint8_t dirPin) : stepPin_(stepPin), dirPin_(dirPin) {}

void PolledStepper::begin()
{
    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    digitalWrite(stepPin_, stepInverted_ ? HIGH : LOW);
}

void PolledStepper::setPinsInverted(bool directionInvert, bool stepInvert)
{
    dirInverted_ = directionInvert;
    stepInverted_ = stepInvert;
}

void PolledStepper::pulse(bool forward)
{
    // DIR first, then a STEP pulse of at least 1 us (TMC2209 needs 100 ns).
    digitalWrite(dirPin_, (forward != dirInverted_) ? HIGH : LOW);
    digitalWrite(stepPin_, stepInverted_ ? LOW : HIGH);
    delayMicroseconds(1);
    digitalWrite(stepPin_, stepInverted_ ? HIGH : LOW);
}

bool PolledStepper::run()
{
    const uint32_t interval = planner_.stepIntervalUs();
    if (interval) {
        const uint32_t now = micros();
        if (now - lastStepUs_ >= interval) {
            pulse(planner_.forward());
            lastStepUs_ = now;
            planner_.stepTaken();
        }
    }
    return planner_.isRunning();
}
