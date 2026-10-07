#include "AccelStepper.h"
#include "IWatchdog.h"

IWatchdogClass IWatchdog;

void AccelStepper::setSpeed(float speed)
{
    const float limit = stepper_.planner().maxSpeed();
    constantSpeed_ = speed > limit ? limit : (speed < -limit ? -limit : speed);
}

bool AccelStepper::runSpeed()
{
    if (constantSpeed_ == 0.0f) return false;
    const uint32_t interval = (uint32_t)(1000000.0f / fabsf(constantSpeed_));
    const uint32_t now = micros();
    if (now - lastConstantStepUs_ < interval) return false;
    lastConstantStepUs_ = now;
    stepper_.stepNow(constantSpeed_ > 0);
    return true;
}
