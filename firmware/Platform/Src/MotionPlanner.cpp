#include "MotionPlanner.h"
#include <math.h>

MotionPlanner::MotionPlanner()
{
    // 1 step/s and 1 step/s^2 until configured.
    setAcceleration(1.0f);
    setMaxSpeed(1.0f);
}

long MotionPlanner::brakingSteps() const
{
    // Eq. 16: steps needed to stop from the current speed.
    return (long)((velocity_ * velocity_) / (2.0f * accel_));
}

void MotionPlanner::plan()
{
    const long remaining = target_ - position_;
    const long braking = brakingSteps();

    if (remaining == 0 && braking <= 1) {           // arrived and (nearly) stopped
        velocity_ = 0.0f;
        intervalUs_ = 0;
        ramp_ = 0;
        return;
    }

    const int8_t wanted = remaining > 0 ? 1 : -1;
    if (ramp_ > 0) {
        // Speeding up or cruising: brake if the target is behind us or too close.
        if (wanted != heading_ || braking >= labs(remaining)) ramp_ = -braking;
    } else if (ramp_ < 0) {
        // Braking: resume accelerating if the target moved away ahead of us.
        if (wanted == heading_ && braking < labs(remaining)) ramp_ = -ramp_;
    }

    if (ramp_ == 0) {                               // starting from rest
        delayUs_ = firstDelayUs_;
        heading_ = wanted;
    } else {                                        // eq. 13 (n < 0 lengthens the delay)
        delayUs_ -= (2.0f * delayUs_) / (4.0f * (float)ramp_ + 1.0f);
        if (delayUs_ < minDelayUs_) delayUs_ = minDelayUs_;
    }
    ++ramp_;
    intervalUs_ = (uint32_t)delayUs_;
    velocity_ = (float)heading_ * (1000000.0f / delayUs_);
}

void MotionPlanner::stepTaken()
{
    position_ += heading_;
    plan();
}

void MotionPlanner::moveTo(long absolute)
{
    if (absolute == target_) return;
    target_ = absolute;
    plan();
}

void MotionPlanner::stop()
{
    if (velocity_ == 0.0f) return;
    moveTo(position_ + heading_ * (brakingSteps() + 1));
}

void MotionPlanner::setCurrentPosition(long position)
{
    position_ = target_ = position;
    ramp_ = 0;
    velocity_ = 0.0f;
    intervalUs_ = 0;
}

void MotionPlanner::setMaxSpeed(float stepsPerSecond)
{
    stepsPerSecond = fabsf(stepsPerSecond);
    if (stepsPerSecond == maxSpeed_ || stepsPerSecond == 0.0f) return;
    maxSpeed_ = stepsPerSecond;
    minDelayUs_ = 1000000.0f / maxSpeed_;
    if (ramp_ > 0) {
        // Re-enter the ramp at the step matching the current speed so a lower
        // limit is approached by braking, not by a jump.
        ramp_ = brakingSteps();
        plan();
    }
}

void MotionPlanner::setAcceleration(float stepsPerSecond2)
{
    stepsPerSecond2 = fabsf(stepsPerSecond2);
    if (stepsPerSecond2 == 0.0f || stepsPerSecond2 == accel_) return;
    // v^2 = 2 a n: rescale the ramp position so the current speed is kept.
    ramp_ = (long)((float)ramp_ * (accel_ / stepsPerSecond2));
    accel_ = stepsPerSecond2;
    firstDelayUs_ = 0.676f * sqrtf(2.0f / accel_) * 1000000.0f;
    plan();
}
