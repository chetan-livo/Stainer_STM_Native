#include "StepperMotor.h"

StepperMotor::StepperMotor(uint8_t interface, uint8_t stepPin, uint8_t dirPin)
    : accelStepper(interface, stepPin, dirPin), stepPin(stepPin), dirPin(dirPin) {
    // Note: AccelStepper itself calls pinMode for step and dir pins.
    sCurveActive  = false;
    sStartPos         = 0;
    sTargetPos        = 0;
    sDir              = 1.0f;
    s_vMax            = 0.0f;
    s_aMax            = 0.0f;
    s_jMax            = 0.0f;
    s_totalTime       = 0.0f;
    s_tStartMicros    = 0;
    s_lastPrintMicros = 0;
}

StepperMotor::~StepperMotor() {
}

namespace {
StepperMotor* interruptCapable[StepEngine::MaxSteppers] = {};
uint8_t interruptCapableCount = 0;
}

void StepperMotor::loopMotor() {
    if (isrMode) {
        // The step interrupt issues pulses; servicing renews its lease.
        isr.serviced();
        return;
    }
    if (sCurveActive) {
        // Run our own S-curve profile
        updateSCurve();
    } else {
        // Default AccelStepper trapezoidal profile
        accelStepper.run();
    }
}

bool StepperMotor::enableInterruptStepping() {
#ifdef LIVO_STEP_ENGINE
    if (!isr.attach(stepPin, dirPin)) return false;
    if (!isrCapable) {
        if (interruptCapableCount >= StepEngine::MaxSteppers) return false;
        interruptCapable[interruptCapableCount++] = this;
        isrCapable = true;
    }
    return setInterruptStepping(true);
#else
    return false;
#endif
}

void StepperMotor::disableInterruptStepping() {
    setInterruptStepping(false);
    isrCapable = false;
}

bool StepperMotor::setInterruptStepping(bool on) {
    if (on == isrMode) return true;
    if (on && !isrCapable) return false;
    if (isRunning()) return false;
    if (on) {
        const long position = accelStepper.currentPosition();
        StepEngine::Guard guard;
        isr.setCurrentPosition(position);
        isr.setActive(true);
        isrMode = true;
    } else {
        long position;
        {
            StepEngine::Guard guard;
            isr.setActive(false);
            position = isr.currentPosition();
            isrMode = false;
        }
        accelStepper.setCurrentPosition(position);
    }
    return true;
}

bool StepperMotor::setAllInterruptStepping(bool on) {
    for (uint8_t i = 0; i < interruptCapableCount; ++i)
        if (interruptCapable[i]->isRunning()) return false;
    for (uint8_t i = 0; i < interruptCapableCount; ++i)
        interruptCapable[i]->setInterruptStepping(on);
    return true;
}

uint8_t StepperMotor::interruptSteppingCount() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < interruptCapableCount; ++i)
        if (interruptCapable[i]->isrMode) ++count;
    return count;
}

void StepperMotor::moveTo(long absolute) {
    if(displayDebug){
		Serial.print("moveTo: ");
		Serial.print(absolute);
        Serial.println(" ");
	}
    if (isrMode) { StepEngine::Guard guard; isr.moveTo(absolute); }
    else accelStepper.moveTo(absolute);
}

void StepperMotor::move(long relative) {
    if(displayDebug){
		Serial.print("move: ");
		Serial.print(relative);
        Serial.println(" ");
	}
    if (isrMode) { StepEngine::Guard guard; isr.move(relative); }
    else accelStepper.move(relative);
}

void StepperMotor::setMaxSpeed(float speed) { // Setter, remains non-const
    accelStepper.setMaxSpeed(speed);
    StepEngine::Guard guard;
    isr.setMaxSpeed(speed);
}

float StepperMotor::maxSpeed() const { // Added const
    return accelStepper.maxSpeed();
}

void StepperMotor::setAcceleration(float acceleration) { // Setter, remains non-const
    accelStepper.setAcceleration(acceleration);
    StepEngine::Guard guard;
    isr.setAcceleration(acceleration);
}

float StepperMotor::acceleration() const { // Added const
    return accelStepper.acceleration();
}

void StepperMotor::setCurrentPosition(long position) { // Setter, remains non-const
    accelStepper.setCurrentPosition(position);
    StepEngine::Guard guard;
    isr.setCurrentPosition(position);
}

long StepperMotor::currentPosition() const { // Added const
    if (isrMode) { StepEngine::Guard guard; return isr.currentPosition(); }
    return accelStepper.currentPosition();
}

long StepperMotor::distanceToGo() const { // Added const
    if (isrMode) { StepEngine::Guard guard; return isr.distanceToGo(); }
    return accelStepper.distanceToGo();
}

bool StepperMotor::isRunning() const { // Added const
    if (isrMode) { StepEngine::Guard guard; return isr.isRunning(); }
    return accelStepper.isRunning();
}

void StepperMotor::stop() {
    if (isrMode) { StepEngine::Guard guard; isr.stop(); }
    else accelStepper.stop();

    // Optional
    sCurveActive = false;
}

// Polled-mode stepping primitives; the step interrupt owns pulses otherwise.
void StepperMotor::run() {
    if (isrMode) return;
    accelStepper.run();
}

void StepperMotor::runSpeed() {
    if (isrMode) return;
    accelStepper.runSpeed();
}

void StepperMotor::setSpeed(float speed) {
    accelStepper.setSpeed(speed);
}

void StepperMotor::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert) {
    accelStepper.setPinsInverted(directionInvert, stepInvert, enableInvert);
    StepEngine::Guard guard;
    isr.setPinsInverted(directionInvert, stepInvert);
}

void StepperMotor::enableOutputs() {

}

void StepperMotor::disableOutputs() {
    
}

long StepperMotor::getOffset() const {
    return offset;
}

void StepperMotor::setOffset(long position) {
    offset = position;
    StepperMotor::setCurrentPosition(-position);
}

void StepperMotor::setHomingOffset(long offset) {
    this->homingOffset = offset;
}

long StepperMotor::getHomingOffset() const {
    return homingOffset;
}

void StepperMotor::setJerk(float value) {
    jerk = value;
}

float StepperMotor::getJerk() {
    return jerk;
}

void StepperMotor::computeUsableProfile(long distanceSteps, float vReq, float aReq, float jReq, float &vUse, float &aUse, float &jUse)
{
  // Distance in steps (always positive)
  float D = (distanceSteps >= 0) ? (float)distanceSteps : (float)(-distanceSteps);
  if (D < 1.0f) D = 1.0f;

  // Requested limits (absolute values)
  float v = fabs(vReq);
  float a = fabs(aReq);
  float j = fabs(jReq);

  // Basic safety guards
  if (v <= 0.0f) v = 1.0f;
  if (a <= 0.0f) a = v;      // reasonable default
  if (j <= 0.0f) j = a;      // reasonable default

  // -------------------------------------------------------
  // 1) Distance constraint:
  //
  // D >= v*(a/j + v/a)
  //
  // Rearranged as a quadratic in v:
  // (1/a) v^2 + (a/j) v - D <= 0
  //
  // Multiply by 'a':
  // v^2 + (a^2/j) v - a*D <= 0
  //
  // The positive root of:
  // v^2 + (a^2/j) v - a*D = 0
  // is the maximum allowed v (v_max) for given a, j, D.
  // -------------------------------------------------------
  float b    = (a * a) / j;                  // a^2 / j
  float disc = b * b + 4.0f * a * D;         // b^2 - 4*c, with c = -a*D
  float vMax = (-b + sqrtf(disc)) * 0.5f;    // positive root

  // -------------------------------------------------------
  // 2) Jerk / 7-seg constraint:
  //
  // For t2, t6 > 0, we need:
  //   v > a^2 / j
  // i.e. v > vMinJerk
  // -------------------------------------------------------
  float vMinJerk = (a * a) / j;  // a^2 / j

  // -------------------------------------------------------
  // 3) Choose vUse:
  //
  // Valid S-curve region in v is:
  //   (vMinJerk, vMax]
  //
  // - If vReq <= vMax   -> we can use vReq (or above vMinJerk if you want full 7-seg).
  // - If vReq >  vMax   -> we clamp to vMax.
  //
  // Note: any v <= vMax satisfies distance constraint;
  //       we'll fix jerk constraint by adjusting 'a' next.
  // -------------------------------------------------------
  if (vMax < 1.0f) vMax = 1.0f;   // tiny moves safety

  if (v <= vMax) {
    // Requested speed fits within distance constraint
    vUse = v;
  } else {
    // Requested speed too high -> clamp to maximum allowed by distance
    vUse = vMax;
  }

  // -------------------------------------------------------
  // 4) Adjust 'a' to satisfy jerk constraint at final vUse:
  //
  // vUse > aUse^2 / j   =>   aUse < sqrt(j * vUse)
  //
  // So the maximum valid acceleration is:
  //   a_jerk_max = sqrt(j * vUse)
  //
  // We must not exceed user limit aReq, so:
  //   aUse = min(aReq, a_jerk_max)
  // -------------------------------------------------------
  float aJerkMax = sqrtf(j * vUse);
  aUse = (a > aJerkMax) ? aJerkMax : a;

  if (aUse < 1.0f) aUse = 1.0f;   // avoid degenerate zero accel

  // Jerk stays as requested (you can also clamp it if you want)
  jUse = j;
}


// void StepperMotor::computeUsableProfile(long distanceSteps, float vReq, float aReq, float jReq, float &vUse, float &aUse, float &jUse) {
//     float D = (distanceSteps >= 0) ? (float)distanceSteps : (float)(-distanceSteps);
//     if (D < 1.0f) D = 1.0f;

//     float v = fabs(vReq);
//     float a = fabs(aReq);
//     float j = fabs(jReq);

//     if (v < 1.0f) v = 1.0f;
//     if (a < 1.0f) a = v;
//     if (j < 1.0f) j = a;

//     for (int iter = 0; iter < 10; ++iter) {
//         float jMin = (a * a) / v;
//         if (j <= jMin) {
//             float aNew = sqrt(j * v * 0.9f);
//             if (aNew < 1.0f)      aNew = 1.0f;
//             if (aNew > fabs(aReq)) aNew = fabs(aReq);
//             a = aNew;
//         }

//         float Dmin = v * (a / j + v / a);
//         if (D >= Dmin) {
//             break;
//         }

//         float scale = (D / Dmin) * 0.9f;
//         if (scale > 1.0f) scale = 1.0f;
//         if (scale < 0.1f) scale = 0.1f;
//         v *= scale;

//         if (v < 1.0f) {
//             v = 1.0f;
//             break;
//         }
//     }

//     vUse = (v > fabs(vReq)) ? fabs(vReq) : v;
//     aUse = (a > fabs(aReq)) ? fabs(aReq) : a;
//     jUse = (j > fabs(jReq)) ? fabs(jReq) : j;
// }

float StepperMotor::sCurveVelocityAt(float t) const {
    const float j = s_jMax;
    const float a = s_aMax;

    if (t <= 0.0f)         return 0.0f;
    if (t >= s_totalTime)  return 0.0f;

    // Phase 1
    if (t < s_tCum[0]) {
        float tau = t;
        return 0.5f * j * tau * tau;
    }
    // Phase 2
    if (t < s_tCum[1]) {
        float tau = t - s_tCum[0];
        return s_vBoundary[0] + a * tau;
    }
    // Phase 3
    if (t < s_tCum[2]) {
        float tau = t - s_tCum[1];
        return s_vBoundary[1] + a * tau - 0.5f * j * tau * tau;
    }
    // Phase 4 (cruise)
    if (t < s_tCum[3]) {
        return s_vBoundary[3];
    }
    // Phase 5
    if (t < s_tCum[4]) {
        float tau = t - s_tCum[3];
        return s_vBoundary[3] - 0.5f * j * tau * tau;
    }
    // Phase 6
    if (t < s_tCum[5]) {
        float tau = t - s_tCum[4];
        return s_vBoundary[4] - a * tau;
    }
    // Phase 7
    if (t < s_tCum[6]) {
        float tau = t - s_tCum[5];
        return s_vBoundary[5] - a * tau + 0.5f * j * tau * tau;
    }

    return 0.0f;
}

void StepperMotor::startSCurveMove(long targetSteps, float vMaxReq, float aMaxReq, float jMaxReq) {
    if (isrMode) return; // S-curve drives AccelStepper directly (polled mode only; unused)
    long cur   = accelStepper.currentPosition();
    long delta = targetSteps - cur;
    if (delta == 0) {
        return;
    }

    float vUse, aUse, jUse;
    computeUsableProfile(delta, vMaxReq, aMaxReq, jMaxReq, vUse, aUse, jUse);

    float D   = (delta > 0) ? (float)delta : (float)(-delta);
    float dir = (delta > 0) ? 1.0f : -1.0f;

    float v_f1 = (aUse * aUse) / (2.0f * jUse);
    float v_f2 = vUse - v_f1;
    float v_f3 = vUse;
    float v_f4 = vUse;
    float v_f5 = vUse - v_f1;
    float v_f6 = v_f1;

    float t1 = aUse / jUse;
    float t2 = (v_f2 - v_f1) / aUse;
    float t3 = aUse / jUse;
    float t5 = aUse / jUse;
    float t6 = (v_f5 - v_f6) / aUse;
    float t7 = aUse / jUse;

    float t4_num = (aUse * jUse * D) - (vUse * aUse * aUse) - (jUse * vUse * vUse);
    float t4_den = jUse * aUse * vUse;
    float t4     = t4_num / t4_den;

    if (t1 < 0.0f) t1 = 0.0f;
    if (t2 < 0.0f) t2 = 0.0f;
    if (t3 < 0.0f) t3 = 0.0f;
    if (t4 < 0.0f) t4 = 0.0f;
    if (t5 < 0.0f) t5 = 0.0f;
    if (t6 < 0.0f) t6 = 0.0f;
    if (t7 < 0.0f) t7 = 0.0f;

    sCurveActive = true;
    sStartPos    = cur;
    sTargetPos   = targetSteps;
    sDir         = dir;
    s_vMax       = vUse;
    s_aMax       = aUse;
    s_jMax       = jUse;

    s_t[0] = t1;
    s_t[1] = t2;
    s_t[2] = t3;
    s_t[3] = t4;
    s_t[4] = t5;
    s_t[5] = t6;
    s_t[6] = t7;

    s_tCum[0] = s_t[0];
    for (int i = 1; i < 7; ++i) {
        s_tCum[i] = s_tCum[i - 1] + s_t[i];
    }
    s_totalTime = s_tCum[6];

    s_vBoundary[0] = v_f1;
    s_vBoundary[1] = v_f2;
    s_vBoundary[2] = v_f3;
    s_vBoundary[3] = v_f4;
    s_vBoundary[4] = v_f5;
    s_vBoundary[5] = v_f6;
    s_vBoundary[6] = 0.0f;   

    s_tStartMicros = micros();

    accelStepper.setAcceleration(0.0f);
}

bool StepperMotor::isSCurveActive() const
{
    return sCurveActive;
}

void StepperMotor::updateSCurve()
{
    if (!sCurveActive) {
        return;
    }

    unsigned long now = micros();
    float t = (now - s_tStartMicros) * 1e-6f;

    if (t >= s_totalTime) {
        sCurveActive = false;
        accelStepper.setSpeed(0.0f);
        accelStepper.setCurrentPosition(sTargetPos);
        return;
    }

    float vScalar = sCurveVelocityAt(t);
    float vCmd    = sDir * vScalar;

    accelStepper.setSpeed(vCmd);
    accelStepper.runSpeed();
}