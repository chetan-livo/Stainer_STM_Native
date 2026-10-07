/*
 * DCFan.cpp
 *
 *  Created on: Jan 23, 2026
 *      Author: Varalakshmi
 */

#include "DCFan.h"

#ifdef Nozzle_Mount_PCB
    DCFan dcFan1(Fan_1);
    DCFan dcFan2(Fan_2);
    DCFan dcFan3(Fan_3, Fan_3_Tacho, (uintptr_t)TIM3, 4, 25000);        // TIM3_CH4 for Fan_3 pin (PC9)
    DCFan dcFan4(Fan_4, Fan_4_Tacho, (uintptr_t)TIM9, 1, 25000);        // TIM9_CH1 for Fan_4 pin (PE5)
#endif

#ifdef Stainer_Master_PCB
    // MIXD / MIXR are the H-bridge inputs. The dcFan1 / dcFan2 objects are kept
    // here only because they're referenced via `extern DCFan dcFan1/2;` from
    // DCFan.h; on master they're no longer driven (S2MIX uses analogWrite on
    // MIXD/MIXR directly).
    DCFan dcFan1(MIXD);
    DCFan dcFan2(MIXR);
#endif


DCFan* DCFan::_isr0 = nullptr;
DCFan* DCFan::_isr1 = nullptr;

void DCFan::tachThunk0() { if (_isr0) _isr0->_tachPulses++; }
void DCFan::tachThunk1() { if (_isr1) _isr1->_tachPulses++; }

DCFan::DCFan(int fanPin) {
    _fanPin = fanPin;
    _timer = nullptr;
}

DCFan::DCFan(int pwmCtrlPin, int tachPin, uintptr_t timerInstance, uint8_t timerChannel, uint32_t pwmFreqHz)
{
    _pwmCtrlPin = pwmCtrlPin;
    _tachPin = tachPin;
    _timerInstance = timerInstance;
    _timerChannel = timerChannel;
    _pwmFreqHz = pwmFreqHz;

    _timer = new HardwareTimer((TIM_TypeDef*)timerInstance);
}

DCFan::~DCFan() {
	detachTachInterrupt();

    if (_timer) {
        _timer->pause();
        delete _timer;
        _timer = nullptr;
    }
}

void DCFan::setup() {
    if (_timer == nullptr) {
        // NORMAL FAN
        pinMode(_fanPin, OUTPUT);
        analogWrite(_fanPin, 0);
        return;
    }
    // HIGH-SPEED FAN
    pinMode(_pwmCtrlPin, OUTPUT);
    pinMode(_tachPin, INPUT_PULLUP);
    
    _timer->setMode(_timerChannel, TIMER_OUTPUT_COMPARE_PWM1, _pwmCtrlPin);
    _timer->setOverflow(_pwmFreqHz, HERTZ_FORMAT);
    _timer->setCaptureCompare(_timerChannel, 0, PERCENT_COMPARE_FORMAT);
    _timer->resume();

    _tachPulses = 0;
    _lastSampleMs = millis();
    _lastRpm = 0.0f;
    attachTachInterrupt();
}

void DCFan::runFan(int pwmValue) {
    if (_timer == nullptr) {
        // Normal Fan
        analogWrite(_fanPin, pwmValue);
        return;
    }
    // High Speed Fan
    runHighSpeedFanPWM(pwmValue);
}

void DCFan::stopFan() {
    if (_timer == nullptr) {
        // Normal Fan
        analogWrite(_fanPin, 0);
        return;
    }
    // High Speed Fan
    stopHighSpeedFan();
}

void DCFan::runHighSpeedFanPWM(uint8_t pwm255) {
    if (_timer == nullptr) return;  

    // Convert 0–255 → 0–100 %
    uint8_t duty = map(pwm255, 0, 255, 0, 100);

    // // Prevent stalling of motor 
    // if (duty > 0 && duty < 20) duty = 20;

    _timer->setCaptureCompare(_timerChannel, duty, PERCENT_COMPARE_FORMAT);
}

void DCFan::runHighSpeedFanPercent(uint8_t percent)
{
    if (_timer == nullptr) return;

    uint8_t duty = constrain(percent, 0, 100);

    // // Prevent stall
    // if (duty > 0 && duty < 20) duty = 20;

    _timer->setCaptureCompare(_timerChannel, duty, PERCENT_COMPARE_FORMAT);
}

void DCFan::stopHighSpeedFan()
{
    if (_timer == nullptr) return;
    _timer->setCaptureCompare(_timerChannel, 0, PERCENT_COMPARE_FORMAT);
}

float DCFan::getHighSpeedFanSpeed(uint8_t pulsesPerRev)
{
    if (_timer == nullptr) return 0.0f;
    if (_tachPin < 0) return 0.0f;
    if (pulsesPerRev == 0) pulsesPerRev = 2;

    const uint32_t SAMPLE_MS = 500;
    uint32_t now = millis();
    if (now - _lastSampleMs < SAMPLE_MS) return _lastRpm;

    noInterrupts();
    uint32_t pulses = _tachPulses;
    _tachPulses = 0;
    interrupts();

    const uint32_t elapsedMs = now - _lastSampleMs;
    _lastSampleMs = now;

    float pulsesPerSecond = (pulses * 1000.0f) / (float)elapsedMs;
    _lastRpm = (pulsesPerSecond * 60.0f) / (float)pulsesPerRev;
    return _lastRpm;
}

// Helpers
void DCFan::attachTachInterrupt()
{
    if (_tachPin < 0) return;      // no tach connected
    if (_timer == nullptr) return; // only high-speed uses tach here

    pinMode(_tachPin, INPUT_PULLUP);

    // Register in one of two slots
    if (_isr0 == nullptr) {
        _isr0 = this;
        attachInterrupt(digitalPinToInterrupt(_tachPin), DCFan::tachThunk0, FALLING);
    } else if (_isr1 == nullptr) {
        _isr1 = this;
        attachInterrupt(digitalPinToInterrupt(_tachPin), DCFan::tachThunk1, FALLING);
    } else {
        // If you need >2 tach fans, tell me; I'll extend routing to an array.
    }
}

void DCFan::detachTachInterrupt()
{
    if (_tachPin < 0) return;

    detachInterrupt(digitalPinToInterrupt(_tachPin));

    if (_isr0 == this) _isr0 = nullptr;
    if (_isr1 == this) _isr1 = nullptr;
}
