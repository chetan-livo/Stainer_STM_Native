#pragma once
// The STM32duino HardwareTimer subset DCFan uses (25 kHz fan PWM), mapped
// onto the platform PWM driver. The pin's timer channel comes from the pin
// table, which matches the timer DCFan names (PC9 = TIM3 CH4, PE5 = TIM9 CH1).
#include "Arduino.h"

enum TimerModes_t { TIMER_OUTPUT_COMPARE_PWM1 = 6 };
enum TimerFormat_t { TICK_FORMAT, MICROSEC_FORMAT, HERTZ_FORMAT };
enum TimerCompareFormat_t { PERCENT_COMPARE_FORMAT, TICK_COMPARE_FORMAT };

class HardwareTimer {
public:
    explicit HardwareTimer(TIM_TypeDef* instance) : instance_(instance) {}
    void setMode(uint32_t, TimerModes_t, uint32_t pin) { pin_ = (uint8_t)pin; }
    void setOverflow(uint32_t value, TimerFormat_t format = TICK_FORMAT)
    {
        if (format == HERTZ_FORMAT) frequencyHz_ = value;
    }
    void setCaptureCompare(uint32_t, uint32_t value, TimerCompareFormat_t format = TICK_COMPARE_FORMAT)
    {
        if (format == PERCENT_COMPARE_FORMAT && pin_ != NC) pwmWritePercent(pin_, frequencyHz_, value);
    }
    void pause() { if (pin_ != NC) pwmStop(pin_); }
    void resume() {}
    TIM_TypeDef* instance() const { return instance_; }

private:
    TIM_TypeDef* instance_;
    uint8_t pin_ = NC;
    uint32_t frequencyHz_ = 1000;
};
