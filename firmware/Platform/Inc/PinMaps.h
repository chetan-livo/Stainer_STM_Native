#pragma once
#include <stddef.h>
#include "Pins.h"
#include "stm32f4xx_hal.h"

// Pin -> peripheral tables for the active MCU (generated, see PinMaps.cpp).
struct AdcPin { uint8_t pin; ADC_TypeDef* adc; uint8_t channel; };
struct TimPin { uint8_t pin; TIM_TypeDef* tim; uint8_t alternate; uint8_t channel; uint8_t complementary; };
struct AfPin  { uint8_t pin; void* instance; uint8_t alternate; };

extern const AdcPin adcPins[];
extern const TimPin timPins[];
extern const AfPin i2cSdaPins[];
extern const AfPin i2cSclPins[];
extern const size_t adcPinCount, timPinCount, i2cSdaPinCount, i2cSclPinCount;

template <typename T> const T* findPin(const T* table, size_t count, uint8_t pin)
{
    for (size_t i = 0; i < count; ++i) if (table[i].pin == pin) return &table[i];
    return nullptr;
}
