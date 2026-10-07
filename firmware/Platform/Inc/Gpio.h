#pragma once
#include "Pins.h"

enum PinMode : uint8_t {
    INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT_OPEN_DRAIN, INPUT_ANALOG
};
constexpr uint8_t LOW = 0, HIGH = 1;

void pinMode(uint8_t pin, uint8_t mode);
// Route a pin to a peripheral alternate function (UART, TIM, I2C, ...).
void pinAlternate(uint8_t pin, uint8_t alternate, bool openDrain = false, bool pullUp = false);

// Single-register accesses; safe from interrupts.
inline void digitalWrite(uint8_t pin, uint8_t value)
{
    GPIO_TypeDef* gpio = pins::port(pin);
    if (gpio) gpio->BSRR = value ? pins::mask(pin) : ((uint32_t)pins::mask(pin) << 16);
}
inline int digitalRead(uint8_t pin)
{
    GPIO_TypeDef* gpio = pins::port(pin);
    return (gpio && (gpio->IDR & pins::mask(pin))) ? HIGH : LOW;
}
inline void digitalToggle(uint8_t pin)
{
    GPIO_TypeDef* gpio = pins::port(pin);
    if (gpio) digitalWrite(pin, (gpio->ODR & pins::mask(pin)) ? LOW : HIGH);
}
