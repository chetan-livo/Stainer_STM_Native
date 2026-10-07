#pragma once
#include <stdint.h>

enum InterruptEdge : uint8_t { RISING = 1, FALLING = 2, CHANGE = 3 };

// One callback per EXTI line (pin number 0..15, any port); attaching a second
// pin with the same number replaces the first, as the hardware requires.
// Callbacks run in interrupt context at IRQ_PRIO_EXTI.
void attachInterrupt(uint8_t pin, void (*callback)(), uint8_t edge);
void detachInterrupt(uint8_t pin);
inline uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }

void noInterrupts();
void interrupts();
