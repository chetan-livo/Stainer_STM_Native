#pragma once
#include <stdint.h>

// Arduino-compatible time base. millis() is the 1 kHz SysTick count;
// micros() adds the SysTick sub-millisecond phase (both wrap like Arduino:
// ~49.7 days and ~71.6 minutes). cycles() is the DWT cycle counter.
void timeInit();
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

inline uint32_t cycles() { return *(volatile uint32_t*)0xE0001004U; } // DWT->CYCCNT
uint32_t cyclesPerMicrosecond();
