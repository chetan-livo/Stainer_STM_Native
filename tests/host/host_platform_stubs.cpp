// Platform time functions for host tests that link Platform sources.
#include "Timebase.h"
static uint32_t fakeMs = 0;
uint32_t millis() { return fakeMs++; }
uint32_t micros() { return fakeMs * 1000U; }
void delay(uint32_t ms) { fakeMs += ms; }
void delayMicroseconds(uint32_t) {}
