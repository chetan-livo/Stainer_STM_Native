#pragma once
#include <stdint.h>

// Independent watchdog (IWDG, ~32 kHz LSI). Once started it cannot be
// stopped until reset, and it keeps running while a debugger halts the core
// unless DBGMCU freezes it (done in Debug builds).
namespace Watchdog {
bool begin(uint32_t timeoutMs);   // 1 ms .. ~32 s; false if out of range
void reload();
bool started();
// Reset cause captured at boot; call resetCauseCapture() once before
// anything else clears RCC->CSR.
void resetCauseCapture();
bool lastResetWasWatchdog();
uint32_t lastResetFlags();        // raw RCC->CSR reset flags
}
