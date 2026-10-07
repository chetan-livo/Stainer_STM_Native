#pragma once
// STM32duino IWatchdog interface (magazine/Hall code) on Platform Watchdog.
#include "Arduino.h"

class IWatchdogClass {
public:
    void begin(uint32_t timeoutUs) { Watchdog::begin(timeoutUs / 1000U); }
    void reload() { Watchdog::reload(); }
    bool isEnabled() { return Watchdog::started(); }
    bool isReset(bool clear = false) { (void)clear; return Watchdog::lastResetWasWatchdog(); }
};
extern IWatchdogClass IWatchdog;
