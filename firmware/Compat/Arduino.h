#pragma once
// Arduino API surface used by the ported Livo application, mapped onto the
// native platform. New code should include Platform.h directly; this header
// exists so the application could move across with minimal edits.
#include "Platform.h"
#include "stm32f4xx_hal.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <initializer_list>  // STM32duino's Arduino.h provided it implicitly
#include <new>

typedef uint8_t byte;
typedef bool boolean;

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define PROGMEM
#define pgm_read_float_near(address) (*(const float*)(address))
#define pgm_read_byte(address) (*(const uint8_t*)(address))
#define pgm_read_word(address) (*(const uint16_t*)(address))
#define F(text) (text)

// Arduino's min/max are type-tolerant templates (ArduinoCore-API).
template <class T, class L> inline auto min(const T& a, const L& b) -> decltype((b < a) ? b : a) { return (b < a) ? b : a; }
template <class T, class L> inline auto max(const T& a, const L& b) -> decltype((b < a) ? b : a) { return (a < b) ? b : a; }
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
inline long map(long x, long inMin, long inMax, long outMin, long outMax)
{
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
inline bool isDigit(int c) { return isdigit(c) != 0; }
inline bool isAlpha(int c) { return isalpha(c) != 0; }
inline bool isSpace(int c) { return isspace(c) != 0; }
typedef unsigned int word;   // Arduino type (used as a cast by the DHT driver)
inline uint32_t clockCyclesPerMicrosecond() { return SystemCoreClock / 1000000UL; }
#define microsecondsToClockCycles(us) ((us) * clockCyclesPerMicrosecond())

// Marks native builds in the ID reply ("BUILD:NATIVE <date> <time>") without
// changing the FW: version the ESP32 compares (Constants.h honours this macro).
#ifndef LIVO_BUILD_STAMP
#define LIVO_BUILD_STAMP "NATIVE " __DATE__ " " __TIME__
#endif
