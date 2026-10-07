#pragma once
#include <stdint.h>

// analogRead(): each ADC (ADC1, plus ADC3 for the F446 port F inputs) runs a
// continuous scan of every pin that has been read, written by circular DMA.
// Reads return the latest sample (at most one scan, about 20 us, old) instead
// of stalling for a conversion. A pin's first read adds it to the scan and
// waits for one fresh pass.
//
// Conversion settings match STM32duino on F4: ADC clock PCLK2/4, 15-cycle
// sampling, 12-bit, right aligned, so raw values and calibrations carry over.
// As in STM32duino, reading a pin puts it into analog mode (no pull).
int analogRead(uint8_t pin);
void analogReadResolution(int bits); // result scaling only; conversion is 12-bit

// PWM output with STM32duino semantics: 1 kHz default, 8-bit values, CCR =
// (ARR + 1) * value / max, so 255 is 100 %. Pins without a timer channel are
// driven digitally (value >= half scale is HIGH). Changing the frequency
// retunes the whole timer, as in STM32duino.
void analogWrite(uint8_t pin, int value);
void analogWriteResolution(int bits);
void analogWriteFrequency(uint32_t hz);
// Explicit PWM for fans and similar loads; duty in percent (0..100).
bool pwmWritePercent(uint8_t pin, uint32_t frequencyHz, uint32_t percent);
void pwmStop(uint8_t pin); // channel off, pin driven LOW
