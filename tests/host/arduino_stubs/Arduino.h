#pragma once
// Minimal Arduino API so third-party Arduino libraries (TMCStepper,
// AccelStepper) compile on the host as references for equivalence tests.
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
typedef bool boolean;
typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define MSBFIRST 1
#define LSBFIRST 0
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
// Renamed so they never collide with the Platform layer's own definitions.
#define pinMode arduino_pinMode
#define digitalWrite arduino_digitalWrite
#define digitalRead arduino_digitalRead
#define millis arduino_millis
#define micros arduino_micros
#define delay arduino_delay
#define delayMicroseconds arduino_delayMicroseconds
#define yield arduino_yield
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield(void);

// Renamed so it can coexist with the Platform Stream in one test binary.
#define Stream ArduinoStream
class ArduinoStream {
public:
    virtual ~ArduinoStream() {}
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t* b, size_t n) { size_t i = 0; for (; i < n; ++i) write(b[i]); return i; }
    virtual void flush() {}
};
