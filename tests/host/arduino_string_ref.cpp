// Builds the Arduino String (ArduinoCore-API, from the local STM32duino core)
// under the name arduino::ArduinoString so it can coexist with Platform
// String in one test program. Test-only; never part of the firmware.
#define String ArduinoString
#include "api/String.cpp"
