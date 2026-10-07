// Builds the Arduino Print class (ArduinoCore-API) as arduino::ArduinoPrint
// next to Platform Print for differential tests. Test-only.
#define String ArduinoString
#define Print ArduinoPrint
// Print::printf casts `this` to int for vdprintf; unused here and not
// representable on a 64-bit host, so the call (and the cast) is dropped.
#define vdprintf(fd, format, args) 0
#include "api/Print.cpp"
