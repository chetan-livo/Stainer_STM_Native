// Differential test: Platform Print number/float formatting against the
// Arduino Print the firmware used (STM32duino 3.0.0 ArduinoCore-API).
#define String ArduinoString
#define Print ArduinoPrint
#include "api/Print.h"
#undef Print
#undef String
#undef DEC   // Arduino macros; Platform Print.h declares them as enumerators
#undef HEX
#undef OCT
#undef BIN
#include "Print.h"
#include "check.h"
#include <string>

struct RefSink : arduino::ArduinoPrint {
    std::string text;
    size_t write(uint8_t c) override { text += (char)c; return 1; }
};
struct MySink : ::Print {
    std::string text;
    size_t write(uint8_t c) override { text += (char)c; return 1; }
    using Print::write;
};

static uint32_t rng = 777;
static uint32_t next() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

int main()
{
    RefSink r; MySink m;
    const long ints[] = {0, 1, -1, 7, 10, 255, -255, 4095, 65535, -32768, 2147483647L, -2147483647L - 1};
    for (long v : ints)
        for (int base : {10, 16, 2, 8}) {
            r.print(v, base); m.print(v, base); r.print(' '); m.print(' ');
            r.print((unsigned long)v, base); m.print((unsigned long)v, base); r.print(' '); m.print(' ');
            r.print((int)v, base); m.print((int)v, base); r.println(); m.println();
        }
    for (int i = 0; i < 20000; ++i) {
        const double v = ((int32_t)next() % 2000000) / 997.0;
        const int digits = (int)(next() % 5);
        r.print(v, digits); m.print(v, digits); r.print(','); m.print(',');
        r.print((float)v); m.print((float)v); r.print(';'); m.print(';');
        const uint8_t b = (uint8_t)next();
        r.print(b, HEX); m.print(b, HEX); r.print(b); m.print(b); r.println(); m.println();
    }
    r.print("PROFILE"); m.print("PROFILE"); r.println("=MG"); m.println("=MG");
    CHECK(r.text.size() == m.text.size());
    size_t first = 0;
    while (first < r.text.size() && first < m.text.size() && r.text[first] == m.text[first]) ++first;
    if (r.text != m.text)
        printf("first difference at %zu: ref \"%s\" mine \"%s\"\n", first,
               r.text.substr(first > 20 ? first - 20 : 0, 60).c_str(), m.text.substr(first > 20 ? first - 20 : 0, 60).c_str());
    CHECK(r.text == m.text);
    printf("compared %zu bytes of output\n", r.text.size());
    return report("Print vs Arduino Print (STM32duino 3.0.0)");
}
