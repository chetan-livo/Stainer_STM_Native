// Differential test: Platform String against the Arduino String the firmware
// was built with (STM32duino 3.0.0 / ArduinoCore-API, LGPL-2.1, compiled from
// the local core into this test program only; it is in namespace arduino).
//
// Random operation sequences biased toward the command parser's patterns,
// plus real Livo command lines through the parser's split/trim/case/toInt
// steps. After every operation the contents and results must be identical.
#define String ArduinoString   // see arduino_string_ref.cpp
#include "api/String.h"
#undef String
#include "WString.h"
#include "check.h"
#include <initializer_list>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

using Ref = arduino::ArduinoString;
using Mine = ::String;

static uint32_t rng = 12345;
static uint32_t next() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static const char* const fragments[] = {
    "", " ", "  ", "\t", "\r\n", "G;", "N;", "M;", ";", ",", "=", "X", "XS", "GXHO", "PROFILE MG",
    "FEED 120", "REQ:", "BS2 T 1A 2B 3C 4D", "DLOCK 1", "-", "0", "7", "42", "-15", "1200", "abc",
    "x12abc", "  pad  ", "FLASH G 4096 0x1A2B", "M14ML", "S2MIX", "DCW1=255", "LOOPSTAT",
};

static bool same(const Ref& r, const Mine& m)
{
    return r.length() == m.length() && strcmp(r.c_str(), m.c_str()) == 0;
}

static int mismatches = 0;
static void expectSame(const Ref& r, const Mine& m, const char* op)
{
    ++checks_run;
    if (!same(r, m)) {
        ++checks_failed;
        if (++mismatches <= 10) printf("FAIL after %s: ref \"%s\" mine \"%s\"\n", op, r.c_str(), m.c_str());
    }
}
static void expectEq(long a, long b, const char* op)
{
    ++checks_run;
    if (a != b) { ++checks_failed; if (++mismatches <= 10) printf("FAIL %s: ref %ld mine %ld\n", op, a, b); }
}

int main()
{
    // Number and float formatting.
    const long ints[] = {0, 1, -1, 9, 10, 255, 256, -255, 2147483647L, -2147483647L - 1, 4095, 123456789};
    for (long v : ints) {
        expectSame(Ref(v), Mine(v), "String(long)");
        expectSame(Ref((unsigned long)v, 16), Mine((unsigned long)v, 16), "String(ulong,HEX)");
        expectSame(Ref((int)v), Mine((int)v), "String(int)");
        expectSame(Ref((unsigned char)v), Mine((unsigned char)v), "String(uchar)");
    }
    const float floats[] = {0.0f, 1.5f, -1.5f, 3.14159f, 0.125f, 100.0f, 0.004f, 2.675f, 12345.678f, -0.5f};
    for (float f : floats)
        for (unsigned char d : {0, 1, 2, 3})
            expectSame(Ref(f, d), Mine(f, d), "String(float,dec)");

    // Random sequences.
    for (int round = 0; round < 20000; ++round) {
        Ref r = "";
        Mine m = "";
        const int ops = 1 + (int)(next() % 12);
        for (int k = 0; k < ops; ++k) {
            const char* frag = fragments[next() % (sizeof(fragments) / sizeof(fragments[0]))];
            const int len = (int)r.length();
            const unsigned a = len ? next() % (unsigned)(len + 2) : 0, b = len ? next() % (unsigned)(len + 2) : 0;
            switch (next() % 14) {
            case 0: r += frag; m += frag; expectSame(r, m, "+= text"); break;
            case 1: { long n = (long)(next() % 4000) - 2000; r += n; m += n; expectSame(r, m, "+= long"); break; }
            case 2: r += frag[0] ? frag[0] : 'q'; m += frag[0] ? frag[0] : 'q'; expectSame(r, m, "+= char"); break;
            case 3: r.trim(); m.trim(); expectSame(r, m, "trim"); break;
            case 4: r.toUpperCase(); m.toUpperCase(); expectSame(r, m, "toUpperCase"); break;
            case 5: { Ref rs = r.substring(a); Mine ms = m.substring(a); expectSame(rs, ms, "substring(a)"); break; }
            case 6: { Ref rs = r.substring(a, b); Mine ms = m.substring(a, b); expectSame(rs, ms, "substring(a,b)"); break; }
            case 7: expectEq(r.indexOf(frag[0] ? frag[0] : ';'), m.indexOf(frag[0] ? frag[0] : ';'), "indexOf(char)");
                    expectEq(r.indexOf(frag), m.indexOf(frag), "indexOf(text)");
                    expectEq(r.indexOf(';', a), m.indexOf(';', a), "indexOf(char,from)");
                    expectEq(r.lastIndexOf(' '), m.lastIndexOf(' '), "lastIndexOf"); break;
            case 8: expectEq(r.startsWith(frag), m.startsWith(frag), "startsWith");
                    expectEq(r.endsWith(frag), m.endsWith(frag), "endsWith");
                    // Arduino's startsWith(s, offset) over-reads when s is longer than
                    // the string (unsigned underflow in its bounds check; found by ASan
                    // here). Compare only where the reference is well defined; ours
                    // returns false outside it.
                    if (strlen(frag) <= r.length() && a <= r.length())
                        expectEq(r.startsWith(frag, a), m.startsWith(frag, a), "startsWith(offset)");
                    else expectEq(0, m.startsWith(frag, a) && a > m.length(), "startsWith(offset) outside");
                    break;
            case 9: expectEq(r.toInt(), m.toInt(), "toInt");
                    expectEq(r.equals(frag), m.equals(frag), "equals");
                    expectEq(r == frag, m == frag, "=="); break;
            case 10: expectEq((long)(unsigned char)r.charAt(a), (long)(unsigned char)m.charAt(a), "charAt");
                     expectEq((long)r.length(), (long)m.length(), "length"); break;
            case 11: { r = r.substring(a) + frag; m = m.substring(a) + frag; expectSame(r, m, "substring+concat"); break; }
            case 12: { Ref t = Ref(frag) + r; Mine u = Mine(frag) + m; r = t; m = u; expectSame(r, m, "text+string"); break; }
            case 13: r.remove(a, b); m.remove(a, b); expectSame(r, m, "remove"); break;
            }
        }
    }

    // Real command lines through the parser's steps.
    const char* const commands[] = {
        "G; GXHO 1200", "  N; PROFILE MG  \r", "M;FEED 120", "FEED 90", "REQ:DSPS1", "X1000,XS5000,XA200",
        "DCW1=255,DCW2=0", "PROFILE 2", "FLASH G 131072 0x1A2B3C4D", "BS2 A 1 2F 0 9C3D", "DTEST 0000ABCD",
        "PARA CHECK", "M14ML=150", "x12abc", "", ";", "G;", "LOOPSTAT", "STEPISR 1",
    };
    for (const char* line : commands) {
        Ref r = line; Mine m = line;
        r.trim(); m.trim();
        expectSame(r, m, "cmd trim");
        Ref ru = r; Mine mu = m; ru.toUpperCase(); mu.toUpperCase();
        expectSame(ru, mu, "cmd upper");
        const int rs = r.indexOf(';'), ms = m.indexOf(';');
        expectEq(rs, ms, "cmd indexOf ;");
        Ref rb = r.substring(rs + 1); Mine mb = m.substring(ms + 1);
        rb.trim(); mb.trim();
        expectSame(rb, mb, "cmd body");
        const int rsp = rb.indexOf(' '), msp = mb.indexOf(' ');
        expectEq(rb.substring(rsp + 1).toInt(), mb.substring(msp + 1).toInt(), "cmd value");
        const int req = r.indexOf('='), meq = m.indexOf('=');
        expectSame(r.substring(0, req), m.substring(0, meq), "cmd key");
        expectSame(r.substring(req + 1), m.substring(meq + 1), "cmd value text");
    }
    return report("String vs Arduino String (STM32duino 3.0.0)");
}
