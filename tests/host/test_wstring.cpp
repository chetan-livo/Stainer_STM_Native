// Host test for Platform WString (the Arduino String subset the firmware uses).
#include "WString.h"
#include "check.h"
#include <string.h>

// Keeps the String alive for the comparison (c_str() of a temporary dangles).
#define CHECK_S(expr, expected) do { const String held_ = (expr); CHECK_STR(held_.c_str(), expected); } while (0)

int main()
{
    // Construction and numbers (Arduino formats HEX in lower case).
    CHECK_S(String("abc"), "abc");
    CHECK_S(String(42), "42");
    CHECK_S(String(-42L), "-42");
    CHECK_S(String(255, 16), "ff");
    CHECK_S(String(0xDEADBEEFUL, 16), "deadbeef");
    CHECK_S(String((unsigned char)7), "7");
    CHECK_S(String(3.14159f), "3.14");
    CHECK_S(String(2.5, 3), "2.500");
    CHECK_S(String(-0.125, 2), "-0.13");
    CHECK_S(String('x'), "x");

    // Protocol-parser patterns: trim, case, prefix, split, toInt.
    String line = "  G; GXHO 1200  \r\n";
    line.trim();
    CHECK_STR(line.c_str(), "G; GXHO 1200");
    CHECK(line.startsWith("G;"));
    CHECK(!line.startsWith("N;"));
    CHECK(line.startsWith("GXHO", 3));
    CHECK(line.endsWith("1200"));
    const int sep = line.indexOf(';');
    CHECK(sep == 1);
    String body = line.substring(sep + 1);
    body.trim();
    CHECK_STR(body.c_str(), "GXHO 1200");
    CHECK(body.substring(body.indexOf(' ') + 1).toInt() == 1200);
    String upper = "profile mg";
    upper.toUpperCase();
    CHECK(upper.equals("PROFILE MG"));
    CHECK(upper == "PROFILE MG" && "PROFILE MG" == upper && upper != "X");
    CHECK(String("X12abc").substring(1).toInt() == 12);   // atol semantics
    CHECK(String("-7").toInt() == -7);
    CHECK(String("").toInt() == 0);
    CHECK(String("FLASH G 4096 0x1A").lastIndexOf(' ') == 12);
    CHECK(String("a=b=c").lastIndexOf("=") == 3);
    CHECK(String("KEY=value").indexOf("=") == 3);
    CHECK(String("abc").indexOf('z') == -1);
    CHECK(String("abc").indexOf('a', 5) == -1);

    // Out-of-range access never faults.
    String s = "abc";
    CHECK(s.charAt(10) == '\0');
    CHECK(s.substring(10).length() == 0);
    CHECK_S(s.substring(2, 1), "b");       // swapped bounds
    CHECK_S(s.substring(1, 99), "bc");
    CHECK(!s.startsWith("abcd"));
    CHECK(!s.endsWith("zabc"));

    // Concatenation, including self-append and growth.
    String built;
    for (int i = 0; i < 100; ++i) built += (char)('a' + i % 26);
    CHECK(built.length() == 100);
    built += built;
    CHECK(built.length() == 200 && built.charAt(126) == built.charAt(26));
    String row = String("DROW ") + 7 + " " + String(0x2AUL, 16) + " PASS";
    CHECK_STR(row.c_str(), "DROW 7 2a PASS");
    String self = "xyz";
    self = self.c_str() + 1;                           // assign from own buffer
    CHECK_STR(self.c_str(), "yz");

    // Copies are independent; moves leave the source empty.
    String a = "one", b = a;
    b += "!";
    CHECK_STR(a.c_str(), "one");
    String moved = static_cast<String&&>(b);
    CHECK_STR(moved.c_str(), "one!");
    CHECK(b.length() == 0 && b.c_str()[0] == '\0');

    // Editing helpers.
    String e = "a,b,,c";
    e.replace(',', ';');
    CHECK_STR(e.c_str(), "a;b;;c");
    e.remove(3, 1);
    CHECK_STR(e.c_str(), "a;b;c");
    CHECK(String("Abc").equalsIgnoreCase(String("aBC")));
    CHECK(String("abc") < String("abd"));

    return report("wstring");
}
