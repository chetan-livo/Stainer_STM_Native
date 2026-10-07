#include "Print.h"
#include "Timebase.h"
#include <math.h>

size_t Print::write(const uint8_t* buffer, size_t size)
{
    size_t written = 0;
    while (size--) {
        if (!write(*buffer++)) break;
        ++written;
    }
    return written;
}

size_t Print::printUnsigned(unsigned long long n, int base)
{
    if (base < 2 || base > 16) base = 10;
    char text[65];
    char* p = &text[sizeof(text) - 1];
    *p = '\0';
    do {
        const unsigned digit = (unsigned)(n % (unsigned)base);
        *--p = (char)(digit < 10 ? '0' + digit : 'A' + digit - 10);
        n /= (unsigned)base;
    } while (n);
    return write(p);
}

size_t Print::printSigned(long long n, int base)
{
    // Arduino prints negative numbers in non-decimal bases as unsigned
    // two's complement of the argument's own width; long is 32 bits here.
    if (base != DEC) return printUnsigned((unsigned long long)(uint32_t)n, base);
    if (n >= 0) return printUnsigned((unsigned long long)n, base);
    return write('-') + printUnsigned((unsigned long long)(-(n + 1)) + 1U, base);
}

size_t Print::print(double value, int digits)
{
    if (isnan(value)) return write("nan");
    if (isinf(value)) return write("inf");
    if (value > 4294967040.0 || value < -4294967040.0) return write("ovf");
    size_t n = 0;
    if (value < 0.0) { n += write('-'); value = -value; }
    if (digits < 0) digits = 0;
    double rounding = 0.5;
    for (int i = 0; i < digits; ++i) rounding /= 10.0;
    value += rounding;
    unsigned long integer = (unsigned long)value;
    double remainder = value - (double)integer;
    n += printUnsigned(integer, DEC);
    if (digits > 0) n += write('.');
    while (digits-- > 0) {
        remainder *= 10.0;
        const unsigned digit = (unsigned)remainder;
        n += write((uint8_t)('0' + digit));
        remainder -= digit;
    }
    return n;
}

int Stream::timedRead()
{
    const uint32_t start = millis();
    do {
        const int c = read();
        if (c >= 0) return c;
    } while (millis() - start < timeoutMs);
    return -1;
}

size_t Stream::readBytes(uint8_t* buffer, size_t length)
{
    size_t count = 0;
    while (count < length) {
        const int c = timedRead();
        if (c < 0) break;
        buffer[count++] = (uint8_t)c;
    }
    return count;
}
