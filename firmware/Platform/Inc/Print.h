#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum : uint8_t { DEC = 10, HEX = 16, OCT = 8, BIN = 2 };

// Arduino-compatible text output. Sinks implement write(); everything else
// formats without printf or heap allocation.
class Print {
public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t* buffer, size_t size);
    virtual int availableForWrite() { return 0; }
    virtual void flush() {}

    size_t write(const char* s) { return s ? write((const uint8_t*)s, strlen(s)) : 0; }
    size_t write(const char* buffer, size_t size) { return write((const uint8_t*)buffer, size); }

    size_t print(const char* s) { return write(s); }
    size_t print(char c) { return write((uint8_t)c); }
    size_t print(unsigned char n, int base = DEC) { return printUnsigned(n, base); }
    size_t print(int n, int base = DEC) { return printSigned(n, base); }
    size_t print(unsigned int n, int base = DEC) { return printUnsigned(n, base); }
    size_t print(long n, int base = DEC) { return printSigned(n, base); }
    size_t print(unsigned long n, int base = DEC) { return printUnsigned(n, base); }
    size_t print(long long n, int base = DEC) { return printSigned(n, base); }
    size_t print(unsigned long long n, int base = DEC) { return printUnsigned(n, base); }
    size_t print(double value, int digits = 2);

    size_t println() { return write("\r\n"); }
    template <typename T> size_t println(const T& value) { size_t n = print(value); return n + println(); }
    template <typename T> size_t println(const T& value, int format) { size_t n = print(value, format); return n + println(); }

private:
    size_t printUnsigned(unsigned long long n, int base);
    size_t printSigned(long long n, int base);
};

// Arduino-compatible byte input stream.
class Stream : public Print {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;

    void setTimeout(uint32_t ms) { timeoutMs = ms; }
    uint32_t getTimeout() const { return timeoutMs; }
    // Read up to `length` bytes, waiting at most the timeout between bytes.
    size_t readBytes(uint8_t* buffer, size_t length);
    size_t readBytes(char* buffer, size_t length) { return readBytes((uint8_t*)buffer, length); }

protected:
    int timedRead();
    uint32_t timeoutMs = 1000;
};
