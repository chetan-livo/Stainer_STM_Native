#include "WString.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace {
// Formats into `out` (at least 66 bytes); returns the length.
size_t formatUnsigned(unsigned long value, unsigned base, char* out)
{
    if (base < 2 || base > 16) base = 10;
    char digits[33];
    size_t n = 0;
    do {
        const unsigned d = (unsigned)(value % base);
        digits[n++] = (char)(d < 10 ? '0' + d : 'a' + d - 10); // Arduino String uses lower case
        value /= base;
    } while (value);
    for (size_t i = 0; i < n; ++i) out[i] = digits[n - 1 - i];
    out[n] = '\0';
    return n;
}

size_t formatSigned(long value, unsigned base, char* out)
{
    if (base == 10 && value < 0) {
        out[0] = '-';
        return 1 + formatUnsigned(0UL - (unsigned long)value, 10, out + 1);
    }
    return formatUnsigned((unsigned long)value, base, out);
}

size_t formatFloat(double value, unsigned decimals, char* out, size_t size)
{
    if (isnan(value)) { strcpy(out, "nan"); return 3; }
    if (isinf(value)) { strcpy(out, "inf"); return 3; }
    if (decimals > 9) decimals = 9;
    size_t n = 0;
    if (value < 0) { out[n++] = '-'; value = -value; }
    double rounding = 0.5;
    for (unsigned i = 0; i < decimals; ++i) rounding /= 10.0;
    value += rounding;
    if (value >= 4294967296.0) { strcpy(out + n, "ovf"); return n + 3; }
    unsigned long integer = (unsigned long)value;
    double fraction = value - (double)integer;
    n += formatUnsigned(integer, 10, out + n);
    if (decimals) out[n++] = '.';
    while (decimals-- && n + 1 < size) {
        fraction *= 10.0;
        const unsigned d = (unsigned)fraction;
        out[n++] = (char)('0' + d);
        fraction -= d;
    }
    out[n] = '\0';
    return n;
}
}

String::String(const char* text) { assign(text ? text : "", text ? strlen(text) : 0); }
String::String(const String& other) { assign(other.c_str(), other.length_); }
String::String(String&& other) noexcept
    : buffer_(other.buffer_), length_(other.length_), capacity_(other.capacity_)
{
    other.buffer_ = nullptr; other.length_ = other.capacity_ = 0;
}
String::String(char c) { assign(&c, 1); }
String::String(unsigned char value, unsigned char base) { char t[34]; assign(t, formatUnsigned(value, base, t)); }
String::String(int value, unsigned char base) { char t[34]; assign(t, formatSigned(value, base, t)); }
String::String(unsigned int value, unsigned char base) { char t[34]; assign(t, formatUnsigned(value, base, t)); }
String::String(long value, unsigned char base) { char t[34]; assign(t, formatSigned(value, base, t)); }
String::String(unsigned long value, unsigned char base) { char t[34]; assign(t, formatUnsigned(value, base, t)); }
String::String(float value, unsigned char decimals) { char t[32]; assign(t, formatFloat(value, decimals, t, sizeof(t))); }
String::String(double value, unsigned char decimals) { char t[32]; assign(t, formatFloat(value, decimals, t, sizeof(t))); }
String::~String() { release(); }

void String::release()
{
    free(buffer_);
    buffer_ = nullptr;
    length_ = capacity_ = 0;
}

bool String::reserve(size_t capacity)
{
    if (buffer_ && capacity <= capacity_) return true;
    char* grown = (char*)realloc(buffer_, capacity + 1);
    if (!grown) return false;
    if (!buffer_) grown[0] = '\0';
    buffer_ = grown;
    capacity_ = capacity;
    return true;
}

bool String::assign(const char* text, size_t count)
{
    if (!reserve(count)) { release(); return false; }
    memmove(buffer_, text, count);
    buffer_[count] = '\0';
    length_ = count;
    return true;
}

String& String::operator=(const String& other)
{
    if (this != &other) assign(other.c_str(), other.length_);
    return *this;
}

String& String::operator=(String&& other) noexcept
{
    if (this != &other) {
        release();
        buffer_ = other.buffer_; length_ = other.length_; capacity_ = other.capacity_;
        other.buffer_ = nullptr; other.length_ = other.capacity_ = 0;
    }
    return *this;
}

String& String::operator=(const char* text)
{
    if (!text) text = "";
    // `text` may point into this buffer (e.g. s = s.c_str() + 2); assign() uses memmove.
    assign(text, strlen(text));
    return *this;
}

char& String::operator[](size_t index)
{
    static char dummy;
    if (index >= length_) { dummy = '\0'; return dummy; }
    return buffer_[index];
}

bool String::concat(const char* text, size_t count)
{
    if (!text) return false;
    if (!count) return true;
    const size_t total = length_ + count;
    // Self-append: remember the offset in case reserve() moves the buffer.
    const bool inside = buffer_ && text >= buffer_ && text < buffer_ + length_;
    const size_t offset = inside ? (size_t)(text - buffer_) : 0;
    if (total > capacity_ && !reserve(total > 2 * capacity_ ? total : 2 * capacity_)) return false;
    memmove(buffer_ + length_, inside ? buffer_ + offset : text, count);
    length_ = total;
    buffer_[length_] = '\0';
    return true;
}

bool String::concat(const char* text) { return text && concat(text, strlen(text)); }

bool String::equals(const String& other) const
{
    return length_ == other.length_ && memcmp(c_str(), other.c_str(), length_) == 0;
}

bool String::equals(const char* text) const
{
    if (!text) return length_ == 0;
    return strcmp(c_str(), text) == 0;
}

bool String::equalsIgnoreCase(const String& other) const
{
    if (length_ != other.length_) return false;
    for (size_t i = 0; i < length_; ++i)
        if (tolower((unsigned char)buffer_[i]) != tolower((unsigned char)other.buffer_[i])) return false;
    return true;
}

int String::compareTo(const String& other) const { return strcmp(c_str(), other.c_str()); }

bool String::startsWith(const String& prefix, size_t offset) const
{
    if (offset > length_ || prefix.length_ > length_ - offset) return false;
    return strncmp(c_str() + offset, prefix.c_str(), prefix.length_) == 0;
}

bool String::endsWith(const String& suffix) const
{
    if (suffix.length_ > length_) return false;
    return strcmp(c_str() + length_ - suffix.length_, suffix.c_str()) == 0;
}

int String::indexOf(char c, size_t from) const
{
    if (from >= length_) return -1;
    const char* hit = strchr(c_str() + from, c);
    return hit ? (int)(hit - c_str()) : -1;
}

int String::indexOf(const String& text, size_t from) const
{
    if (from > length_) return -1;
    const char* hit = strstr(c_str() + from, text.c_str());
    return hit ? (int)(hit - c_str()) : -1;
}

int String::lastIndexOf(char c) const
{
    const char* hit = length_ ? strrchr(c_str(), c) : nullptr;
    return hit ? (int)(hit - c_str()) : -1;
}

int String::lastIndexOf(const String& text) const
{
    if (text.length_ > length_) return -1;
    for (size_t i = length_ - text.length_ + 1; i-- > 0;)
        if (strncmp(c_str() + i, text.c_str(), text.length_) == 0) return (int)i;
    return -1;
}

String String::substring(size_t from, size_t to) const
{
    if (from > to) { const size_t t = from; from = to; to = t; }
    if (from >= length_) return String();
    if (to > length_) to = length_;
    String out;
    out.assign(c_str() + from, to - from);
    return out;
}

void String::trim()
{
    if (!length_) return;
    size_t begin = 0, end = length_;
    while (begin < end && isspace((unsigned char)buffer_[begin])) ++begin;
    while (end > begin && isspace((unsigned char)buffer_[end - 1])) --end;
    length_ = end - begin;
    if (begin) memmove(buffer_, buffer_ + begin, length_);
    buffer_[length_] = '\0';
}

void String::toUpperCase() { for (size_t i = 0; i < length_; ++i) buffer_[i] = (char)toupper((unsigned char)buffer_[i]); }
void String::toLowerCase() { for (size_t i = 0; i < length_; ++i) buffer_[i] = (char)tolower((unsigned char)buffer_[i]); }

void String::remove(size_t index, size_t count)
{
    if (index >= length_) return;
    if (count > length_ - index) count = length_ - index;
    memmove(buffer_ + index, buffer_ + index + count, length_ - index - count + 1);
    length_ -= count;
}

void String::replace(char find, char with)
{
    for (size_t i = 0; i < length_; ++i) if (buffer_[i] == find) buffer_[i] = with;
}

long String::toInt() const { return length_ ? atol(buffer_) : 0; }
float String::toFloat() const { return (float)toDouble(); }
double String::toDouble() const { return length_ ? atof(buffer_) : 0.0; }

String operator+(const String& l, const String& r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, const char* r) { String s(l); s.concat(r); return s; }
String operator+(const char* l, const String& r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, char r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, int r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, unsigned int r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, long r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, unsigned long r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, float r) { String s(l); s.concat(r); return s; }
String operator+(const String& l, double r) { String s(l); s.concat(r); return s; }
