#pragma once
#include <stddef.h>
#include <stdint.h>

// Heap string with the Arduino String interface used by the Livo firmware.
// Independent implementation; semantics follow the Arduino reference:
// out-of-range indices yield empty results rather than faults, toInt()
// parses like atol(), numbers format like Print. Allocation failures leave
// the string empty (invalid) instead of throwing.
//
// Hot paths should move to fixed buffers (porting plan, milestone 8).
class String {
public:
    String(const char* text = "");
    String(const String& other);
    String(String&& other) noexcept;
    explicit String(char c);
    explicit String(unsigned char value, unsigned char base = 10);
    explicit String(int value, unsigned char base = 10);
    explicit String(unsigned int value, unsigned char base = 10);
    explicit String(long value, unsigned char base = 10);
    explicit String(unsigned long value, unsigned char base = 10);
    explicit String(float value, unsigned char decimals = 2);
    explicit String(double value, unsigned char decimals = 2);
    ~String();

    String& operator=(const String& other);
    String& operator=(String&& other) noexcept;
    String& operator=(const char* text);

    size_t length() const { return length_; }
    const char* c_str() const { return buffer_ ? buffer_ : ""; }
    bool reserve(size_t capacity);

    char charAt(size_t index) const { return index < length_ ? buffer_[index] : '\0'; }
    char operator[](size_t index) const { return charAt(index); }
    char& operator[](size_t index);
    void setCharAt(size_t index, char c) { if (index < length_) buffer_[index] = c; }

    bool concat(const char* text, size_t count);
    bool concat(const char* text);
    bool concat(const String& other) { return concat(other.c_str(), other.length_); }
    bool concat(char c) { return concat(&c, 1); }
    bool concat(unsigned char value) { return concat(String(value)); }
    bool concat(int value) { return concat(String(value)); }
    bool concat(unsigned int value) { return concat(String(value)); }
    bool concat(long value) { return concat(String(value)); }
    bool concat(unsigned long value) { return concat(String(value)); }
    bool concat(float value) { return concat(String(value)); }
    bool concat(double value) { return concat(String(value)); }
    template <typename T> String& operator+=(const T& value) { concat(value); return *this; }

    bool equals(const String& other) const;
    bool equals(const char* text) const;
    bool equalsIgnoreCase(const String& other) const;
    int compareTo(const String& other) const;
    bool operator==(const String& other) const { return equals(other); }
    bool operator==(const char* text) const { return equals(text); }
    bool operator!=(const String& other) const { return !equals(other); }
    bool operator!=(const char* text) const { return !equals(text); }
    bool operator<(const String& other) const { return compareTo(other) < 0; }
    bool operator>(const String& other) const { return compareTo(other) > 0; }

    bool startsWith(const String& prefix) const { return startsWith(prefix, 0); }
    bool startsWith(const String& prefix, size_t offset) const;
    bool endsWith(const String& suffix) const;

    int indexOf(char c, size_t from = 0) const;
    int indexOf(const String& text, size_t from = 0) const;
    int lastIndexOf(char c) const;
    int lastIndexOf(const String& text) const;
    String substring(size_t from) const { return substring(from, length_); }
    String substring(size_t from, size_t to) const;

    void trim();
    void toUpperCase();
    void toLowerCase();
    void remove(size_t index, size_t count = (size_t)-1);
    void replace(char find, char with);

    long toInt() const;
    float toFloat() const;
    double toDouble() const;

private:
    bool assign(const char* text, size_t count);
    void release();
    char* buffer_ = nullptr;
    size_t length_ = 0, capacity_ = 0;
};

String operator+(const String& left, const String& right);
String operator+(const String& left, const char* right);
String operator+(const char* left, const String& right);
String operator+(const String& left, char right);
String operator+(const String& left, int right);
String operator+(const String& left, unsigned int right);
String operator+(const String& left, long right);
String operator+(const String& left, unsigned long right);
String operator+(const String& left, float right);
String operator+(const String& left, double right);
inline bool operator==(const char* left, const String& right) { return right.equals(left); }
inline bool operator!=(const char* left, const String& right) { return !right.equals(left); }
