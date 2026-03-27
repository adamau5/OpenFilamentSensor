/**
 * Arduino Environment Mocks
 *
 * Provides mock implementations of Arduino functions and classes
 * for desktop unit testing.
 */

#ifndef ARDUINO_MOCKS_H
#define ARDUINO_MOCKS_H

#include <cstdint>
#include <cstring>
#include <cstdlib>

// Include test_mocks.h for millis() and time functions
#include "test_mocks.h"

// Arduino type definitions
typedef uint8_t byte;
typedef bool boolean;

// Arduino flash string helper (for PROGMEM strings)
class __FlashStringHelper;
#define PSTR(s) (s)
#undef F
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))

// Arduino constants
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Mock delay function
inline void delay(unsigned long ms) {
    _mockMillis += ms;
}

inline void delayMicroseconds(unsigned int us) {
    // Approximate - just advance time slightly
    if (us >= 1000) {
        _mockMillis += us / 1000;
    }
}

// Mock Arduino String class
class String {
public:
    String() : data(nullptr), len(0), capacity(0) {}

    String(const char* str) {
        if (str) {
            len = strlen(str);
            capacity = len + 1;
            data = new char[capacity];
            strcpy(data, str);
        } else {
            data = nullptr;
            len = 0;
            capacity = 0;
        }
    }

    String(const String& other) {
        if (other.data) {
            len = other.len;
            capacity = len + 1;
            data = new char[capacity];
            strcpy(data, other.data);
        } else {
            data = nullptr;
            len = 0;
            capacity = 0;
        }
    }

    String(int value) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", value);
        len = strlen(buf);
        capacity = len + 1;
        data = new char[capacity];
        strcpy(data, buf);
    }

    String(float value, int decimals = 2) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", decimals, value);
        len = strlen(buf);
        capacity = len + 1;
        data = new char[capacity];
        strcpy(data, buf);
    }

    ~String() {
        delete[] data;
    }

    String& operator=(const String& other) {
        if (this != &other) {
            delete[] data;
            if (other.data) {
                len = other.len;
                capacity = len + 1;
                data = new char[capacity];
                strcpy(data, other.data);
            } else {
                data = nullptr;
                len = 0;
                capacity = 0;
            }
        }
        return *this;
    }

    String& operator=(const char* str) {
        delete[] data;
        if (str) {
            len = strlen(str);
            capacity = len + 1;
            data = new char[capacity];
            strcpy(data, str);
        } else {
            data = nullptr;
            len = 0;
            capacity = 0;
        }
        return *this;
    }

    const char* c_str() const { return data ? data : ""; }

    bool isEmpty() const { return len == 0; }

    size_t length() const { return len; }

    operator bool() const { return data != nullptr && len > 0; }

    bool operator==(const String& other) const {
        if (len != other.len) return false;
        if (data == nullptr && other.data == nullptr) return true;
        if (data == nullptr || other.data == nullptr) return false;
        return strcmp(data, other.data) == 0;
    }

    bool operator==(const char* str) const {
        if (str == nullptr) return data == nullptr || len == 0;
        if (data == nullptr) return strlen(str) == 0;
        return strcmp(data, str) == 0;
    }

    bool operator!=(const String& other) const {
        return !(*this == other);
    }

    bool operator!=(const char* str) const {
        return !(*this == str);
    }

    String& operator+=(const char* str) {
        if (!str) return *this;
        size_t addLen = strlen(str);
        size_t newLen = len + addLen;
        char* newData = new char[newLen + 1];
        if (data) {
            strcpy(newData, data);
            strcat(newData, str);
        } else {
            strcpy(newData, str);
        }
        delete[] data;
        data = newData;
        len = newLen;
        capacity = newLen + 1;
        return *this;
    }

    String& operator+=(const String& other) {
        return *this += other.c_str();
    }

    String operator+(const char* str) const {
        String result(*this);
        result += str;
        return result;
    }

    String operator+(const String& other) const {
        return *this + other.c_str();
    }

    char charAt(size_t index) const {
        if (index >= len) return 0;
        return data[index];
    }

    char operator[](size_t index) const {
        return charAt(index);
    }

    int indexOf(char c) const {
        if (!data) return -1;
        for (size_t i = 0; i < len; i++) {
            if (data[i] == c) return (int)i;
        }
        return -1;
    }

    int indexOf(const char* str) const {
        if (!data || !str) return -1;
        char* found = strstr(data, str);
        if (!found) return -1;
        return (int)(found - data);
    }

    String substring(size_t start) const {
        if (start >= len) return String();
        return String(data + start);
    }

    String substring(size_t start, size_t end) const {
        if (start >= len) return String();
        if (end > len) end = len;
        if (start >= end) return String();

        size_t subLen = end - start;
        char* buf = new char[subLen + 1];
        strncpy(buf, data + start, subLen);
        buf[subLen] = '\0';
        String result(buf);
        delete[] buf;
        return result;
    }

    void trim() {
        if (!data || len == 0) return;

        size_t start = 0;
        while (start < len && (data[start] == ' ' || data[start] == '\t' ||
                                data[start] == '\n' || data[start] == '\r')) {
            start++;
        }

        size_t end = len;
        while (end > start && (data[end-1] == ' ' || data[end-1] == '\t' ||
                               data[end-1] == '\n' || data[end-1] == '\r')) {
            end--;
        }

        if (start > 0 || end < len) {
            size_t newLen = end - start;
            memmove(data, data + start, newLen);
            data[newLen] = '\0';
            len = newLen;
        }
    }

    int toInt() const {
        if (!data) return 0;
        return atoi(data);
    }

    float toFloat() const {
        if (!data) return 0.0f;
        return (float)atof(data);
    }

private:
    char* data;
    size_t len;
    size_t capacity;
};

// Serial mock (for debug output compatibility)
class MockSerial {
public:
    void begin(long baud) {}
    void print(const char* str) {}
    void print(int val) {}
    void print(float val) {}
    void println(const char* str = "") {}
    void println(int val) {}
    void println(float val) {}
    void printf(const char* fmt, ...) {}
    bool available() { return false; }
    int read() { return -1; }
};

extern MockSerial Serial;

// Pin functions (no-op for testing)
inline void pinMode(int pin, int mode) {}
inline int digitalRead(int pin) { return LOW; }
inline void digitalWrite(int pin, int value) {}
inline int analogRead(int pin) { return 0; }
inline void analogWrite(int pin, int value) {}

// Random functions
inline long random(long max) { return rand() % max; }
inline long random(long min, long max) { return min + (rand() % (max - min)); }

// Min/max macros (Arduino style)
// Note: These are NOT defined here to avoid conflicts with C++ std library
// when mixing mocks with real Arduino headers. The Arduino framework
// provides these macros in its own headers.
// 
// #ifndef min
// #define min(a,b) ((a)<(b)?(a):(b))
// #endif
// 
// #ifndef max
// #define max(a,b) ((a)>(b)?(a):(b))
// #endif

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

// Map function
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // ARDUINO_MOCKS_H
