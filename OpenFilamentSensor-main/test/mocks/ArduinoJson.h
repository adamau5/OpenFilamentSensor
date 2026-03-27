/**
 * ArduinoJson Mock Header
 *
 * This file intercepts #include <ArduinoJson.h> when -Imocks is in include path.
 * It includes the json_mocks.h and exposes types in global namespace.
 */

#ifndef ARDUINOJSON_H
#define ARDUINOJSON_H

#include "json_mocks.h"

// Bring TestMocks types into global namespace for ArduinoJson API compatibility
using TestMocks::JsonVariant;
using TestMocks::JsonObject;
using TestMocks::JsonArray;
using TestMocks::JsonDocument;
using TestMocks::StaticJsonDocument;
using TestMocks::DynamicJsonDocument;

// Mock deserialization functions
inline int deserializeJson(JsonDocument& doc, const char* json) {
    // Return 0 for success (mock always succeeds)
    return 0;
}

inline int deserializeJson(JsonDocument& doc, const String& json) {
    return 0;
}

// Mock serialization functions
inline size_t serializeJson(const JsonDocument& doc, char* buffer, size_t size) {
    // Return a minimal JSON
    const char* minJson = "{}";
    size_t len = strlen(minJson);
    if (len < size) {
        strcpy(buffer, minJson);
        return len;
    }
    return 0;
}

inline size_t serializeJson(const JsonDocument& doc, String& output) {
    output = "{}";
    return 2;
}

// Measure JSON functions
inline size_t measureJson(const JsonDocument& doc) {
    return 2;  // "{}"
}

#endif // ARDUINOJSON_H
