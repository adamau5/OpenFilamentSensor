/**
 * ArduinoJson Mocks
 *
 * Mock implementations of ArduinoJson classes for unit testing.
 */

#ifndef JSON_MOCKS_H
#define JSON_MOCKS_H

#include <cstring>
#include <cstdio>
#include "arduino_mocks.h"

namespace TestMocks {

/**
 * Mock JsonVariant - holds a single value
 */
class JsonVariant {
public:
    enum Type { TYPE_NULL, TYPE_BOOL, TYPE_INT, TYPE_FLOAT, TYPE_STRING };

    JsonVariant() : type(TYPE_NULL), floatVal(0.0f), intVal(0), boolVal(false), strVal(nullptr) {}

    ~JsonVariant() {
        delete[] strVal;
    }

    JsonVariant(const JsonVariant& other) : type(other.type), floatVal(other.floatVal),
                                            intVal(other.intVal), boolVal(other.boolVal), strVal(nullptr) {
        if (other.strVal) {
            strVal = new char[strlen(other.strVal) + 1];
            strcpy(strVal, other.strVal);
        }
    }

    JsonVariant& operator=(const JsonVariant& other) {
        if (this != &other) {
            delete[] strVal;
            type = other.type;
            floatVal = other.floatVal;
            intVal = other.intVal;
            boolVal = other.boolVal;
            if (other.strVal) {
                strVal = new char[strlen(other.strVal) + 1];
                strcpy(strVal, other.strVal);
            } else {
                strVal = nullptr;
            }
        }
        return *this;
    }

    bool isNull() const { return type == TYPE_NULL; }

    template <typename T>
    T as() const;

    void setFloat(float val) {
        type = TYPE_FLOAT;
        floatVal = val;
    }

    void setInt(int val) {
        type = TYPE_INT;
        intVal = val;
    }

    void setBool(bool val) {
        type = TYPE_BOOL;
        boolVal = val;
    }

    void setString(const char* val) {
        type = TYPE_STRING;
        delete[] strVal;
        if (val) {
            strVal = new char[strlen(val) + 1];
            strcpy(strVal, val);
        } else {
            strVal = nullptr;
        }
    }

private:
    Type type;
    float floatVal;
    int intVal;
    bool boolVal;
    char* strVal;
};

// Template specializations for as<T>()
template<>
inline float JsonVariant::as<float>() const {
    switch (type) {
        case TYPE_FLOAT: return floatVal;
        case TYPE_INT: return (float)intVal;
        default: return 0.0f;
    }
}

template<>
inline int JsonVariant::as<int>() const {
    switch (type) {
        case TYPE_INT: return intVal;
        case TYPE_FLOAT: return (int)floatVal;
        default: return 0;
    }
}

template<>
inline bool JsonVariant::as<bool>() const {
    switch (type) {
        case TYPE_BOOL: return boolVal;
        case TYPE_INT: return intVal != 0;
        case TYPE_FLOAT: return floatVal != 0.0f;
        default: return false;
    }
}

template<>
inline const char* JsonVariant::as<const char*>() const {
    return strVal ? strVal : "";
}

/**
 * Mock JsonObject - key-value store
 */
class JsonObject {
public:
    static const int MAX_KEYS = 32;

    JsonObject() : keyCount(0) {
        for (int i = 0; i < MAX_KEYS; i++) {
            keys[i] = nullptr;
        }
    }

    ~JsonObject() {
        for (int i = 0; i < keyCount; i++) {
            delete[] keys[i];
        }
    }

    JsonObject(const JsonObject& other) : keyCount(other.keyCount) {
        for (int i = 0; i < keyCount; i++) {
            if (other.keys[i]) {
                keys[i] = new char[strlen(other.keys[i]) + 1];
                strcpy(keys[i], other.keys[i]);
            } else {
                keys[i] = nullptr;
            }
            values[i] = other.values[i];
        }
        for (int i = keyCount; i < MAX_KEYS; i++) {
            keys[i] = nullptr;
        }
    }

    bool containsKey(const char* key) const {
        for (int i = 0; i < keyCount; i++) {
            if (keys[i] && strcmp(keys[i], key) == 0) return true;
        }
        return false;
    }

    JsonVariant operator[](const char* key) const {
        for (int i = 0; i < keyCount; i++) {
            if (keys[i] && strcmp(keys[i], key) == 0) return values[i];
        }
        return JsonVariant();
    }

    void set(const char* key, float value) {
        int idx = findOrCreate(key);
        if (idx >= 0) {
            values[idx].setFloat(value);
        }
    }

    void set(const char* key, int value) {
        int idx = findOrCreate(key);
        if (idx >= 0) {
            values[idx].setInt(value);
        }
    }

    void set(const char* key, bool value) {
        int idx = findOrCreate(key);
        if (idx >= 0) {
            values[idx].setBool(value);
        }
    }

    void set(const char* key, const char* value) {
        int idx = findOrCreate(key);
        if (idx >= 0) {
            values[idx].setString(value);
        }
    }

    bool isNull() const { return keyCount == 0; }

private:
    char* keys[MAX_KEYS];
    JsonVariant values[MAX_KEYS];
    int keyCount;

    int findOrCreate(const char* key) {
        // Find existing
        for (int i = 0; i < keyCount; i++) {
            if (keys[i] && strcmp(keys[i], key) == 0) return i;
        }
        // Create new
        if (keyCount < MAX_KEYS) {
            keys[keyCount] = new char[strlen(key) + 1];
            strcpy(keys[keyCount], key);
            return keyCount++;
        }
        return -1;
    }
};

/**
 * Mock JsonArray - simple array storage
 */
class JsonArray {
public:
    static const int MAX_ELEMENTS = 32;

    JsonArray() : count(0) {}

    void add(int val) {
        if (count < MAX_ELEMENTS) {
            intValues[count++] = val;
        }
    }

    void add(float val) {
        if (count < MAX_ELEMENTS) {
            floatValues[count++] = val;
        }
    }

    size_t size() const { return count; }

    bool isNull() const { return false; }

private:
    int intValues[MAX_ELEMENTS];
    float floatValues[MAX_ELEMENTS];
    int count;
};

/**
 * Mock JsonDocument - root container
 */
class JsonDocument {
public:
    JsonDocument() {}

    JsonObject createNestedObject(const char* key) {
        return JsonObject();
    }

    JsonArray createNestedArray(const char* key) {
        return JsonArray();
    }

    JsonObject as() {
        return rootObject;
    }

    template<typename T>
    void operator[](const char* key) {}

    void set(const char* key, const String& val) { rootObject.set(key, val.c_str()); }
    void set(const char* key, const char* val) { rootObject.set(key, val); }
    void set(const char* key, unsigned long val) { rootObject.set(key, (int)val); }
    void set(const char* key, int val) { rootObject.set(key, val); }
    void set(const char* key, float val) { rootObject.set(key, val); }
    void set(const char* key, bool val) { rootObject.set(key, val); }

    bool containsKey(const char* key) const { return rootObject.containsKey(key); }

private:
    JsonObject rootObject;
};

// Aliases for ArduinoJson compatibility
template<size_t N>
class StaticJsonDocument : public JsonDocument {};

class DynamicJsonDocument : public JsonDocument {
public:
    DynamicJsonDocument(size_t capacity = 1024) : JsonDocument() {}
};

}  // namespace TestMocks

#endif // JSON_MOCKS_H
