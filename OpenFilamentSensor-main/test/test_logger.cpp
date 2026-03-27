/**
 * Unit Tests for Logger
 *
 * Tests the circular buffer logging system, log levels,
 * formatting, and output generation.
 */

#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdarg>

// Define mock globals before including mocks
unsigned long _mockMillis = 0;
int testsPassed = 0;
int testsFailed = 0;

// Include shared mocks
#include "mocks/test_mocks.h"
#include "mocks/arduino_mocks.h"

// Mock Serial
MockSerial Serial;

// Log levels (from Logger.h)
enum LogLevel : uint8_t {
    LOG_NORMAL      = 0,
    LOG_VERBOSE     = 1,
    LOG_PIN_VALUES  = 2
};

// Log entry structure
struct LogEntry {
    char          uuid[37];
    unsigned long timestamp;
    char          message[256];
    LogLevel      level;
};

// Testable Logger class (non-singleton for testing)
class TestableLogger {
private:
    static const int MAX_LOG_ENTRIES = 50;  // Smaller for testing
    LogEntry logBuffer[MAX_LOG_ENTRIES];
    int currentIndex;
    int totalEntries;
    uint32_t uuidCounter;
    LogLevel currentLogLevel;

    void generateUUID(char* buffer) {
        // Simple counter-based UUID for testing
        snprintf(buffer, 37, "test-uuid-%08x", uuidCounter++);
    }

public:
    TestableLogger() : currentIndex(0), totalEntries(0), uuidCounter(0), currentLogLevel(LOG_NORMAL) {
        clearLogs();
    }

    void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

    LogLevel getLogLevel() const {
        return currentLogLevel;
    }

    void log(const char* message, LogLevel level = LOG_NORMAL) {
        // Only log if level is at or below current setting
        if (level > currentLogLevel) {
            return;
        }

        LogEntry& entry = logBuffer[currentIndex];
        generateUUID(entry.uuid);
        entry.timestamp = millis();
        entry.level = level;

        // Truncate message if too long
        strncpy(entry.message, message, 255);
        entry.message[255] = '\0';

        currentIndex = (currentIndex + 1) % MAX_LOG_ENTRIES;
        if (totalEntries < MAX_LOG_ENTRIES) {
            totalEntries++;
        }
    }

    void logf(const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(buffer, LOG_NORMAL);
    }

    void logf(LogLevel level, const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(buffer, level);
    }

    void logNormal(const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(buffer, LOG_NORMAL);
    }

    void logVerbose(const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(buffer, LOG_VERBOSE);
    }

    void logPinValues(const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(buffer, LOG_PIN_VALUES);
    }

    String getLogsAsText(int maxEntries = -1) {
        String result;
        int count = totalEntries;
        if (maxEntries > 0 && maxEntries < count) {
            count = maxEntries;
        }

        // Calculate starting index
        int startIdx;
        if (totalEntries < MAX_LOG_ENTRIES) {
            startIdx = 0;
        } else {
            startIdx = currentIndex;  // Oldest entry
        }

        // Skip entries if we're limiting
        int skip = totalEntries - count;
        startIdx = (startIdx + skip) % MAX_LOG_ENTRIES;

        for (int i = 0; i < count; i++) {
            int idx = (startIdx + i) % MAX_LOG_ENTRIES;
            LogEntry& entry = logBuffer[idx];

            char line[512];
            snprintf(line, sizeof(line), "[%s] %lu: %s\n",
                     entry.uuid, entry.timestamp, entry.message);
            result += line;
        }

        return result;
    }

    void clearLogs() {
        currentIndex = 0;
        totalEntries = 0;
        for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
            logBuffer[i].message[0] = '\0';
            logBuffer[i].uuid[0] = '\0';
            logBuffer[i].timestamp = 0;
            logBuffer[i].level = LOG_NORMAL;
        }
    }

    int getLogCount() const {
        return totalEntries;
    }

    // Test helper to get raw entry
    const LogEntry& getEntry(int index) const {
        int actualIdx;
        if (totalEntries < MAX_LOG_ENTRIES) {
            actualIdx = index;
        } else {
            actualIdx = (currentIndex + index) % MAX_LOG_ENTRIES;
        }
        return logBuffer[actualIdx];
    }
};

// Tests

void testLogAddsEntry() {
    TEST_SECTION("log() Adds Entry to Buffer");

    resetMockTime();
    TestableLogger logger;

    TEST_ASSERT(logger.getLogCount() == 0, "Buffer should be empty initially");

    logger.log("Test message");

    TEST_ASSERT(logger.getLogCount() == 1, "Buffer should have 1 entry after log");

    logger.log("Second message");
    logger.log("Third message");

    TEST_ASSERT(logger.getLogCount() == 3, "Buffer should have 3 entries");

    TEST_PASS("log() adds entry to buffer");
}

void testCircularBufferWrap() {
    TEST_SECTION("Circular Buffer Wraps at MAX_LOG_ENTRIES");

    resetMockTime();
    TestableLogger logger;

    // Log more than MAX_LOG_ENTRIES (50)
    for (int i = 0; i < 60; i++) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Message %d", i);
        logger.log(msg);
        advanceTime(10);
    }

    TEST_ASSERT(logger.getLogCount() == 50, "Count should cap at MAX_LOG_ENTRIES");

    // Check that oldest entries were overwritten
    String logs = logger.getLogsAsText();
    TEST_ASSERT(logs.indexOf("Message 59") >= 0, "Most recent should be present");
    TEST_ASSERT(logs.indexOf("Message 10") >= 0, "Entry 10 should still be present");

    TEST_PASS("Circular buffer wraps at MAX_LOG_ENTRIES");
}

void testLogLevelFiltering() {
    TEST_SECTION("Log Level Filtering");

    resetMockTime();
    TestableLogger logger;

    // Set to NORMAL level - should only log NORMAL
    logger.setLogLevel(LOG_NORMAL);

    logger.logNormal("Normal message");
    logger.logVerbose("Verbose message");  // Should be filtered
    logger.logPinValues("Pin values");      // Should be filtered

    TEST_ASSERT(logger.getLogCount() == 1, "Only NORMAL should be logged at NORMAL level");

    // Set to VERBOSE level
    logger.clearLogs();
    logger.setLogLevel(LOG_VERBOSE);

    logger.logNormal("Normal message");
    logger.logVerbose("Verbose message");
    logger.logPinValues("Pin values");  // Should be filtered

    TEST_ASSERT(logger.getLogCount() == 2, "NORMAL and VERBOSE should be logged at VERBOSE level");

    // Set to PIN_VALUES level (all)
    logger.clearLogs();
    logger.setLogLevel(LOG_PIN_VALUES);

    logger.logNormal("Normal message");
    logger.logVerbose("Verbose message");
    logger.logPinValues("Pin values");

    TEST_ASSERT(logger.getLogCount() == 3, "All levels should be logged at PIN_VALUES level");

    TEST_PASS("Log levels filter correctly (NORMAL, VERBOSE, PIN_VALUES)");
}

void testGetLogsAsText() {
    TEST_SECTION("getLogsAsText() Returns Formatted Output");

    resetMockTime();
    TestableLogger logger;

    _mockMillis = 1000;
    logger.log("First message");

    _mockMillis = 2000;
    logger.log("Second message");

    String logs = logger.getLogsAsText();

    TEST_ASSERT(logs.length() > 0, "Logs should not be empty");
    TEST_ASSERT(logs.indexOf("First message") >= 0, "Should contain first message");
    TEST_ASSERT(logs.indexOf("Second message") >= 0, "Should contain second message");
    TEST_ASSERT(logs.indexOf("test-uuid-") >= 0, "Should contain UUID");

    TEST_PASS("getLogsAsText() returns formatted output");
}

void testClearLogs() {
    TEST_SECTION("clearLogs() Empties Buffer");

    TestableLogger logger;

    logger.log("Message 1");
    logger.log("Message 2");
    logger.log("Message 3");

    TEST_ASSERT(logger.getLogCount() == 3, "Should have 3 entries before clear");

    logger.clearLogs();

    TEST_ASSERT(logger.getLogCount() == 0, "Should have 0 entries after clear");

    String logs = logger.getLogsAsText();
    TEST_ASSERT(logs.length() == 0, "Logs text should be empty after clear");

    TEST_PASS("clearLogs() empties buffer");
}

void testUuidIncrementsCorrectly() {
    TEST_SECTION("UUID Increments Correctly");

    TestableLogger logger;

    logger.log("First");
    logger.log("Second");
    logger.log("Third");

    const LogEntry& entry1 = logger.getEntry(0);
    const LogEntry& entry2 = logger.getEntry(1);
    const LogEntry& entry3 = logger.getEntry(2);

    // UUIDs should be different
    TEST_ASSERT(strcmp(entry1.uuid, entry2.uuid) != 0, "UUIDs should be different");
    TEST_ASSERT(strcmp(entry2.uuid, entry3.uuid) != 0, "UUIDs should be different");
    TEST_ASSERT(strcmp(entry1.uuid, entry3.uuid) != 0, "UUIDs should be different");

    TEST_PASS("UUID increments correctly");
}

void testTimestampFormatting() {
    TEST_SECTION("Timestamp Formatting");

    resetMockTime();
    TestableLogger logger;

    _mockMillis = 12345;
    logger.log("Test message");

    const LogEntry& entry = logger.getEntry(0);
    TEST_ASSERT(entry.timestamp == 12345, "Timestamp should be recorded");

    String logs = logger.getLogsAsText();
    TEST_ASSERT(logs.indexOf("12345") >= 0, "Timestamp should appear in output");

    TEST_PASS("Timestamp formatting works correctly");
}

void testMessageTruncation() {
    TEST_SECTION("Buffer Doesn't Overflow on Long Messages");

    TestableLogger logger;

    // Create a very long message (> 256 chars)
    char longMessage[512];
    memset(longMessage, 'A', 511);
    longMessage[511] = '\0';

    // Should not crash
    logger.log(longMessage);

    TEST_ASSERT(logger.getLogCount() == 1, "Long message should be logged");

    const LogEntry& entry = logger.getEntry(0);
    TEST_ASSERT(strlen(entry.message) <= 255, "Message should be truncated to 255 chars");

    TEST_PASS("Buffer doesn't overflow on long messages (256 char limit)");
}

void testLogfFormatting() {
    TEST_SECTION("logf() Formatting");

    TestableLogger logger;

    logger.logf("Value: %d, Float: %.2f, String: %s", 42, 3.14f, "test");

    const LogEntry& entry = logger.getEntry(0);
    TEST_ASSERT(strstr(entry.message, "Value: 42") != nullptr, "Should contain formatted int");
    TEST_ASSERT(strstr(entry.message, "3.14") != nullptr, "Should contain formatted float");
    TEST_ASSERT(strstr(entry.message, "test") != nullptr, "Should contain formatted string");

    TEST_PASS("logf() formats messages correctly");
}

void testGetLogLevelReturnsCurrentLevel() {
    TEST_SECTION("getLogLevel() Returns Current Level");

    TestableLogger logger;

    logger.setLogLevel(LOG_NORMAL);
    TEST_ASSERT(logger.getLogLevel() == LOG_NORMAL, "Should return NORMAL");

    logger.setLogLevel(LOG_VERBOSE);
    TEST_ASSERT(logger.getLogLevel() == LOG_VERBOSE, "Should return VERBOSE");

    logger.setLogLevel(LOG_PIN_VALUES);
    TEST_ASSERT(logger.getLogLevel() == LOG_PIN_VALUES, "Should return PIN_VALUES");

    TEST_PASS("getLogLevel() returns current level");
}

void testGetLogsWithMaxEntries() {
    TEST_SECTION("getLogsAsText() with maxEntries limit");

    TestableLogger logger;

    for (int i = 0; i < 10; i++) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Message %d", i);
        logger.log(msg);
    }

    // Get only 3 most recent
    String logs = logger.getLogsAsText(3);

    TEST_ASSERT(logs.indexOf("Message 9") >= 0, "Should have most recent");
    TEST_ASSERT(logs.indexOf("Message 8") >= 0, "Should have second most recent");
    TEST_ASSERT(logs.indexOf("Message 7") >= 0, "Should have third most recent");
    TEST_ASSERT(logs.indexOf("Message 0") < 0, "Should not have oldest");

    TEST_PASS("getLogsAsText(maxEntries) limits output correctly");
}

int main() {
    TEST_SUITE_BEGIN("Logger Unit Test Suite");

    testLogAddsEntry();
    testCircularBufferWrap();
    testLogLevelFiltering();
    testGetLogsAsText();
    testClearLogs();
    testUuidIncrementsCorrectly();
    testTimestampFormatting();
    testMessageTruncation();
    testLogfFormatting();
    testGetLogLevelReturnsCurrentLevel();
    testGetLogsWithMaxEntries();

    TEST_SUITE_END();
}
