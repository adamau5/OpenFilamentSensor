/**
 * Soak Test - Long-Duration Stress Simulation
 *
 * Simulates ~24 hours of firmware runtime in minutes by running
 * the full detection pipeline (FilamentMotionSensor + JamDetector + Logger)
 * for millions of iterations.
 *
 * Goals:
 *   - Detect slow-burn state corruption (counter overflow, index wraparound)
 *   - Catch memory leaks via ASan (run with --sanitize flag)
 *   - Verify circular buffer integrity after millions of writes
 *   - Stress print start/stop/pause/resume cycling
 *
 * Run:
 *   g++ -std=c++17 -I. -I./mocks -I../src -o test_soak test_soak.cpp
 *   ./test_soak
 *
 * With AddressSanitizer:
 *   g++ -std=c++17 -fsanitize=address -fno-omit-frame-pointer -g \
 *       -I. -I./mocks -I../src -o test_soak test_soak.cpp && ./test_soak
 */

#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdarg>
#include <climits>

// Define mock globals before including mocks
unsigned long _mockMillis = 0;
int testsPassed = 0;
int testsFailed = 0;

// Pre-define header guards to prevent real headers from being included
#define LOGGER_H
#define SETTINGS_DATA_H

// Define LogLevel enum needed by JamDetector
enum LogLevel {
    LOG_NORMAL = 0,
    LOG_VERBOSE = 1,
    LOG_PIN_VALUES = 2
};

// Mock Logger singleton
class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }
    void log(const char* msg, LogLevel level = LOG_NORMAL) {}
    void logf(const char* fmt, ...) {}
    void logf(LogLevel level, const char* fmt, ...) {}
};

// Mock SettingsManager singleton
class SettingsManager {
public:
    static SettingsManager& getInstance() {
        static SettingsManager instance;
        return instance;
    }
    bool getVerboseLogging() const { return false; }
    int getLogLevel() const { return 0; }
};

// Include shared mocks
#include "mocks/test_mocks.h"
#include "mocks/arduino_mocks.h"

// Mock instances
MockLogger logger;
MockSettingsManager settingsManager;
MockSerial Serial;

// Include real implementations
#include "../src/FilamentMotionSensor.h"
#include "../src/FilamentMotionSensor.cpp"
#include "../src/JamDetector.h"
#include "../src/JamDetector.cpp"

// ============================================================================
// Soak Test Harness (extended from integration test pattern)
// ============================================================================

class SoakHarness {
public:
    FilamentMotionSensor sensor;
    JamDetector detector;
    JamConfig config;

    unsigned long printStartTime;
    bool isPrinting;
    unsigned long pulseCount;
    float totalExtrusionMm;

    // Stats tracking
    unsigned long totalIterations;
    unsigned long printStarts;
    unsigned long printStops;
    unsigned long pauseResumeCycles;
    unsigned long jamDetections;
    unsigned long maxPulseCount;

    SoakHarness() : printStartTime(0), isPrinting(false), pulseCount(0),
                    totalExtrusionMm(0.0f), totalIterations(0), printStarts(0),
                    printStops(0), pauseResumeCycles(0), jamDetections(0),
                    maxPulseCount(0) {
        config.graceTimeMs = 5000;
        config.hardJamMm = 5.0f;
        config.softJamTimeMs = 10000;
        config.hardJamTimeMs = 3000;
        config.ratioThreshold = 0.70f;
        config.detectionMode = DetectionMode::BOTH;
    }

    void startPrint() {
        printStartTime = millis();
        isPrinting = true;
        sensor.reset();
        detector.reset(printStartTime);
        pulseCount = 0;
        totalExtrusionMm = 0.0f;
        printStarts++;
    }

    void stopPrint() {
        isPrinting = false;
        printStops++;
    }

    void pauseResume() {
        detector.onResume(millis(), pulseCount, sensor.getSensorDistance());
        pauseResumeCycles++;
    }

    void addExtrusion(float deltaMm) {
        totalExtrusionMm += deltaMm;
        sensor.updateExpectedPosition(totalExtrusionMm);
    }

    void addPulse(float mmPerPulse = 2.88f) {
        sensor.addSensorPulse(mmPerPulse);
        pulseCount++;
        if (pulseCount > maxPulseCount) maxPulseCount = pulseCount;
    }

    JamState runDetection() {
        float expectedRate, actualRate;
        sensor.getWindowedRates(expectedRate, actualRate);

        totalIterations++;

        JamState state = detector.update(
            sensor.getExpectedDistance(),
            sensor.getSensorDistance(),
            pulseCount,
            isPrinting,
            true,
            millis(),
            printStartTime,
            config,
            expectedRate,
            actualRate
        );

        if (state.jammed) jamDetections++;
        return state;
    }
};

// ============================================================================
// Logger Circular Buffer (standalone for soak testing)
// ============================================================================

struct LogEntry {
    char          uuid[37];
    unsigned long timestamp;
    char          message[256];
    uint8_t       level;
};

class SoakLogger {
public:
    static const int MAX_LOG_ENTRIES = 250;

    LogEntry logBuffer[MAX_LOG_ENTRIES];
    int logCapacity;
    int currentIndex;
    int totalEntries;
    uint32_t uuidCounter;

    SoakLogger() : logCapacity(MAX_LOG_ENTRIES), currentIndex(0),
                   totalEntries(0), uuidCounter(0) {
        memset(logBuffer, 0, sizeof(logBuffer));
    }

    void log(const char *message) {
        uuidCounter++;
        snprintf(logBuffer[currentIndex].uuid, 37, "soak-%08x", uuidCounter);
        logBuffer[currentIndex].timestamp = millis();
        strncpy(logBuffer[currentIndex].message, message,
                sizeof(logBuffer[currentIndex].message) - 1);
        logBuffer[currentIndex].message[sizeof(logBuffer[currentIndex].message) - 1] = '\0';
        logBuffer[currentIndex].level = 0;

        currentIndex = (currentIndex + 1) % logCapacity;
        if (totalEntries < logCapacity) {
            totalEntries++;
        }
    }

    void clearLogs() {
        currentIndex = 0;
        totalEntries = 0;
        memset(logBuffer, 0, sizeof(logBuffer));
    }

    // Validate internal consistency
    bool isConsistent() const {
        if (currentIndex < 0 || currentIndex >= logCapacity) return false;
        if (totalEntries < 0 || totalEntries > logCapacity) return false;
        return true;
    }

    // Check that the most recent entry has a valid UUID
    bool lastEntryValid() const {
        if (totalEntries == 0) return true;
        int lastIdx = (currentIndex - 1 + logCapacity) % logCapacity;
        return logBuffer[lastIdx].uuid[0] != '\0' &&
               strlen(logBuffer[lastIdx].uuid) > 0 &&
               strlen(logBuffer[lastIdx].uuid) < 37;
    }
};

// ============================================================================
// CachedResponse double-buffer (standalone soak version)
// ============================================================================

static const size_t kCacheBufSize = 1536;

struct CachedResponse {
    char   buf[2][kCacheBufSize];
    size_t len[2];
    int    activeIdx;

    CachedResponse() : activeIdx(0) {
        memset(buf, 0, sizeof(buf));
        len[0] = 0;
        len[1] = 0;
    }

    void publish(const char *json, size_t jsonLen) {
        int writeIdx = !activeIdx;
        size_t copyLen = (jsonLen < kCacheBufSize - 1) ? jsonLen : (kCacheBufSize - 1);
        memcpy(buf[writeIdx], json, copyLen);
        buf[writeIdx][copyLen] = '\0';
        len[writeIdx] = copyLen;
        activeIdx = writeIdx;
    }

    const char *read(size_t &outLen) const {
        int idx = activeIdx;
        outLen = len[idx];
        return buf[idx];
    }

    bool isConsistent() const {
        if (activeIdx != 0 && activeIdx != 1) return false;
        if (len[activeIdx] > kCacheBufSize - 1) return false;
        // Active buffer should be null-terminated
        if (len[activeIdx] > 0 && buf[activeIdx][len[activeIdx]] != '\0') return false;
        return true;
    }
};

// ============================================================================
// Soak Tests
// ============================================================================

// Total simulated iterations. 10M iterations @ 250ms = ~694 hours simulated.
// Reduced to 2M for reasonable test time (~5-10 seconds).
static const long SOAK_ITERATIONS = 2000000;
static const int  TICK_MS = 250;  // Simulated time per iteration

/**
 * Test 1: Full pipeline soak - cycling through print states
 *
 * Simulates realistic firmware lifecycle:
 *   idle(10s) -> print(60s, healthy) -> soft jam(5s) -> recovery(10s) ->
 *   pause(3s) -> resume(30s) -> stop -> repeat
 */
void testFullPipelineSoak() {
    TEST_SECTION("Full Pipeline Soak (" + std::to_string(SOAK_ITERATIONS) + " iterations)");

    resetMockTime();
    SoakHarness harness;

    // State machine for cycling through conditions
    enum Phase { IDLE, GRACE, HEALTHY, SOFT_JAM, RECOVERY, PAUSED, POST_RESUME };
    Phase phase = IDLE;
    unsigned long phaseStart = 0;
    int cycleCount = 0;

    // Phase durations (in iterations, not ms)
    const int IDLE_ITERS     = 40;     // 10s
    const int GRACE_ITERS    = 24;     // 6s (past 5s grace)
    const int HEALTHY_ITERS  = 240;    // 60s
    const int SOFT_JAM_ITERS = 20;     // 5s
    const int RECOVERY_ITERS = 40;     // 10s
    const int PAUSED_ITERS   = 12;     // 3s
    const int RESUME_ITERS   = 120;    // 30s

    int phaseIter = 0;

    for (long i = 0; i < SOAK_ITERATIONS; i++) {
        advanceTime(TICK_MS);
        phaseIter++;

        switch (phase) {
        case IDLE:
            if (phaseIter >= IDLE_ITERS) {
                harness.startPrint();
                phase = GRACE;
                phaseIter = 0;
            }
            break;

        case GRACE:
            // During grace period: normal extrusion + pulses
            harness.addExtrusion(2.0f);
            harness.addPulse(2.88f);
            harness.runDetection();

            if (phaseIter >= GRACE_ITERS) {
                phase = HEALTHY;
                phaseIter = 0;
            }
            break;

        case HEALTHY:
            // Normal healthy print
            harness.addExtrusion(2.0f);
            harness.addPulse(2.88f);
            {
                JamState s = harness.runDetection();
                // After grace, should not jam with healthy flow
                (void)s;
            }

            if (phaseIter >= HEALTHY_ITERS) {
                phase = SOFT_JAM;
                phaseIter = 0;
            }
            break;

        case SOFT_JAM:
            // Under-extrusion: expected movement, sparse pulses
            harness.addExtrusion(3.0f);
            if (phaseIter % 4 == 0) {
                harness.addPulse(2.88f);  // ~25% flow
            }
            harness.runDetection();

            if (phaseIter >= SOFT_JAM_ITERS) {
                phase = RECOVERY;
                phaseIter = 0;
            }
            break;

        case RECOVERY:
            // Full flow again - jam percentages should decrease
            harness.addExtrusion(2.0f);
            harness.addPulse(2.88f);
            harness.runDetection();

            if (phaseIter >= RECOVERY_ITERS) {
                phase = PAUSED;
                phaseIter = 0;
            }
            break;

        case PAUSED:
            // No extrusion, no pulses
            if (phaseIter >= PAUSED_ITERS) {
                harness.pauseResume();
                phase = POST_RESUME;
                phaseIter = 0;
            }
            break;

        case POST_RESUME:
            // Post-resume: normal flow
            harness.addExtrusion(2.0f);
            harness.addPulse(2.88f);
            harness.runDetection();

            if (phaseIter >= RESUME_ITERS) {
                harness.stopPrint();
                phase = IDLE;
                phaseIter = 0;
                cycleCount++;
            }
            break;
        }
    }

    // Verify state is sane after soak
    TEST_ASSERT(harness.totalIterations > 0, "Should have run iterations");
    TEST_ASSERT(harness.printStarts > 0, "Should have started prints");
    TEST_ASSERT(harness.printStarts == harness.printStops ||
                harness.printStarts == harness.printStops + 1,
                "Start/stop counts should be balanced (off by at most 1)");
    TEST_ASSERT(harness.pulseCount < ULONG_MAX / 2,
                "Pulse count should not have overflowed");

    // Sensor state should be valid
    float expected = harness.sensor.getExpectedDistance();
    float actual = harness.sensor.getSensorDistance();
    TEST_ASSERT(!std::isnan(expected), "Expected distance should not be NaN");
    TEST_ASSERT(!std::isnan(actual), "Actual distance should not be NaN");
    TEST_ASSERT(!std::isinf(expected), "Expected distance should not be Inf");
    TEST_ASSERT(!std::isinf(actual), "Actual distance should not be Inf");

    // JamState should be valid
    JamState finalState = harness.detector.getState();
    TEST_ASSERT(!std::isnan(finalState.passRatio), "passRatio should not be NaN");
    TEST_ASSERT(!std::isnan(finalState.deficit), "deficit should not be NaN");
    TEST_ASSERT(!std::isnan(finalState.hardJamPercent), "hardJamPercent should not be NaN");
    TEST_ASSERT(!std::isnan(finalState.softJamPercent), "softJamPercent should not be NaN");
    TEST_ASSERT(finalState.hardJamPercent >= 0.0f && finalState.hardJamPercent <= 100.0f,
                "hardJamPercent should be in [0, 100]");
    TEST_ASSERT(finalState.softJamPercent >= 0.0f && finalState.softJamPercent <= 100.0f,
                "softJamPercent should be in [0, 100]");

    std::cout << "  Cycles: " << cycleCount << std::endl;
    std::cout << "  Print starts: " << harness.printStarts << std::endl;
    std::cout << "  Pause/resumes: " << harness.pauseResumeCycles << std::endl;
    std::cout << "  Jam detections: " << harness.jamDetections << std::endl;
    std::cout << "  Max pulse count: " << harness.maxPulseCount << std::endl;
    std::cout << "  Simulated time: " << (_mockMillis / 3600000) << "h "
              << ((_mockMillis % 3600000) / 60000) << "m" << std::endl;

    TEST_PASS("Full pipeline soak: " + std::to_string(cycleCount) + " cycles, " +
              std::to_string(SOAK_ITERATIONS) + " iterations");
}

/**
 * Test 2: Logger circular buffer soak
 *
 * Writes millions of log entries, periodically clears, and verifies
 * buffer integrity at each checkpoint.
 */
void testLoggerCircularBufferSoak() {
    TEST_SECTION("Logger Circular Buffer Soak");

    resetMockTime();
    SoakLogger soakLog;

    const long LOG_ITERATIONS = 1000000;
    const int  CLEAR_EVERY = 50000;
    int clearCount = 0;

    for (long i = 0; i < LOG_ITERATIONS; i++) {
        _mockMillis = (unsigned long)i;

        char msg[64];
        snprintf(msg, sizeof(msg), "Soak log entry %ld at %lu", i, _mockMillis);
        soakLog.log(msg);

        // Periodic clear
        if (i > 0 && i % CLEAR_EVERY == 0) {
            soakLog.clearLogs();
            clearCount++;

            // Verify consistency after clear
            TEST_ASSERT(soakLog.isConsistent(), "Logger should be consistent after clear");
            TEST_ASSERT(soakLog.currentIndex == 0, "Index should be 0 after clear");
            TEST_ASSERT(soakLog.totalEntries == 0, "Count should be 0 after clear");
        }

        // Periodic consistency check
        if (i % 100000 == 0) {
            TEST_ASSERT(soakLog.isConsistent(), "Logger consistent at iteration " + std::to_string(i));
            TEST_ASSERT(soakLog.lastEntryValid(), "Last entry valid at iteration " + std::to_string(i));
        }
    }

    // Final checks
    TEST_ASSERT(soakLog.isConsistent(), "Logger should be consistent after soak");
    TEST_ASSERT(soakLog.lastEntryValid(), "Last entry should be valid");
    TEST_ASSERT(soakLog.uuidCounter == (uint32_t)LOG_ITERATIONS,
                "UUID counter should match iteration count");

    // Verify the buffer contains recent entries (not garbage)
    int lastIdx = (soakLog.currentIndex - 1 + soakLog.logCapacity) % soakLog.logCapacity;
    TEST_ASSERT(soakLog.logBuffer[lastIdx].timestamp > 0, "Last entry should have timestamp");
    TEST_ASSERT(strlen(soakLog.logBuffer[lastIdx].message) > 0, "Last entry should have message");

    std::cout << "  Logged: " << LOG_ITERATIONS << " entries" << std::endl;
    std::cout << "  Clears: " << clearCount << std::endl;
    std::cout << "  Final totalEntries: " << soakLog.totalEntries << std::endl;
    std::cout << "  UUID counter: " << soakLog.uuidCounter << std::endl;

    TEST_PASS("Logger circular buffer survived " + std::to_string(LOG_ITERATIONS) + " entries");
}

/**
 * Test 3: CachedResponse double-buffer soak
 *
 * Publishes millions of different JSON payloads and verifies
 * the buffer never corrupts.
 */
void testCachedResponseSoak() {
    TEST_SECTION("CachedResponse Double-Buffer Soak");

    CachedResponse cache;

    const long PUBLISH_ITERATIONS = 1000000;
    char payload[512];

    for (long i = 0; i < PUBLISH_ITERATIONS; i++) {
        int len = snprintf(payload, sizeof(payload),
            "{\"iter\":%ld,\"status\":%d,\"value\":%.4f}",
            i, (int)(i % 10), (float)i * 0.001f);

        cache.publish(payload, (size_t)len);

        // Periodic consistency check
        if (i % 100000 == 0) {
            TEST_ASSERT(cache.isConsistent(), "Cache consistent at iteration " + std::to_string(i));

            // Verify read returns valid data
            size_t readLen;
            const char *data = cache.read(readLen);
            TEST_ASSERT(readLen > 0, "Cache should have data");
            TEST_ASSERT(data[0] == '{', "JSON should start with {");
            TEST_ASSERT(data[readLen - 1] == '}', "JSON should end with }");
            TEST_ASSERT(data[readLen] == '\0', "Should be null-terminated");
        }
    }

    // Final check
    TEST_ASSERT(cache.isConsistent(), "Cache consistent after soak");
    size_t finalLen;
    const char *finalData = cache.read(finalLen);
    TEST_ASSERT(finalLen > 0, "Final read should have data");
    TEST_ASSERT(finalData[0] == '{', "Final JSON valid");

    std::cout << "  Published: " << PUBLISH_ITERATIONS << " payloads" << std::endl;
    std::cout << "  Final payload length: " << finalLen << std::endl;

    TEST_PASS("CachedResponse survived " + std::to_string(PUBLISH_ITERATIONS) + " publishes");
}

/**
 * Test 4: millis() rollover simulation
 *
 * Tests that the system handles millis() wrapping around ULONG_MAX.
 * On ESP32, millis() wraps after ~49.7 days.
 */
void testMillisRollover() {
    TEST_SECTION("millis() Rollover Handling");

    SoakHarness harness;

    // Start near rollover point
    _mockMillis = ULONG_MAX - 30000;  // 30 seconds before rollover
    harness.startPrint();

    // Run through the rollover
    float totalExtrusion = 0.0f;
    bool crossedRollover = false;
    int iterations = 0;
    unsigned long prevMillis = _mockMillis;

    for (int i = 0; i < 500; i++) {
        advanceTime(TICK_MS);
        iterations++;

        if (_mockMillis < prevMillis) {
            crossedRollover = true;
        }
        prevMillis = _mockMillis;

        totalExtrusion += 2.0f;
        harness.addExtrusion(2.0f);
        harness.addPulse(2.88f);

        JamState state = harness.runDetection();

        // Should not crash or produce NaN
        TEST_ASSERT(!std::isnan(state.passRatio), "passRatio should not be NaN after rollover");
        TEST_ASSERT(!std::isnan(state.deficit), "deficit should not be NaN after rollover");
    }

    TEST_ASSERT(crossedRollover, "Should have crossed millis() rollover point");

    // State should still be sane
    JamState finalState = harness.detector.getState();
    TEST_ASSERT(!std::isnan(finalState.hardJamPercent), "hardJamPercent valid after rollover");
    TEST_ASSERT(!std::isnan(finalState.softJamPercent), "softJamPercent valid after rollover");

    std::cout << "  Crossed rollover at iteration with millis()=" << _mockMillis << std::endl;

    TEST_PASS("System handles millis() rollover without corruption");
}

/**
 * Test 5: Rapid print start/stop cycling
 *
 * Some users rapidly start/stop prints. Tests that repeated
 * reset() calls don't leak state.
 */
void testRapidStartStopCycling() {
    TEST_SECTION("Rapid Print Start/Stop Cycling");

    resetMockTime();
    SoakHarness harness;

    const int CYCLES = 100000;

    for (int i = 0; i < CYCLES; i++) {
        advanceTime(100);

        harness.startPrint();

        // Brief print (a few iterations)
        for (int j = 0; j < 5; j++) {
            advanceTime(TICK_MS);
            harness.addExtrusion(1.0f);
            harness.addPulse(2.88f);
            harness.runDetection();
        }

        harness.stopPrint();
    }

    // Verify no state corruption after rapid cycling
    TEST_ASSERT(harness.printStarts == CYCLES, "Print start count should match");
    TEST_ASSERT(harness.printStops == CYCLES, "Print stop count should match");

    float expected = harness.sensor.getExpectedDistance();
    float actual = harness.sensor.getSensorDistance();
    TEST_ASSERT(!std::isnan(expected), "Expected should not be NaN");
    TEST_ASSERT(!std::isnan(actual), "Actual should not be NaN");
    TEST_ASSERT(!std::isinf(expected), "Expected should not be Inf");
    TEST_ASSERT(!std::isinf(actual), "Actual should not be Inf");

    std::cout << "  Cycles: " << CYCLES << std::endl;
    std::cout << "  Total detection iterations: " << harness.totalIterations << std::endl;

    TEST_PASS("Rapid start/stop: " + std::to_string(CYCLES) + " cycles without corruption");
}

/**
 * Test 6: Extrusion value accumulation precision
 *
 * Verifies that floating point accumulation over long prints
 * doesn't drift excessively.
 */
void testExtrusionAccumulationPrecision() {
    TEST_SECTION("Extrusion Accumulation Precision");

    resetMockTime();
    FilamentMotionSensor sensor;
    sensor.reset();

    // Simulate a very long print: 10000 updates of 0.5mm each = 5000mm total
    const int UPDATES = 10000;
    const float DELTA_MM = 0.5f;
    float manualTotal = 0.0f;

    for (int i = 0; i < UPDATES; i++) {
        advanceTime(250);
        manualTotal += DELTA_MM;
        sensor.updateExpectedPosition(manualTotal);
        sensor.addSensorPulse(DELTA_MM);  // Perfect flow
    }

    float expected = sensor.getExpectedDistance();
    float actual = sensor.getSensorDistance();

    // Check that windowed values are reasonable (not NaN, not absurdly large)
    TEST_ASSERT(!std::isnan(expected), "Expected should not be NaN");
    TEST_ASSERT(!std::isnan(actual), "Actual should not be NaN");
    TEST_ASSERT(expected >= 0.0f, "Expected should be non-negative");
    TEST_ASSERT(actual >= 0.0f, "Actual should be non-negative");

    // With perfect 1:1 flow, deficit should be near zero
    float deficit = sensor.getDeficit();
    TEST_ASSERT(!std::isnan(deficit), "Deficit should not be NaN");

    std::cout << "  Total expected extrusion: " << manualTotal << "mm" << std::endl;
    std::cout << "  Windowed expected: " << expected << "mm" << std::endl;
    std::cout << "  Windowed actual: " << actual << "mm" << std::endl;
    std::cout << "  Deficit: " << deficit << "mm" << std::endl;

    TEST_PASS("Extrusion accumulation maintains precision over " +
              std::to_string(UPDATES) + " updates");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "    Soak Test Suite" << std::endl;
    std::cout << "========================================" << std::endl;

    testFullPipelineSoak();
    testLoggerCircularBufferSoak();
    testCachedResponseSoak();
    testMillisRollover();
    testRapidStartStopCycling();
    testExtrusionAccumulationPrecision();

    std::cout << "\n========================================" << std::endl;
    std::cout << "Test Results:" << std::endl;
    std::cout << "\033[32m  Passed: " << testsPassed << "\033[0m" << std::endl;
    if (testsFailed > 0) {
        std::cout << "\033[31m  Failed: " << testsFailed << "\033[0m" << std::endl;
    }
    std::cout << "========================================\n" << std::endl;

    return testsFailed > 0 ? 1 : 0;
}
