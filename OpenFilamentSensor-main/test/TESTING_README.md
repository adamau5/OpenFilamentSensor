# OpenFilamentSensor Testing Suite

**Master Documentation**  
*Consolidated from: Project Testing Summary, Quick Test Guide, and Component Readmes.*

## 1. Overview
This directory contains the comprehensive test suite for the OpenFilamentSensor project. It covers:
- **Firmware Logic (C++):** Core jam detection, protocol parsing, and sensor motion algorithms.
- **Build Tools (Python):** Validation of build scripts, board configurations, and release workflows.
- **Web Components (JavaScript):** Verification of the web distributor, manifest files, and lightweight UI assets.

The suite is designed to be **fast, deterministic, and CI/CD ready**, allowing developers to validate changes without requiring physical hardware for the majority of logic.

---

## 2. Quick Start

To run the **entire** test suite (C++, Python, and Node.js tests) in one go:

```bash
cd test
./build_and_run_all_tests.sh
```

**Expected Output:**
- A sequence of builds and test runs.
- Green checkmarks (`✓`) for passed tests.
- A final summary: `ALL TESTS PASSED`.
- Exit code `0` on success.

---

## 3. Test Definitions & Validation Goals

### A. C++ Firmware Tests

#### 1. `pulse_simulator.cpp` (Core Sensor Logic)
Simulates the `FilamentMotionSensor` class against complex printing scenarios to ensure reliability.

| Test Scenario | Goal |
| :--- | :--- |
| **Normal Healthy Print** | Verifies no false positives occur during 30 seconds of continuous, healthy printing. |
| **Hard Jam Detection** | Ensures complete blockages (no encoder pulses) are detected within ~2 seconds. |
| **Soft Jam Detection** | Ensures partial clogs (under-extrusion < 25%) are detected within ~10 seconds. |
| **Sparse Infill** | Validates that travel moves with minimal extrusion do not trigger false jams. |
| **Retraction Handling** | Verifies that grace periods are correctly applied after retraction events. |
| **Ironing / Low-Flow** | Ensures micro-movements and ironing patterns don't trigger false positives. |
| **Transient Spike Resistance** | Tests hysteresis; a single bad ratio spike shouldn't trigger a jam. |
| **Minimum Movement Threshold** | Verifies that tiny movements below 1mm are ignored to prevent noise. |
| **Grace Period Duration** | Checks that the 500ms grace period correctly protects against SDCP look-ahead issues. |
| **Normal Print with Hard Snag** | Validates jam detection works even after a long period of healthy printing. |
| **Complex Flow Sequence** | A stress test combining travel, retractions, and ironing in one long sequence. |

#### 2. `test_jam_detector.cpp` (Unit Tests)
Isolates the `JamDetector` class to verify internal state machines and algorithms.

| Test Case | Goal |
| :--- | :--- |
| **testReset** | Verifies clean initialization and state reset. |
| **testGracePeriodStartup** | Checks that jams are ignored immediately after startup (grace period). |
| **testHardJamDetection** | Validates immediate detection logic when flow stops completely. |
| **testSoftJamDetection** | Validates the integral-based detection for slow, accumulating failures. |
| **testJamRecovery** | Ensures the system can recover and re-arm after a jam is cleared. |
| **testResumeGrace** | Verifies a specific grace period is applied after a Pause -> Resume cycle. |
| **testDetectionModes** | Tests `HARD_ONLY`, `SOFT_ONLY`, and `BOTH` filtering modes. |
| **testRateBasedDetection** | Verifies calculations for flow rate ratios. |
| **testMinimumThresholds** | Ensures thresholds (distance/time) must be met before evaluation begins. |
| **testPauseRequestHandling** | Validates the flag management for requesting a printer pause. |
| **testEdgeCaseZeroExpected** | Handling of zero-expected-movement updates (should be ignored). |
| **testNotPrintingState** | Ensures no detection occurs when the printer state is not "Printing". |

#### 3. `test_sdcp_protocol.cpp` (Protocol Parsing)
Validates the `SDCPProtocol` utility class.

| Test Case | Goal |
| :--- | :--- |
| **testBuildCommandMessage** | Verifies correct JSON formatting for SDCP commands. |
| **testTryReadExtrusionValueNormalKey** | Tests reading standard "current_feed_state" JSON keys. |
| **testTryReadExtrusionValueHexKey** | Tests fallback logic for hex-encoded keys used in some firmware versions. |
| **testTryReadExtrusionValueNotFound** | Ensures graceful failure when keys are missing. |
| **testSDCPConstants** | Validates protocol version strings and port constants. |

#### 4. `test_additional_edge_cases.cpp` (Integration Scenarios)
Tests complex interactions and boundary conditions.

| Test Case | Goal |
| :--- | :--- |
| **testJamDetectorRapidStateChanges** | Tests rapid alternation between good and bad flow (stability check). |
| **testJamDetectorVeryLongPrint** | Simulates a 24-hour print to check for variable overflows or drift. |
| **testJamDetectorExtremelySlowPrinting** | Validates detection at very slow speeds (0.1mm/sec). |
| **testJamDetectorTelemetryLoss** | Simulates SDCP connection loss; ensures no false jams. |
| **testJamDetectorMultipleResumeGraces** | Tests multiple consecutive pause/resume cycles. |
| **testSDCPProtocolEmptyMainboardId** | Graceful degradation if Mainboard ID is missing. |
| **testSDCPProtocolVeryLongRequestId** | Buffer overflow protection for long UUIDs. |
| **testIntegrationJamRecoveryWithResume** | Full cycle: Detect Jam -> Pause -> Resume -> Normal (Integration). |
| **testIntegrationMixedJamTypes** | Tests transition from Soft Jam condition directly to Hard Jam. |

### B. Python Tooling Tests (`test_tools.py`)

| Test Class | Goal |
| :--- | :--- |
| **TestBoardConfig** | Validates `board_config.py` correctly loads board definitions and handles invalid inputs. |
| **TestExtractLogData** | Tests the log parser used for visualization and debugging. |
| **TestGenerateTestSettings** | Ensures `user_settings.json` generation works for test environments. |
| **TestBuildScripts** | Integrity checks for `build_and_release.py` and other CI scripts. |
| **TestFixtures** | Validates that required test log fixtures exist and are readable. |
| **TestDataFiles** | Checks the integrity and JSON validity of `data/user_settings.json`. |

### C. JavaScript/Web Tests (`test_distributor.js`)

| Test Function | Goal |
| :--- | :--- |
| **testDistributorFilesExist** | Checks that `distributor/index.html` and assets exist. |
| **testBoardsJsonValid** | Validates `boards.json` schema and version consistency. |
| **testManifestJsonValid** | Validates firmware manifest files for OTA updates. |
| **testWifiPatcherStructure** | Checks `wifiPatcher.js` for expected class structures. |
| **testAppJsStructure** | Validates `app.js` (WebSerial logic) structure. |
| **testIndexHtmlValid** | Basic HTML validation for the distributor page. |
| **testWebUILiteFiles** | Ensures `webui_lite` build artifacts exist. |
| **testLiteOTAJsStructure** | Validates client-side OTA logic structure. |
| **testDevServerJs** | Checks the local development server script. |

---

## 4. Prerequisites

To run the full suite, your development environment needs:

1.  **C++ Compiler:** `g++` with C++11 support.
    *   *Linux/macOS:* Standard via `build-essential` or Xcode CLI tools.
    *   *Windows:* MinGW or MSYS2.
2.  **Python:** Version 3.6 or later.
3.  **Node.js:** Version 12.0 or later.

---

## 5. Detailed Usage Guide

### Running Individual Test Suites

If you only want to test specific components, you can run them individually:

**1. Pulse Simulator (Sensor Logic)**
```bash
cd test
./build_tests.sh
# Or manually:
# g++ -std=c++11 -o pulse_simulator pulse_simulator.cpp -I.. && ./pulse_simulator
```

**2. Specific C++ Unit Tests**
```bash
# Jam Detector
g++ -std=c++11 -o test_jam_detector test_jam_detector.cpp -I. -I.. && ./test_jam_detector

# SDCP Protocol
g++ -std=c++11 -o test_sdcp_protocol test_sdcp_protocol.cpp -I. -I.. && ./test_sdcp_protocol
```

**3. Python Tests**
```bash
python3 test/test_tools.py
```

**4. JavaScript Tests**
```bash
node test/test_distributor.js
```

### Visualizing Flow Data
The `pulse_simulator` can export CSV data to visualize how the jam detection logic reacts to filament movement.

1.  **Generate Log:**
    ```bash
    ./test/pulse_simulator --log test/render/filament_log.csv
    ```
2.  **Start Server:**
    ```bash
    cd test/render
    python3 -m http.server
    ```
3.  **View:** Open `http://localhost:8000` in your browser. You can replay the test scenarios and see the "Expected" vs "Actual" extrusion graphs.

---

## 6. Developing New Tests

### Adding C++ Tests
1.  Create a new file `test/test_<component>.cpp`.
2.  Include `Arduino.h` (the mock version in `test/`) and your target header.
3.  Follow the pattern of `test_jam_detector.cpp`:
    *   Use `printTestHeader("Description")`.
    *   Perform logic.
    *   Use `assert()` or simple boolean checks.
    *   Call `recordTest(description, result)`.
4.  Add your new file to `build_and_run_all_tests.sh` to include it in the main run.

### Adding Python Tests
1.  Open `test/test_tools.py`.
2.  Add a new class inheriting from `unittest.TestCase`.
3.  The main runner will automatically pick it up.

### Adding JS Tests
1.  Open `test/test_distributor.js`.
2.  Add a new async function `testYourFeature()`.
3.  Add the function call to the `runAllTests()` list at the bottom of the file.

---

## 7. Current Coverage & Limitations

**Coverage Statistics (as of Dec 2025):**
*   **Total Tests:** 100+
*   **Core Logic:** 100% coverage of `JamDetector`, `SDCPProtocol`, and `FilamentMotionSensor`.
*   **Tools:** High coverage of board config and build scripts.

**Limitations (Integration Testing Required):**
The following components have dependencies that are mocked in these tests. Changes to these specific files should be verified with **Integration Tests** on actual hardware:
*   `SystemServices` (WiFi, NTP)
*   `ElegooCC` (Real printer SDCP connection)
*   `Logger` & `SettingsManager` (Real LittleFS filesystem operations)

---

## 8. CI/CD Integration

This suite is used in the GitHub Actions workflow (`.github/workflows/run-tests.yml`).
*   **Exit Code 0:** All tests passed.
*   **Exit Code 1:** Failure.

When submitting PRs, ensure `./build_and_run_all_tests.sh` passes locally.