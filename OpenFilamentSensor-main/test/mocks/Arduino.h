/**
 * Arduino Mock Header
 *
 * Provides comprehensive Arduino environment mocks for desktop testing.
 * This file should be included BEFORE any actual Arduino framework headers.
 */

#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

// Include core Arduino mocks
#include "arduino_mocks.h"
#include "test_mocks.h"

// Include JSON support
#include "json_mocks.h"
#include "ArduinoJson.h"

// Include other required mocks but be careful with class redefinitions
// These will be included only if not already defined by real headers
#include "Logger.h"
#include "SettingsManager.h"

#endif // ARDUINO_MOCK_H
