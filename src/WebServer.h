#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncEventSource.h>
#include <ElegantOTA.h>
#include <LittleFS.h>

#include "SettingsManager.h"
#include "ElegooCC.h"

// Define SPIFFS as LittleFS
#define SPIFFS LittleFS

// Maximum SSE clients allowed simultaneously
static constexpr int kMaxSSEClients = 4;

class WebServer
{
   private:
    AsyncWebServer server;
    AsyncEventSource statusEvents;
    unsigned long lastStatusBroadcastMs = 0;
    unsigned long statusBroadcastIntervalMs = 5000;

    // --- Thread-safe command queue (async handlers set flags, loop() processes) ---

    // Pending settings update: async handler parses JSON into this doc, loop() applies it
    volatile bool pendingSettingsUpdate = false;
    StaticJsonDocument<1024> pendingSettingsDoc;
    portMUX_TYPE pendingMutex = portMUX_INITIALIZER_UNLOCKED;

    // Pending action commands from web handlers
    volatile bool pendingPause = false;
    volatile bool pendingResume = false;
    volatile bool pendingDiscovery = false;
    volatile bool pendingReconnect = false;  // Set when IP changed during settings update

    // --- Pre-built cached responses (built in loop(), served from async handlers) ---

    // Cached sensor status JSON (rebuilt every loop iteration)
    String cachedSensorStatusJson;
    sdcp_print_status_t cachedPrintStatus = SDCP_PRINT_STATUS_IDLE;
    portMUX_TYPE cacheMutex = portMUX_INITIALIZER_UNLOCKED;

    // Cached settings JSON (rebuilt when settings change)
    String cachedSettingsJson;
    volatile bool settingsJsonDirty = true;  // Start dirty to build initial cache

    // --- SSE deduplication ---
    // CRC32 of last idle payload for dedup (replaces full String comparison)
    uint32_t lastIdlePayloadCrc = 0;
    bool hasLastIdlePayload = false;

    // SSE client cleanup tracking
    unsigned long lastSSECleanupMs = 0;

    void buildStatusJson(StaticJsonDocument<768> &jsonDoc, const printer_info_t &elegooStatus);
    void broadcastStatusUpdate();
    void processPendingCommands();
    void refreshCachedResponses();
    void cleanupSSEClients();

    static uint32_t crc32(const char *data, size_t length);

   public:
    WebServer(int port = 80);
    void begin();
    void loop();
};

#endif  // WEB_SERVER_H
