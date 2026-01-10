# Time Synchronization System Implementation

## Overview

This document describes the time synchronization architecture implemented for the BP_Bridge-node project. The system provides source-agnostic time management with support for multiple time sources (RTC, NTP, WebSocket Server).

## Architecture

```
                    ┌─────────────────────────────────────┐
                    │         EXTERNAL SOURCES            │
                    │  ┌─────────┐     ┌─────────────┐   │
                    │  │ Server  │     │    NTP      │   │
                    │  │  Time   │     │   Pool      │   │
                    │  └────┬────┘     └──────┬──────┘   │
                    │       │                 │          │
                    └───────┼─────────────────┼──────────┘
                            │                 │
                            └────────┬────────┘
                                     │
                                     ▼
                    ┌─────────────────────────────────────┐
                    │   syncTime_fromExternal()           │
                    │   - Check drift threshold           │
                    │   - Update DEVICE_UNIX_TIME         │
                    │   - Update RTC hardware (optional)  │
                    └─────────────────┬───────────────────┘
                                      │
              ┌───────────────────────┼───────────────────────┐
              │                       │                       │
              ▼                       ▼                       ▼
    ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
    │ DEVICE_UNIX_TIME│     │  RTC Hardware   │     │ deviceTimeSync  │
    │   (RAM)         │     │  (Persistent)   │     │   Point (ms)    │
    └─────────────────┘     └─────────────────┘     └─────────────────┘
              │
              ▼
    ┌─────────────────────────────────────┐
    │   syncTime_getRelative()            │
    │   DEVICE_UNIX_TIME + elapsed millis │
    └─────────────────────────────────────┘
              │
              ▼
    ┌─────────────────────────────────────┐
    │           OUTPUTS                   │
    │  - CSV Logging (UnixTime)           │
    │  - WebSocket Publishing             │
    │  - Display (formatted)              │
    └─────────────────────────────────────┘
```

## Design Principles

1. **Source-Agnostic**: `DEVICE_UNIX_TIME` can come from ANY source (RTC, NTP, Server)
2. **Millisecond Precision**: Uses `millis()` to add sub-second precision to low-resolution sources
3. **Automatic Resync**: Periodic sync every 60 seconds with drift detection
4. **RTC Write-Back**: External time updates can optionally persist to RTC hardware

## Libraries Modified/Created

### 1. WIFI32_util (lib/WIFI32_util/)

WiFi initialization extracted from main.cpp with NTP support added.

**Header (WIFI32_util.h):**
```cpp
// WiFi initialization
void initWiFi(const char* ssid, const char* password, int attempt);
bool WiFi32_isConnected();
int  WiFi32_getRSSI();

// NTP Time functions
void WiFi32_initNTP(const char* ntpServer1 = "pool.ntp.org",
                    const char* ntpServer2 = "time.nist.gov",
                    long gmtOffsetSec = 0);
uint64_t WiFi32_getNTPTime();  // Returns Unix timestamp in ms, 0 if unavailable
bool WiFi32_isNTPSynced();
```

### 2. syncTime_util (lib/syncTime_util/)

Core time synchronization library.

**Key Functions:**
```cpp
// Set absolute time from any source
void syncTime_setAbsolute(uint64_t &deviceAbsoluteTime, uint64_t unixTimeMs);

// Get current time (base + elapsed millis)
uint64_t syncTime_getRelative(uint64_t deviceAbsoluteTime);

// Check if synced
bool syncTime_isSynced();

// Sync from external source with optional RTC write-back
bool syncTime_fromExternal(uint64_t &deviceTime, uint64_t externalTimeMs,
                           void* rtcPtr = nullptr, uint64_t driftThreshold_ms = 1000ULL);

// Get drift without triggering sync
int64_t syncTime_getDrift(uint64_t deviceTime, uint64_t externalTimeMs);

// Formatting
void syncTime_formatUnix(char* outBuf, uint64_t unixMs, int timezoneOffsetHours);
void syncTime_formatUnix_UTC(char* outBuf, uint64_t unixMs);
```

### 3. BP_mobile_util (lib/BP_mobile_util/)

WebSocket server time functions (standalone, no existing code modified).

**New Functions:**
```cpp
// Request server time via WebSocket
void BPMobile_requestServerTime(WebSocketsClient* ws);

// Parse server time from incoming message
uint64_t BPMobile_parseServerTime(const char* payload);

// Get last received server time
uint64_t BPMobile_getLastServerTime();
```

## Usage in node.cpp

### Setup
```cpp
// Connect to WiFi
initWiFi(ssid, password, 10);
if (WiFi.status() == WL_CONNECTED) {
  // Initialize NTP after WiFi connects
  WiFi32_initNTP();

  // Initialize WebSocket...
}

// Sync time from RTC initially
if (RTCavailable) {
  DateTime now = rtc.now();
  syncTime_setAbsolute(DEVICE_UNIX_TIME, (uint64_t)now.unixtime() * 1000ULL);
}
```

### Loop (Periodic Sync)
```cpp
// Every 60 seconds
if (SESSION_TIME - lastExternalSync >= EXTERNAL_SYNC_INTERVAL) {
  uint64_t externalTime = 0;
  bool gotExternalTime = false;

  // Try server time first
  uint64_t serverTime = BPMobile_getLastServerTime();
  if (BPsocketstatus->isConnected && serverTime > 0) {
    externalTime = serverTime;
    gotExternalTime = true;
  }
  // Fallback to NTP
  else if (WiFi32_isNTPSynced()) {
    externalTime = WiFi32_getNTPTime();
    if (externalTime > 0) {
      gotExternalTime = true;
    }
  }

  // Sync if drift > 1 second
  if (gotExternalTime) {
    syncTime_fromExternal(DEVICE_UNIX_TIME, externalTime,
                          RTCavailable ? &rtc : nullptr);
  }

  lastExternalSync = SESSION_TIME;
}
```

### Reading Time for Logging
```cpp
// Get current Unix time (auto-adds elapsed millis)
uint64_t CURRENT_UNIX_TIME = syncTime_getRelative(DEVICE_UNIX_TIME);

// Log to CSV
// UnixTime field: CURRENT_UNIX_TIME
// SessionTime field: millis()
```

## Time Source Priority

1. **Server Time** (preferred) - via WebSocket `BPMobile_getLastServerTime()`
2. **NTP Time** (fallback) - via `WiFi32_getNTPTime()`
3. **RTC Time** (initial) - via `rtc.now().unixtime()`

## Drift Handling

- Default threshold: 1000ms (1 second)
- If drift < threshold: Skip resync (avoid unnecessary updates)
- If drift >= threshold: Update DEVICE_UNIX_TIME and optionally RTC hardware

## DS3231 RTC Notes

- Drift: ~2 ppm = ~0.17 seconds/day = ~1 second/week
- For racing sessions (hours), RTC drift is negligible
- Periodic sync mainly catches:
  - Initial RTC error (was it set correctly?)
  - Power loss scenarios
  - Server/device clock desync

## Server Time Protocol

**Request:**
```json
{"type": "time_request"}
```

**Expected Response:**
```json
{"type": "time_response", "server_time": 1736000000000}
```

## Files Summary

| File | Purpose |
|------|---------|
| `lib/WIFI32_util/WIFI32_util.h` | WiFi + NTP function declarations |
| `lib/WIFI32_util/WIFI32_util.cpp` | WiFi + NTP implementation |
| `lib/syncTime_util/syncTime_util.h` | Time sync API declarations |
| `lib/syncTime_util/syncTime_util.cpp` | Time sync implementation |
| `lib/BP_mobile_util/BP_mobile_util.h` | Server time function declarations |
| `lib/BP_mobile_util/BP_mobile_util.cpp` | Server time implementation |
| `src/node.cpp` | Main application using all libraries |

## Configuration

```cpp
// In node.cpp
const unsigned long EXTERNAL_SYNC_INTERVAL = 60000;  // 1 minute
uint64_t DEVICE_UNIX_TIME = 0;  // Central time variable
```

## Build Info

- Platform: ESP32-S3
- Framework: Arduino
- RAM Usage: ~14.9%
- Flash Usage: ~29.9%
