# Time Synchronization System Implementation (Mk2)

## Overview

This document describes the time synchronization architecture implemented for the BP_Bridge-node project. The system uses a **two-tier RTC architecture**: ESP32's internal RTC for fast reads and DS3231 hardware RTC for persistence across power loss.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    REMOTE TIME SOURCES                              │
│         ┌─────────────┐           ┌─────────────┐                   │
│         │   Server    │           │    NTP      │                   │
│         │    Time     │           │   Pool      │                   │
│         └──────┬──────┘           └──────┬──────┘                   │
│                │ (preferred)             │ (fallback)               │
└────────────────┼─────────────────────────┼──────────────────────────┘
                 │                         │
                 └───────────┬─────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼ (every 1s)         │                    ▼ (every 60s, if drift > 1s)
┌───────────────────┐        │        ┌───────────────────────────────┐
│  ESP32 Internal   │        │        │   DS3231 Hardware RTC         │
│  RTC (time.h)     │        │        │   (Battery-backed persistent) │
│                   │◄───────┘        │                               │
│  settimeofday()   │                 │   rtc.adjust(DateTime(...))   │
└─────────┬─────────┘                 └───────────────────────────────┘
          │                                       │
          │ ◄─────────────────────────────────────┘
          │        (fallback if no Server/NTP)
          ▼
┌─────────────────────────────────────────────────────────────────────┐
│   DEVICE_UNIX_TIME = (uint64_t)time(NULL) * 1000ULL                 │
│   - Reads ESP32 internal RTC each loop iteration                    │
│   - Fast, low-overhead system call                                  │
└─────────────────────────────┬───────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│   syncTime_getElapse(DEVICE_UNIX_TIME)                              │
│   - Adds millis() offset for sub-second precision                   │
│   - Returns: DEVICE_UNIX_TIME + (millis() - sync_point)             │
└─────────────────────────────┬───────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         OUTPUTS                                     │
│   - CSV Logging (UnixTime in ms)                                    │
│   - WebSocket Publishing (timestamps)                               │
│   - Display (formatted via syncTime_formatUnix)                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Design Principles

1. **Two-Tier RTC**: ESP32 internal RTC for fast reads, DS3231 for power-loss persistence
2. **Millisecond Precision**: Uses `millis()` offset to add sub-second precision
3. **Tiered Sync Intervals**:
   - ESP32 RTC: every 1 second from best available source
   - DS3231 hardware: every 60 seconds (only if drift > 1s)
4. **Priority-Based Sources**: Server > NTP > DS3231 (fallback chain)
5. **Drift Detection**: Only update DS3231 when drift exceeds threshold (saves I2C writes)

## Libraries

### 1. WIFI32_util (lib/WIFI32_util/)

WiFi initialization with NTP support.

**Key Functions:**
```cpp
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

Core time synchronization library using ESP32's internal RTC.

**Key Functions:**
```cpp
// Set time and update ESP32 internal RTC via settimeofday()
void syncTime(uint64_t &localTime, uint64_t Timesource_ms);

// Get current time with millis() offset for sub-second precision
uint64_t syncTime_getElapse(uint64_t localTime);

// Check if synced
bool syncTime_isSynced();

// Get drift without triggering sync
int64_t syncTime_getDrift(uint64_t localTime, uint64_t TimeSource_ms);

// Sync from external source with optional DS3231 write-back
bool syncTime_fromExternal(uint64_t &localTime, uint64_t TimeSource_ms,
                           void* rtcPtr = nullptr, uint64_t driftThreshold_ms = 1000ULL);

// Formatting
void syncTime_formatUnix(char* outBuf, uint64_t unixMs, int timezoneOffsetHours);
void syncTime_formatUnix_UTC(char* outBuf, uint64_t unixMs);
```

### 3. BP_mobile_util (lib/BP_mobile_util/)

WebSocket server time functions.

**Key Functions:**
```cpp
void BPMobile_requestServerTime(WebSocketsClient* ws);
uint64_t BPMobile_parseServerTime(const char* payload);
uint64_t BPMobile_getLastServerTime();
```

## Usage in node.cpp

### Setup
```cpp
// Connect to WiFi
initWiFi(ssid, password, 10);
if (WiFi.status() == WL_CONNECTED) {
  WiFi32_initNTP();  // Initialize NTP after WiFi connects
  // Initialize WebSocket...
}

// Initialize DS3231 hardware RTC
RTCavailable = RTCinit(&Wire1);

// Initial time sync: prefer NTP, fallback to DS3231
syncTime(DEVICE_UNIX_TIME, WiFi32_isNTPSynced() ? WiFi32_getNTPTime() : RTC_getUnix());
```

### Loop - ESP32 RTC Sync (Every 1 Second)
```cpp
// Update ESP32 internal RTC from best available source
static unsigned long lastTimeSourceSync = 0;
if (SESSION_TIME - lastTimeSourceSync >= 1000) {
  uint64_t sourceTimeMs = 0;

  // Priority: Server > NTP > DS3231
  uint64_t serverTime = BPMobile_getLastServerTime();
  if (BPsocketstatus->isConnected && serverTime > 0) {
    sourceTimeMs = serverTime;
  } else if (WiFi32_isNTPSynced()) {
    sourceTimeMs = WiFi32_getNTPTime();
  } else if (RTCavailable) {
    sourceTimeMs = (uint64_t)rtc.now().unixtime() * 1000ULL;
  }

  if (sourceTimeMs > 0) {
    syncTime(DEVICE_UNIX_TIME, sourceTimeMs);  // Updates ESP32 RTC via settimeofday()
  }
  lastTimeSourceSync = SESSION_TIME;
}
```

### Loop - DS3231 Hardware Sync (Every 60 Seconds)
```cpp
// Persist network time to DS3231 for power-loss recovery
if (SESSION_TIME - lastExternalSync >= EXTERNAL_SYNC_INTERVAL) {
  uint64_t externalTime = 0;
  bool gotExternalTime = false;

  // Only sync from network sources (Server/NTP) to DS3231
  uint64_t serverTime = BPMobile_getLastServerTime();
  if (BPsocketstatus->isConnected && serverTime > 0) {
    externalTime = serverTime;
    gotExternalTime = true;
  } else if (WiFi32_isNTPSynced()) {
    externalTime = WiFi32_getNTPTime();
    if (externalTime > 0) gotExternalTime = true;
  }

  // Update DS3231 hardware RTC (only if drift > 1s)
  if (gotExternalTime && RTCavailable) {
    int64_t drift = syncTime_getDrift(DEVICE_UNIX_TIME, externalTime);
    if (llabs(drift) >= 1000) {
      rtc.adjust(DateTime((uint32_t)(externalTime / 1000ULL)));
      Serial.printf("[TimeSync] DS3231 updated, drift was %lld ms\n", drift);
    }
  }
  lastExternalSync = SESSION_TIME;
}
```

### Reading Time for Logging
```cpp
// Read ESP32 RTC (fast system call)
DEVICE_UNIX_TIME = (uint64_t)time(NULL) * 1000ULL;

// Get current Unix time with sub-second precision
uint64_t CURRENT_UNIX_TIME = syncTime_getElapse(DEVICE_UNIX_TIME);

// Use for CSV logging and WebSocket publishing
```

## Time Source Priority

| Priority | Source | Update Target | Interval |
|----------|--------|---------------|----------|
| 1 | Server Time (WebSocket) | ESP32 RTC | 1s |
| 2 | NTP Time | ESP32 RTC | 1s |
| 3 | DS3231 RTC | ESP32 RTC (fallback) | 1s |
| - | Server/NTP | DS3231 (persistence) | 60s |

## Sync Flow Summary

```
┌──────────────────────────────────────────────────────────────┐
│                        EVERY 1 SECOND                        │
├──────────────────────────────────────────────────────────────┤
│  Source: Server > NTP > DS3231                               │
│  Target: ESP32 internal RTC (via settimeofday)               │
│  Purpose: Keep ESP32 RTC accurate for time(NULL) reads       │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                       EVERY 60 SECONDS                       │
├──────────────────────────────────────────────────────────────┤
│  Source: Server > NTP (network sources only)                 │
│  Target: DS3231 hardware RTC (via rtc.adjust)                │
│  Condition: Only if drift > 1 second                         │
│  Purpose: Persist accurate time for power-loss recovery      │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                        EVERY LOOP                            │
├──────────────────────────────────────────────────────────────┤
│  DEVICE_UNIX_TIME = time(NULL) * 1000  (read ESP32 RTC)      │
│  CURRENT_UNIX_TIME = syncTime_getElapse(DEVICE_UNIX_TIME)    │
│  Purpose: Get timestamp for logging/publishing               │
└──────────────────────────────────────────────────────────────┘
```

## Drift Handling

- **ESP32 RTC sync**: No threshold, updates every second
- **DS3231 sync**: 1000ms threshold (only update if drift >= 1s)
- Drift calculation: `device_time - external_time` (positive = device ahead)

## DS3231 RTC Notes

- Drift: ~2 ppm = ~0.17 seconds/day = ~1 second/week
- For racing sessions (hours), RTC drift is negligible
- Primary role: **Persistence** across power loss, not accuracy
- 60-second sync interval sufficient for maintaining < 1s accuracy

## Server Time Protocol

**Request:**
```json
{"type": "time_request"}
```

**Response:**
```json
{"type": "time_response", "server_time": 1736000000000}
```

## Files Summary

| File | Purpose |
|------|---------|
| `lib/WIFI32_util/WIFI32_util.h` | WiFi + NTP function declarations |
| `lib/WIFI32_util/WIFI32_util.cpp` | WiFi + NTP implementation |
| `lib/syncTime_util/syncTime_util.h` | Time sync API declarations |
| `lib/syncTime_util/syncTime_util.cpp` | Time sync implementation (uses settimeofday) |
| `lib/BP_mobile_util/BP_mobile_util.h` | Server time function declarations |
| `lib/BP_mobile_util/BP_mobile_util.cpp` | Server time implementation |
| `src/node.cpp` | Main application with two-tier sync logic |

## Configuration

```cpp
// In node.cpp
const unsigned long EXTERNAL_SYNC_INTERVAL = 60000;  // DS3231 sync interval (1 minute)
uint64_t DEVICE_UNIX_TIME = 0;  // Central time variable (read from ESP32 RTC)
```

## Build Info

- Platform: ESP32-S3
- Framework: Arduino
- Dependencies: RTClib, time.h (ESP-IDF)
