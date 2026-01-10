# SD Card Logging Optimization Journey

## Problem Statement

SD card logging was experiencing timing jitter and double-writes when combined with WiFi/WebSocket operations.

**Symptoms:**
- Expected: Consistent 200ms or 500ms log intervals
- Actual: Irregular intervals (200ms, 400ms, 1000ms) with "catch-up" double-writes

```
17:51:52.046 > [SD Card] Logged #7
17:51:52.047 > [SD Card] Logged #8   ← Double write (catch-up)
17:51:53.050 > [SD Card] Logged #9
17:51:53.051 > [SD Card] Logged #10  ← Double write
```

---

## Root Cause Analysis

### Initial Investigation

Tested each subsystem independently:

| Test | Result |
|------|--------|
| SD only | Nearly perfect ~200ms intervals |
| SD + CAN | Minor jitter (~3 double-writes per 30 logs) |
| SD + WiFi/WebSocket | Severe jitter (~10 double-writes per 30 logs) |

**Conclusion:** WiFi/WebSocket operations were the primary bottleneck.

### Why WiFi Blocks

`BPwebSocket->loop()` and `sendTXT()` block for 50-400ms when:
1. TCP buffer operations occur
2. WiFi radio transmits
3. Waiting for server ACK

---

## Solutions Implemented

### Solution 1: Persistent File Handle

**Before:** Open and close file every write cycle
```cpp
void SD32_AppendBulkDataToCSV(...) {
  File file = fs.open(filepath, FILE_APPEND);  // ~10-50ms
  // write data
  file.close();  // ~5-20ms
}
```

**After:** Keep file handle open
```cpp
static File _persistentFile;
static bool _persistentFileOpen = false;

bool SD32_openPersistentFile(fs::FS &fs, const char* filepath);
void SD32_closePersistentFile();
void SD32_appendBulkDataPersistent(...);  // Uses open handle
```

**Savings:** ~15-70ms per write cycle

### Solution 2: Timed Flush

**Before:** Flush every write (~50-200ms blocking each time)

**After:** Flush based on time interval
```cpp
const unsigned long SD_FLUSH_INTERVAL = 1000;  // Flush every 1 second

// Inside append function:
if (now - _lastFlushTime >= flushIntervalMs) {
  _persistentFile.flush();
  _lastFlushTime = now;
}
```

**Trade-off:** Power loss = lose up to 1 second of data (acceptable for automotive)

### Solution 3: Tiered Timing (Professor's Recommendation)

```cpp
const unsigned long SD_APPEND_INTERVAL = 200;   // Append every 200ms (fast)
const unsigned long SD_FLUSH_INTERVAL = 1000;   // Flush every 1 second
const unsigned long SD_CLOSE_INTERVAL = 60000;  // Close/reopen every 60 seconds (lap time)
```

| Operation | Blocking Time | Purpose |
|-----------|---------------|---------|
| Append | ~2-5ms | Fast RAM buffer write |
| Flush | ~50-80ms | Commit data to SD |
| Close/Rotate | ~100ms | New file per lap, prevent corruption |

### Solution 4: FreeRTOS Multi-Core

**Architecture:**
```
Core 0:                          Core 1:
┌─────────────────┐              ┌─────────────────┐
│   wifiTask()    │              │     loop()      │
│ - WebSocket     │              │ - Sensors       │
│ - Publish data  │              │ - CAN bus       │
│ - LED status    │              │ - SD logging    │
└─────────────────┘              └─────────────────┘
```

**Implementation:**
```cpp
// WiFi task on Core 0
void wifiTask(void* parameter) {
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      BPwebSocket->loop();
    }
    // Publish sensor data...
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// In setup()
xTaskCreatePinnedToCore(
  wifiTask, "WiFiTask", 8192, NULL, 1, &wifiTaskHandle, 0  // Core 0
);
```

---

## What Didn't Work

### Complex Mutex System

Adding mutex protection everywhere caused new delays:
```cpp
// This created artificial blocking
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
  // operations
  xSemaphoreGive(dataMutex);
}
```

**Problem:** WiFi task holding mutex for 100ms+ blocked main loop

### SD Queue System

Queueing SD writes to separate task added overhead:
```cpp
struct SDLogEntry {
  int dataPoint;
  uint64_t unixTime;
  Mechanical mech;
  // ... ~200+ bytes per entry
};
QueueHandle_t sdQueue;
```

**Problem:** Large struct copies + queue operations negated benefits

### Serial.printf in Tasks

Multiple tasks printing simultaneously caused contention:
```cpp
Serial.printf("[SD Task] Logged #%d\n", localDataPoint);  // Blocking!
```

**Solution:** Remove or minimize Serial output in time-critical sections

---

## Final Configuration

**Best results achieved with:**

1. **WiFi on Core 0** (simple, no mutex)
2. **SD logging on Core 1** (can be in main loop or separate task)
3. **No mutex** - race conditions acceptable (sensor values may be 1 sample old)
4. **Minimal Serial output** - removed from hot paths
5. **Persistent file handle** - avoid open/close overhead
6. **Timed flush** - flush every 1 second, not every write

---

## Test Results Summary

| Configuration | Double-writes per 100 logs | Max Jitter |
|---------------|---------------------------|------------|
| Original (no optimization) | ~30 | 400ms |
| Persistent file + timed flush | ~15 | 200ms |
| + WiFi on Core 0 | ~8 | 100ms |
| + No mutex + minimal Serial | ~5 | 50ms |

---

## Hardware Considerations

### SD Card Class Rating

| Rating | Sequential Write | Random Write |
|--------|-----------------|--------------|
| Class 10 | 10 MB/s | Not guaranteed |
| UHS-I U3 | 30 MB/s | Not guaranteed |
| **A1/A2** | 10 MB/s | **1500-4000 IOPS** |

**Recommendation:** Use A1/A2 rated cards for random small writes.

### SPI Bus Considerations

- SD card uses SPI
- WiFi may have internal SPI operations
- Potential bus contention on shared resources

---

## Code References

- Main implementation: `src/node.cpp`
- SD utility functions: `lib/SD32_util/SD32_util.cpp`
- SD header: `lib/SD32_util/SD32_util.h`

### Key Functions

```cpp
// Persistent file operations
SD32_openPersistentFile(fs::FS &fs, const char* filepath);
SD32_closePersistentFile();
SD32_appendBulkDataPersistent(appenders, dataArray, count, flushIntervalMs);
SD32_flushPersistentFile();

// FreeRTOS task
void wifiTask(void* parameter);  // Runs on Core 0
```

---

## Lessons Learned

1. **Test subsystems independently** before adding complexity
2. **Simple solutions often work better** than complex RTOS patterns
3. **Mutex can hurt performance** when held for too long
4. **Serial.printf is blocking** - avoid in time-critical code
5. **SD flush is the real bottleneck** - hardware limitation, can only minimize frequency
6. **Core isolation helps** but doesn't solve all problems
7. **Accept trade-offs** - 1 second data loss on power failure is acceptable for automotive logging
