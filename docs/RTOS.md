# SD Logging + WebSocket Streaming on FreeRTOS

How the nodes keep SD logging stable while a WiFi/WebSocket task runs on the same MCU. Current reference: [src/node_rear.cpp](../src/node_rear.cpp) and [lib/SD32_util/](../lib/SD32_util/).

---

## Goal

One ESP32-S3 must do three things concurrently without the SD cadence drifting:

1. Read sensors at a steady rate (20–50 Hz).
2. Append a CSV row to SD at a fixed interval (e.g. every 200 ms).
3. Publish JSON frames over a WSS WebSocket at 1–2 Hz.

Naive single-threaded polling collapsed under WiFi: `BPwebSocket->loop()` and `sendTXT()` block 50–400 ms while the radio transmits or waits for a server ACK, which pushes SD writes into "catch-up" double-writes.

---

## Architecture Overview

```
Core 0 (Comms)                  Core 1 (Real-time)
┌──────────────────┐             ┌──────────────────┐
│ BPMobileTask     │             │ sensorTask       │
│ pri 1, 10 ms     │             │ pri 5, 20 ms     │
│ WebSocket loop + │             │ Read ADC/I2C/    │
│ JSON publish     │             │ GPS/IMU → local  │
└──────────────────┘             │ copy to shared   │
                                 │ struct           │
┌──────────────────┐             └──────────────────┘
│ timeSyncTask     │             ┌──────────────────┐
│ pri 3, 100 ms    │             │ sdTask           │
│ RTC poll 1 s     │             │ pri 2, queue-    │
│ NTP recal 60 s   │             │ blocked          │
└──────────────────┘             │ Dequeue → append │
                                 │ CSV → flush      │
                                 └──────────────────┘
                                 ┌──────────────────┐
                                 │ loop()           │
                                 │ SD enqueue +     │
                                 │ debug/teleplot   │
                                 └──────────────────┘
```

Four FreeRTOS tasks plus `loop()`. Core 0 owns network + time; Core 1 owns sensors + SD. Shared state is a handful of structs (`Mechanical`, `Odometry`, etc.) guarded by a short-held mutex. SD work is decoupled from `loop()` by a queue.

<!-- ```
┌─────────────────────────────────────────────────────┐
│                    ESP32-S3 Node                    │
│                                                     │
│  Core 1 (Real-time)        Core 0 (Comms)           │
│  ┌──────────────┐          ┌──────────────────┐     │
│  │ sensorTask   │          │ BPMobileTask     │     │
│  │ (50–200 Hz)  │          │ Publish JSON 1Hz │     │
│  │  ADC/I2C/    │          │ WebSocket (WSS)  │     │
│  │  CAN/UART    │          └──────────────────┘     │
│  └──────┬───────┘                                   │
│         │ mutex               ┌──────────────────┐  │
│  ┌──────▼───────┐             │ TimeSyncTask     │  │
│  │ Shared Struct│             │ RTC poll 1s      │  │
│  │ (Mechanical, │             │ NTP recal 60s    │  │
│  │  Electrical, │             └──────────────────┘  │
│  │  Odometry,   │                                   │
│  │  BAMOCar,    │  ┌──────────────────────────┐     │
│  │  BMUdata)    │  │ sdTask (Core 1)          │     │
│  └──────┬───────┘  │ Dequeue → append CSV     │     │
│         │          │ Flush every 5s           │     │
│  ┌──────▼───────┐  │ Rotate file every 1000 r │     │
│  │  SDLogEntry  │  └──────────────────────────┘     │
│  │  Queue(30)   │                                   │
│  └──────────────┘                                   │
└─────────────────────────────────────────────────────┘
         │                          │
    SD Card (CSV)           WebSocket Server
    /Node_session_N/        blackpearl-ws.onrender.com
    File_0..N.csv           WSS:443
``` -->

---

## Task Responsibilities

| Task | Core | Pri | Period | Role |
|------|------|-----|--------|------|
| `BPMobileTask` | 0 | 1 | 10 ms | Run WebSocket loop, snapshot shared structs, publish JSON at sensor-group rates |
| `timeSyncTask` | 0 | 3 | 100 ms | Poll RTC every 1 s for `RTC_UNIX_TIME`; every 60 s recalibrate against NTP if drift > 1 s |
| `sensorTask` | 1 | 5 | 20 ms | Read sensors into local copies, then briefly take `dataMutex` to write into shared structs |
| `sdTask` | 1 | 2 | queue | `xQueueReceive(portMAX_DELAY)` → append CSV row → flush on interval → rotate file on row limit |
| `loop()` | 1 | — | 20 ms | At `SD_APPEND_INTERVAL`, snapshot shared struct and enqueue `SDLogEntry`. Also handles debug/teleplot output |

`sensorTask` has the highest priority so sensor cadence isn't preempted by logging or publishing. `sdTask` is lowest on Core 1 — it only runs between sensor samples and only when a queue entry is waiting.

---

## Persistent File Logging

SD file handling avoids per-write `open()`/`close()` (~15–70 ms each) by keeping one file open for the session and flushing on a separate cadence.

### Three-tier timing

```cpp
const unsigned long SD_APPEND_INTERVAL = 200;   // append RAM → file buffer
const unsigned long SD_FLUSH_INTERVAL  = 1000;  // commit buffer → SD
const unsigned long SD_CLOSE_INTERVAL  = 60000; // optional: rotate file
const int SD_MAX_ROWS = ...;                    // row-based rotation
```

| Op | Cost | Purpose |
|----|------|---------|
| Append | 2–5 ms | Fast — writes into the open `File` object's RAM buffer |
| Flush | 50–80 ms | Commits buffered data to the card |
| Close/rotate | ~100 ms | New file per lap / every N rows, caps data loss on corruption |

Trade-off: on power loss, up to `SD_FLUSH_INTERVAL` worth of data is lost. Acceptable for automotive logging.

### File rotation

`sdTask` counts rows and rotates when `localDataPoint % SD_MAX_ROWS == 0`:

```cpp
SD32_closePersistentFile();
fileIndex++;
SD32_generateFilenameInDir(csvFilename, sessionDirPath, "File", fileIndex);
SD32_createCSVFile(csvFilename, csvHeaderBuffer);
SD32_openPersistentFile(csvFilename);
```

Files land in `/<Node>_session_N/File_0.csv`, `File_1.csv`, … AMS uses a multi-file variant (`SD32_openPersistentFileArray`) to keep 8 BMU files + 1 AMS file open simultaneously — see [AMS_datalog_reference.md](AMS_datalog_reference.md).

---

## Shared State + Mutex Pattern

One mutex (`dataMutex`) guards the shared sensor structs. The rule: **hold it only long enough to copy.**

```cpp
// Producer (sensorTask, Core 1)
Mechanical localMech;
StrokesensorUpdate(&localMech, ...);          // slow, OUTSIDE the lock
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
  myMechData = localMech;                     // fast struct assign
  xSemaphoreGive(dataMutex);
}

// Consumer (BPMobileTask, Core 0)
Mechanical localMech;
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
  localMech = myMechData;                     // fast struct copy
  xSemaphoreGive(dataMutex);
}
publishMechData(&localMech);                  // slow WiFi call OUTSIDE the lock
```

Critical sections are struct-copy-only (tens of microseconds). Blocking I/O — sensor reads, `BPwebSocket->sendTXT()`, SD writes — always happens outside the lock. Race windows are at most one sample stale, which is fine for telemetry.

---

## SD Queue Pattern

`loop()` produces; `sdTask` consumes. Decoupling means a slow SD flush can't delay `loop()` or the sensor task.

```cpp
struct SDLogEntry {
  int dataPoint;
  uint64_t unixTime;
  uint64_t sessionTime;
  Mechanical mech;
  Odometry odom;
};
QueueHandle_t sdQueue = xQueueCreate(30, sizeof(SDLogEntry));
```

- Producer (`loop()`): every `SD_APPEND_INTERVAL`, snapshot under mutex, `xQueueSend(sdQueue, &entry, 0)` with 0 timeout — drops on overflow rather than blocking.
- Consumer (`sdTask`): `xQueueReceive(sdQueue, &entry, portMAX_DELAY)` blocks idle, wakes only when data is queued.
- Depth 30 = ~6 s of buffer at 200 ms cadence — absorbs transient SD stalls from flushes or rotation.

The entry is ~150 bytes; the copy cost is negligible compared to flush time.

---

## Results

| Config | Double-writes / 100 logs | Max jitter |
|--------|--------------------------|------------|
| Original (single-threaded, open/close per write) | ~30 | 400 ms |
| Persistent file + timed flush | ~15 | 200 ms |
| + WiFi on Core 0 | ~8 | 100 ms |
| + Short-held mutex + queued SD | ~0–5 | < 50 ms |

---

## Hardware Notes

- **SD card**: use A1/A2 rated cards — their 1500–4000 random-write IOPS matter more than sequential MB/s for small CSV appends.
- **SPI**: SD uses SPI; WiFi does not share the bus but competes for CPU during radio bursts. Core pinning is the mitigation.

---

## Key Functions

```cpp
// Persistent single file (used by Front/Rear/Bamo nodes)
SD32_openPersistentFile(fs::FS &fs, const char* filepath);
SD32_closePersistentFile();
SD32_appendBulkDataPersistent(appenders, data, count, flushMs, closeMs);
SD32_flushPersistentFile();

// Multi-file persistent (used by AMS node)
SD32_openPersistentFileArray(fs::FS &fs, const char* filepath);  // returns index
SD32_appendToPersistentFile(int index, AppenderFunc, void* data);
SD32_flushAllPersistentFiles();
```

---

## Appendix A — What Didn't Work Initially

During early development, several patterns caused regressions. All were later reintroduced successfully, but it's useful to record the original failure modes so the mistakes aren't repeated.

### A.1 Mutex held across blocking I/O

Taking `dataMutex` before calling `BPwebSocket->sendTXT()` or a sensor read caused the WiFi task to hold the lock for 100+ ms, starving the sensor and SD tasks.

```cpp
// Broken pattern
xSemaphoreTake(dataMutex, portMAX_DELAY);
BPwebSocket->sendTXT(msg);   // 50–400 ms with the lock still held
xSemaphoreGive(dataMutex);
```

### A.2 SD writes queued without a dedicated consumer

Queueing SD writes while leaving the consumer logic in `loop()` only moved the bottleneck. Large `SDLogEntry` copies happened, but flushes still blocked the same core as the producer.

### A.3 Serial.printf in hot paths

Multiple tasks printing simultaneously contended on the UART and added unpredictable 5–20 ms stalls inside time-critical sections.

### A.4 Rejected approach: no mutex at all

At one point all locking was removed to "just let it race." This worked for scalar fields but produced torn reads on multi-field structs (e.g. GPS lat/lng half-updated), which corrupted downstream logs. Short-held mutex replaced it.

---

## Appendix B — Why Some of These Work Now

The patterns in Appendix A aren't inherently broken — they failed under specific integrations. The current architecture reintroduces them safely:

| Pattern | Why it failed before | Why it works now |
|---------|----------------------|------------------|
| Mutex | Held across 100+ ms WiFi call | Held only for struct copy (~µs). Blocking I/O moved outside the critical section |
| SD queue | Consumer still on same execution path as producer | Dedicated `sdTask` on Core 1, blocks on `xQueueReceive` — producer never waits for SD |
| Large struct in queue | Copy cost was blamed, but real bottleneck was flush timing | Flush decoupled via `SD_FLUSH_INTERVAL`; struct copy is trivial vs. flush |
| Serial debug | Printed from every task unconditionally | Gated behind `DEBUG_MODE` and kept out of `sensorTask` / `sdTask` hot paths |

**Rule of thumb:** the primitives (mutex, queue, cross-core tasks) are fine. What matters is *what runs inside them*. Keep critical sections to memory copies, keep blocking I/O in dedicated tasks, and keep consumers on a different core or priority level than producers.

*This appendix can be deleted once the team no longer needs the historical context.*
