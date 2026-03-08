# BP_Bridge-node

Distributed telemetry and data logging firmware for the **BP16B Formula Student Electric Vehicle**. Each ESP32-S3 board acts as a region-specific sensor node — collecting, timestamping, logging to SD card, and publishing live data to a WebSocket server over WiFi.

---

## What This Project Does

The car is split into four instrumented regions, each with its own ESP32-S3 board:

| Node | Region | Data Collected |
|------|--------|----------------|
| **Front** | Front suspension + electrical | Stroke sensors, pedal position (APPS/BPPS), current, motor temp, fault signals |
| **Rear** | Rear suspension + odometry | Wheel RPM, stroke sensors, GPS, 9-axis IMU |
| **BAMO** | Motor controller (Bamocar D3) | DC voltage, DC current, motor temp, controller temp via CAN bus |
| **AMS** | Battery management (8 BMUs) | Per-cell voltages, temperatures, fault states via CAN bus |

Data flows through two parallel paths simultaneously:
- **Live stream** → WebSocket publish (JSON, 1–2 Hz) to a cloud relay server
- **Persistent log** → SD card CSV, session-partitioned with automatic file rotation

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                    ESP32-S3 Node                    │
│                                                     │
│  Core 1 (Real-time)        Core 0 (Comms)          │
│  ┌──────────────┐          ┌──────────────────┐    │
│  │ sensorTask   │          │ BPMobileTask      │    │
│  │ (50–200 Hz)  │          │ Publish JSON 1Hz  │    │
│  │  ADC/I2C/    │          │ WebSocket (WSS)   │    │
│  │  CAN/UART    │          └──────────────────┘    │
│  └──────┬───────┘                                   │
│         │ mutex                ┌──────────────────┐ │
│  ┌──────▼───────┐             │ TimeSyncTask      │ │
│  │ Shared Struct│             │ RTC poll 1s       │ │
│  │ (Mechanical, │             │ NTP recal 60s     │ │
│  │  Electrical, │             └──────────────────┘ │
│  │  Odometry,   │                                   │
│  │  BAMOCar,    │  ┌──────────────────────────┐    │
│  │  BMUdata)    │  │ sdTask (Core 1)           │    │
│  └──────┬───────┘  │ Dequeue → append CSV      │    │
│         │          │ Flush every 5s            │    │
│  ┌──────▼───────┐  │ Rotate file every 1000 r  │    │
│  │  SDLogEntry  │  └──────────────────────────┘    │
│  │  Queue(30)   │                                   │
│  └──────────────┘                                   │
└─────────────────────────────────────────────────────┘
         │                          │
    SD Card (CSV)           WebSocket Server
    /Node_session_N/        blackpearl-ws.onrender.com
    File_0..N.csv           WSS:443
```

---

## Project Structure

```
BP_Bridge-node/
├── src/
│   ├── node_front.cpp      # Front node entry point + FreeRTOS tasks
│   ├── node_rear.cpp       # Rear node entry point + FreeRTOS tasks
│   ├── node_bamo.cpp       # BAMO node entry point + FreeRTOS tasks
│   ├── node_ams.cpp        # AMS node entry point + FreeRTOS tasks
│   ├── base_sensors.cpp    # ADC/encoder/digital-IO sensor reads
│   ├── bamo_helper.cpp     # Bamocar CAN encode/decode + NTC LUT
│   └── motion_sensors.cpp  # GPS (TinyGPS++) + IMU (MPU6050) reads
├── include/
│   ├── shared_config.h     # All pin defs, scaling constants, network config, debug flags
│   ├── base_sensors.h      # Mechanical/Electrical structs + function declarations
│   ├── bamo_helper.h       # BAMOCar struct + Bamocar CAN helpers
│   └── motion_sensors.h    # Odometry struct + GPS/IMU declarations
├── lib/
│   ├── BP_mobile_util/     # WebSocket lifecycle + pub/sub client
│   ├── SD32_util/          # SD card CSV appender + session/file management
│   ├── syncTime_util/      # RTC/NTP time sync, drift detection, Unix formatting
│   ├── CAN32_util/         # ESP32 TWAI wrapper (send/receive/filter)
│   ├── DS3221_util/        # DS3231 RTC init, set, and get via I2C
│   ├── WIFI32_util/        # WiFi connect + NTP time retrieval
│   └── ams_data_util/      # BMU/AMS data structs, CAN parsing, publish helpers
├── docs/                   # Architecture and protocol references
└── platformio.ini          # Build environments (front/rear/bamo/ams)
```

---

## Build & Dependencies

### Supported Board

**Espressif ESP32-S3-DevKitC-1** (`esp32-s3-devkitc-1`)
- USB CDC enabled at boot (`-D ARDUINO_USB_CDC_ON_BOOT=1`)
- AMS node uses `huge_app.csv` partition (required for AMS library size)

### External Libraries (auto-fetched by PlatformIO)

| Library | Version | Used By |
|---------|---------|---------|
| `links2004/WebSockets` | 2.7.1 | All nodes |
| `bblanchon/ArduinoJson` | 7.4.2 | All nodes |
| `adafruit/Adafruit BusIO` | 1.17.4 | All nodes |
| `adafruit/RTClib` | 2.1.4 | All nodes (DS3231) |
| `mikalhart/TinyGPSPlus` | 1.1.0 | Rear only |
| `rfetick/MPU6050_light` | 1.1.0 | Rear only |

### Build

```bash
# Install PlatformIO CLI
pip install platformio

# Build a specific node
pio run -e front
pio run -e rear
pio run -e bamo
pio run -e ams

# Upload to board
pio run -e front --target upload
```

Each environment compiles only its own source files. The `src_filter` in `platformio.ini` isolates per-node entry points — you cannot accidentally link the wrong node together.

---

## Configuration (`shared_config.h`)

All tunable constants live here. Key groups:

**Pin assignments** — GPIO numbers for encoders, ADC channels, SD SPI, I2C (RTC/IMU), UART (GPS), CAN (TWAI).

**Sensor scaling**
- Stroke sensors: 0–4095 ADC → 0–75 mm linear
- Current (Hall effect): `I = (V - 2.5) / 0.1` A
- APPS/BPPS: `position = (V / 3.3) × 75` mm
- Steering: 0–180° full range

**Network**
- WiFi SSID/password
- WebSocket server host + port (WSS/443)
- Publish rate and NTP servers

**Debug flags** — flip these at compile time, no code changes needed:

| Flag | Values | Effect |
|------|--------|--------|
| `DEBUG_MODE` | 0 / 1 / 2 | Off / Serial / Teleplot |
| `TIME_SRC` | 0 / 1 | RTC only / WiFi NTP |
| `MOCK_FLAG` | 0 / 1 | Real hardware / synthetic test data |

---

## Data Structures & Buffering

### Sensor Structs (per node)

```cpp
// Shared between sensorTask and output tasks (mutex protected)
Mechanical myMechData;   // Wheel RPM L/R, Heave/Roll stroke (mm)
Electrical myElectData;  // Current (A), Temp (°C), APPS/BPPS (%), fault bools, steering
Odometry   myOdomData;   // GPS lat/lng/speed/course/age, IMU accel/gyro xyz
BAMOCar    myBamoData;   // DC V, DC A, Power, motor temp, controller temp + validity flags
BMUdata    bmuArray[8];  // 8× { V_CELL[10], TEMP_SENSE[2], V_MODULE, DV, fault thresholds }
```

### SD Buffering — Queue Pattern

```
sensorTask writes → shared struct (mutex)
                          │
                     loop() samples at 1Hz
                          │
                    SDLogEntry (timestamp + copies of all structs)
                          │
                    xQueueSend(sdQueue, max 30 entries)
                          │
                     sdTask dequeues → appends CSV row
                          │
                     File flush every 5s → file rotate every 1000 rows
```

The queue decouples the fast sensor task from the slow SD writes. 30 entries provides a ~30-second cushion if the SD card stalls. The queue blocks on full — this is a known design trade-off (sensor reads may lag instead of dropping).

**File layout on SD:**
```
/Front_session_1/File_0.csv   (rows 0–999)
/Front_session_1/File_1.csv   (rows 1000–1999)
/AMS_session_1/AMS_p0/bmu_0.csv .. bmu_7.csv   (AMS has one file per BMU)
```

Sessions increment on each reboot. Partitions rotate every 1000 rows.

---

## Custom Libraries

| Library | What It Does |
|---------|-------------|
| **BP_mobile_util** | Manages the WebSocket client lifecycle — connects, reconnects, registers topics, and dispatches publish calls. Accepts callbacks for topic registration and time sourcing. |
| **SD32_util** | SD card CSV logging. Uses a function-pointer *appender* pattern: each data group provides an `AppenderFunc` that writes its own columns into an open `File` handle. Handles session dirs, file rotation, and periodic flush. |
| **syncTime_util** | Abstracts over RTC and NTP. Sets a sync point (records `millis()` offset at last known Unix time), computes relative session elapsed time, and detects drift to trigger RTC recalibration only when drift exceeds 1 s. |
| **CAN32_util** | Thin wrapper around ESP32's TWAI peripheral. Handles bus init, acceptance filters, non-blocking send/receive, and debug frame printing. |
| **DS3231_util** | Initialises DS3231 RTC over I2C, reads Unix timestamp, and sets time from a Unix epoch value. |
| **WIFI32_util** | WiFi connect with configurable retry count, NTP initialisation (dual-server fallback), and returns current Unix time in milliseconds. |
| **ams_data_util** | Defines `BMUdata`/`AMSdata`/`OBCdata` structs, CAN frame parser for BMU messages, WebSocket publish helpers for per-cell and per-fault groups, and Teleplot/mock data output. |

---

## Debug Strategies

**1. Teleplot (`DEBUG_MODE 2`)**
Set `DEBUG_MODE 2` in `shared_config.h`. Sensor values print as `>variable:value` on Serial. Open the [Teleplot VSCode extension](https://github.com/nesnes/teleplot) to get live graphs without any external tooling.

**2. Serial console (`DEBUG_MODE 1`)**
Human-readable structured output for task state, WiFi status, RTC sync events, CAN frame dumps.

**3. Mock data (`MOCK_FLAG 1`)**
Each node has `mock*()` functions that generate synthetic time-varying values. Set `MOCK_FLAG 1` to run the full pipeline (SD + WebSocket) without any physical hardware connected.

**4. RTC calibration mode**
Set `calibrate_RTC 1` in the node source. The board runs only the RTC-set loop and prints the result — use this to verify the DS3231 is responding before a full deployment.

---

## Design Critique

> A personal note on what works, what doesn't, and where the real problems lie.

The region-per-node split is a reasonable embedded systems decision — it keeps CAN bus topologies short, limits the number of peripherals per MCU, and maps naturally to the physical layout of the car. The four-node architecture is not the design problem.

**The actual problem is the per-node duplication of encode/decode logic.**

Every node independently owns its own CSV column layout, its own WebSocket topic group names, its own JSON serialization code, and its own appender functions. If you rename a field, add a sensor, or change units, you touch a minimum of four files across two layers (struct header + CSV appender + WebSocket publisher + docs). There is no single source of truth for "what does the front node publish."

This is a schema ownership problem. The structs in `base_sensors.h` are the closest thing to a schema, but they carry no metadata about column names, units, or wire format — those are scattered across appender functions and string literals in each node file.

A cleaner approach would be a **descriptor table**: a static array of `{field_name, unit, getter_fn, scale}` entries defined once per struct type. The CSV appender, WebSocket serializer, and documentation generator all derive from the same table. Adding a sensor means adding one row — not hunting through five files.

The counter-argument is complexity cost: descriptor tables in C++ require either macros, templates, or runtime reflection, all of which add cognitive overhead in a codebase that needs to be readable by Formula Student team members who may not be C++ experts. The current duplication is verbose but debuggable by anyone who can read a struct.

The pragmatic fix that doesn't require rearchitecting: **move all topic names, CSV headers, and column order into `shared_config.h` as string constants.** The logic stays distributed; at least the names don't. That alone would halve the risk of the front node and the WebSocket consumer disagreeing about what `front.elect` contains.

---

## WebSocket Protocol

Topics follow the pattern `node.group` (e.g. `front.elect`, `bamo.power`, `bmu3.faults`).
See [docs/websocket_protocol_reference.md](docs/websocket_protocol_reference.md) for the full message schema.

---

## Related Docs

- [docs/Timesync_architecture.md](docs/Timesync_architecture.md) — RTC/NTP sync strategy
- [docs/SD_csv_appender.md](docs/SD_csv_appender.md) — Appender pattern explained
- [docs/Make SD&websocket_multitask.md](docs/Make%20SD%26websocket_multitask.md) — FreeRTOS task design
- [docs/teleplot_debug_implementation.md](docs/teleplot_debug_implementation.md) — Teleplot setup
- [docs/datalog_update_session.md](docs/datalog_update_session.md) — Session/partition rotation logic
