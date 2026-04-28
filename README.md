# BP_Bridge-node

Educational example ESP32-S3 firmware for **BP16B Formula Student EV**. <br>
Intended for future development after BP16B <br>
Each MCU reads and processes sensor data, logs timestamped rows to SD card, and publishes live telemetry over WebSocket at the same time.


## What This Project Does

The car has two physical board placements. Each board runs a unified firmware binary that reads every sensor type at boot — unconnected hardware simply logs zeros and does not block the others.

| Board | Physical location | Data collected |
|-------|------------------|----------------|
| **Node** (×2) | Front + Rear suspension | Heave/Roll stroke, Wheel RPM, APPS/BPPS pedal, electrical fault flags, GPS, 9-axis IMU (BNO055), Bamocar CAN (DC V/A, motor temp) |
| **AMS** | Battery management system | Per-cell voltages, temperatures, fault states (8 BMU modules) |

The unified approach partially solves the BP16B problem of duplicated sensor code across four separate node files — all non-AMS sensors now share a single source and one compiled binary.

---

### Project Structure
```
├── src/
│   ├── node_unified.cpp    # Unified node: Front, Rear, and BAMO in one binary
│   ├── helper.cpp          # Sensor read/publish/CSV append helpers
│   └── node_ams.cpp        # AMS node (separate: needs 16 MB flash + FatFS)
├── include/
│   ├── shared_config.h     # Pin defs, network config, build flags
│   └── helper.h
├── lib/                    (custom libraries)
│   ├── BP_mobile_util/     # WebSocket class + pub/sub client
│   ├── SD32_util/          # SD card CSV handling, persistent file API
│   ├── syncTime_util/      # Sub-second timestamp interpolation from RTC
│   ├── CAN32_util/         # ESP32 TWAI wrapper
│   ├── DS3231_util/        # DS3231 RTC read/set
│   ├── WIFI32_util/        # WiFi connect + NTP time sync
│   └── ams_data_util/      # BMU/AMS data structs, CAN frame parser
```

> The old per-node split (`node_front`, `node_rear`, `node_bamo`) is preserved as commented-out environments in `platformio.ini` for reference.

---

## Quick start

### 1. Check Supported Board

**ESP32-S3** (recommended) and **ESP32dev or Node32S** or any ESP32 model that has dual-core, and TWAI for CAN Bus.

**ESP32-S2**, **ESP32-C3**: Single-core chips. no parallel task capability; sensor and comms tasks share threads. Untested.

The boards used during TSAE 2026 development and testing are custom PCB [here](https://github.com/BlackPearlFSAE/Bridge-PCB)

---
### 2. Build & Upload
Project are written and built with PlatformIO IDE<br>
Install it as vscode extension<br> 
Or install the shell command into your terminal [here](https://docs.platformio.org/en/latest/core/installation/index.html)

```bash
# Build
pio run -e Node   # unified firmware (Front/Rear/BAMO boards)
pio run -e ams    # AMS node (separate binary)

# Upload
pio run -e Node --target upload
pio run -e ams  --target upload
```

Each environment compiles only its own source files via [`build_src_filter`](https://docs.platformio.org/en/stable/projectconf/sections/env/options/build/build_src_filter.html) in `platformio.ini`.

---
### 3. Tweak `shared_config.h` to fit the test
All tunable constants live here. Key groups:

**Pin assignments** — GPIO numbers for encoders, ADC channels, SD SPI, I2C (RTC/IMU), UART (GPS), CAN (TWAI).

**Sensor scaling factor**
- Stroke sensors: 0–4095 ADC → 0–75 mm linear
- Current (Hall effect): `I = (V - 2.5) / 0.1` A
- APPS/BPPS: `position = (V / 3.3) × 75` mm
- Steering: 0–180° full range
- NTC thermistor: Steinhart-Hart equation

**Sensor data structs**
```cpp
Mechanical myMechData;   // Wheel RPM L/R, Heave/Roll stroke (mm)
Electrical myElectData;  // Current (A), Temp (°C), APPS/BPPS (%), fault bools, steering
Odometry   myOdomData;   // GPS lat/lng/speed/course/age, IMU accel/gyro xyz
BAMOCar    myBamoData;   // DC V, DC A, Power, motor temp, controller temp + validity flags
BMUdata    bmuArray[8];  // 8× { V_CELL[10], TEMP_SENSE[2], V_MODULE, DV, fault thresholds }
```

**Network config**
- WiFi SSID/password
- WebSocket server host + port to backend (WSS/443)
- Publish rate and NTP servers

#### Build flags 
flip these at compile time to change operation mode

| Flag | Values | Effect |
|------|--------|--------|
| `DEBUG_MODE` | `0/1/2` | 0. No serial output <br> 1. Concise human-readable serial debug for all tasks<br> 2. [Teleplot](https://github.com/nesnes/teleplot) live graphs in VSCode|
| `TIME_SRC` | `0/1` | 0. RTC only -> DS3231 is the time source<br> 1. WiFi NTP -> NTP server is the time source|
| `MOCK_FLAG` | `0/1` | 0. Real hardware<br> 1. Disable sensors and run mock data for testing |
| `calibrate_RTC` | `1` | Runs only when need to sync RTC with NTP server time |
<br>
---

## Program Flow
Three task are managed by freeRTOS
- **Sensor Reading**: &ensp; ADC/Digital IO and CAN Bus messages are processed and buffer
- **SDcard loggin**: &emsp;&nbsp; Log the buffered data into a micro SD card in as .csv, has basic session management logic (e.g. file rotation, renaming)
- **Live streaming**: &ensp;&nbsp; WebSocket publish (JSON, 5 Hz) to a server or a rented cloud. For dashboard display

![](/asset/architecture.png)
<br>

For more details please check
- [/docs folders](/docs/) to understand the idea of bridge-node design architecture it is vibe coded document, but it should get you the idea
- [/lib folders Readme file](/lib/) to understand the custom library that makes up the building block for the program flow

---
## Design Critique for later BP telemetry

#### Problem: Every node duplicates everything — partially solved

BP16B had four separate node files each owning their own structs, scaling constants, WebSocket topic names, and CSV layout. Adding or renaming a sensor touched at minimum four files across two layers.

`node_unified.cpp` merges Front, Rear, and BAMO into one binary. Struct definitions and CSV headers are now shared; unconnected sensors gracefully degrade to zero rather than preventing compilation. The AMS node remains separate due to its flash and FatFS requirements.

The remaining gap is a formal shared data contract — schema is still embedded in source rather than defined externally.

#### Suggested Direction: Standardized on CAN-DBC

Move all sensors data onto the CAN bus. Define scaling, units, and field names once in a `.DBC` file. 
Make one Node a Master unit that only log and stream raw CAN frames --the receiver-side software then handles decoding. Adding a sensor becomes just another one DBC entry that shared across the car.<br>
[Design Suggestion document here](https://docs.google.com/document/d/1JCUlqwzH1Ghhc9liws8qCY-q6IBD5o8sZY9PAH-eIoI/edit?tab=t.0#heading=h.81brb99mqf9b) 

#### Reduce Hardware size

Minimise GPIO count per MCU, silkscreen pin assignments directly on the PCB, and size each node's peripherals to its physical region of the car.<br>
[Example PCB layout](https://github.com/BlackPearlFSAE/Bridge-PCB)
<!-- This is still the old PCB, I need to update to new one that show the concept -->


## Related Docs

Deeper design notes live in `/docs`:
- [docs/Timesync_architecture.md](docs/Timesync_architecture.md) — two-tier RTC/NTP sync strategy
- [docs/SD_csv_appender.md](docs/SD_csv_appender.md) — generic CSV appender pattern
- [docs/Make SD&websocket_multitask.md](docs/Make%20SD%26websocket_multitask.md) — FreeRTOS task design for logging + streaming without stutter
- [docs/AMS_datalog_reference.md](docs/AMS_datalog_reference.md) — AMS BMU scaling, multi-file persistent SD logging
- [docs/websocket_protocol_reference.md](docs/websocket_protocol_reference.md) — per-node WebSocket registration and data schema