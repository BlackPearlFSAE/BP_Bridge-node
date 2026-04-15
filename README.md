# BP_Bridge-node

Educational example ESP32-S3 firmware for **BP16B Formula Student EV**. <br>
Intended for future development after BP16B <br>
Each MCU read and process sensors data, logged timestamped data to SD card , and publish live Telemetry data to the server at the same time: 


## What This Project Does
The car is split into two instrumented regions placement, each with its own ESP32-S3 board as Telemetry Bridge:

| Node | Region | Data Collected |
|------|--------|----------------|
| **Front** | Front suspension + Electrical | Heave and Roll distance, Wheel RPM pedal position, electrical fault, etc. |
| **Rear** | Rear suspension + Odometry | Heave and Roll distance, Wheel RPM, GPS, 9-axis IMU |
| **BAMO** | Motor controller <br>(Bamocar D3) | DC voltage, DC current, Motor temp, Controller temp |
| **AMS** | Battery management <br>(8 Modules) | Per-cell voltages, temperatures, fault states |

---

### Project Structure
```
├── src/
│   ├── node_front.cpp      # Source for Front Node
│   ├── node_rear.cpp       # Source for Rear Node
│   ├── node_bamo.cpp       # Source for Bamocar P3 D400 Node
│   ├── node_ams.cpp        # Source for AMS Node
│   ├── base_sensors.cpp    # Sensor reading
│   ├── bamo_helper.cpp     # Bamocar CAN Bus reading (Canopen)
│   └── motion_sensors.cpp  # GPS + IMU reading
├── include/
│   ├── shared_config.h     # pin defs, config string, debug flags
│   ├── base_sensors.h      
│   ├── bamo_helper.h       
│   └── motion_sensors.h    
├── lib/                    (For custom library)
│   ├── BP_mobile_util/     # WebSocket class + pub/sub client
│   ├── SD32_util/          # SD card CSV handling
│   ├── syncTime_util/      # Sync time across different source
│   ├── CAN32_util/         # ESP32 TWAI wrapper
│   ├── DS3221_util/        # DS3231 RTC functions
│   ├── WIFI32_util/        # WiFi connection + NTP time sync
│   └── ams_data_util/      # BMU/AMS data structs, CAN parsing
```
Quite bloated; it can be improved, see critique on this design on [Design critique section](#design-critique-for-later-bp-telemetry)

---

## Quick start

### 1. Check Supported Board

**ESP32-S3** (recommended) and **ESP32dev or Node32S** or any ESP32 model that has dual-core, and TWAI for CAN Bus.

**ESP32-S2**, **ESP32-C3**: Single-core chips. no parallel task capability; sensor and comms tasks share threads. Untested.

The boards used during TSAE 2026 development and testing are custom PCB [here](Links)

---
### 2. Build & Upload
Project are written and built with PlatformIO IDE<br>
Install it in vscode [here](https://platformio.org/install/ide?install=vscode)<br> 
Or install the shell command into your terminal [here](https://docs.platformio.org/en/latest/core/installation/index.html)

```bash
# Build a specific node environment
pio run -e front
pio run -e rear
pio run -e bamo
pio run -e ams

# Upload to board
pio run -e front --target upload
```

Each environment compiles only its own source files. See [`build_src_filter`](https://docs.platformio.org/en/stable/projectconf/sections/env/options/build/build_src_filter.html) in `platformio.ini`.<br>

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

#### Problem: Every node duplicates everything

Each node owns its own structs, scaling constants, WebSocket topic names, and CSV layout. Adding or renaming a sensor touches at minimum four files across two layers. The four-node split is not the problem  --the lack of a shared data contract is.

#### Suggested Direction: Standardized on CAN-DBC

Move all sensors data onto the CAN bus. Define scaling, units, and field names once in a `.DBC` file. 
Make one Node a Master unit that only log and stream raw CAN frames --the receiver-side software then handles decoding. Adding a sensor becomes just another one DBC entry that shared across the car.<br>
[Design Suggestion document here](https://docs.google.com/document/d/1JCUlqwzH1Ghhc9liws8qCY-q6IBD5o8sZY9PAH-eIoI/edit?tab=t.0#heading=h.81brb99mqf9b) 

#### Reduce Hardware size

Minimise GPIO count per MCU, silkscreen pin assignments directly on the PCB, and size each node's peripherals to its physical region of the car.<br>
[Example PCB layout](Link)


## Related Docs

Deeper design notes live in `/docs`:
- [docs/Timesync_architecture.md](docs/Timesync_architecture.md) — two-tier RTC/NTP sync strategy
- [docs/SD_csv_appender.md](docs/SD_csv_appender.md) — generic CSV appender pattern
- [docs/Make SD&websocket_multitask.md](docs/Make%20SD%26websocket_multitask.md) — FreeRTOS task design for logging + streaming without stutter
- [docs/AMS_datalog_reference.md](docs/AMS_datalog_reference.md) — AMS BMU scaling, multi-file persistent SD logging
- [docs/websocket_protocol_reference.md](docs/websocket_protocol_reference.md) — per-node WebSocket registration and data schema