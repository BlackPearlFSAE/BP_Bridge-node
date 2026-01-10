# BP_Bridge-node Refactoring Session Summary

## Session Overview
This document summarizes the refactoring work done on the BP_Bridge-node ESP32-S3 project.

---

## Task 1: ESP32-S3 Strapping Pin Check

### Question
Check if any pin definitions in main.cpp overlap with ESP32-S3 strapping pins.

### ESP32-S3 Strapping Pins
- **GPIO0**: Boot mode selection
- **GPIO3**: JTAG signal source
- **GPIO45**: VDD_SPI voltage
- **GPIO46**: Boot mode / ROM messages

### Finding
- **GPIO3** (`WIFI_LED_INDICATOR`) overlaps with strapping pin
- **Resolution**: Safe to use since it's just an LED output with no external pull-up/down resistors

---

## Task 2: BAMOCar Global Variable Refactoring

### Problem
Code had global BAMOCar instance and alias variables (bad practice from "overvibecoding").

### Solution
1. Keep BAMOCar struct instantiation local in `loop()`
2. Remove all global instances and backward compatibility aliases
3. Pass `BAMOCar*` pointer to functions (consistent with `Mechanical*`, `Electrical*`, `Odometry*`)

### Changes Made

#### Struct Definition (lines 85-100)
```cpp
typedef struct{
  float motorTemp1 = 0.0;
  float motorTemp2 = 0.0;
  float controllerTemp = 0.0;
  float canVoltage = 0.0;
  float canCurrent = 0.0;
  float power = 0.0;
  bool motorTempValid = false;
  bool controllerTempValid = false;
  bool canVoltageValid = false;
  bool canCurrentValid = false;
} BAMOCar;
```

#### Functions Updated to Accept BAMOCar*
- `publishBAMOpower(BAMOCar* bamocar)`
- `publishBAMOtemp(BAMOCar* bamocar)`
- `logBAMOcarDataToSD(char* csvFilename, BAMOCar* bamocar)`
- `process_ResponseBamocarMsg(BAMOCar* bamocar)`

---

## Task 3: Electrical Sensor Publisher Functions

### Requirement
Complete empty stub functions `publishElectData()` and `publishElectFaultState()` following the same pattern as existing publishers.

### Sensor Categories
- **ElectSensors (Analog)**: I_SENSE, TMP, APPS, BPPS
- **ElectFaultState (Digital)**: AMS_OK, IMD_OK, HV_ON, BSPD_OK

### Sampling Rates (from lines 269-271)
```cpp
const float ELECT_SENSORS_SAMPLING_RATE = 2.0;    // 2 Hz
const float ELECT_FAULT_STAT_SAMPLING_RATE = 1.0; // 1 Hz
```

### Implementation

#### publishElectData() - lines 685-747
Publishes analog electrical sensor data:
- `electrical/current_sense` - Current sense (A)
- `electrical/temperature` - Electrical system temperature (°C)
- `electrical/apps` - Accelerator Pedal Position Sensor (%)
- `electrical/bpps` - Brake Pedal Position Sensor (%)

#### publishElectFaultState() - lines 749-805
Publishes digital fault state signals:
- `electrical/ams_ok` - Accumulator Management System status
- `electrical/imd_ok` - Insulation Monitoring Device status
- `electrical/hv_on` - High Voltage system status
- `electrical/bspd_ok` - Brake System Plausibility Device status

#### registerClient() Updates - lines 807-936
Added 8 new topics with metadata:
```cpp
topics.add("electrical/current_sense");
topics.add("electrical/temperature");
topics.add("electrical/apps");
topics.add("electrical/bpps");
topics.add("electrical/ams_ok");
topics.add("electrical/imd_ok");
topics.add("electrical/hv_on");
topics.add("electrical/bspd_ok");
```

#### Timing Variables - lines 150-156
```cpp
unsigned long lastBAMOsend_power = 0;
unsigned long lastBAMOsend_temp = 0;
unsigned long lastMechSend = 0;
unsigned long lastElectSend = 0;
unsigned long lastElectFaultSend = 0;
unsigned long lastOdometrySend = 0;
```

#### Call Sites in loop() - lines 439-453
```cpp
// Send Mechanical data
if (now - lastMechSend >= (1000.0 / MECH_SENSORS_SAMPLING_RATE)) {
  publishMechData(&myMechSensors);
  lastMechSend = now;
}
// Send Electrical analog data
if (now - lastElectSend >= (1000.0 / ELECT_SENSORS_SAMPLING_RATE)) {
  publishElectData(&myElectSensors);
  lastElectSend = now;
}
// Send Electrical fault state data
if (now - lastElectFaultSend >= (1000.0 / ELECT_FAULT_STAT_SAMPLING_RATE)) {
  publishElectFaultState(&myElectSensors);
  lastElectFaultSend = now;
}
```

---

## Electrical Struct Definition (lines 163-170)
```cpp
typedef struct{
  float I_SENSE = 0.0;
  float TMP = 0.0;
  float APPS = 0.0;
  float BPPS = 0.0;
  bool AMS_OK = 0;
  bool IMD_OK = 0;
  bool HV_ON = 0;
  bool BSPD_OK = 0;
} Electrical;
```

---

## Publisher Function Pattern

All publisher functions follow this pattern:
```cpp
void publishXxxData(StructType* data) {
  unsigned long long unixTimestamp = getSynchronizedTime();
  if (unixTimestamp < 1000000000000ULL) return;

  JsonDocument doc;
  doc["type"] = "data";
  doc["topic"] = "category/sensor_name";
  doc["data"]["value"] = data->field;
  doc["data"]["sensor_id"] = "SENSOR_ID";
  doc["timestamp"] = unixTimestamp;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);

  Serial.print("[BP Mobile] ...");
}
```

---

## Build Status
- **Platform**: ESP32-S3 (esp32-s3-devkitc-1)
- **Framework**: Arduino
- **RAM Usage**: 14.6% (47,896 / 327,680 bytes)
- **Flash Usage**: 29.6% (990,529 / 3,342,336 bytes)
- **Compilation**: SUCCESS

---

## Deferred Tasks
- Create separate logging functions for Mechanical, Electrical, and Odometry data with unique filenames (user said "Maybe let's forget that first")

---

## Files Modified
- `src/main.cpp` - Primary changes

## Key Libraries Used
- ArduinoJson (7.4.2)
- WebSockets (2.7.2)
- WiFi, SPI, Wire, SD
- Custom: BP_mobile_util, CAN32_util, SD32_util, syncTime_util