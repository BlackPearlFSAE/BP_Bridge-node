# WebSocket Protocol Reference

All nodes communicate via WebSocket using JSON messages. Each node sends a **registration** message once on connection, then **data** messages periodically.

**Server must respond to registration with:**
```json
{ "type": "registration_response", "status": "accepted", "system_time": { "timestamp_ms": 1706000000000 } }
```

`DEFAULT_PUBLISH_RATE = 2.0 Hz`

---

## Front_Node (`src/node_front.cpp`)

### Registration
```json
{
  "type": "register",
  "client_name": "Front_Node",
  "groups": [
    { "group": "front.mech",   "rate_hz": 2.0 },
    { "group": "front.elect",  "rate_hz": 2.0 },
    { "group": "front.faults", "rate_hz": 2.0 }
  ],
  "schema": [
    { "key": "front.mech.STR_Heave_mm", "type": "float", "unit": "mm", "scale": 1, "offset": 0, "group": "front.mech" },
    { "key": "front.mech.STR_Roll_mm",  "type": "float", "unit": "mm", "scale": 1, "offset": 0, "group": "front.mech" },
    { "key": "front.elect.I_SENSE",     "type": "float", "unit": "A",  "scale": 1, "offset": 0, "group": "front.elect" },
    { "key": "front.elect.TMP",         "type": "float", "unit": "C",  "scale": 1, "offset": 0, "group": "front.elect" },
    { "key": "front.elect.APPS",        "type": "float", "unit": "%",  "scale": 1, "offset": 0, "group": "front.elect" },
    { "key": "front.elect.BPPS",        "type": "float", "unit": "%",  "scale": 1, "offset": 0, "group": "front.elect" },
    { "key": "front.faults.AMS_OK",     "type": "bool",  "unit": "",   "scale": 1, "offset": 0, "group": "front.faults" },
    { "key": "front.faults.IMD_OK",     "type": "bool",  "unit": "",   "scale": 1, "offset": 0, "group": "front.faults" },
    { "key": "front.faults.HV_ON",      "type": "bool",  "unit": "",   "scale": 1, "offset": 0, "group": "front.faults" },
    { "key": "front.faults.BSPD_OK",    "type": "bool",  "unit": "",   "scale": 1, "offset": 0, "group": "front.faults" }
  ]
}
```

### Data: front.mech (2 Hz)
```json
{ "type": "data", "group": "front.mech", "ts": 1706000000000, "d": { "STR_Heave_mm": 12.34, "STR_Roll_mm": 5.67 } }
```

### Data: front.elect (2 Hz)
```json
{ "type": "data", "group": "front.elect", "ts": 1706000000000, "d": { "I_SENSE": 3.21, "TMP": 42.5, "APPS": 75.0, "BPPS": 12.3 } }
```

### Data: front.faults (2 Hz)
```json
{ "type": "data", "group": "front.faults", "ts": 1706000000000, "d": { "AMS_OK": true, "IMD_OK": true, "HV_ON": false, "BSPD_OK": true } }
```

---

## Rear_Node (`src/node_rear.cpp`)

### Registration
```json
{
  "type": "register",
  "client_name": "Rear_Node",
  "groups": [
    { "group": "rear.mech", "rate_hz": 2.0 },
    { "group": "rear.odom", "rate_hz": 2.0 }
  ],
  "schema": [
    { "key": "rear.mech.Wheel_RPM_L",  "type": "float",  "unit": "RPM",   "scale": 1, "offset": 0, "group": "rear.mech" },
    { "key": "rear.mech.Wheel_RPM_R",  "type": "float",  "unit": "RPM",   "scale": 1, "offset": 0, "group": "rear.mech" },
    { "key": "rear.mech.STR_Heave_mm", "type": "float",  "unit": "mm",    "scale": 1, "offset": 0, "group": "rear.mech" },
    { "key": "rear.mech.STR_Roll_mm",  "type": "float",  "unit": "mm",    "scale": 1, "offset": 0, "group": "rear.mech" },
    { "key": "rear.odom.gps_lat",      "type": "double", "unit": "deg",   "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.gps_lng",      "type": "double", "unit": "deg",   "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.gps_age",      "type": "double", "unit": "ms",    "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.gps_course",   "type": "double", "unit": "deg",   "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.gps_speed",    "type": "double", "unit": "km/h",  "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.imu_accel_x",  "type": "float",  "unit": "m/s2",  "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.imu_accel_y",  "type": "float",  "unit": "m/s2",  "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.imu_accel_z",  "type": "float",  "unit": "m/s2",  "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.imu_gyro_x",   "type": "float",  "unit": "deg/s", "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.imu_gyro_y",   "type": "float",  "unit": "deg/s", "scale": 1, "offset": 0, "group": "rear.odom" },
    { "key": "rear.odom.imu_gyro_z",   "type": "float",  "unit": "deg/s", "scale": 1, "offset": 0, "group": "rear.odom" }
  ]
}
```

### Data: rear.mech (2 Hz)
```json
{ "type": "data", "group": "rear.mech", "ts": 1706000000000, "d": { "Wheel_RPM_L": 320.5, "Wheel_RPM_R": 318.2, "STR_Heave_mm": 11.0, "STR_Roll_mm": 4.3 } }
```

### Data: rear.odom (2 Hz)
```json
{ "type": "data", "group": "rear.odom", "ts": 1706000000000, "d": { "gps_lat": 3.1234567, "gps_lng": 101.7654321, "gps_age": 150.0, "gps_course": 270.5, "gps_speed": 85.3, "imu_accel_x": 0.12, "imu_accel_y": -0.05, "imu_accel_z": 9.81, "imu_gyro_x": 1.2, "imu_gyro_y": -0.3, "imu_gyro_z": 0.8 } }
```

---

## BAMO_Node (`src/node_bamo.cpp`)

### Registration
```json
{
  "type": "register",
  "client_name": "BAMO_Node",
  "groups": [
    { "group": "bamo.power", "rate_hz": 2.0 },
    { "group": "bamo.temp",  "rate_hz": 2.0 }
  ],
  "schema": [
    { "key": "bamo.power.canVoltage",      "type": "float", "unit": "V", "scale": 1, "offset": 0, "group": "bamo.power" },
    { "key": "bamo.power.canCurrent",      "type": "float", "unit": "A", "scale": 1, "offset": 0, "group": "bamo.power" },
    { "key": "bamo.power.power",           "type": "float", "unit": "W", "scale": 1, "offset": 0, "group": "bamo.power" },
    { "key": "bamo.power.canVoltageValid", "type": "bool",  "unit": "",  "scale": 1, "offset": 0, "group": "bamo.power" },
    { "key": "bamo.power.canCurrentValid", "type": "bool",  "unit": "",  "scale": 1, "offset": 0, "group": "bamo.power" },
    { "key": "bamo.temp.motorTemp",        "type": "float", "unit": "C", "scale": 1, "offset": 0, "group": "bamo.temp" },
    { "key": "bamo.temp.controllerTemp",   "type": "float", "unit": "C", "scale": 1, "offset": 0, "group": "bamo.temp" },
    { "key": "bamo.temp.motorTempValid",   "type": "bool",  "unit": "",  "scale": 1, "offset": 0, "group": "bamo.temp" },
    { "key": "bamo.temp.ctrlTempValid",    "type": "bool",  "unit": "",  "scale": 1, "offset": 0, "group": "bamo.temp" }
  ]
}
```

### Data: bamo.power (2 Hz)
```json
{ "type": "data", "group": "bamo.power", "ts": 1706000000000, "d": { "canVoltage": 400.5, "canCurrent": 120.3, "power": 48180.15, "canVoltageValid": true, "canCurrentValid": true } }
```

### Data: bamo.temp (2 Hz)
```json
{ "type": "data", "group": "bamo.temp", "ts": 1706000000000, "d": { "motorTemp": 65.2, "controllerTemp": 48.7, "motorTempValid": true, "ctrlTempValid": true } }
```

---

## AMS_Node (`src/node_ams.cpp`)

8 BMU modules (bmu0-bmu7), each with a `.cells` and `.faults` group.

### Registration
```json
{
  "type": "register",
  "client_name": "AMS_Node",
  "groups": [
    { "group": "bmu0.cells",  "rate_hz": 2.0 },
    { "group": "bmu0.faults", "rate_hz": 2.0 },
    { "group": "bmu1.cells",  "rate_hz": 2.0 },
    { "group": "bmu1.faults", "rate_hz": 2.0 },
    { "group": "bmu2.cells",  "rate_hz": 2.0 },
    { "group": "bmu2.faults", "rate_hz": 2.0 },
    { "group": "bmu3.cells",  "rate_hz": 2.0 },
    { "group": "bmu3.faults", "rate_hz": 2.0 },
    { "group": "bmu4.cells",  "rate_hz": 2.0 },
    { "group": "bmu4.faults", "rate_hz": 2.0 },
    { "group": "bmu5.cells",  "rate_hz": 2.0 },
    { "group": "bmu5.faults", "rate_hz": 2.0 },
    { "group": "bmu6.cells",  "rate_hz": 2.0 },
    { "group": "bmu6.faults", "rate_hz": 2.0 },
    { "group": "bmu7.cells",  "rate_hz": 2.0 },
    { "group": "bmu7.faults", "rate_hz": 2.0 }
  ],
  "schema": [
    { "key": "bmu0.cells.V_MODULE",   "type": "uint16",   "unit": "V",     "scale": 0.02, "offset": 0,   "group": "bmu0.cells" },
    { "key": "bmu0.cells.V_CELL",     "type": "uint8[]",  "unit": "V",     "scale": 0.02, "offset": 0,   "group": "bmu0.cells", "length": 10 },
    { "key": "bmu0.cells.TEMP_SENSE", "type": "uint16[]", "unit": "C",     "scale": 0.5,  "offset": -40, "group": "bmu0.cells", "length": 2 },
    { "key": "bmu0.cells.DV",         "type": "uint8",    "unit": "V",     "scale": 0.1,  "offset": 0,   "group": "bmu0.cells" },
    { "key": "bmu0.cells.connected",  "type": "bool",     "unit": "",      "scale": 1,    "offset": 0,   "group": "bmu0.cells" },
    { "key": "bmu0.faults.OV_WARN",   "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.OV_CRIT",   "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.LV_WARN",   "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.LV_CRIT",   "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.OT_WARN",   "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.OT_CRIT",   "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.ODV_WARN",  "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.ODV_CRIT",  "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.BAL_CELLS", "type": "uint16",   "unit": "flags", "scale": 1, "offset": 0, "group": "bmu0.faults" },
    { "key": "bmu0.faults.NEED_BAL",  "type": "bool",     "unit": "",      "scale": 1, "offset": 0, "group": "bmu0.faults" },
    "... same 15 entries repeated for bmu1 through bmu7 ..."
  ]
}
```

### Data: bmuN.cells (2 Hz, x8 messages per cycle)
```json
{ "type": "data", "group": "bmu0.cells", "ts": 1706000000000, "d": { "V_MODULE": 3724, "V_CELL": [186,187,185,188,186,187,185,186,188,187], "TEMP_SENSE": [165,162], "DV": 3, "connected": true } }
```

**Decoding raw integers:**
- `V_MODULE`: `3724 * 0.02 = 74.48 V`
- `V_CELL[0]`: `186 * 0.02 = 3.72 V`
- `TEMP_SENSE[0]`: `165 * 0.5 + (-40) = 42.5 C`
- `DV`: `3 * 0.1 = 0.3 V`

### Data: bmuN.faults (2 Hz, x8 messages per cycle)
```json
{ "type": "data", "group": "bmu0.faults", "ts": 1706000000000, "d": { "OV_WARN": 0, "OV_CRIT": 0, "LV_WARN": 0, "LV_CRIT": 0, "OT_WARN": 0, "OT_CRIT": 0, "ODV_WARN": 0, "ODV_CRIT": 0, "BAL_CELLS": 1023, "NEED_BAL": false } }
```

---

## Legacy Protocol — Superseded

> **Note:** The format below is the old per-topic protocol used before the schema-driven refactor. It is **no longer emitted by any node** and is kept here only for historical reference when migrating old dashboards or server code.

### Registration (old format)
```json
{
  "type": "register",
  "client_name": "Front_Node",
  "topics": [
    "power/can_voltage",
    "power/can_current",
    "power/power",
    "motor/temperature",
    "motor/controller_temperature",
    "wheel/left_rpm",
    "wheel/right_rpm",
    "sensors/stroke_Heave_distanceMM",
    "sensors/stroke_Roll_distanceMM",
    "electrical/current_sense",
    "electrical/temperature",
    "electrical/apps",
    "electrical/bpps",
    "electrical/ams_ok",
    "electrical/imd_ok",
    "electrical/hv_on",
    "electrical/bspd_ok"
  ],
  "topic_metadata": {
    "power/can_voltage":              { "description": "DC Link Voltage (Bamocar CAN)",              "unit": "V",    "sampling_rate": 2.0 },
    "power/can_current":              { "description": "Motor DC Current (Bamocar CAN)",              "unit": "A",    "sampling_rate": 2.0 },
    "power/power":                    { "description": "Power consumption (calculated)",               "unit": "W",    "sampling_rate": 2.0 },
    "motor/temperature":              { "description": "Motor temperature (Bamocar)",                  "unit": "C",    "sampling_rate": 2.0 },
    "motor/controller_temperature":   { "description": "Motor controller/IGBT temperature (Bamocar)",  "unit": "C",    "sampling_rate": 2.0 },
    "wheel/left_rpm":                 { "description": "Left wheel speed",                             "unit": "RPM",  "sampling_rate": 2.0 },
    "wheel/right_rpm":                { "description": "Right wheel speed",                            "unit": "RPM",  "sampling_rate": 2.0 },
    "sensors/stroke_Heave_distanceMM":{ "description": "Stroke Heave",                                "unit": "mm",   "sampling_rate": 2.0 },
    "sensors/stroke_Roll_distanceMM": { "description": "Stroke Roll",                                 "unit": "mm",   "sampling_rate": 2.0 },
    "electrical/current_sense":       { "description": "Current sense",                                "unit": "A",    "sampling_rate": 2.0 },
    "electrical/temperature":         { "description": "Electrical system temperature",                "unit": "C",    "sampling_rate": 2.0 },
    "electrical/apps":                { "description": "Accelerator Pedal Position Sensor",            "unit": "%",    "sampling_rate": 2.0 },
    "electrical/bpps":                { "description": "Brake Pedal Position Sensor",                  "unit": "%",    "sampling_rate": 2.0 },
    "electrical/ams_ok":              { "description": "Accumulator Management System status",         "unit": "bool", "sampling_rate": 0.4 },
    "electrical/imd_ok":              { "description": "Insulation Monitoring Device status",          "unit": "bool", "sampling_rate": 0.4 },
    "electrical/hv_on":               { "description": "High Voltage system status",                   "unit": "bool", "sampling_rate": 0.4 },
    "electrical/bspd_ok":             { "description": "Brake System Plausibility Device status",      "unit": "bool", "sampling_rate": 0.4 }
  }
}
```

### Data (old format - one message per value)
```json
{ "type": "data", "topic": "power/can_voltage",              "data": { "value": 400.5, "sensor_id": "BAMOCAR_CAN" },   "timestamp": 1706000000000 }
{ "type": "data", "topic": "power/can_current",              "data": { "value": 120.3, "sensor_id": "BAMOCAR_CAN" },   "timestamp": 1706000000000 }
{ "type": "data", "topic": "power/power",                    "data": { "value": 48180.15, "sensor_id": "CALCULATED" },  "timestamp": 1706000000000 }
{ "type": "data", "topic": "motor/temperature",              "data": { "value": 65.2, "sensor_id": "BAMOCAR_MOTOR" },   "timestamp": 1706000000000 }
{ "type": "data", "topic": "motor/controller_temperature",   "data": { "value": 48.7, "sensor_id": "BAMOCAR_CTRL" },    "timestamp": 1706000000000 }
{ "type": "data", "topic": "wheel/left_rpm",                 "data": { "value": 320.5, "sensor_id": "ENC_LEFT" },       "timestamp": 1706000000000 }
{ "type": "data", "topic": "wheel/right_rpm",                "data": { "value": 318.2, "sensor_id": "ENC_RIGHT" },      "timestamp": 1706000000000 }
{ "type": "data", "topic": "sensors/stroke_Heave_distanceMM","data": { "value": 12.34, "sensor_id": "STR_Heave" },      "timestamp": 1706000000000 }
{ "type": "data", "topic": "sensors/stroke_Roll_distanceMM", "data": { "value": 5.67, "sensor_id": "STR_Roll" },        "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/current_sense",       "data": { "value": 3.21, "sensor_id": "I_SENSE" },         "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/temperature",         "data": { "value": 42.5, "sensor_id": "TMP" },             "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/apps",                "data": { "value": 75.0, "sensor_id": "APPS" },            "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/bpps",                "data": { "value": 12.3, "sensor_id": "BPPS" },            "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/ams_ok",              "data": { "value": 1, "sensor_id": "AMS" },                "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/imd_ok",              "data": { "value": 1, "sensor_id": "IMD" },                "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/hv_on",               "data": { "value": 0, "sensor_id": "HV" },                 "timestamp": 1706000000000 }
{ "type": "data", "topic": "electrical/bspd_ok",             "data": { "value": 1, "sensor_id": "BSPD" },               "timestamp": 1706000000000 }
```

---

## Message Volume Summary

| Node       | Groups | Messages per cycle | Rate  |
|------------|--------|--------------------|-------|
| Front_Node | 3      | 3                  | 2 Hz  |
| Rear_Node  | 2      | 2                  | 2 Hz  |
| BAMO_Node  | 2      | 2                  | 2 Hz  |
| AMS_Node   | 16     | 16                 | 2 Hz  |
| **Total**  | **23** | **23**             |       |

Legacy Front_Node (temp/) sent **up to 17 individual messages per cycle**.
