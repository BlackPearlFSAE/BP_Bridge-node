# SD Card Logging Refactor - Implementation Summary

## Objective

Transform the SD card logging system into a flexible, generic appender-based architecture with togglable CSV write fields.

---

## Requirements

1. **Generic Function Pointer Type**: `typedef void (*AppenderFunc)(File&, void*);`
2. **Refactored Main Function Signature**: `void logData(const char* filename, AppenderFunc* appenders, void** dataArray, size_t count)`
3. **Typecast Pattern**: All appenders cast `void*` to specific struct type as first line
4. **Split CSV Headers**: Modular header segments for concatenation
5. **New Appenders**: Timestamp, Electrical, Odometry (in addition to existing BAMO and Mechanical)

---

## Implementation Details

### 1. AppenderFunc Typedef

**Location:** `main.cpp:62`

```cpp
typedef void (*AppenderFunc)(File&, void*);
```

### 2. Modular CSV Header Segments

**Location:** `main.cpp:67-71`

```cpp
const char* header_Timestamp = "Timestamp,DateTime,DataPoint";
const char* header_BAMO = "CAN_Voltage(V),CAN_Current(A),Power(W),MotorTemp(C),ControllerTemp(C)";
const char* header_Mechanical = "Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm";
const char* header_Electrical = "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK";
const char* header_Odometry = "GPS_Lat,GPS_Lng,GPS_Age,GPS_Course,GPS_Speed,IMU_AccelX,IMU_AccelY,IMU_AccelZ,IMU_GyroX,IMU_GyroY,IMU_GyroZ";

char csvHeaderBuffer[512] = "";
```

### 3. Appender Functions

All appenders follow the same pattern:
- Signature: `void append_XXX_toCSVFile(File& dataFile, void* data)`
- First line: typecast `void*` to specific struct
- Write fields with `dataFile.print()` followed by `dataFile.print(",")`
- Do NOT call `println()` - main function handles line ending

#### Timestamp Appender
```cpp
void append_Timestamp_toCSVFile(File& dataFile, void* data) {
  (void)data;  // Unused - access globals directly

  uint64_t syncTime = getSynchronizedTime();
  uint64_t timestamp;
  char dateTimeStr[32] = "";

  if (timeIsSynchronized) {
    timestamp = (syncTime / 1000ULL);
    formatDateTimeBangkok(dateTimeStr, syncTime);
  } else {
    timestamp = millis() / 1000;
    strcpy(dateTimeStr, "NOT_SYNCED");
  }

  dataFile.print(timestamp);
  dataFile.print(",");
  dataFile.print(dateTimeStr);
  dataFile.print(",");
  dataFile.print(dataPoint);
  dataFile.print(",");
}
```

#### BAMOCar Appender
```cpp
void append_BAMOdata_toCSVFile(File& dataFile, void* data) {
  BAMOCar* bamocar = static_cast<BAMOCar*>(data);

  dataFile.print((bamocar->canVoltageValid) ? bamocar->canVoltage : 0.0, 2);
  dataFile.print(",");
  // ... other fields
}
```

#### Mechanical Sensors Appender
```cpp
void append_MechSensors_toCSVFile(File& dataFile, void* data) {
  Mechanical* MechSensors = static_cast<Mechanical*>(data);

  dataFile.print(MechSensors->Wheel_RPM_L, 2);
  dataFile.print(",");
  // ... other fields
}
```

#### Electrical Sensors Appender
```cpp
void append_ElectricalSensors_toCSVFile(File& dataFile, void* data) {
  Electrical* ElectSensors = static_cast<Electrical*>(data);

  // Analog sensors (2 decimal places)
  dataFile.print(ElectSensors->I_SENSE, 2);
  dataFile.print(",");
  dataFile.print(ElectSensors->TMP, 2);
  dataFile.print(",");
  dataFile.print(ElectSensors->APPS, 2);
  dataFile.print(",");
  dataFile.print(ElectSensors->BPPS, 2);
  dataFile.print(",");

  // Digital fault status (0/1)
  dataFile.print(ElectSensors->AMS_OK ? 1 : 0);
  dataFile.print(",");
  dataFile.print(ElectSensors->IMD_OK ? 1 : 0);
  dataFile.print(",");
  dataFile.print(ElectSensors->HV_ON ? 1 : 0);
  dataFile.print(",");
  dataFile.print(ElectSensors->BSPD_OK ? 1 : 0);
  dataFile.print(",");
}
```

#### Odometry Sensors Appender
```cpp
void append_OdometrySensors_toCSVFile(File& dataFile, void* data) {
  Odometry* OdomSensors = static_cast<Odometry*>(data);

  // GPS data (lat/lng: 6 decimals, others: 2 decimals)
  dataFile.print(OdomSensors->gps_lat, 6);
  dataFile.print(",");
  dataFile.print(OdomSensors->gps_lng, 6);
  dataFile.print(",");
  // ... other GPS fields

  // IMU accelerometer (3 decimals)
  dataFile.print(OdomSensors->imu_accelx, 3);
  dataFile.print(",");
  // ... other IMU fields
}
```

### 4. Generic logData Function

```cpp
void logData(const char* filename, AppenderFunc* appenders, void** dataArray, size_t count) {
  File dataFile = SD.open(filename, FILE_APPEND);

  if (!dataFile) {
    Serial.println("[SD Card] ERROR: Could not open file!");
    return;
  }

  // Call all appenders
  for (size_t i = 0; i < count; i++) {
    appenders[i](dataFile, dataArray[i]);
  }

  // End CSV line
  dataFile.println();

  dataFile.flush();
  dataFile.close();

  dataPoint++;
  Serial.print("[SD Card] Logged data point #");
  Serial.println(dataPoint - 1);
}
```

---

## Critical Bug Fixes

### 1. File Parameter Bug (CRITICAL)

**Original (Broken):**
```cpp
void append_MechSensors_toCSVFile(File dataFile, Mechanical *MechSensors)
```

**Fixed:**
```cpp
void append_MechSensors_toCSVFile(File& dataFile, void* data)
```

Passing `File` by value creates a copy of the file handle. Operations on the copy may not persist to the actual file.

### 2. Inconsistent Line Ending

**Original:** `append_MechSensors_toCSVFile` called `println()` at end, but `append_BAMOdata_toCSVFile` did not.

**Fixed:** All appenders use `print(",")` at end. The main `logData()` function calls `println()` to end the line.

---

## Usage Patterns

### Approach A: Comment-Based Toggle (Recommended - User's Choice)

**setup() - Header Building:**
```cpp
strcat(csvHeaderBuffer, header_Timestamp);
strcat(csvHeaderBuffer, header_Mechanical); strcat(csvHeaderBuffer, ",");
strcat(csvHeaderBuffer, header_Electrical); strcat(csvHeaderBuffer, ",");
strcat(csvHeaderBuffer, header_Odometry);   strcat(csvHeaderBuffer, ",");
strcat(csvHeaderBuffer, header_BAMO);
```

**loop() - Appender Arrays:**
```cpp
int appenderCount = 5;
AppenderFunc appenders[appenderCount] = {
  append_Timestamp_toCSVFile,
  append_MechData_toCSVFile,      // Comment to disable
  append_ElectData_toCSVFile,     // Comment to disable
  append_OdometryData_toCSVFile,  // Comment to disable
  append_BAMOdata_toCSVFile
};
void *structArray[appenderCount] = {
  &DeviceTime_Absolute,
  &myMechData,
  &myElectData,
  &myOdometryData,
  &myBAMOCar
};

logDataToSD(csvFilename, appenders, structArray, appenderCount);
```

**Advantages:**
- Simple and readable
- Direct array initialization
- Easy to reorder sensors
- Quick toggle by commenting lines
- Visual alignment between headers and data

**Tradeoffs:**
- Must sync header (setup) and data (loop) manually
- Must update `appenderCount` when changing

### Approach B: Compile-Time Toggle (Alternative)

**Configuration:**
```cpp
#define ENABLE_TIMESTAMP_LOG 1
#define ENABLE_BAMO_LOG 1
#define ENABLE_MECHANICAL_LOG 1
#define ENABLE_ELECTRICAL_LOG 1
#define ENABLE_ODOMETRY_LOG 1
```

**Header Builder Function:**
```cpp
void buildCSVHeader(char* buffer, size_t bufferSize) {
  buffer[0] = '\0';
  bool needsComma = false;

  #if ENABLE_TIMESTAMP_LOG
    strcat(buffer, header_Timestamp);
    needsComma = true;
  #endif

  #if ENABLE_BAMO_LOG
    if (needsComma) strcat(buffer, ",");
    strcat(buffer, header_BAMO);
    needsComma = true;
  #endif
  // ... etc
}
```

**Advantages:**
- Guaranteed sync between header and data
- True compile-time optimization (smaller binary)
- Single point of configuration

**Tradeoffs:**
- More complex preprocessor logic
- Less readable
- Slower iteration during development

---

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Timestamp access | NULL pointer, access globals | Timestamp appender needs `getSynchronizedTime()`, `timeIsSynchronized`, `dataPoint` |
| Header style | Static (all columns always present) | Consistent column positions across logs |
| File parameter | By reference (`File&`) | Avoid copying file handle |
| Line ending | Main function handles `println()` | Consistent behavior, appenders don't need to know if they're last |
| Decimal precision | Varies by sensor type | GPS lat/lng: 6 decimals, analog: 2 decimals, IMU: 3 decimals |
| Boolean output | `0` or `1` | Compact and unambiguous |

---

## Data Structures Reference

### BAMOCar
```cpp
typedef struct {
  float motorTemp1, motorTemp2, controllerTemp;
  float canVoltage, canCurrent, power;
  bool motorTempValid, controllerTempValid;
  bool canVoltageValid, canCurrentValid;
} BAMOCar;
```

### Mechanical
```cpp
typedef struct {
  float Wheel_RPM_L, Wheel_RPM_R;
  float STR_Heave_mm, STR_Roll_mm;
} Mechanical;
```

### Electrical
```cpp
typedef struct {
  float I_SENSE, TMP, APPS, BPPS;  // Analog
  bool AMS_OK, IMD_OK, HV_ON, BSPD_OK;  // Digital fault status
} Electrical;
```

### Odometry
```cpp
typedef struct {
  double gps_lat, gps_lng, gps_age, gps_course, gps_speed;  // GPS
  float imu_accelx, imu_accely, imu_accelz;  // Accelerometer
  float imu_gyrox, imu_gyroy, imu_gyroz;  // Gyroscope
} Odometry;
```

---

## Files Modified

- `src/main.cpp` - All logging logic, appenders, and configuration

---

## Future Extensibility

To add a new sensor type:

1. Define header segment: `const char* header_NewSensor = "Field1,Field2,...";`
2. Create appender function following the pattern
3. Add header to concatenation in `setup()`
4. Add appender and struct pointer to arrays in `loop()`
5. Update `appenderCount`

---

*Document generated from Claude Code conversation - 2026-01-10*
