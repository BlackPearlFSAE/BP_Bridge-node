# AMS Datalog Reference

Reference for the AMS node's data scaling, SD card file-handle limits, and multi-file persistent logging API. Current source: `src/node_ams.cpp` + `lib/SD32_util/`.

---

## 1. BMU/AMS Data Scaling

### Context
BCU/BMU payloads arrive over CAN in encoded form. CSV and WebSocket outputs must decode before writing, otherwise logs contain raw integers instead of volts/°C.

### Changes Made

#### BMU CSV Output (`append_BMU_toCSVFile`)
| Field | Before | After |
|-------|--------|-------|
| `V_MODULE` | Raw uint16_t | `* 0.02f` (actual volts) |
| `TEMP_SENSE[]` | Raw uint8_t | `(val * 0.5f) - 40.0f` (Celsius) |
| `DV` | Raw uint8_t | `* 0.1f` (volts) |
| `V_CELL[]` | Raw uint8_t | `* 0.02f` (volts) |
| Fault flags | Decimal | HEX format for bitmask readability |

#### AMS CSV Output (`append_AMS_toCSVFile`)
- Bool flags now explicitly output as `0` or `1` using ternary operator

#### Mock Data Section
- Fixed field name: `bmu_id` → `BMU_ID`
- Cell voltages encoded correctly (180-205 = 3.6-4.1V range)
- Temperature encoded correctly (130 = 25C with offset/factor)
- `V_MODULE` computed as sum of encoded cells

#### WebSocket Publishing (`publishBMUcells`)
- `v_module`, `cells[]`, `temps[]`, `dv` now publish decoded/scaled values

---

## 2. SD Card File Descriptor Exhaustion Fix

### Problem
```
E (5654) vfs_fat: open: no free file descriptors
[SD Card] ERROR: Could not open BMU file 5
```
ESP32 FAT VFS default limit is ~5 open files. Datalog needs 9 (8 BMU + 1 AMS).

### Solution
Modified `SD32_initSDCard()` in `SD32_util.cpp`:

```cpp
// Before
SD.begin(sd_cs)

// After - with max_files parameter
SD.begin(sd_cs, SPI, 4000000, "/sd", 20, false)
```

The 5th parameter sets `max_files = 20`, allowing up to 20 concurrent file handles.

---

## 3. Multi-File Persistent API Added to SD32_util

### New Functions in `SD32_util.h`

```cpp
#define SD32_MAX_PERSISTENT_FILES 16

void SD32_initPersistentFileArray();
int  SD32_openPersistentFileArray(fs::FS &fs, const char* filepath);  // Returns index
void SD32_closePersistentFileByIndex(int index);
void SD32_closeAllPersistentFiles();
bool SD32_isPersistentFileOpenByIndex(int index);
int  SD32_getOpenFileCount();
void SD32_appendToPersistentFile(int index, AppenderFunc appender, void* data);
void SD32_flushPersistentFileByIndex(int index);
void SD32_flushAllPersistentFiles();
File* SD32_getPersistentFile(int index);  // For direct writes
```

### Usage Example (for future datalog.cpp refactor)

```cpp
int bmuFileIndices[MODULE_NUM];
int amsFileIndex;

void openAllFiles() {
  for (int i = 0; i < MODULE_NUM; i++) {
    bmuFileIndices[i] = SD32_openPersistentFileArray(SD, bmuFilePaths[i]);
  }
  amsFileIndex = SD32_openPersistentFileArray(SD, amsFilePath);
}

void closeAllFiles() {
  SD32_closeAllPersistentFiles();
}

void flushAllFiles() {
  SD32_flushAllPersistentFiles();
}

// Direct write
File* f = SD32_getPersistentFile(bmuFileIndices[0]);
if (f) f->println("data");
```

---

## Related Files

| File | Role |
|------|------|
| `src/node_ams.cpp` | AMS node — CSV/WebSocket output with decoded scaling |
| `lib/SD32_util/SD32_util.h` | Multi-file persistent API declarations |
| `lib/SD32_util/SD32_util.cpp` | Multi-file implementation, `max_files=20` |
| `lib/ams_data_util/` | BMU/AMS/OBC structs and CAN parser |

---

## Encoding Reference

| Data | Encoding | Decode Formula |
|------|----------|----------------|
| Cell Voltage | uint8_t, 0.02V/bit | `val * 0.02f` |
| Module Voltage | uint16_t, sum of cells | `val * 0.02f` |
| Temperature | uint8_t, offset -40, 0.5C/bit | `(val * 0.5f) - 40.0f` |
| Delta Voltage | uint8_t, 0.1V/bit | `val * 0.1f` |
| Fault Flags | uint16_t bitmask | Cell 1 = bit 9, Cell 10 = bit 0 |
