# Teleplot Debug Implementation

## Overview
Added teleplot-compatible serial debug functions for real-time visualization using the VSCode Teleplot extension.

## Changes Made

### 1. Library Functions (lib/ams_data_util/)

**ams_data_util.h** - New function declarations:
```cpp
void teleplotAMSstate(AMSdata *myAMS);
void teleplotBMUModule(BMUdata *myBMU, int moduleNum);
void teleplotBMUCellVoltages(BMUdata *myBMU, int moduleNum);
void teleplotBMUTemperatures(BMUdata *myBMU, int moduleNum);
void teleplotBMUFaults(BMUdata *myBMU, int moduleNum);
void teleplotOBCmsg(OBCdata *myOBC);
void teleplotAllModules(BMUdata *BMU_Package, int moduleCount);
void teleplotLocalCells(float *cellvoltages, int cellCount, const char* prefix);
```

**ams_data_util.cpp** - Implementations using teleplot format `>variable:value`

### 2. BCU Integration (src/bcu_integrate.cpp)

DEBUG_MODE 2 section (lines 228-243):
- `teleplotAMSstate()` - AMS aggregate voltage and fault flags
- `teleplotAllModules()` - All module voltages overview
- `teleplotLocalCells()` - BCU local cell voltages (raw float)
- BCU temperatures: `>BCU_T1:val|BCU_T2:val`
- System flags: `>ChgPlugged:val|AMS_OK:val`, `>AccumFull:val|AccumLow:val`

### 3. BMU (src/bmu.cpp)

DEBUG_MODE 2 section (lines 167-192):
- `teleplotLocalCells()` - Raw cell voltages
- Temperatures: `>BMU_T1:val|BMU_T2:val`
- Module status: `>BMU_DV:val`, `>BMU_NeedBal:val|BCU_AllowBal:val`
- Module voltage: `>BMU_ModuleV:val`
- Fault counts: `>BMU_OVfault:val|BMU_LVfault:val`

### 4. Bug Fix

Moved `ModuleNumber` variable declaration before `setup()` to fix compilation error (was declared after usage in setup printf).

## Usage

To enable/disable debug output, change the `DEBUG_MODE` define in each node file:
```cpp
#define DEBUG_MODE 2  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot
```

### Debug Modes
- **Mode 0**: Debug output completely disabled (production build)
- **Mode 1**: Regular serial debug with human-readable output
- **Mode 2**: Teleplot format for VSCode Teleplot extension visualization

### Node Files with DEBUG_MODE Support
- `src/node.cpp` - Production build (all sensors)
- `src/node_ams.cpp` - AMS/BMS node
- `src/node_front.cpp` - Front sensor node (BAMO, Mechanical, Electrical)
- `src/node_rear.cpp` - Rear sensor node (Odometry, Mechanical, Electrical)

## Teleplot Format Reference

- Single value: `>variable_name:value`
- Multiple values: `>var1:val1|var2:val2`
- Array plotting: Use loop with indexed names like `>BMU_C1:3.7`

## Build Status

Both environments build successfully:
- BMU: SUCCESS
- BCU: SUCCESS
