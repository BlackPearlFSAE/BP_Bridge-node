/* BMS Data Logger - AMS Node
 *
 * ESP32-S3 BMS data logger for Formula Student vehicle telemetry.
 * - Core 0: WiFi/WebSocket communication (BPMobile)
 * - Core 1: SD logging
 *
 * Platform: ESP32-S3 | Framework: Arduino + FreeRTOS
 */

/************************* Includes ***************************/
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Wire.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <driver/twai.h>

#include "BP_mobile_util.h"
#include "SD32_util.h"
#include "WIFI32_util.h"
#include "syncTime_util.h"
#include "DS3231_util.h"
#include "CAN32_util.h"
#include "ams_data_util.h"
#include "shared_config.h"

/************************* Pin Definitions ***************************/

// WebSocket and Network config
const char* ssid = DEFAULT_SSID;
const char* password = DEFAULT_PASSWORD;
const char* serverHost = DEFAULT_SERVER_HOST;
const int serverPort = DEFAULT_SERVER_PORT;
const char* clientName = "AMS_Node";

WebSocketsClient webSockets;
socketstatus webSocketStatus;

BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

// Sampling Rates (Hz)
const float BMU_CELLS_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float BMU_FAULT_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;

// AMS CAN Timing
#define BMS_RX_INTERVAL 10  // Poll CAN RX every 10ms

/************************* Global Variables ***************************/

// Peripherals
RTC_DS3231 rtc;
bool I2C1_connect = false;
bool RTCavailable = false;
bool sdCardReady = false;
bool canBusReady = false;

// Timing Intervals (ms)
const unsigned long LOCAL_SYNC_INTERVAL = DEFAULT_LOCAL_SYNC_INTERVAL;
const unsigned long REMOTE_SYNC_INTERVAL = DEFAULT_REMOTE_SYNC_INTERVAL;

const unsigned long SD_APPEND_INTERVAL = DEFAULT_SD_LOG_INTERVAL;
const unsigned long SD_FLUSH_INTERVAL = DEFAULT_SD_FLUSH_INTERVAL;
const unsigned long SD_CLOSE_INTERVAL = DEFAULT_SD_CLOSE_INTERVAL;
const int SD_MAX_ROWS = DEFAULT_SD_ROW_LIMIT;

uint64_t RTC_UNIX_TIME = 0;
unsigned long lastSDLog = 0;
unsigned long lastTeleplotDebug = 0;
const unsigned long TELEPLOT_DEBUG_INTERVAL = 200;
int dataPoint = 1;
int sessionNumber = 0;

// SD Card
char sessionDirPath[48] = {0};
char partDirPath[64] = {0};
int partIndex = 0;
char bmuFilePaths[MODULE_NUM][80] = {0};

// CSV Headers
const char* header_BMU = "DataPoint,UnixTime,SessionTime,V_MODULE,TEMP1,TEMP2,DV,"
                         "Cell1,Cell2,Cell3,Cell4,Cell5,Cell6,Cell7,Cell8,Cell9,Cell10,"
                         "OV_WARN,OV_CRIT,LV_WARN,LV_CRIT,OT_WARN,OT_CRIT,ODV_WARN,ODV_CRIT,"
                         "BAL_CELLS,Connected,ReadyToCharge";
// Sensor Data
BMUdata BMU_Package[MODULE_NUM];


/************************* Function Declarations ***************************/

// BPMobile Publishers
void publishBMUcells(BMUdata* bmu, int moduleNum);
void publishBMUfaults(BMUdata* bmu, int moduleNum);
void registerClient(const char* clientName);

// File Management
void showDeviceStatus();
void append_BMU_toCSVFile(File& dataFile, BMUdata* bmu, int dp, uint64_t Timestamp, uint64_t session);
void createPartitionDir();
void openAllFiles();
void closeAllFiles();
void flushAllFiles();

/************************* FreeRTOS ***************************/

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;  // For cross-core Serial prints
TaskHandle_t BPMobileTaskHandle = NULL;
TaskHandle_t timeSyncTaskHandle = NULL;
TaskHandle_t canTaskHandle = NULL;
TaskHandle_t sdTaskHandle = NULL;
QueueHandle_t sdQueue = NULL;

// SD Queue data structure
struct SDLogEntry {
  int dataPoint;
  uint64_t unixTime;
  uint64_t sessionTime;
  BMUdata bmu[MODULE_NUM];
};

// File handles for persistent logging (one per BMU)
static File bmuFiles[MODULE_NUM];
static bool filesOpen = false;
static volatile bool closeRequested = false;  // Signal from loop() to sdTask

// Core 0: WiFi/WebSocket Task
void BPMobileTask(void* parameter) {
  // Task-local timing variables
  unsigned long taskLastBMUcells = 0;
  unsigned long taskLastBMUfaults = 0;

  // Local copies of sensor data
  BMUdata localBMU[MODULE_NUM];

  while (true) {
    unsigned long SESSION_TIME = millis();

    // Handle WebSocket communication
    if (WiFi.status() == WL_CONNECTED) {
      BPwebSocket->loop();
    }

    // Update LED status
    // (digitalWriteWIFI_LED, WiFi.status() == WL_CONNECTED ? 1 : 0);
    // digitalWrite(WS_LED, BPsocketstatus->isConnected ? 1 : 0);

    // Publish data if registered and connected
    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      // Quick copy: take mutex briefly, copy data, release immediately
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(localBMU, BMU_Package, sizeof(BMU_Package));
        xSemaphoreGive(dataMutex);
      }

      // Publish BMU cell voltages
      if (SESSION_TIME - taskLastBMUcells >= (unsigned long)(1000.0 / BMU_CELLS_SAMPLING_RATE)) {
        for (int i = 0; i < MODULE_NUM; i++) {
          publishBMUcells(&localBMU[i], i);
        }
        taskLastBMUcells = SESSION_TIME;
      }

      // Publish BMU faults
      if (SESSION_TIME - taskLastBMUfaults >= (unsigned long)(1000.0 / BMU_FAULT_SAMPLING_RATE)) {
        for (int i = 0; i < MODULE_NUM; i++) {
          publishBMUfaults(&localBMU[i], i);
        }
        taskLastBMUfaults = SESSION_TIME;
      }
    }

    // Small delay to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Core 1: SD Card Logger Task
// Consumes SDLogEntry from queue, writes to MODULE_NUM persistent BMU CSV files.
// AMS uses two-level dirs: /AMS_session_000/AMS_p0/bmu_0.csv ... bmu_7.csv
// File rotation: after SD_MAX_ROWS rows, closes all BMU files, creates next
// partition subdir (AMS_p1/, AMS_p2/, ...) with fresh files in the same session.
// Session dir only renews on reboot.
void sdTask(void* parameter) {
  SDLogEntry entry;
  unsigned long lastFlushTime = 0;
  int localDataPoint = 0;

  while (true) {
    // Block until loop() enqueues a log entry
    if (xQueueReceive(sdQueue, &entry, portMAX_DELAY) == pdTRUE) {
      // Handle close request from loop() safely within sdTask context
      if (closeRequested) {
        closeAllFiles();
        sdCardReady = false;
        closeRequested = false;
        Serial.println("[SD Card] Card removed - files closed safely");
        continue;
      }

      if (!sdCardReady || !filesOpen) {
        continue;
      }

      // Row-based rotation: close all BMU files, create next partition subdir
      if (localDataPoint > 0 && localDataPoint % SD_MAX_ROWS == 0) {
        closeAllFiles();
        partIndex++;
        createPartitionDir();  // creates AMS_pN/ dir, CSV files, and opens them
        Serial.printf("[SD] Row limit reached, rotated to partition: %s\n", partDirPath);
      }

      unsigned long SESSION_TIME = millis();

      // Write one row to each BMU file (one file per battery module)
      for (int i = 0; i < MODULE_NUM; i++) {
        if (bmuFiles[i]) {
          append_BMU_toCSVFile(bmuFiles[i], &entry.bmu[i], entry.dataPoint, entry.unixTime, entry.sessionTime);
        }
      }

      // Flush all BMU files periodically to prevent data loss on power cut
      if (SESSION_TIME - lastFlushTime >= SD_FLUSH_INTERVAL) {
        flushAllFiles();
        lastFlushTime = SESSION_TIME;
      }

      localDataPoint++;
      if (localDataPoint % 50 == 0) {
        Serial.printf("[SD Task] Logged %d entries\n", localDataPoint);
      }
    }
  }
}

// Core 0: Time Synchronization Task
void timeSyncTask(void* parameter) {
  unsigned long lastLocalSync = 0;
  unsigned long lastRemoteSync = 0;

  while (true) {
    unsigned long SESSION_TIME = millis();

    // Local RTC poll every 1s
    if (SESSION_TIME - lastLocalSync >= LOCAL_SYNC_INTERVAL) {
      #if TIME_SRC == 0
      uint64_t t = (uint64_t)RTC_getUnix(rtc, RTCavailable) * 1000ULL;
      #elif TIME_SRC == 1
      uint64_t t = WiFi32_getNTPTime();
      #endif
      if (t > 0) syncTime_setSyncPoint(RTC_UNIX_TIME, t);
      lastLocalSync = SESSION_TIME;
    }

    // Remote NTP recalibration every 60s
    if (SESSION_TIME - lastRemoteSync >= REMOTE_SYNC_INTERVAL) {
      uint64_t externalTime = WiFi32_getNTPTime();
      if (externalTime > 0 && RTCavailable) {
        if (syncTime_ifDrifted(RTC_UNIX_TIME, externalTime, 1000))
          RTCcalibrate(rtc, RTC_UNIX_TIME / 1000ULL, RTCavailable);
      }
      lastRemoteSync = SESSION_TIME;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Core 1: CAN Bus Task (BMU communication - placeholder for AMS Master logic)
void canTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  twai_message_t rxmsg;

  while (true) {
    if (!canBusReady) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    // RX poll: receive BMU data frames
    if (CAN32_receiveCAN(&rxmsg) == ESP_OK) {
      // TODO: Implement BMU message parsing (AMS Master logic)
      // Expected: parse rxmsg based on BMU CAN IDs (BCU_ADD range)
      // and populate BMU_Package[] under dataMutex
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // process_BMU_CANmsg(&rxmsg, BMU_Package, MODULE_NUM);
        xSemaphoreGive(dataMutex);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BMS_RX_INTERVAL));
  }
}

/************************* Setup ***************************/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(UART0_BAUD);
  delay(100);

  Serial.println("==================================================");
  Serial.println("       BMS Data Logger - Initializing");
  Serial.println("==================================================");

  // I2C for RTC
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  Wire1.setTimeout(2);

  // RTC Init
  RTCavailable = RTCinit(rtc,&Wire1);
  // Sync ESP32 internal clock from DS3231 so FAT file timestamps are correct
  if (RTCavailable) {
    #if TIME_SRC == 0
    struct timeval tv = { .tv_sec = (time_t)rtc.now().unixtime(), .tv_usec = 0 };
    #elif TIME_SRC == 1
    struct timeval tv = { .tv_sec = (time_t)WiFi32_getNTPTime(), .tv_usec = 0 };
    #endif
    settimeofday(&tv, NULL);
    Serial.println("[RTC] ESP32 system clock set from DS3231");
  }

  // CAN Bus Init
  canBusReady = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN,
    TWAI_TIMING_CONFIG_250KBITS(), TWAI_FILTER_CONFIG_ACCEPT_ALL());

  // Connect to WiFi and Websocket
  initWiFi(ssid, password, 10);
  int ntpready; 
  if (WiFi.status() == WL_CONNECTED) {
    ntpready = WiFi32_initNTP();
    BPMobile.setClientName(clientName);
    BPMobile.setRegisterCallback(registerClient);
    BPMobile.initWebSocketSSL(serverHost, serverPort, clientName);
  }

  // SD Card Init
  SD32_initSDCard(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN, sdCardReady);
  if (sdCardReady) {
    SD32_createSessionDir(sessionNumber, sessionDirPath, "AMS");
    createPartitionDir();
  }

  #if calibrate_RTC == 1
    RTCcalibrate(rtc,(ntpready) ? (WiFi32_getNTPTime()/1000ULL): 1000000000000ULL ,RTCavailable);
  #endif
  syncTime_setSyncPoint(RTC_UNIX_TIME, RTCavailable ? RTC_getUnix(rtc,RTCavailable) : 1000000000000ULL);

  // FreeRTOS Setup
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) Serial.println("[ERROR] Failed to create dataMutex!");
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) Serial.println("[ERROR] Failed to create serialMutex!");
  sdQueue = xQueueCreate(30, sizeof(SDLogEntry));
  if (sdQueue == NULL) Serial.println("[ERROR] Failed to create SD queue!");

  // Core 0 tasks
  xTaskCreatePinnedToCore(BPMobileTask, "BPMobileTask", 8192, NULL, 1, &BPMobileTaskHandle, 0);
  Serial.println("[RTOS] BPMobile task on Core 0 (pri 1)");

  xTaskCreatePinnedToCore(timeSyncTask, "TimeSyncTask", 4096, NULL, 3, &timeSyncTaskHandle, 0);
  Serial.println("[RTOS] TimeSync task on Core 0 (pri 3)");

  // Core 1 tasks
  #if MOCK_FLAG == 0
    xTaskCreatePinnedToCore(canTask, "CANTask", 4096, NULL, 5, &canTaskHandle, 1);
    Serial.println("[RTOS] CAN task on Core 1 (pri 5)");
  #else
    Serial.println("[RTOS] CAN task SKIPPED (MOCK_FLAG=1)");
  #endif

  xTaskCreatePinnedToCore(sdTask, "SDTask", 8192, NULL, 2, &sdTaskHandle, 1);
  Serial.println("[RTOS] SD Logger task on Core 1 (pri 2)");

  Serial.println( "==================================================");
  Serial.println( "       BMS Data Logger - Ready");
  Serial.printf(  "       Session: %d | BMU Count: %d\n", sessionNumber, MODULE_NUM);
  Serial.printf(  "       Client: %s\n", clientName);
  Serial.println( "==================================================");
}

/************************* Main Loop ***************************/

void loop() {

  uint64_t SESSION_TIME_MS = millis();
  uint64_t CURRENT_UNIX_TIME_MS = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  #if calibrate_RTC == 1
    char timeBuf[32];
    syncTime_formatUnix(timeBuf, CURRENT_UNIX_TIME_MS, 7);  // UTC+7
    Serial.println(timeBuf);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  #endif

  // Debug Output
  #if DEBUG_MODE > 0
  if (SESSION_TIME_MS - lastTeleplotDebug >= TELEPLOT_DEBUG_INTERVAL) {
    BMUdata debugBMU;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      debugBMU = BMU_Package[0];
      xSemaphoreGive(dataMutex);
    }
    #if DEBUG_MODE == 1
      debugBMUModule(&debugBMU, 0);
    #elif DEBUG_MODE == 2
      teleplotBMU(&debugBMU, 0);
    #endif
    lastTeleplotDebug = SESSION_TIME_MS;
  }
  #endif

  // Mock data for testing (remove in production)
  #if MOCK_FLAG == 1
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      for (int i = 0; i < MODULE_NUM; i++) {
        mockBMU(&BMU_Package[i], i);
      }
      xSemaphoreGive(dataMutex);
    }
  #endif

  // Check SD card health — signal sdTask to close files safely
  if (sdCardReady && !SD32_checkSDconnect()) {
    closeRequested = true;
  }

  // Queue SD log entry
  if (sdCardReady && sdQueue != NULL && (SESSION_TIME_MS - lastSDLog >= SD_APPEND_INTERVAL)) {
    SDLogEntry entry;
    entry.dataPoint = dataPoint;
    entry.unixTime = CURRENT_UNIX_TIME_MS;
    entry.sessionTime = SESSION_TIME_MS;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(entry.bmu, BMU_Package, sizeof(BMU_Package));
      xSemaphoreGive(dataMutex);
    }

    if (xQueueSend(sdQueue, &entry, 0) == pdTRUE) {
      dataPoint++;
    } else {
      Serial.println("[Loop] SD queue full - data dropped");
    }

    lastSDLog = SESSION_TIME_MS;
  }

  vTaskDelay(pdMS_TO_TICKS(20));
}

/************************* File Management ***************************/

// Creates partition subdir inside session dir, generates BMU CSV files, and opens them.
// Called on boot and on every row-based rotation (every SD_MAX_ROWS entries).
// Result: /AMS_session_000/AMS_p0/bmu_0.csv ... bmu_7.csv
void createPartitionDir() {
  snprintf(partDirPath, sizeof(partDirPath), "%s/AMS_p%d", sessionDirPath, partIndex);
  SD.mkdir(partDirPath);
  Serial.printf("[SD] Created partition directory: %s\n", partDirPath);

  for (int i = 0; i < MODULE_NUM; i++) {
    SD32_generateFilenameInDir(bmuFilePaths[i], partDirPath, "bmu", i);
    SD32_createCSVFile(bmuFilePaths[i], header_BMU);
    Serial.printf("  BMU %d file: %s\n", i, bmuFilePaths[i]);
  }
  openAllFiles();
}

void openAllFiles() {
  for (int i = 0; i < MODULE_NUM; i++) {
    bmuFiles[i] = SD.open(bmuFilePaths[i], FILE_APPEND);
    if (!bmuFiles[i]) {
      Serial.printf("[SD Card] ERROR: Could not open BMU file %d\n", i);
    }
  }
  filesOpen = true;
  Serial.println("[SD Card] All files opened for logging");
}

void closeAllFiles() {
  for (int i = 0; i < MODULE_NUM; i++) {
    if (bmuFiles[i]) {
      bmuFiles[i].flush();
      bmuFiles[i].close();
    }
  }
  filesOpen = false;
  Serial.println("[SD Card] All files closed");
}

void flushAllFiles() {
  for (int i = 0; i < MODULE_NUM; i++) {
    if (bmuFiles[i]) {
      bmuFiles[i].flush();
    }
  }
}

/************************* CSV Appenders ***************************/

void append_BMU_toCSVFile(File& dataFile, BMUdata* bmu, int dp, uint64_t Timestamp, uint64_t session) {
  // DataPoint, UnixTime, SessionTime
  dataFile.print(dp); dataFile.print(",");
  dataFile.print(Timestamp); dataFile.print(",");
  dataFile.print(session); dataFile.print(",");

  // V_MODULE (scaled: 0.02V per bit), TEMP1, TEMP2 (decoded: (val * 0.5) - 40), DV (scaled: 0.1V per bit)
  dataFile.print(bmu->V_MODULE * 0.02f, 2); dataFile.print(",");
  dataFile.print((bmu->TEMP_SENSE[0] * 0.5f) - 40.0f, 1); dataFile.print(",");
  dataFile.print((bmu->TEMP_SENSE[1] * 0.5f) - 40.0f, 1); dataFile.print(",");
  dataFile.print(bmu->DV * 0.1f, 2); dataFile.print(",");

  // Cell voltages (10 cells, scaled: 0.02V per bit)
  for (int i = 0; i < CELL_NUM; i++) {
    dataFile.print(bmu->V_CELL[i] * 0.02f, 3);
    dataFile.print(",");
  }

  // Fault flags (16-bit bitmasks, log as hex for readability)
  dataFile.print(bmu->OVERVOLTAGE_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERVOLTAGE_CRITICAL, HEX); dataFile.print(",");
  dataFile.print(bmu->LOWVOLTAGE_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->LOWVOLTAGE_CRITICAL, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERTEMP_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERTEMP_CRITICAL, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERDIV_VOLTAGE_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERDIV_VOLTAGE_CRITICAL, HEX); dataFile.print(",");

  // Status (balancing as hex bitmask)
  dataFile.print(bmu->BalancingDischarge_Cells, HEX); dataFile.print(",");
  dataFile.print(bmu->BMUconnected); dataFile.print(",");
  dataFile.println(bmu->BMUneedBalance);
}

/************************* BPMobile Publishers ***************************/

void registerClient(const char* name) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = name;

  // Groups: one cells + one faults group per BMU module
  JsonArray groups = doc["groups"].to<JsonArray>();
  for (int i = 0; i < MODULE_NUM; i++) {
    char buf[24];
    JsonObject gc = groups.add<JsonObject>();
    snprintf(buf, sizeof(buf), "bmu%d.cells", i);
    gc["group"] = (const char*)buf; gc["rate_hz"] = BMU_CELLS_SAMPLING_RATE;

    JsonObject gf = groups.add<JsonObject>();
    snprintf(buf, sizeof(buf), "bmu%d.faults", i);
    gf["group"] = (const char*)buf; gf["rate_hz"] = BMU_FAULT_SAMPLING_RATE;
  }

  // Schema
  JsonArray schema = doc["schema"].to<JsonArray>();
  char key[48], grp[24];

  for (int i = 0; i < MODULE_NUM; i++) {
    snprintf(grp, sizeof(grp), "bmu%d.cells", i);

    auto addCell = [&](const char* field, const char* type, const char* unit, float scale, float offset = 0, int length = 0) {
      JsonObject e = schema.add<JsonObject>();
      snprintf(key, sizeof(key), "%s.%s", grp, field);
      e["key"] = (const char*)key; e["type"] = type; e["unit"] = unit;
      e["scale"] = scale; e["offset"] = offset; e["group"] = (const char*)grp;
      if (length > 0) e["length"] = length;
    };

    addCell("V_MODULE",   "uint16",   "V", 0.02);
    addCell("V_CELL",     "uint8[]",  "V", 0.02, 0, CELL_NUM);
    addCell("TEMP_SENSE", "uint16[]", "C", 0.5, -40, TEMP_SENSOR_NUM);
    addCell("DV",         "uint8",    "V", 0.1);
    addCell("connected",  "bool",     "",  1);

    snprintf(grp, sizeof(grp), "bmu%d.faults", i);

    auto addFault = [&](const char* field, const char* type, const char* unit) {
      JsonObject e = schema.add<JsonObject>();
      snprintf(key, sizeof(key), "%s.%s", grp, field);
      e["key"] = (const char*)key; e["type"] = type; e["unit"] = unit;
      e["scale"] = 1; e["offset"] = 0; e["group"] = (const char*)grp;
    };

    addFault("OV_WARN",   "uint16", "flags");
    addFault("OV_CRIT",   "uint16", "flags");
    addFault("LV_WARN",   "uint16", "flags");
    addFault("LV_CRIT",   "uint16", "flags");
    addFault("OT_WARN",   "uint16", "flags");
    addFault("OT_CRIT",   "uint16", "flags");
    addFault("ODV_WARN",  "uint16", "flags");
    addFault("ODV_CRIT",  "uint16", "flags");
    addFault("BAL_CELLS", "uint16", "flags");
    addFault("NEED_BAL",  "bool",   "");
  }

  String registration;
  serializeJson(doc, registration);
  Serial.println("[WS] Sending registration...");
  BPwebSocket->sendTXT(registration);
}

void publishBMUcells(BMUdata* bmu, int moduleNum) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  JsonDocument doc;
  doc["type"] = "data";

  char groupBuf[24];
  snprintf(groupBuf, sizeof(groupBuf), "bmu%d.cells", moduleNum);
  doc["group"] = groupBuf;
  doc["ts"] = timestamp;

  JsonObject d = doc["d"].to<JsonObject>();
  d["V_MODULE"] = bmu->V_MODULE;

  JsonArray cells = d["V_CELL"].to<JsonArray>();
  for (int i = 0; i < CELL_NUM; i++) {
    cells.add(bmu->V_CELL[i]);
  }

  JsonArray temps = d["TEMP_SENSE"].to<JsonArray>();
  for (int i = 0; i < TEMP_SENSOR_NUM; i++) {
    temps.add(bmu->TEMP_SENSE[i]);
  }

  d["DV"] = bmu->DV;
  d["connected"] = bmu->BMUconnected;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishBMUfaults(BMUdata* bmu, int moduleNum) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  JsonDocument doc;
  doc["type"] = "data";

  char groupBuf[24];
  snprintf(groupBuf, sizeof(groupBuf), "bmu%d.faults", moduleNum);
  doc["group"] = groupBuf;
  doc["ts"] = timestamp;

  JsonObject d = doc["d"].to<JsonObject>();
  d["OV_WARN"]   = bmu->OVERVOLTAGE_WARNING;
  d["OV_CRIT"]   = bmu->OVERVOLTAGE_CRITICAL;
  d["LV_WARN"]   = bmu->LOWVOLTAGE_WARNING;
  d["LV_CRIT"]   = bmu->LOWVOLTAGE_CRITICAL;
  d["OT_WARN"]   = bmu->OVERTEMP_WARNING;
  d["OT_CRIT"]   = bmu->OVERTEMP_CRITICAL;
  d["ODV_WARN"]  = bmu->OVERDIV_VOLTAGE_WARNING;
  d["ODV_CRIT"]  = bmu->OVERDIV_VOLTAGE_CRITICAL;
  d["BAL_CELLS"] = bmu->BalancingDischarge_Cells;
  d["NEED_BAL"]  = bmu->BMUneedBalance;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}


/************************* Teleplot Debug Functions ***************************/

void teleplotBMU(BMUdata *bmu, int moduleNum) {
  Serial.printf(">BMU%d_V_MODULE:%.2f\n", moduleNum, bmu->V_MODULE * 0.02f);
  Serial.printf(">BMU%d_TEMP1:%.1f\n", moduleNum, (bmu->TEMP_SENSE[0] * 0.5f) - 40.0f);
  Serial.printf(">BMU%d_TEMP2:%.1f\n", moduleNum, (bmu->TEMP_SENSE[1] * 0.5f) - 40.0f);
  Serial.printf(">BMU%d_DV:%.2f\n", moduleNum, bmu->DV * 0.1f);
  Serial.printf(">BMU%d_Connected:%d\n", moduleNum, bmu->BMUconnected ? 1 : 0);
}

void showDeviceStatus() {
  Serial.println("╔═════════════════════════════════════════════╗");
  Serial.println("║         AMS NODE - SYSTEM STATUS            ║");
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.printf("║ I2C1:         %s\n", I2C1_connect ? "OK" : "FAIL");
  Serial.printf("║ CAN Bus:      %s\n", canBusReady ? "OK" : "FAIL");
  Serial.printf("║ SD Card:      %s\n", sdCardReady ? "OK" : "FAIL");
  Serial.printf("║ WiFi:         %s (RSSI: %d)\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL", WiFi.RSSI());
  Serial.printf("║ RTC:          %s\n", RTCavailable ? "OK" : "FAIL");
  Serial.printf("║ WebSocket:    %s\n", BPsocketstatus->isConnected ? "OK" : "FAIL");
  Serial.printf("║ Time Sync:    %s\n", syncTime_isSynced() ? "OK" : "FAIL");
  Serial.println("╚═════════════════════════════════════════════╝");
}