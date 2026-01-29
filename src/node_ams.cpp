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
void mockBMUData();
void showDeviceStatus();
void append_BMU_toCSVFile(File& dataFile, BMUdata* bmu, int dp, uint64_t Timestamp, uint64_t session);
void createPartitionDir();
void openAllFiles();
void closeAllFiles();
void flushAllFiles();

/************************* Build Flags ***************************/

#define MOCK_DATA 0
#define calibrate_RTC 0
#define DEBUG_MODE 0  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot

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
      uint64_t t = (uint64_t)RTC_getUnix(rtc, RTCavailable) * 1000ULL;
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
    BPMobile.initWebSocket(serverHost, serverPort, clientName);
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
  #if MOCK_DATA == 0
    xTaskCreatePinnedToCore(canTask, "CANTask", 4096, NULL, 5, &canTaskHandle, 1);
    Serial.println("[RTOS] CAN task on Core 1 (pri 5)");
  #else
    Serial.println("[RTOS] CAN task SKIPPED (MOCK_DATA=1)");
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
    #if DEBUG_MODE == 1
      debugBMUModule(BMU_Package, 0);
    #elif DEBUG_MODE == 2
      teleplotBMU(&BMU_Package[0], 0);
    #endif
    lastTeleplotDebug = SESSION_TIME_MS;
  }
  #endif

  // Mock data for testing (remove in production)
  #if MOCK_DATA == 1
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      mockBMUData();
      xSemaphoreGive(dataMutex);
    }
  #endif

  // Check SD card health
  if (sdCardReady && !SD32_checkSDconnect()) {
    closeAllFiles();
    sdCardReady = false;
    Serial.println("[SD Card] Card removed - files closed safely");
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
  dataFile.print(bmu->BMUneedBalance);
}

/************************* BPMobile Publishers ***************************/

void registerClient(const char* name) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = name;

  // Define topics for BMS data
  JsonArray topics = doc["topics"].to<JsonArray>();

  // BMU cell topics (one per module)
  for (int i = 0; i < MODULE_NUM; i++) {
    char topic[32];
    snprintf(topic, sizeof(topic), "bms/bmu%d/cells", i);
    topics.add(topic);
    snprintf(topic, sizeof(topic), "bms/bmu%d/faults", i);
    topics.add(topic);
  }

  // Define topic metadata
  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();

  for (int i = 0; i < MODULE_NUM; i++) {
    char topic[32];
    snprintf(topic, sizeof(topic), "bms/bmu%d/cells", i);
    JsonObject cellMeta = metadata[topic].to<JsonObject>();
    cellMeta["description"] = "BMU cell voltages";
    cellMeta["unit"] = "V";
    cellMeta["sampling_rate"] = BMU_CELLS_SAMPLING_RATE;

    snprintf(topic, sizeof(topic), "bms/bmu%d/faults", i);
    JsonObject faultMeta = metadata[topic].to<JsonObject>();
    faultMeta["description"] = "BMU fault flags";
    faultMeta["unit"] = "flags";
    faultMeta["sampling_rate"] = BMU_FAULT_SAMPLING_RATE;
  }

  String registration;
  serializeJson(doc, registration);

  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println("[WebSocket] Sending registration...");
    xSemaphoreGive(serialMutex);
  }
  BPwebSocket->sendTXT(registration);
}

void publishBMUcells(BMUdata* bmu, int moduleNum) {
  uint64_t timestamp = WiFi32_getNTPTime();
  if (timestamp == 0) timestamp = millis();

  JsonDocument doc;
  doc["type"] = "data";

  char topic[32];
  snprintf(topic, sizeof(topic), "bms/bmu%d/cells", moduleNum);
  doc["topic"] = topic;

  JsonObject data = doc["data"].to<JsonObject>();
  data["module"] = moduleNum;
  data["v_module"] = bmu->V_MODULE * 0.02f;  // Scaled to actual volts

  JsonArray cells = data["cells"].to<JsonArray>();
  for (int i = 0; i < CELL_NUM; i++) {
    cells.add(bmu->V_CELL[i] * 0.02f);  // Scaled to actual volts
  }

  JsonArray temps = data["temps"].to<JsonArray>();
  for (int i = 0; i < TEMP_SENSOR_NUM; i++) {
    temps.add((bmu->TEMP_SENSE[i] * 0.5f) - 40.0f);  // Decoded to Celsius
  }

  data["dv"] = bmu->DV * 0.1f;  // Scaled to actual volts
  data["connected"] = bmu->BMUconnected;

  doc["timestamp"] = timestamp;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishBMUfaults(BMUdata* bmu, int moduleNum) {
  uint64_t timestamp = WiFi32_getNTPTime();
  if (timestamp == 0) timestamp = millis();

  JsonDocument doc;
  doc["type"] = "data";

  char topic[32];
  snprintf(topic, sizeof(topic), "bms/bmu%d/faults", moduleNum);
  doc["topic"] = topic;

  JsonObject data = doc["data"].to<JsonObject>();
  data["module"] = moduleNum;
  data["ov_warn"] = bmu->OVERVOLTAGE_WARNING;
  data["ov_crit"] = bmu->OVERVOLTAGE_CRITICAL;
  data["lv_warn"] = bmu->LOWVOLTAGE_WARNING;
  data["lv_crit"] = bmu->LOWVOLTAGE_CRITICAL;
  data["ot_warn"] = bmu->OVERTEMP_WARNING;
  data["ot_crit"] = bmu->OVERTEMP_CRITICAL;
  data["odv_warn"] = bmu->OVERDIV_VOLTAGE_WARNING;
  data["odv_crit"] = bmu->OVERDIV_VOLTAGE_CRITICAL;
  data["BalancingCells"] = bmu->BalancingDischarge_Cells;
  data["NeedBalancing"] = bmu->BMUneedBalance;

  doc["timestamp"] = timestamp;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void mockBMUData() {
    // Half identical (good modules)
    for (int i = 0; i < MODULE_NUM / 2; i++) {
        BMU_Package[i].BMU_ID = 0x18200001 + (i << 16);
        BMU_Package[i].BMUconnected = true;
        BMU_Package[i].BMUneedBalance = 1;
        BMU_Package[i].DV = 5;
        BMU_Package[i].TEMP_SENSE[0] = 0xC8;
        BMU_Package[i].TEMP_SENSE[1] = 0xC8;
        
        for (int j = 0; j < CELL_NUM; j++) {
            BMU_Package[i].V_CELL[j] = 185;
        }
        
        BMU_Package[i].OVERVOLTAGE_WARNING = 0x0000;
        BMU_Package[i].OVERVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].LOWVOLTAGE_WARNING = 0x0000;
        BMU_Package[i].LOWVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].OVERTEMP_WARNING = 0x0000;
        BMU_Package[i].OVERTEMP_CRITICAL = 0x0000;
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = 0x0000;
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].BalancingDischarge_Cells = 0x0000;
    }
    
    // Half mixed bag (faulty modules)
    for (int i = MODULE_NUM / 2; i < MODULE_NUM; i++) {
        BMU_Package[i].BMU_ID = 0x18200001 + (i << 16);
        BMU_Package[i].BMUconnected = true;
        BMU_Package[i].BMUneedBalance = 0;
        BMU_Package[i].DV = 15;
        BMU_Package[i].TEMP_SENSE[0] = 0xFA;
        BMU_Package[i].TEMP_SENSE[1] = 0xD0;
        
        BMU_Package[i].V_CELL[0] = 210;
        BMU_Package[i].V_CELL[1] = 205;
        BMU_Package[i].V_CELL[2] = 160;
        BMU_Package[i].V_CELL[3] = 185;
        BMU_Package[i].V_CELL[4] = 190;
        BMU_Package[i].V_CELL[5] = 155;
        BMU_Package[i].V_CELL[6] = 200;
        BMU_Package[i].V_CELL[7] = 185;
        BMU_Package[i].V_CELL[8] = 195;
        BMU_Package[i].V_CELL[9] = 175;
        
        BMU_Package[i].OVERVOLTAGE_WARNING = 0x0200;
        BMU_Package[i].OVERVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].LOWVOLTAGE_WARNING = 0x0090;
        BMU_Package[i].LOWVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].OVERTEMP_WARNING = 0x0200;
        BMU_Package[i].OVERTEMP_CRITICAL = 0x0000;
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = 0x0094;
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].BalancingDischarge_Cells = 0x0201;
    }
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
  Serial.println("║         REAR NODE - SYSTEM STATUS           ║");
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.printf("║ I2C1:         %s\n", I2C1_connect ? "OK" : "FAIL");
  // Serial.printf("║ I2C2:         %s\n", I2C2_connect ? "OK" : "FAIL");
  Serial.printf("║ SD Card:      %s\n", sdCardReady ? "OK" : "FAIL");
  Serial.printf("║ WiFi:         %s (RSSI: %d)\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL", WiFi.RSSI());
  Serial.printf("║ RTC:          %s\n", RTCavailable ? "OK" : "FAIL");
  Serial.printf("║ WebSocket:    %s\n", BPsocketstatus->isConnected ? "OK" : "FAIL");
  Serial.printf("║ Time Sync:    %s\n", syncTime_isSynced() ? "OK" : "FAIL");
  // Serial.println("╠═════════════════════════════════════════════╣");
  // Serial.printf("║ IMU:          %s\n", IMUavailable ? "OK" : "FAIL");
  // Serial.printf("║ GPS:          %s\n", gpsSerial.available() > 0 ? "OK" : "FAIL");
  Serial.println("╚═════════════════════════════════════════════╝");
}