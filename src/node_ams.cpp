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

#include "BP_mobile_util.h"
#include "SD32_util.h"
#include "WIFI32_util.h"
#include "syncTime_util.h"
#include "DS3231_util.h"
#include "ams_data_util.h"

/************************* Pin Definitions ***************************/

// Network
const char* ssid = "realme C55";
const char* password = "realme1234";
const char* serverHost = "10.18.211.132";
const int serverPort = 3000;
const char* clientName = "AMS_Node";

WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

// Sampling Rates (Hz)
const float BMU_CELLS_SAMPLING_RATE = (DEFAULT_PUBLISH_RATE/5);
const float BMU_FAULT_SAMPLING_RATE = (DEFAULT_PUBLISH_RATE/5);

// LEDs
#define WIFI_LED 3
#define WS_LED 4

// I2C
#define I2C1_SDA 18
#define I2C1_SCL 17

// SD SPI
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_SCK_PIN 12

// UART
#define UART0_BAUD 115200

/************************* Global Variables ***************************/

// Peripherals
RTC_DS3231 rtc;
bool I2C1_connect = false;
bool RTCavailable = false;
bool sdCardReady = false;

// Timing Intervals (ms)
const unsigned long LOCAL_SYNC_INTERVAL = 1000;
const unsigned long REMOTE_SYNC_INTERVAL = 60000;
const unsigned long SD_APPEND_INTERVAL = DEFAULT_SD_LOG_INTERVAL;
const unsigned long SD_FLUSH_INTERVAL = 1000;
const unsigned long SD_CLOSE_INTERVAL = 10000;

unsigned long lastTimeSourceSync = 0;
unsigned long lastExternalSync = 0;
uint64_t RTC_UNIX_TIME = 0;
unsigned long lastSDLog = 0;
unsigned long lastTeleplotDebug = 0;
const unsigned long TELEPLOT_DEBUG_INTERVAL = 200;
int dataPoint = 1;
int sessionNumber = 0;

// SD Card
char sessionDirPath[32] = {0};
char bmuFilePaths[MODULE_NUM][48] = {0};

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
void append_BMU_toCSVFile(File& dataFile, BMUdata* bmu, int dp, uint64_t Timestamp, uint64_t session);
void openAllFiles();
void closeAllFiles();
void flushAllFiles();

/************************* FreeRTOS ***************************/

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;  // For cross-core Serial prints
TaskHandle_t BPMobileTaskHandle = NULL;
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
    unsigned long now = millis();

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
      if (now - taskLastBMUcells >= (unsigned long)(1000.0 / BMU_CELLS_SAMPLING_RATE)) {
        for (int i = 0; i < MODULE_NUM; i++) {
          publishBMUcells(&localBMU[i], i);
        }
        taskLastBMUcells = now;
      }

      // Publish BMU faults
      if (now - taskLastBMUfaults >= (unsigned long)(1000.0 / BMU_FAULT_SAMPLING_RATE)) {
        for (int i = 0; i < MODULE_NUM; i++) {
          publishBMUfaults(&localBMU[i], i);
        }
        taskLastBMUfaults = now;
      }
    }

    // Small delay to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Core 1: SD Card Logger Task
void sdTask(void* parameter) {
  SDLogEntry entry;
  unsigned long lastFlushTime = 0;
  int localDataPoint = 0;

  while (true) {
    // Wait for data from queue
    if (xQueueReceive(sdQueue, &entry, portMAX_DELAY) == pdTRUE) {
      if (!sdCardReady || !filesOpen) {
        continue;
      }

      unsigned long now = millis();
      // Write to all BMU files (every entry = 200ms)
      for (int i = 0; i < MODULE_NUM; i++) {
        if (bmuFiles[i]) {
          append_BMU_toCSVFile(bmuFiles[i], &entry.bmu[i], entry.dataPoint, entry.unixTime, entry.sessionTime);
        }
      }
      // Flush periodically
      if (now - lastFlushTime >= SD_FLUSH_INTERVAL) {
        flushAllFiles();
        lastFlushTime = now;
      }

      localDataPoint++;
      if (localDataPoint % 50 == 0) {
        Serial.printf("[SD Task] Logged %d entries\n", localDataPoint);
      }
    }
  }
}

/************************* Setup ***************************/

#define MOCK_DATA 0
#define calibrate_RTC 0
#define DEBUG_MODE 2  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot

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

  // Connect to WiFi and Websocket
  initWiFi(ssid, password, 10);
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
    // Create session directory
    SD32_createSessionDir(sessionNumber, sessionDirPath);

    // Generate file paths for each BMU
    for (int i = 0; i < MODULE_NUM; i++) {
      SD32_generateFilenameInDir(bmuFilePaths[i], sessionDirPath, "bmu", i);
      Serial.printf("  BMU %d file: %s\n", i, bmuFilePaths[i]);
    }

    // Create CSV files with headers
    for (int i = 0; i < MODULE_NUM; i++) 
    SD32_createCSVFile(bmuFilePaths[i], header_BMU);
    
    // Open all files for persistent logging
    openAllFiles();
  }

  #if calibrate_RTC == 1
    RTCcalibrate(rtc,(ntpready) ? (WiFi32_getNTPTime()/1000ULL): 1000000000000ULL ,RTCavailable);
  #endif
  syncTime_setSyncPoint(RTC_UNIX_TIME, RTCavailable ? RTC_getUnix(rtc,RTCavailable) : 1000000000000ULL);
  Serial.print("Time synced: ");
  Serial.println(RTC_getISO(rtc,RTCavailable));

  // Create mutex for shared sensor data
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) Serial.println("[ERROR] Failed to create dataMutex!");
  
  // Create mutex for Serial prints (prevents garbled output across cores)
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) Serial.println("[ERROR] Failed to create serialMutex!");
  
  // Create SD queue (hold up to 30 entries)
  sdQueue = xQueueCreate(30, sizeof(SDLogEntry));
  if (sdQueue == NULL) Serial.println("[ERROR] Failed to create SD queue!");

  // Create BPMobile task on Core 0
  xTaskCreatePinnedToCore(BPMobileTask,"BPMobileTask",8192,NULL,1,
    &BPMobileTaskHandle,0);
  Serial.println("[FreeRTOS] BPMobile task started on Core 0");

  // Create SD task on Core 1
  xTaskCreatePinnedToCore(sdTask,"SDTask",8192,NULL,1,&sdTaskHandle,1);
  Serial.println("[FreeRTOS] SD Logger task started on Core 1");

  Serial.println("==================================================");
  Serial.println("       BMS Data Logger - Ready");
  Serial.printf("       Session: %d | BMU Count: %d\n", sessionNumber, MODULE_NUM);
  Serial.printf("       Client: %s\n", clientName);
  Serial.println("==================================================");
}

/************************* Main Loop ***************************/

void loop() {

  uint64_t SESSION_TIME_MS = millis();
  // Time Sync: fetch RTC DS3231 every 1s 
  uint64_t Time_placeholder = 0;
  if (SESSION_TIME_MS - lastTimeSourceSync >= LOCAL_SYNC_INTERVAL) {
    Time_placeholder = (uint64_t)RTC_getUnix(rtc,RTCavailable)*1000ULL;
    lastTimeSourceSync = SESSION_TIME_MS;
  }
  // Set syncpoint , and calculate UnixTime_ms + ElapseTime_ms 
  if (Time_placeholder > 0) syncTime_setSyncPoint(RTC_UNIX_TIME, Time_placeholder);
  uint64_t CURRENT_UNIX_TIME_MS = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  #if calibrate_RTC == 1
    char timeBuf[32];
    syncTime_formatUnix(timeBuf, CURRENT_UNIX_TIME_MS, 7);  // UTC+7
    Serial.println(timeBuf);
    // Serial.println(CURRENT_UNIX_TIME_MS);
    return;
  #endif

  // Time Sync: Remote source -> DS3231 (60s period)
  if (SESSION_TIME_MS - lastExternalSync >= REMOTE_SYNC_INTERVAL) {
    uint64_t externalTime = WiFi32_getNTPTime();              // NTP
    // uint64_t externalTime = BPMobile_getLastServerTime();  // Server

    if (externalTime > 0 && RTCavailable) {
      if(syncTime_ifDrifted(RTC_UNIX_TIME, externalTime,1000))
        RTCcalibrate(rtc,RTC_UNIX_TIME/1000ULL,RTCavailable);
    }
    lastExternalSync = SESSION_TIME_MS;
  }

  // Debug Output
  #if DEBUG_MODE > 0
  if (SESSION_TIME_MS - lastTeleplotDebug >= TELEPLOT_DEBUG_INTERVAL) {
    #if DEBUG_MODE == 1
      // Regular serial debug
      debugBMUModule(BMU_Package, 0);
    #elif DEBUG_MODE == 2
      // Teleplot format debug
      teleplotBMU(&BMU_Package[0], 0);
    #endif
    lastTeleplotDebug = SESSION_TIME_MS;
  }
  #endif

  // Mock data for testing (remove in production)
  #if MOCK_DATA == 1
    for (int i = 0; i < MODULE_NUM; i++) {
      mockBMUData(i);
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

    // Copy sensor data with mutex
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(entry.bmu, BMU_Package, sizeof(BMU_Package));
      xSemaphoreGive(dataMutex);
    }

    // Non-blocking queue send
    if (xQueueSend(sdQueue, &entry, 0) == pdTRUE) {
      dataPoint++;
    } else {
      Serial.println("[Loop] SD queue full - data dropped");
    }

    lastSDLog = SESSION_TIME_MS;
  }

  // Small delay to prevent watchdog issues
  delay(10);
}

/************************* File Management ***************************/

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
  dataFile.println(bmu->BMUreadytoCharge);
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
  data["balancing"] = bmu->BalancingDischarge_Cells;
  data["ready_charge"] = bmu->BMUreadytoCharge;

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
        BMU_Package[i].BMUreadytoCharge = 1;
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
        BMU_Package[i].BMUreadytoCharge = 0;
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
