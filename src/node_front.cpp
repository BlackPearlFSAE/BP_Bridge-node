/* BP Bridge Sensor Node - Front (Electrical + Mechanical)
 *
 * ESP32-S3 sensor node for Formula Student vehicle telemetry.
 * - Core 0: WiFi/WebSocket communication (BPMobile)
 * - Core 1: SD logging
 *
 * Sensors: Stroke (Heave, Roll), Electrical (I_SENSE, TMP, APPS, BPPS, Faults)
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
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <BP_mobile_util.h>
#include <SD32_util.h>
#include <DS3231_util.h>
#include <syncTime_util.h>
#include <WIFI32_util.h>

#include <base_sensors.h>
#include <shared_config.h>

/************************* Pin Definitions ***************************/
// Mechanical Sensors
#define STR_Roll 5
#define STR_Heave 6

// Electrical Pins
#define I_SENSE_PIN 4
#define TMP_PIN 8
#define APPS_PIN 15
#define BPPS_PIN 7
#define AMS_OK_PIN 37
#define IMD_OK_PIN 38
#define HV_ON_PIN 35
#define BSPD_OK_PIN 36

int ElectPinArray[8] = {
  I_SENSE_PIN, TMP_PIN, APPS_PIN, BPPS_PIN,
  AMS_OK_PIN, IMD_OK_PIN, HV_ON_PIN, BSPD_OK_PIN
};

/************************* Global Variables ***************************/

// WebSocket and Network config
const char* ssid = DEFAULT_SSID;
const char* password = DEFAULT_PASSWORD;
const char* serverHost = DEFAULT_SERVER_HOST;
const int serverPort = DEFAULT_SERVER_PORT;
const char* clientName = "Front_Node";
WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

// Sampling Rates (Hz)
const float MECH_SENSORS_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float ELECT_SENSORS_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float ELECT_FAULT_STAT_SAMPLING_RATE = (DEFAULT_PUBLISH_RATE/5);

// Sensor Data
Mechanical myMechData;
Electrical myElectData;

// Peripherals
RTC_DS3231 rtc;

// Status Flags
bool I2C1_connect = false;
bool sdCardReady = false;
bool RTCavailable = false;

// Timing Intervals (ms)
const unsigned long LOCAL_SYNC_INTERVAL = DEFAULT_LOCAL_SYNC_INTERVAL;
const unsigned long REMOTE_SYNC_INTERVAL = DEFAULT_REMOTE_SYNC_INTERVAL;

const unsigned long SD_APPEND_INTERVAL = DEFAULT_SD_LOG_INTERVAL;
const unsigned long SD_FLUSH_INTERVAL = DEFAULT_SD_FLUSH_INTERVAL;
const unsigned long SD_CLOSE_INTERVAL = DEFAULT_SD_CLOSE_INTERVAL;
const int SD_MAX_ROWS = DEFAULT_SD_ROW_LIMIT;

unsigned long lastSDLog = 0;
unsigned long lastTeleplotDebug = 0;
const unsigned long TELEPLOT_DEBUG_INTERVAL = 200;
int dataPoint = 1;
int sessionNumber = 0;
uint64_t RTC_UNIX_TIME = 0;

// SD Card
char sessionDirPath[48] = {0};
char csvFilename[48] = {0};
int fileIndex = 0;
const char* header_Timestamp = "DataPoint,UnixTime,SessionTime";
const char* header_Mechanical = "Stroke1_mm,Stroke2_mm";
const char* header_Electrical = "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK";
char csvHeaderBuffer[256] = "";
int appenderCount = 5;

/************************* Function Declarations ***************************/

// CSV Appenders
void append_DataPoint_toCSVFile(File& dataFile, void* data);
void append_Timestamp_toCSVFile(File& dataFile, void* data);
void append_SessionTime_toCSVFile(File& dataFile, void* data);
void append_MechData_toCSVFile(File& dataFile, void* data);
void append_ElectData_toCSVFile(File& dataFile, void* data);

// BPMobile Publishers
void publishMechData(Mechanical* MechSensors);
void publishElectData(Electrical* ElectSensors);
void publishElectFaultState(Electrical* ElectSensors);
void registerClient(const char* clientName);

// Debug
void showDeviceStatus();

/************************* Build Flags ***************************/

#define MOCK_FLAG 1
#define calibrate_RTC 0
#define DEBUG_MODE 0  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot
#define TIME_SRC 0 // 0 = RTC , 1 = WiFI NTP Pool

/************************* FreeRTOS ***************************/

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
TaskHandle_t BPMobileTaskHandle = NULL;
TaskHandle_t timeSyncTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t sdTaskHandle = NULL;
QueueHandle_t sdQueue = NULL;

struct SDLogEntry {
  int dataPoint;
  uint64_t unixTime;
  uint64_t sessionTime;
  Mechanical mech;
  Electrical elect;
};

// Core 0: WiFi/WebSocket Task
void BPMobileTask(void* parameter) {
  unsigned long taskLastMech = 0;
  unsigned long taskLastElect = 0;
  unsigned long taskLastElectFault = 0;

  Mechanical localMech;
  Electrical localElect;

  while (true) {
    unsigned long SESSION_TIME = millis();

    if (WiFi.status() == WL_CONNECTED) {
      BPwebSocket->loop();
    }

    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        localMech = myMechData;
        localElect = myElectData;
        xSemaphoreGive(dataMutex);
      }

      if (SESSION_TIME - taskLastMech >= (1000.0 / MECH_SENSORS_SAMPLING_RATE)) {
        publishMechData(&localMech);
        taskLastMech = SESSION_TIME;
      }
      if (SESSION_TIME - taskLastElect >= (1000.0 / ELECT_SENSORS_SAMPLING_RATE)) {
        publishElectData(&localElect);
        taskLastElect = SESSION_TIME;
      }
      if (SESSION_TIME - taskLastElectFault >= (1000.0 / ELECT_FAULT_STAT_SAMPLING_RATE)) {
        publishElectFaultState(&localElect);
        taskLastElectFault = SESSION_TIME;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Core 1: SD Card Logger Task
// Consumes SDLogEntry from queue, writes to persistent CSV file in session dir.
// File rotation: after SD_MAX_ROWS rows, closes current file and opens next
// (e.g. /Front_session_000/File_0.csv -> File_1.csv -> ...).
// Flush is handled inside SD32_appendBulkDataPersistent at SD_FLUSH_INTERVAL.
void sdTask(void* parameter) {
  SDLogEntry entry;
  int localDataPoint = 0;

  while (true) {
    // Block until loop() enqueues a log entry
    if (xQueueReceive(sdQueue, &entry, portMAX_DELAY) == pdTRUE) {
      if (!sdCardReady || !SD32_isPersistentFileOpen()) continue;

      // Row-based rotation: close current file, open next in same session dir
      if (localDataPoint > 0 && localDataPoint % SD_MAX_ROWS == 0) {
        SD32_closePersistentFile();
        fileIndex++;
        SD32_generateFilenameInDir(csvFilename, sessionDirPath, "File", fileIndex);
        SD32_createCSVFile(csvFilename, csvHeaderBuffer);
        SD32_openPersistentFile(csvFilename);
        Serial.printf("[SD] Row limit reached, rotated to: %s\n", csvFilename);
      }

      // Build appender array — each function writes its fields to the open file
      AppenderFunc appenders[appenderCount] = {
        append_DataPoint_toCSVFile,
        append_Timestamp_toCSVFile,
        append_SessionTime_toCSVFile,
        append_MechData_toCSVFile,
        append_ElectData_toCSVFile
      };
      void* structArray[appenderCount] = {
        &entry.dataPoint, &entry.unixTime, &entry.sessionTime,
        &entry.mech, &entry.elect
      };

      // Append one CSV row + newline; flushes to SD at SD_FLUSH_INTERVAL
      SD32_appendBulkDataPersistent(appenders, structArray, appenderCount
                                    , SD_FLUSH_INTERVAL,SD_CLOSE_INTERVAL);
      localDataPoint++;
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

// Core 1: Sensor Reading Task
void sensorTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Mechanical localMech;
  Electrical localElect;

  while (true) {
    // Read sensors into local structs (no mutex needed for hardware reads)
    #if MOCK_FLAG == 0
      StrokesensorUpdate(&localMech, STR_Heave, STR_Roll);
      ElectSensorsUpdate(&localElect, ElectPinArray);
    #else
      mockMechanicalData(&localMech);
      mockElectricalData(&localElect);
    #endif

    // Brief lock to copy results into shared structs
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      myMechData = localMech;
      myElectData = localElect;
      xSemaphoreGive(dataMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));  // 100Hz
  }
}

/************************* Setup ***************************/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(UART0_BAUD);

  // I2C Init
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  Wire1.setTimeout(2);

  // RTC Init
  RTCavailable = RTCinit(rtc, &Wire1);
  // Sync ESP32 internal clock from DS3231 so FAT file timestamps are correct
  if (RTCavailable) {
    struct timeval tv = { .tv_sec = (time_t)rtc.now().unixtime(), .tv_usec = 0 };
    settimeofday(&tv, NULL);
    Serial.println("[RTC] ESP32 system clock set from DS3231");
  }

  // Sensor Init
  StrokesensorInit(STR_Heave, STR_Roll);
  ElectSensorsInit(ElectPinArray);

  // WiFi & WebSocket
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
  strcat(csvHeaderBuffer, header_Timestamp);
  strcat(csvHeaderBuffer, ",");
  strcat(csvHeaderBuffer, header_Mechanical);
  strcat(csvHeaderBuffer, ",");
  strcat(csvHeaderBuffer, header_Electrical);
  if (sdCardReady) {
    SD32_createSessionDir(sessionNumber, sessionDirPath, "Front");
    SD32_generateFilenameInDir(csvFilename, sessionDirPath, "File", fileIndex);
    SD32_createCSVFile(csvFilename, csvHeaderBuffer);
    SD32_openPersistentFile(csvFilename);
  }

  // Time Sync
  #if calibrate_RTC == 1
    RTCcalibrate(rtc,(ntpready) ? (WiFi32_getNTPTime()/1000ULL): 1000000000000ULL ,RTCavailable);
  #endif
  syncTime_setSyncPoint(RTC_UNIX_TIME, (RTCavailable) ? RTC_getUnix(rtc, RTCavailable) : 1000000000000ULL);

  Serial.println("==================================================");
  Serial.println("BP Bridge Sensor Node - Front (Elect+Mech) - Ready");
  Serial.println("==================================================");
  Serial.printf("Client: %s\n\n", clientName);

  // FreeRTOS Setup
  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  sdQueue = xQueueCreate(30, sizeof(SDLogEntry));

  // Core 0 tasks
  xTaskCreatePinnedToCore(BPMobileTask, "BPMobileTask", 8192, NULL, 1, &BPMobileTaskHandle, 0);
  Serial.println("[RTOS] BPMobile task on Core 0 (pri 1)");

  xTaskCreatePinnedToCore(timeSyncTask, "TimeSyncTask", 4096, NULL, 3, &timeSyncTaskHandle, 0);
  Serial.println("[RTOS] TimeSync task on Core 0 (pri 3)");

  // Core 1 tasks
  #if MOCK_FLAG == 0
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 5, &sensorTaskHandle, 1);
    Serial.println("[RTOS] Sensor task on Core 1 (pri 5)");
  #else
    Serial.println("[RTOS] Sensor task SKIPPED (MOCK_FLAG=1)");
  #endif

  xTaskCreatePinnedToCore(sdTask, "SDTask", 4096, NULL, 2, &sdTaskHandle, 1);
  Serial.println("[RTOS] SD Logger task on Core 1 (pri 2)");
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
    Mechanical debugMech;
    Electrical debugElect;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      debugMech = myMechData;
      debugElect = myElectData;
      xSemaphoreGive(dataMutex);
    }
    #if DEBUG_MODE == 1
      Serial.printf("[DEBUG] Mech: Heave=%.1f Roll=%.1f\n", debugMech.STR_Heave_mm, debugMech.STR_Roll_mm);
      Serial.printf("[DEBUG] Elect: I=%.2fA APPS=%.1f BPPS=%.1f\n", debugElect.I_SENSE, debugElect.APPS, debugElect.BPPS);
    #elif DEBUG_MODE == 2
      teleplotMechanical(&debugMech);
      teleplotElectrical(&debugElect);
    #endif
    lastTeleplotDebug = SESSION_TIME_MS;
  }
  #endif

  // Debug Console
  if (Serial.available() && Serial.peek() == '`') {
    Serial.read();
    while (1) {
      showDeviceStatus();
      if (Serial.available() && Serial.read() == '~') break;
      delay(200);
    }
  }

  // Mock data when sensor task is not running
  #if MOCK_FLAG == 1
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      mockMechanicalData(&myMechData);
      mockElectricalData(&myElectData);
      xSemaphoreGive(dataMutex);
    }
  #endif

  // SD card close file once removed
  if (sdCardReady && !SD32_checkSDconnect()) {
    SD32_closePersistentFile();
    sdCardReady = false;
    Serial.println("[SD] Card removed");
  }

  // Queue SD Log Entry
  if (sdCardReady && sdQueue != NULL && (SESSION_TIME_MS - lastSDLog >= SD_APPEND_INTERVAL)) {
    SDLogEntry entry;
    entry.dataPoint = dataPoint;
    entry.unixTime = CURRENT_UNIX_TIME_MS;
    entry.sessionTime = SESSION_TIME_MS;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      entry.mech = myMechData;
      entry.elect = myElectData;
      xSemaphoreGive(dataMutex);
    }

    if (xQueueSend(sdQueue, &entry, 0) == pdTRUE) {
      dataPoint++;
    } else {
      Serial.println("[SD] Queue full - data dropped");
    }
    lastSDLog = SESSION_TIME_MS;
  }

  vTaskDelay(pdMS_TO_TICKS(20));
}

/************************* Debug Functions ***************************/

void showDeviceStatus() {
  Serial.println("╔═════════════════════════════════════════════╗");
  Serial.println("║     FRONT NODE (Elect+Mech) - STATUS        ║");
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.printf("║ I2C1:         %s\n", I2C1_connect ? "OK" : "FAIL");
  Serial.printf("║ SD Card:      %s\n", sdCardReady ? "OK" : "FAIL");
  Serial.printf("║ WiFi:         %s (RSSI: %d)\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL", WiFi.RSSI());
  Serial.printf("║ RTC:          %s\n", RTCavailable ? "OK" : "FAIL");
  Serial.printf("║ WebSocket:    %s\n", BPsocketstatus->isConnected ? "OK" : "FAIL");
  Serial.printf("║ Time Sync:    %s\n", syncTime_isSynced() ? "OK" : "FAIL");
  Serial.println("╚═════════════════════════════════════════════╝");
}

/************************* BPMobile Publishers ***************************/

void publishMechData(Mechanical* MechSensors) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  JsonDocument sH;
  sH["type"] = "data";
  sH["topic"] = "sensors/stroke_Heave_distanceMM";
  sH["data"]["value"] = MechSensors->STR_Heave_mm;
  sH["data"]["sensor_id"] = "STR_Heave";
  sH["timestamp"] = timestamp;
  String msg2;
  serializeJson(sH, msg2);
  BPwebSocket->sendTXT(msg2);

  JsonDocument sR;
  sR["type"] = "data";
  sR["topic"] = "sensors/stroke_Roll_distanceMM";
  sR["data"]["value"] = MechSensors->STR_Roll_mm;
  sR["data"]["sensor_id"] = "STR_Roll";
  sR["timestamp"] = timestamp;
  String msg1;
  serializeJson(sR, msg1);
  BPwebSocket->sendTXT(msg1);
}

void publishElectData(Electrical* ElectSensors) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  JsonDocument iSenseDoc;
  iSenseDoc["type"] = "data";
  iSenseDoc["topic"] = "electrical/current_sense";
  iSenseDoc["data"]["value"] = ElectSensors->I_SENSE;
  iSenseDoc["data"]["sensor_id"] = "I_SENSE";
  iSenseDoc["timestamp"] = timestamp;
  String iSenseMsg;
  serializeJson(iSenseDoc, iSenseMsg);
  BPwebSocket->sendTXT(iSenseMsg);

  JsonDocument tmpDoc;
  tmpDoc["type"] = "data";
  tmpDoc["topic"] = "electrical/temperature";
  tmpDoc["data"]["value"] = ElectSensors->TMP;
  tmpDoc["data"]["sensor_id"] = "TMP";
  tmpDoc["timestamp"] = timestamp;
  String tmpMsg;
  serializeJson(tmpDoc, tmpMsg);
  BPwebSocket->sendTXT(tmpMsg);

  JsonDocument appsDoc;
  appsDoc["type"] = "data";
  appsDoc["topic"] = "electrical/apps";
  appsDoc["data"]["value"] = ElectSensors->APPS;
  appsDoc["data"]["sensor_id"] = "APPS";
  appsDoc["timestamp"] = timestamp;
  String appsMsg;
  serializeJson(appsDoc, appsMsg);
  BPwebSocket->sendTXT(appsMsg);

  JsonDocument bppsDoc;
  bppsDoc["type"] = "data";
  bppsDoc["topic"] = "electrical/bpps";
  bppsDoc["data"]["value"] = ElectSensors->BPPS;
  bppsDoc["data"]["sensor_id"] = "BPPS";
  bppsDoc["timestamp"] = timestamp;
  String bppsMsg;
  serializeJson(bppsDoc, bppsMsg);
  BPwebSocket->sendTXT(bppsMsg);
}

void publishElectFaultState(Electrical* ElectSensors) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  JsonDocument amsDoc;
  amsDoc["type"] = "data";
  amsDoc["topic"] = "electrical/ams_ok";
  amsDoc["data"]["value"] = ElectSensors->AMS_OK;
  amsDoc["data"]["sensor_id"] = "AMS";
  amsDoc["timestamp"] = timestamp;
  String amsMsg;
  serializeJson(amsDoc, amsMsg);
  BPwebSocket->sendTXT(amsMsg);

  JsonDocument imdDoc;
  imdDoc["type"] = "data";
  imdDoc["topic"] = "electrical/imd_ok";
  imdDoc["data"]["value"] = ElectSensors->IMD_OK;
  imdDoc["data"]["sensor_id"] = "IMD";
  imdDoc["timestamp"] = timestamp;
  String imdMsg;
  serializeJson(imdDoc, imdMsg);
  BPwebSocket->sendTXT(imdMsg);

  JsonDocument hvDoc;
  hvDoc["type"] = "data";
  hvDoc["topic"] = "electrical/hv_on";
  hvDoc["data"]["value"] = ElectSensors->HV_ON;
  hvDoc["data"]["sensor_id"] = "HV";
  hvDoc["timestamp"] = timestamp;
  String hvMsg;
  serializeJson(hvDoc, hvMsg);
  BPwebSocket->sendTXT(hvMsg);

  JsonDocument bspdDoc;
  bspdDoc["type"] = "data";
  bspdDoc["topic"] = "electrical/bspd_ok";
  bspdDoc["data"]["value"] = ElectSensors->BSPD_OK;
  bspdDoc["data"]["sensor_id"] = "BSPD";
  bspdDoc["timestamp"] = timestamp;
  String bspdMsg;
  serializeJson(bspdDoc, bspdMsg);
  BPwebSocket->sendTXT(bspdMsg);
}

void registerClient(const char* clientName) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = clientName;

  JsonArray topics = doc["topics"].to<JsonArray>();
  topics.add("sensors/stroke_Heave_distanceMM");
  topics.add("sensors/stroke_Roll_distanceMM");
  topics.add("electrical/current_sense");
  topics.add("electrical/temperature");
  topics.add("electrical/apps");
  topics.add("electrical/bpps");
  topics.add("electrical/ams_ok");
  topics.add("electrical/imd_ok");
  topics.add("electrical/hv_on");
  topics.add("electrical/bspd_ok");

  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();

  JsonObject sHMeta = metadata["sensors/stroke_Heave_distanceMM"].to<JsonObject>();
  sHMeta["description"] = "Stroke Heave";
  sHMeta["unit"] = "mm";
  sHMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

  JsonObject sRMeta = metadata["sensors/stroke_Roll_distanceMM"].to<JsonObject>();
  sRMeta["description"] = "Stroke Roll";
  sRMeta["unit"] = "mm";
  sRMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

  JsonObject iSenseMeta = metadata["electrical/current_sense"].to<JsonObject>();
  iSenseMeta["description"] = "Current sense";
  iSenseMeta["unit"] = "A";
  iSenseMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  JsonObject tmpMeta = metadata["electrical/temperature"].to<JsonObject>();
  tmpMeta["description"] = "Electrical system temperature";
  tmpMeta["unit"] = "C";
  tmpMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  JsonObject appsMeta = metadata["electrical/apps"].to<JsonObject>();
  appsMeta["description"] = "Accelerator Pedal Position Sensor";
  appsMeta["unit"] = "%";
  appsMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  JsonObject bppsMeta = metadata["electrical/bpps"].to<JsonObject>();
  bppsMeta["description"] = "Brake Pedal Position Sensor";
  bppsMeta["unit"] = "%";
  bppsMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  JsonObject amsMeta = metadata["electrical/ams_ok"].to<JsonObject>();
  amsMeta["description"] = "Accumulator Management System status";
  amsMeta["unit"] = "bool";
  amsMeta["sampling_rate"] = ELECT_FAULT_STAT_SAMPLING_RATE;

  JsonObject imdMeta = metadata["electrical/imd_ok"].to<JsonObject>();
  imdMeta["description"] = "Insulation Monitoring Device status";
  imdMeta["unit"] = "bool";
  imdMeta["sampling_rate"] = ELECT_FAULT_STAT_SAMPLING_RATE;

  JsonObject hvMeta = metadata["electrical/hv_on"].to<JsonObject>();
  hvMeta["description"] = "High Voltage system status";
  hvMeta["unit"] = "bool";
  hvMeta["sampling_rate"] = ELECT_FAULT_STAT_SAMPLING_RATE;

  JsonObject bspdMeta = metadata["electrical/bspd_ok"].to<JsonObject>();
  bspdMeta["description"] = "Brake System Plausibility Device status";
  bspdMeta["unit"] = "bool";
  bspdMeta["sampling_rate"] = ELECT_FAULT_STAT_SAMPLING_RATE;

  String registration;
  serializeJson(doc, registration);

  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println("[WS] Sending registration...");
    xSemaphoreGive(serialMutex);
  }
  BPwebSocket->sendTXT(registration);
}

/************************* CSV Appenders ***************************/

void append_DataPoint_toCSVFile(File& dataFile, void* data) {
  int* dp = (int*)data;
  dataFile.print(*dp);
  dataFile.print(",");
}

void append_Timestamp_toCSVFile(File& dataFile, void* data) {
  uint64_t* t = (uint64_t*)data;
  dataFile.print(*t);
  dataFile.print(",");
}

void append_SessionTime_toCSVFile(File& dataFile, void* data) {
  uint64_t* t = (uint64_t*)data;
  dataFile.print(*t);
  dataFile.print(",");
}

void append_MechData_toCSVFile(File& dataFile, void* data) {
  Mechanical* m = static_cast<Mechanical*>(data);
  dataFile.print(m->STR_Heave_mm, 2);
  dataFile.print(",");
  dataFile.print(m->STR_Roll_mm, 2);
  dataFile.print(",");
}

void append_ElectData_toCSVFile(File& dataFile, void* data) {
  Electrical* e = static_cast<Electrical*>(data);
  dataFile.print(e->I_SENSE, 2);
  dataFile.print(",");
  dataFile.print(e->TMP, 2);
  dataFile.print(",");
  dataFile.print(e->APPS, 2);
  dataFile.print(",");
  dataFile.print(e->BPPS, 2);
  dataFile.print(",");
  dataFile.print(e->AMS_OK ? 1 : 0);
  dataFile.print(",");
  dataFile.print(e->IMD_OK ? 1 : 0);
  dataFile.print(",");
  dataFile.print(e->HV_ON ? 1 : 0);
  dataFile.print(",");
  dataFile.print(e->BSPD_OK ? 1 : 0);
}
