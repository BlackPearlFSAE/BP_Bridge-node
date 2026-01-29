/* BP Bridge Sensor Node - BAMO (BAMOCar CAN)
 *
 * ESP32-S3 CAN bus node for Formula Student vehicle telemetry.
 * - Core 0: WiFi/WebSocket communication (BPMobile)
 * - Core 1: SD logging
 *
 * Data: BAMOCar motor controller via CAN bus (voltage, current, power, temps)
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
#include <driver/twai.h>
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
#include <CAN32_util.h>
#include <WIFI32_util.h>

#include <bamo_helper.h>
#include <shared_config.h> // Shared_pin definition

/************************* Global Variables ***************************/

// WebSocket and Network config
const char* ssid = DEFAULT_SSID;
const char* password = DEFAULT_PASSWORD;
const char* serverHost = DEFAULT_SERVER_HOST;
const int serverPort = DEFAULT_SERVER_PORT;
const char* clientName = "BAMO_Node";
WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;
// Sampling Rates (Hz)
const float BAMO_POWER_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float BAMO_TEMP_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;

// Sensor Data
BAMOCar myBAMOCar;
// Peripherals
RTC_DS3231 rtc;

// Status Flags
bool I2C1_connect = false;
bool sdCardReady = false;
bool canBusReady = false;
bool RTCavailable = false;

// Timing Intervals (ms)
#define BAMOCarREQ_INTERVAL 200
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
const char* header_BAMO = "BAMOVolt(V),BAMOAmp(A),BAMOPower(W),MotorTemp(C),BAMOTemp(C)";
char csvHeaderBuffer[256] = "";
int appenderCount = 4;

/************************* Function Declarations ***************************/

// CSV Appenders
void append_DataPoint_toCSVFile(File& dataFile, void* data);
void append_Timestamp_toCSVFile(File& dataFile, void* data);
void append_SessionTime_toCSVFile(File& dataFile, void* data);
void append_BAMOdata_toCSVFile(File& dataFile, void* data);

// BPMobile Publishers
void publishBAMOpower(BAMOCar* bamocar);
void publishBAMOtemp(BAMOCar* bamocar);
void registerClient(const char* clientName);

// Debug
void showDeviceStatus();

/************************* Build Flags ***************************/

#define MOCK_FLAG 1
#define calibrate_RTC 0
#define DEBUG_MODE 0  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot

/************************* FreeRTOS ***************************/

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
TaskHandle_t BPMobileTaskHandle = NULL;
TaskHandle_t timeSyncTaskHandle = NULL;
TaskHandle_t canTaskHandle = NULL;
TaskHandle_t sdTaskHandle = NULL;
QueueHandle_t sdQueue = NULL;

// Struct Blueprint need for SDtask (deterministic log field)
struct SDLogEntry {
  int dataPoint;
  uint64_t unixTime;
  uint64_t sessionTime;
  BAMOCar bamo;
};

// Core 0: WiFi/WebSocket Task
void BPMobileTask(void* parameter) {
  unsigned long taskLastBAMOpower = 0;
  unsigned long taskLastBAMOtemp = 0;

  BAMOCar localBAMO;

  while (true) {
    unsigned long SESSION_TIME = millis();

    if (WiFi.status() == WL_CONNECTED)
      BPwebSocket->loop();
    

    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        localBAMO = myBAMOCar;
        xSemaphoreGive(dataMutex);
      }

      if (SESSION_TIME - taskLastBAMOpower >= (1000.0 / BAMO_POWER_SAMPLING_RATE)) {
        publishBAMOpower(&localBAMO);
        taskLastBAMOpower = SESSION_TIME;
      }
      if (SESSION_TIME - taskLastBAMOtemp >= (1000.0 / BAMO_TEMP_SAMPLING_RATE)) {
        publishBAMOtemp(&localBAMO);
        taskLastBAMOtemp = SESSION_TIME;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Core 1: SD Card Logger Task
// Consumes SDLogEntry from queue, writes to persistent CSV file in session dir.
// File rotation: after SD_MAX_ROWS rows, closes current file and opens next
// (e.g. /BAMO_session_000/File_0.csv -> File_1.csv -> ...).
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
        append_BAMOdata_toCSVFile
      };
      void* structArray[appenderCount] = {
        &entry.dataPoint, &entry.unixTime, &entry.sessionTime,
        &entry.bamo
      };

      // Append one CSV row + newline; 
      // flushes to SD at SD_FLUSH_INTERVAL , close SD file at SD_CLOSE_INTERVAL 
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

// Core 1: CAN Bus Task (BAMOCar polling)
void canTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long lastRequest = 0;
  uint8_t requestSeq = 0;
  twai_message_t txmsg, rxmsg;

  while (true) {
    unsigned long SESSION_TIME = millis();
    if (!canBusReady) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    // CAN RX: poll for incoming responses
    if (CAN32_receiveCAN(&rxmsg) == ESP_OK) {
      Serial.printf("[CAN RX] ID=0x%03X DLC=%d Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
        rxmsg.identifier, rxmsg.data_length_code,
        rxmsg.data[0], rxmsg.data[1], rxmsg.data[2], rxmsg.data[3],
        rxmsg.data[4], rxmsg.data[5], rxmsg.data[6], rxmsg.data[7]);
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        process_ResponseBamocarMsg(&rxmsg, &myBAMOCar);
        xSemaphoreGive(dataMutex);
      }
    }

    // CAN TX: send register requests every 200ms
    if (SESSION_TIME - lastRequest >= BAMOCarREQ_INTERVAL) {
      switch (requestSeq) {
        case 0: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_MOTOR_TEMP); break;
        case 1: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_CONTROLLER_TEMP); break;
        case 2: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_DC_VOLTAGE); break;
        case 3: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_DC_CURRENT); break;
      }
      CAN32_sendCAN(&txmsg);
      Serial.printf("[CAN TX] ID=0x%03X REG=0x%02X (seq=%d)\n",
        txmsg.identifier, txmsg.data[0], requestSeq);
      requestSeq = (requestSeq + 1) % 4;
      lastRequest = SESSION_TIME;
    }

    // Power calculation
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (myBAMOCar.canVoltageValid && myBAMOCar.canCurrentValid) {
        myBAMOCar.power = myBAMOCar.canVoltage * myBAMOCar.canCurrent;
      }
      xSemaphoreGive(dataMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));  // 200Hz CAN poll
  }
}

/************************* Setup ***************************/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(UART0_BAUD);

  // I2C Init
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  Wire1.setTimeout(2);

  // CAN Bus Init
  canBusReady = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN,
    TWAI_TIMING_CONFIG_250KBITS(), TWAI_FILTER_CONFIG_ACCEPT_ALL());

  // RTC Init
  RTCavailable = RTCinit(rtc, &Wire1);

  // WiFi & WebSocket
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
  strcat(csvHeaderBuffer, header_Timestamp);
  strcat(csvHeaderBuffer, ",");
  strcat(csvHeaderBuffer, header_BAMO);
  if (sdCardReady) {
    SD32_createSessionDir(sessionNumber, sessionDirPath, "BAMO");
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
  Serial.println("BP Bridge Sensor Node - BAMO (CAN) - Ready");
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
    xTaskCreatePinnedToCore(canTask, "CANTask", 4096, NULL, 5, &canTaskHandle, 1);
    Serial.println("[RTOS] CAN task on Core 1 (pri 5)");
  #else
    Serial.println("[RTOS] CAN task SKIPPED (MOCK_FLAG=1)");
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
    #if DEBUG_MODE == 1
      Serial.printf("[DEBUG] BAMO: V=%.1fV I=%.1fA P=%.1fW\n", myBAMOCar.canVoltage, myBAMOCar.canCurrent, myBAMOCar.power);
    #elif DEBUG_MODE == 2
      teleplotBAMOCar(&myBAMOCar);
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

  #if MOCK_FLAG == 1

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      mockBAMOCarData(&myBAMOCar);
      xSemaphoreGive(dataMutex);
    }
  #endif

  // SD card close file once removed
  if (sdCardReady && !SD32_checkSDconnect()) {
    SD32_closePersistentFile();
    sdCardReady = false;
    Serial.println("[SD] Card removed");
  }

  // Manaully Queue the  SD Log Entry
  if (sdCardReady && sdQueue != NULL && (SESSION_TIME_MS - lastSDLog >= SD_APPEND_INTERVAL)) {
    SDLogEntry entry;
    entry.dataPoint = dataPoint;
    entry.unixTime = CURRENT_UNIX_TIME_MS;
    entry.sessionTime = SESSION_TIME_MS;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      entry.bamo = myBAMOCar;
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


/************************* BPMobile Publishers ***************************/

void publishBAMOpower(BAMOCar* bamocar) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  if (bamocar->canVoltageValid) {
    JsonDocument doc;
    doc["type"] = "data";
    doc["topic"] = "power/can_voltage";
    doc["data"]["value"] = bamocar->canVoltage;
    doc["data"]["sensor_id"] = "BAMOCAR_CAN";
    doc["timestamp"] = timestamp;
    String msg;
    serializeJson(doc, msg);
    BPwebSocket->sendTXT(msg);
  }

  if (bamocar->canCurrentValid) {
    JsonDocument doc;
    doc["type"] = "data";
    doc["topic"] = "power/can_current";
    doc["data"]["value"] = bamocar->canCurrent;
    doc["data"]["sensor_id"] = "BAMOCAR_CAN";
    doc["timestamp"] = timestamp;
    String msg;
    serializeJson(doc, msg);
    BPwebSocket->sendTXT(msg);
  }

  JsonDocument doc;
  doc["type"] = "data";
  doc["topic"] = "power/power";
  doc["data"]["value"] = bamocar->power;
  doc["data"]["sensor_id"] = "CALCULATED";
  doc["timestamp"] = timestamp;
  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishBAMOtemp(BAMOCar* bamocar) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  if (bamocar->motorTempValid) {
    JsonDocument doc;
    doc["type"] = "data";
    doc["topic"] = "motor/temperature";
    doc["data"]["value"] = bamocar->motorTemp2;
    doc["data"]["sensor_id"] = "BAMOCAR_MOTOR";
    doc["timestamp"] = timestamp;
    String msg;
    serializeJson(doc, msg);
    BPwebSocket->sendTXT(msg);
  }

  if (bamocar->controllerTempValid) {
    JsonDocument doc;
    doc["type"] = "data";
    doc["topic"] = "motor/controller_temperature";
    doc["data"]["value"] = bamocar->controllerTemp;
    doc["data"]["sensor_id"] = "BAMOCAR_CTRL";
    doc["timestamp"] = timestamp;
    String msg;
    serializeJson(doc, msg);
    BPwebSocket->sendTXT(msg);
  }
}

void registerClient(const char* clientName) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = clientName;

  JsonArray topics = doc["topics"].to<JsonArray>();
  topics.add("power/can_voltage");
  topics.add("power/can_current");
  topics.add("power/power");
  topics.add("motor/temperature");
  topics.add("motor/controller_temperature");

  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();

  JsonObject canVoltMeta = metadata["power/can_voltage"].to<JsonObject>();
  canVoltMeta["description"] = "DC Link Voltage (Bamocar CAN)";
  canVoltMeta["unit"] = "V";
  canVoltMeta["sampling_rate"] = BAMO_POWER_SAMPLING_RATE;

  JsonObject canCurrMeta = metadata["power/can_current"].to<JsonObject>();
  canCurrMeta["description"] = "Motor DC Current (Bamocar CAN)";
  canCurrMeta["unit"] = "A";
  canCurrMeta["sampling_rate"] = BAMO_POWER_SAMPLING_RATE;

  JsonObject powMeta = metadata["power/power"].to<JsonObject>();
  powMeta["description"] = "Power consumption (calculated)";
  powMeta["unit"] = "W";
  powMeta["sampling_rate"] = BAMO_POWER_SAMPLING_RATE;

  JsonObject motorTempMeta = metadata["motor/temperature"].to<JsonObject>();
  motorTempMeta["description"] = "Motor temperature (Bamocar)";
  motorTempMeta["unit"] = "C";
  motorTempMeta["sampling_rate"] = BAMO_TEMP_SAMPLING_RATE;

  JsonObject ctrlTempMeta = metadata["motor/controller_temperature"].to<JsonObject>();
  ctrlTempMeta["description"] = "Motor controller/IGBT temperature (Bamocar)";
  ctrlTempMeta["unit"] = "C";
  ctrlTempMeta["sampling_rate"] = BAMO_TEMP_SAMPLING_RATE;

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

void append_BAMOdata_toCSVFile(File& dataFile, void* data) {
  BAMOCar* b = static_cast<BAMOCar*>(data);
  dataFile.print(b->canVoltageValid ? b->canVoltage : 0.0, 2);
  dataFile.print(",");
  dataFile.print(b->canCurrentValid ? b->canCurrent : 0.0, 2);
  dataFile.print(",");
  dataFile.print(b->power, 2);
  dataFile.print(",");
  dataFile.print(b->motorTempValid ? b->motorTemp2 : 0.0, 1);
  dataFile.print(",");
  dataFile.print(b->controllerTempValid ? b->controllerTemp : 0.0, 1);
}

/************************* Debug Functions ***************************/

void showDeviceStatus() {
  Serial.println("╔═════════════════════════════════════════════╗");
  Serial.println("║        BAMO NODE (CAN) - STATUS             ║");
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.printf("║ CAN Bus:      %s\n", canBusReady ? "OK" : "FAIL");
  Serial.printf("║ I2C1:         %s\n", I2C1_connect ? "OK" : "FAIL");
  Serial.printf("║ SD Card:      %s\n", sdCardReady ? "OK" : "FAIL");
  Serial.printf("║ WiFi:         %s (RSSI: %d)\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL", WiFi.RSSI());
  Serial.printf("║ RTC:          %s\n", RTCavailable ? "OK" : "FAIL");
  Serial.printf("║ WebSocket:    %s\n", BPsocketstatus->isConnected ? "OK" : "FAIL");
  Serial.printf("║ Time Sync:    %s\n", syncTime_isSynced() ? "OK" : "FAIL");
  Serial.println("╚═════════════════════════════════════════════╝");
}