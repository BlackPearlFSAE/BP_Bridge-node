/* BP Bridge Sensor Node - Front
 *
 * ESP32-S3 dual-core sensor aggregator for Formula Student vehicle telemetry.
 * - Core 0: WiFi/WebSocket communication (BPMobile)
 * - Core 1: Sensor polling, CAN bus (BAMO), SD logging
 *
 * Sensors: Wheel RPM, Stroke, Electrical, BAMOCar CAN
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
#include <base_sensors.h>

/************************* Pin Definitions ***************************/

// LEDs
#define WIFI_LED 3
#define WS_LED 4

// SD SPI
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_SCK_PIN 12

// CAN Bus
#define CAN_TX_PIN 48
#define CAN_RX_PIN 47

// UART
#define UART0_BAUD 115200

// I2C Buses
#define I2C1_SDA 18
#define I2C1_SCL 17

// Mechanical Sensors
#define ENCODER_PINL 3
#define ENCODER_PINR 9
#define ENCODER_N 50
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
const float BAMO_POWER_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float BAMO_TEMP_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float MECH_SENSORS_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float ELECT_SENSORS_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float ELECT_FAULT_STAT_SAMPLING_RATE = (DEFAULT_PUBLISH_RATE/5);

// Sensor Data
Mechanical myMechData;
Electrical myElectData;
BAMOCar myBAMOCar;

// Peripherals
RTC_DS3231 rtc;
hw_timer_t* My_timer = nullptr;

// Status Flags
bool I2C1_connect = false;
bool sdCardReady = false;
bool canBusReady = false;
bool RTCavailable = false;

// Timing Intervals (ms)
#define BAMOCarREQ_INTERVAL 200
const unsigned long SD_APPEND_INTERVAL = DEFAULT_SD_LOG_INTERVAL;
const unsigned long SD_FLUSH_INTERVAL = 1000;
const size_t SD_MAX_FILE_SIZE = 2 * 1024 * 1024;
const unsigned long LOCAL_SYNC_INTERVAL = 1000;
const unsigned long REMOTE_SYNC_INTERVAL = 60000;
const unsigned long RPM_CalcInterval = 100;

unsigned long lastSDLog = 0;
unsigned long lastBAMOrequest = 0;
unsigned long lastTimeSourceSync = 0;
unsigned long lastExternalSync = 0;
unsigned long lastTeleplotDebug = 0;
const unsigned long TELEPLOT_DEBUG_INTERVAL = 200;
int dataPoint = 1;
int sessionNumber = 0;
uint8_t CANRequest_sequence = 0;
uint64_t RTC_UNIX_TIME = 0;

// SD Card
char csvFilename[32] = {0};
const char* header_Timestamp = "DataPoint,UnixTime,SessionTime";
const char* header_Mechanical = "Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm";
const char* header_Electrical = "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK";
const char* header_BAMO = "BAMOVolt(V),BAMOAmp(A),BAMOPower(W),MotorTemp(C),BAMOTemp(C)";
char csvHeaderBuffer[256] = "";
int appenderCount = 6;

/************************* Function Declarations ***************************/

// CSV Appenders
void append_DataPoint_toCSVFile(File& dataFile, void* data);
void append_Timestamp_toCSVFile(File& dataFile, void* data);
void append_SessionTime_toCSVFile(File& dataFile, void* data);
void append_MechData_toCSVFile(File& dataFile, void* data);
void append_ElectData_toCSVFile(File& dataFile, void* data);
void append_BAMOdata_toCSVFile(File& dataFile, void* data);

// BPMobile Publishers
void publishBAMOpower(BAMOCar* bamocar);
void publishBAMOtemp(BAMOCar* bamocar);
void publishMechData(Mechanical* MechSensors);
void publishElectData(Electrical* ElectSensors);
void publishElectFaultState(Electrical* ElectSensors);
void registerClient(const char* clientName);

// Debug
void showDeviceStatus();

/************************* FreeRTOS ***************************/

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
TaskHandle_t BPMobileTaskHandle = NULL;
TaskHandle_t sdTaskHandle = NULL;
QueueHandle_t sdQueue = NULL;

struct SDLogEntry {
  int dataPoint;
  uint64_t unixTime;
  uint64_t sessionTime;
  Mechanical mech;
  Electrical elect;
  BAMOCar bamo;
};

// Core 0: WiFi/WebSocket Task
void BPMobileTask(void* parameter) {
  unsigned long taskLastBAMOpower = 0;
  unsigned long taskLastBAMOtemp = 0;
  unsigned long taskLastMech = 0;
  unsigned long taskLastElect = 0;
  unsigned long taskLastElectFault = 0;

  Mechanical localMech;
  Electrical localElect;
  BAMOCar localBAMO;

  while (true) {
    unsigned long now = millis();

    if (WiFi.status() == WL_CONNECTED) {
      BPwebSocket->loop();
    }

    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        localMech = myMechData;
        localElect = myElectData;
        localBAMO = myBAMOCar;
        xSemaphoreGive(dataMutex);
      }

      if (now - taskLastBAMOpower >= (1000.0 / BAMO_POWER_SAMPLING_RATE)) {
        publishBAMOpower(&localBAMO);
        taskLastBAMOpower = now;
      }
      if (now - taskLastBAMOtemp >= (1000.0 / BAMO_TEMP_SAMPLING_RATE)) {
        publishBAMOtemp(&localBAMO);
        taskLastBAMOtemp = now;
      }
      if (now - taskLastMech >= (1000.0 / MECH_SENSORS_SAMPLING_RATE)) {
        publishMechData(&localMech);
        taskLastMech = now;
      }
      if (now - taskLastElect >= (1000.0 / ELECT_SENSORS_SAMPLING_RATE)) {
        publishElectData(&localElect);
        taskLastElect = now;
      }
      if (now - taskLastElectFault >= (1000.0 / ELECT_FAULT_STAT_SAMPLING_RATE)) {
        publishElectFaultState(&localElect);
        taskLastElectFault = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Core 1: SD Card Logger Task
void sdTask(void* parameter) {
  SDLogEntry entry;
  int localDataPoint = 0;

  while (true) {
    if (xQueueReceive(sdQueue, &entry, portMAX_DELAY) == pdTRUE) {
      if (!sdCardReady || !SD32_isPersistentFileOpen()) continue;

      if (SD32_getPersistentFileSize() >= SD_MAX_FILE_SIZE) {
        SD32_closePersistentFile();
        sessionNumber++;
        snprintf(csvFilename, 32, "/datalog_%03d.csv", sessionNumber);
        SD32_createCSVFile(csvFilename, csvHeaderBuffer);
        SD32_openPersistentFile(csvFilename);
        Serial.printf("[SD] Size limit reached, rotated to: %s\n", csvFilename);
      }

      AppenderFunc appenders[appenderCount] = {
        append_DataPoint_toCSVFile,
        append_Timestamp_toCSVFile,
        append_SessionTime_toCSVFile,
        append_MechData_toCSVFile,
        append_ElectData_toCSVFile,
        append_BAMOdata_toCSVFile
      };
      void* structArray[appenderCount] = {
        &entry.dataPoint, &entry.unixTime, &entry.sessionTime,
        &entry.mech, &entry.elect, &entry.bamo
      };

      SD32_appendBulkDataPersistent(appenders, structArray, appenderCount, SD_FLUSH_INTERVAL);
      localDataPoint++;
      Serial.printf("[SD] Logged #%d\n", localDataPoint);
    }
  }
}

/************************* Setup ***************************/

#define MOCK_FLAG 0
#define calibrate_RTC 0
#define DEBUG_MODE 0  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot

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

  // Sensor Init
  // RPMinit_withExtPullUP(ENCODER_PINL, ENCODER_PINR);
  // RPM_setCalcPeriod(My_timer, 100);
  StrokesensorInit(STR_Heave, STR_Roll);
  ElectSensorsInit(ElectPinArray);

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
  SD32_generateUniqueFilename(sessionNumber, csvFilename);
  strcat(csvHeaderBuffer, header_Timestamp);
  strcat(csvHeaderBuffer, ",");
  strcat(csvHeaderBuffer, header_Mechanical);
  strcat(csvHeaderBuffer, ",");
  strcat(csvHeaderBuffer, header_Electrical);
  strcat(csvHeaderBuffer, ",");
  strcat(csvHeaderBuffer, header_BAMO);
  SD32_createCSVFile(csvFilename, csvHeaderBuffer);
  if (sdCardReady) SD32_openPersistentFile(csvFilename);

  // Time Sync
  #if calibrate_RTC == 1
    RTCcalibrate(rtc,(ntpready) ? (WiFi32_getNTPTime()/1000ULL): 1000000000000ULL ,RTCavailable);
  #endif
  syncTime_setSyncPoint(RTC_UNIX_TIME, (RTCavailable) ? RTC_getUnix(rtc, RTCavailable) : 1000000000000ULL);

  Serial.println("==================================================");
  Serial.println("BP Bridge Sensor Node - Front - Ready");
  Serial.println("==================================================");
  Serial.printf("Client: %s\n\n", clientName);

  // FreeRTOS Setup
  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(BPMobileTask, "BPMobileTask", 8192, NULL, 1, &BPMobileTaskHandle, 0);
  Serial.println("[RTOS] BPMobile task on Core 0");

  sdQueue = xQueueCreate(30, sizeof(SDLogEntry));
  xTaskCreatePinnedToCore(sdTask, "SDTask", 4096, NULL, 1, &sdTaskHandle, 1);
  Serial.println("[RTOS] SD Logger task on Core 1");
}

/************************* Main Loop ***************************/

void loop() {

  uint64_t SESSION_TIME_MS = millis();

  // Time Sync: fetch RTC DS3231 every 1s
  uint64_t Time_placeholder = 0;
  if (SESSION_TIME_MS - lastTimeSourceSync >= LOCAL_SYNC_INTERVAL) {
    Time_placeholder = (uint64_t)RTC_getUnix(rtc, RTCavailable) * 1000ULL;
    lastTimeSourceSync = SESSION_TIME_MS;
  }
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
    uint64_t externalTime = WiFi32_getNTPTime();
    if (externalTime > 0 && RTCavailable) {
      if (syncTime_ifDrifted(RTC_UNIX_TIME, externalTime, 1000))
        RTCcalibrate(rtc, RTC_UNIX_TIME / 1000ULL, RTCavailable);
    }
    lastExternalSync = SESSION_TIME_MS;
  }

  // Debug Output
  #if DEBUG_MODE > 0
  if (SESSION_TIME_MS - lastTeleplotDebug >= TELEPLOT_DEBUG_INTERVAL) {
    #if DEBUG_MODE == 1
      // Regular serial debug - placeholder for custom debug output
      Serial.printf("[DEBUG] Mech: RPM_L=%.1f RPM_R=%.1f\n", myMechData.Wheel_RPM_L, myMechData.Wheel_RPM_R);
      Serial.printf("[DEBUG] Elect: I=%.2fA APPS=%.1f BPPS=%.1f\n", myElectData.I_SENSE, myElectData.APPS, myElectData.BPPS);
      Serial.printf("[DEBUG] BAMO: V=%.1fV I=%.1fA P=%.1fW\n", myBAMOCar.canVoltage, myBAMOCar.canCurrent, myBAMOCar.power);
    #elif DEBUG_MODE == 2
      // Teleplot format debug
      teleplotMechanical(&myMechData);
      teleplotElectrical(&myElectData);
      teleplotBAMOCar(&myBAMOCar);
    #endif
    lastTeleplotDebug = SESSION_TIME_MS;
  }
  #endif

  twai_message_t txmsg;
  twai_message_t rxmsg;

  // Debug Console
  if (Serial.available() && Serial.peek() == '`') {
    Serial.read();
    while (1) {
      showDeviceStatus();
      if (Serial.available() && Serial.read() == '~') break;
      delay(200);
    }
  }

  #if MOCK_FLAG == 0
    // Sensor Update
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      // RPMsensorUpdate(&myMechData, RPM_CalcInterval, ENCODER_N);
      StrokesensorUpdate(&myMechData, STR_Heave, STR_Roll);
      ElectSensorsUpdate(&myElectData, ElectPinArray);
      xSemaphoreGive(dataMutex);
    }

    // BAMOCar CAN
    if (canBusReady) {
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
      if (SESSION_TIME_MS - lastBAMOrequest >= BAMOCarREQ_INTERVAL) {
        switch (CANRequest_sequence) {
          case 0: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_MOTOR_TEMP); break;
          case 1: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_CONTROLLER_TEMP); break;
          case 2: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_DC_VOLTAGE); break;
          case 3: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_DC_CURRENT); break;
        }
        CAN32_sendCAN(&txmsg);
        Serial.printf("[CAN TX] ID=0x%03X REG=0x%02X (seq=%d)\n",
          txmsg.identifier, txmsg.data[0], CANRequest_sequence);
        CANRequest_sequence = (CANRequest_sequence + 1) % 4;
        lastBAMOrequest = SESSION_TIME_MS;
      }
    }

    // Power Calculation
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (myBAMOCar.canVoltageValid && myBAMOCar.canCurrentValid) {
        myBAMOCar.power = myBAMOCar.canVoltage * myBAMOCar.canCurrent;
      }
      xSemaphoreGive(dataMutex);
    }
  #endif

  #if MOCK_FLAG == 1
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      mockMechanicalData(&myMechData);
      mockElectricalData(&myElectData);
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

  // Queue SD Log Entry
  if (sdCardReady && sdQueue != NULL && (SESSION_TIME_MS - lastSDLog >= SD_APPEND_INTERVAL)) {
    SDLogEntry entry;
    entry.dataPoint = dataPoint;
    entry.unixTime = CURRENT_UNIX_TIME_MS;
    entry.sessionTime = SESSION_TIME_MS;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      entry.mech = myMechData;
      entry.elect = myElectData;
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

}

/************************* Debug Functions ***************************/

void showDeviceStatus() {
  Serial.println("╔═════════════════════════════════════════════╗");
  Serial.println("║        FRONT NODE - SYSTEM STATUS           ║");
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

void publishMechData(Mechanical* MechSensors) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  JsonDocument docL;
  docL["type"] = "data";
  docL["topic"] = "wheel/left_rpm";
  docL["data"]["value"] = MechSensors->Wheel_RPM_L;
  docL["data"]["sensor_id"] = "ENC_LEFT";
  docL["timestamp"] = timestamp;
  String msgL;
  serializeJson(docL, msgL);
  BPwebSocket->sendTXT(msgL);

  JsonDocument docR;
  docR["type"] = "data";
  docR["topic"] = "wheel/right_rpm";
  docR["data"]["value"] = MechSensors->Wheel_RPM_R;
  docR["data"]["sensor_id"] = "ENC_RIGHT";
  docR["timestamp"] = timestamp;
  String msgR;
  serializeJson(docR, msgR);
  BPwebSocket->sendTXT(msgR);

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
  topics.add("power/can_voltage");
  topics.add("power/can_current");
  topics.add("power/power");
  topics.add("motor/temperature");
  topics.add("motor/controller_temperature");
  topics.add("wheel/left_rpm");
  topics.add("wheel/right_rpm");
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

  JsonObject leftRpmMeta = metadata["wheel/left_rpm"].to<JsonObject>();
  leftRpmMeta["description"] = "Left wheel speed";
  leftRpmMeta["unit"] = "RPM";
  leftRpmMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

  JsonObject rightRpmMeta = metadata["wheel/right_rpm"].to<JsonObject>();
  rightRpmMeta["description"] = "Right wheel speed";
  rightRpmMeta["unit"] = "RPM";
  rightRpmMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

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

void append_MechData_toCSVFile(File& dataFile, void* data) {
  Mechanical* m = static_cast<Mechanical*>(data);
  dataFile.print(m->Wheel_RPM_L, 2);
  dataFile.print(",");
  dataFile.print(m->Wheel_RPM_R, 2);
  dataFile.print(",");
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
  dataFile.print(",");
}
