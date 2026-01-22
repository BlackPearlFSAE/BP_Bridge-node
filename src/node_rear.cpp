/* BP Bridge Sensor Node - Rear
 *
 * ESP32-S3 dual-core sensor aggregator for Formula Student vehicle telemetry.
 * - Core 0: WiFi/WebSocket communication (BPMobile)
 * - Core 1: Sensor polling, SD logging
 *
 * Sensors: Wheel RPM, Stroke, Electrical, GPS, IMU
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
#include <HardwareSerial.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <time.h>
#include <MPU6050_light.h>
#include <TinyGPS++.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <BP_mobile_util.h>
#include <SD32_util.h>
#include <RTClib_helper.h>
#include <syncTime_util.h>
#include <WIFI32_util.h>
#include <motion_sensors.h>
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

// UART
#define UART0_BAUD 115200

// I2C Buses
#define I2C1_SDA 18
#define I2C1_SCL 17
#define I2C2_SDA 42
#define I2C2_SCL 41

// Mechanical Sensors
#define ENCODER_PINL 3
#define ENCODER_PINR 9
#define ENCODER_N 50
#define STR_Roll 5
#define STR_Heave 6

// Motion Sensors
#define GPS_RX_PIN 2
#define GPS_TX_PIN 1
#define GPS_BAUD 115200

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
const char* clientName = "ESP32 Rear Node";
WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

// Sampling Rates (Hz)
const float MECH_SENSORS_SAMPLING_RATE = 2.0;
const float ELECT_SENSORS_SAMPLING_RATE = 2.0;
const float ELECT_FAULT_STAT_SAMPLING_RATE = 2.0;
const float ODOM_SENSORS_SAMPLING_RATE = 2.0;

// Sensor Data
Mechanical myMechData;
Electrical myElectData;
Odometry myOdometryData;

// Peripherals
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
MPU6050 mpu(Wire);
RTC_DS3231 rtc;
hw_timer_t* My_timer = nullptr;

// Status Flags
bool I2C1_connect = false;
bool I2C2_connect = false;
bool sdCardReady = false;
bool GPSavailable = false;
bool IMUavailable = false;
bool RTCavailable = false;

// Timing Intervals (ms)
const unsigned long SD_APPEND_INTERVAL = 200;
const unsigned long SD_FLUSH_INTERVAL = 1000;
const size_t SD_MAX_FILE_SIZE = 2 * 1024 * 1024;
const unsigned long LOCAL_SYNC_INTERVAL = 1000;
const unsigned long REMOTE_SYNC_INTERVAL = 60000;
const unsigned long RPM_CalcInterval = 100;

unsigned long lastSDLog = 0;
unsigned long lastTimeSourceSync = 0;
unsigned long lastExternalSync = 0;
int dataPoint = 1;
int sessionNumber = 0;
uint64_t RTC_UNIX_TIME = 0;

// SD Card
char csvFilename[32] = {0};
const char* header_Timestamp = "DataPoint,UnixTime,SessionTime";
const char* header_Mechanical = "Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm";
const char* header_Electrical = "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK";
const char* header_Odometry = "GPS_Lat,GPS_Lng,GPS_Age,GPS_Course,GPS_Speed,IMU_AccelX,IMU_AccelY,IMU_AccelZ,IMU_GyroX,IMU_GyroY,IMU_GyroZ";
char csvHeaderBuffer[350] = "";
int appenderCount = 6;

/************************* Function Declarations ***************************/

// CSV Appenders
void append_DataPoint_toCSVFile(File& dataFile, void* data);
void append_Timestamp_toCSVFile(File& dataFile, void* data);
void append_SessionTime_toCSVFile(File& dataFile, void* data);
void append_MechData_toCSVFile(File& dataFile, void* data);
void append_ElectData_toCSVFile(File& dataFile, void* data);
void append_OdometryData_toCSVFile(File& dataFile, void* data);

// BPMobile Publishers
void publishMechData(Mechanical* MechSensors);
void publishElectData(Electrical* ElectSensors);
void publishElectFaultState(Electrical* ElectSensors);
void publishOdometryData(Odometry* OdomSensors);
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
  Odometry odom;
};

// Core 0: WiFi/WebSocket Task
void BPMobileTask(void* parameter) {
  unsigned long taskLastMech = 0;
  unsigned long taskLastElect = 0;
  unsigned long taskLastElectFault = 0;
  unsigned long taskLastOdom = 0;

  Mechanical localMech;
  Electrical localElect;
  Odometry localOdom;

  while (true) {
    unsigned long now = millis();

    if (WiFi.status() == WL_CONNECTED) {
      BPwebSocket->loop();
    }

    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        localMech = myMechData;
        localElect = myElectData;
        localOdom = myOdometryData;
        xSemaphoreGive(dataMutex);
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
      if (now - taskLastOdom >= (1000.0 / ODOM_SENSORS_SAMPLING_RATE)) {
        publishOdometryData(&localOdom);
        taskLastOdom = now;
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
        append_OdometryData_toCSVFile
      };
      void* structArray[appenderCount] = {
        &entry.dataPoint, &entry.unixTime, &entry.sessionTime,
        &entry.mech, &entry.elect, &entry.odom
      };

      SD32_appendBulkDataPersistent(appenders, structArray, appenderCount, SD_FLUSH_INTERVAL);
      localDataPoint++;
      Serial.printf("[SD] Logged #%d\n", localDataPoint);
    }
  }
}

/************************* Setup ***************************/

#define MOCK_FLAG 1
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(UART0_BAUD);

  // I2C Init
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  I2C2_connect = Wire.begin(I2C2_SDA, I2C2_SCL);
  Wire1.setTimeout(2);
  Wire.setTimeout(2);

  // RTC Init
  RTCavailable = RTCinit(rtc, &Wire1);

  // Motion Sensor Init
  IMUavailable = IMUinit(&Wire, mpu);
  delay(1000);
  IMUcalibrate(mpu, IMUavailable);
  GPSavailable = GPSinit(gpsSerial, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);

  // Base Sensor Init
  RPMinit_withExtPullUP(ENCODER_PINL, ENCODER_PINR);
  RPM_setCalcPeriod(My_timer, 100);
  StrokesensorInit(STR_Heave, STR_Roll);
  ElectSensorsInit(ElectPinArray);

  // WiFi & WebSocket
  initWiFi(ssid, password, 10);
  if (WiFi.status() == WL_CONNECTED) {
    WiFi32_initNTP();
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
  strcat(csvHeaderBuffer, header_Odometry);
  SD32_createCSVFile(csvFilename, csvHeaderBuffer);
  if (sdCardReady) SD32_openPersistentFile(csvFilename);

  // Time Sync
  syncTime_setSyncPoint(RTC_UNIX_TIME, (RTCavailable) ? RTC_getUnix(rtc, RTCavailable) : 1000000000000ULL);
  Serial.print("Time synced: ");
  Serial.println(RTC_getISO(rtc, RTCavailable));

  Serial.println("==================================================");
  Serial.println("BP Bridge Sensor Node - Rear - Ready");
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

  // Time Sync: Remote source -> DS3231 (60s period)
  if (SESSION_TIME_MS - lastExternalSync >= REMOTE_SYNC_INTERVAL) {
    uint64_t externalTime = WiFi32_getNTPTime();
    if (externalTime > 0 && RTCavailable) {
      if (syncTime_ifDrifted(RTC_UNIX_TIME, externalTime, 1000))
        RTCcalibrate(rtc, RTC_UNIX_TIME / 1000ULL, RTCavailable);
    }
    lastExternalSync = SESSION_TIME_MS;
  }

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
      RPMsensorUpdate(&myMechData, RPM_CalcInterval, ENCODER_N);
      StrokesensorUpdate(&myMechData, STR_Heave, STR_Roll);
      ElectSensorsUpdate(&myElectData, ElectPinArray);
      GPSupdate(&myOdometryData, gpsSerial, gps, GPSavailable);
      IMUupdate(&myOdometryData, mpu, IMUavailable);
      xSemaphoreGive(dataMutex);
    }
  #endif

  #if MOCK_FLAG == 1
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      mockMechanicalData(&myMechData);
      mockElectricalData(&myElectData);
      mockOdometryData(&myOdometryData);
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
      entry.odom = myOdometryData;
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
  Serial.println("║         REAR NODE - SYSTEM STATUS           ║");
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.printf("║ I2C1:         %s\n", I2C1_connect ? "OK" : "FAIL");
  Serial.printf("║ I2C2:         %s\n", I2C2_connect ? "OK" : "FAIL");
  Serial.printf("║ SD Card:      %s\n", sdCardReady ? "OK" : "FAIL");
  Serial.printf("║ WiFi:         %s (RSSI: %d)\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL", WiFi.RSSI());
  Serial.printf("║ RTC:          %s\n", RTCavailable ? "OK" : "FAIL");
  Serial.printf("║ WebSocket:    %s\n", BPsocketstatus->isConnected ? "OK" : "FAIL");
  Serial.printf("║ Time Sync:    %s\n", syncTime_isSynced() ? "OK" : "FAIL");
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.printf("║ IMU:          %s\n", IMUavailable ? "OK" : "FAIL");
  Serial.printf("║ GPS:          %s\n", gpsSerial.available() > 0 ? "OK" : "FAIL");
  Serial.println("╚═════════════════════════════════════════════╝");
}

/************************* BPMobile Publishers ***************************/

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

void publishOdometryData(Odometry* OdomSensors) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  // GPS Data
  JsonDocument gpsDoc;
  gpsDoc["type"] = "data";
  gpsDoc["topic"] = "odometry/gps";
  gpsDoc["data"]["lat"] = OdomSensors->gps_lat;
  gpsDoc["data"]["lng"] = OdomSensors->gps_lng;
  gpsDoc["data"]["age"] = OdomSensors->gps_age;
  gpsDoc["data"]["course"] = OdomSensors->gps_course;
  gpsDoc["data"]["speed"] = OdomSensors->gps_speed;
  gpsDoc["data"]["sensor_id"] = "GPS_AN251";
  gpsDoc["timestamp"] = timestamp;
  String gpsMsg;
  serializeJson(gpsDoc, gpsMsg);
  BPwebSocket->sendTXT(gpsMsg);

  // IMU Accelerometer Data
  JsonDocument accelDoc;
  accelDoc["type"] = "data";
  accelDoc["topic"] = "odometry/imu_accel";
  accelDoc["data"]["x"] = OdomSensors->imu_accelx;
  accelDoc["data"]["y"] = OdomSensors->imu_accely;
  accelDoc["data"]["z"] = OdomSensors->imu_accelz;
  accelDoc["data"]["sensor_id"] = "MPU6050";
  accelDoc["timestamp"] = timestamp;
  String accelMsg;
  serializeJson(accelDoc, accelMsg);
  BPwebSocket->sendTXT(accelMsg);

  // IMU Gyroscope Data
  JsonDocument gyroDoc;
  gyroDoc["type"] = "data";
  gyroDoc["topic"] = "odometry/imu_gyro";
  gyroDoc["data"]["x"] = OdomSensors->imu_gyrox;
  gyroDoc["data"]["y"] = OdomSensors->imu_gyroy;
  gyroDoc["data"]["z"] = OdomSensors->imu_gyroz;
  gyroDoc["data"]["sensor_id"] = "MPU6050";
  gyroDoc["timestamp"] = timestamp;
  String gyroMsg;
  serializeJson(gyroDoc, gyroMsg);
  BPwebSocket->sendTXT(gyroMsg);
}

void registerClient(const char* clientName) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = clientName;

  JsonArray topics = doc["topics"].to<JsonArray>();
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
  topics.add("odometry/gps");
  topics.add("odometry/imu_accel");
  topics.add("odometry/imu_gyro");

  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();

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

  JsonObject gpsMeta = metadata["odometry/gps"].to<JsonObject>();
  gpsMeta["description"] = "GPS position and motion data";
  gpsMeta["unit"] = "mixed";
  gpsMeta["sampling_rate"] = ODOM_SENSORS_SAMPLING_RATE;

  JsonObject accelMeta = metadata["odometry/imu_accel"].to<JsonObject>();
  accelMeta["description"] = "IMU accelerometer (X, Y, Z)";
  accelMeta["unit"] = "m/s^2";
  accelMeta["sampling_rate"] = ODOM_SENSORS_SAMPLING_RATE;

  JsonObject gyroMeta = metadata["odometry/imu_gyro"].to<JsonObject>();
  gyroMeta["description"] = "IMU gyroscope (X, Y, Z)";
  gyroMeta["unit"] = "deg/s";
  gyroMeta["sampling_rate"] = ODOM_SENSORS_SAMPLING_RATE;

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

void append_OdometryData_toCSVFile(File& dataFile, void* data) {
  Odometry* o = static_cast<Odometry*>(data);
  dataFile.print(o->gps_lat, 4);
  dataFile.print(",");
  dataFile.print(o->gps_lng, 4);
  dataFile.print(",");
  dataFile.print(o->gps_age, 2);
  dataFile.print(",");
  dataFile.print(o->gps_course, 2);
  dataFile.print(",");
  dataFile.print(o->gps_speed, 2);
  dataFile.print(",");
  dataFile.print(o->imu_accelx, 3);
  dataFile.print(",");
  dataFile.print(o->imu_accely, 3);
  dataFile.print(",");
  dataFile.print(o->imu_accelz, 3);
  dataFile.print(",");
  dataFile.print(o->imu_gyrox, 3);
  dataFile.print(",");
  dataFile.print(o->imu_gyroy, 3);
  dataFile.print(",");
  dataFile.println(o->imu_gyroz, 3);
}
