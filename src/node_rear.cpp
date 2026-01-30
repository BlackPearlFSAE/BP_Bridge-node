/* BP Bridge Sensor Node - Rear
 *
 * ESP32-S3 dual-core sensor aggregator for Formula Student vehicle telemetry.
 * - Core 0: WiFi/WebSocket communication (BPMobile)
 * - Core 1: Sensor polling, SD logging
 *
 * Sensors: Wheel RPM, Stroke, GPS, IMU
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
#include <DS3231_util.h>
#include <syncTime_util.h>
#include <WIFI32_util.h>

#include <motion_sensors.h>
#include <base_sensors.h>
#include <shared_config.h>

/************************* Pin Definitions ***************************/

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
#define IMU_SDA 42
#define IMU_SCL 41

/************************* Global Variables ***************************/

// WebSocket and Network config
const char* ssid = DEFAULT_SSID;
const char* password = DEFAULT_PASSWORD;
const char* serverHost = DEFAULT_SERVER_HOST;
const int serverPort = DEFAULT_SERVER_PORT;
const char* clientName = "Rear_Node";
WebSocketsClient webSockets;
socketstatus webSocketStatus;

BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

// Sampling Rates (Hz)
const float MECH_SENSORS_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;
const float ODOM_SENSORS_SAMPLING_RATE = DEFAULT_PUBLISH_RATE;

// Sensor Data
Mechanical myMechData;
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
const unsigned long LOCAL_SYNC_INTERVAL = DEFAULT_LOCAL_SYNC_INTERVAL;
const unsigned long REMOTE_SYNC_INTERVAL = DEFAULT_REMOTE_SYNC_INTERVAL;

const unsigned long SD_APPEND_INTERVAL = DEFAULT_SD_LOG_INTERVAL;
const unsigned long SD_FLUSH_INTERVAL = DEFAULT_SD_FLUSH_INTERVAL;
const unsigned long SD_CLOSE_INTERVAL = DEFAULT_SD_CLOSE_INTERVAL;
const int SD_MAX_ROWS = DEFAULT_SD_ROW_LIMIT;
const unsigned long RPM_CalcInterval = 100;

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
const char* header_Mechanical = "Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm";
const char* header_Odometry = "GPS_Lat,GPS_Lng,GPS_Age,GPS_Course,GPS_Speed,IMU_AccelX,IMU_AccelY,IMU_AccelZ,IMU_GyroX,IMU_GyroY,IMU_GyroZ";
char csvHeaderBuffer[350] = "";
int appenderCount = 5;

/************************* Function Declarations ***************************/

// CSV Appenders
void append_DataPoint_toCSVFile(File& dataFile, void* data);
void append_Timestamp_toCSVFile(File& dataFile, void* data);
void append_SessionTime_toCSVFile(File& dataFile, void* data);
void append_MechData_toCSVFile(File& dataFile, void* data);
void append_OdometryData_toCSVFile(File& dataFile, void* data);

// BPMobile Publishers
void publishMechData(Mechanical* MechSensors);
void publishOdometryData(Odometry* OdomSensors);
void registerClient(const char* clientName);

// Debug
void showDeviceStatus();

/************************* Build Flags ***************************/

#define MOCK_FLAG 0
#define calibrate_RTC 0
#define DEBUG_MODE 0  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot

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
  Odometry odom;
};

// Core 0: WiFi/WebSocket Task
void BPMobileTask(void* parameter) {
  unsigned long taskLastMech = 0;
  unsigned long taskLastOdom = 0;

  Mechanical localMech;
  Odometry localOdom;

  while (true) {
    unsigned long SESSION_TIME = millis();

    if (WiFi.status() == WL_CONNECTED) {
      BPwebSocket->loop();
    }

    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        localMech = myMechData;
        localOdom = myOdometryData;
        xSemaphoreGive(dataMutex);
      }

      if (SESSION_TIME - taskLastMech >= (1000.0 / MECH_SENSORS_SAMPLING_RATE)) {
        publishMechData(&localMech);
        taskLastMech = SESSION_TIME;
      }
      if (SESSION_TIME - taskLastOdom >= (1000.0 / ODOM_SENSORS_SAMPLING_RATE)) {
        publishOdometryData(&localOdom);
        taskLastOdom = SESSION_TIME;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Core 1: SD Card Logger Task
// Consumes SDLogEntry from queue, writes to persistent CSV file in session dir.
// File rotation: after SD_MAX_ROWS rows, closes current file and opens next
// (e.g. /Rear_session_000/File_0.csv -> File_1.csv -> ...).
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
        append_OdometryData_toCSVFile
      };
      void* structArray[appenderCount] = {
        &entry.dataPoint, 
        &entry.unixTime, 
        &entry.sessionTime,
        &entry.mech, 
        &entry.odom
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

// Core 1: Sensor Reading Task
void sensorTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Mechanical localMech;
  Odometry localOdom;

  while (true) {
    // Read sensors into local structs
    #if MOCK_FLAG == 0
      RPMsensorUpdate(&localMech, RPM_CalcInterval, ENCODER_N);
      StrokesensorUpdate(&localMech, STR_Heave, STR_Roll);
      GPSupdate(&localOdom, gpsSerial, gps, GPSavailable);
      IMUupdate(&localOdom, mpu, IMUavailable);
    #else
      mockMechanicalData(&localMech);
      mockOdometryData(&localOdom);
    #endif

    // Brief Mutex protection to copy results into shared structs
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      myMechData = localMech;
      myOdometryData = localOdom;
      xSemaphoreGive(dataMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // 50Hz
  }
}

/************************* Setup ***************************/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(UART0_BAUD);

  // I2C Init
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  I2C2_connect = Wire.begin(IMU_SDA, IMU_SCL);
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
  strcat(csvHeaderBuffer, header_Odometry);
  if (sdCardReady) {
    SD32_createSessionDir(sessionNumber, sessionDirPath, "Rear");
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
  Serial.println("BP Bridge Sensor Node - Rear - Ready");
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
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 5, &sensorTaskHandle, 1);
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
    Odometry debugOdom;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      debugMech = myMechData;
      debugOdom = myOdometryData;
      xSemaphoreGive(dataMutex);
    }
    #if DEBUG_MODE == 1
      Serial.printf("[DEBUG] Mech: RPM_L=%.1f RPM_R=%.1f\n", debugMech.Wheel_RPM_L, debugMech.Wheel_RPM_R);
      Serial.printf("[DEBUG] Odom: GPS(%.3f,%.3f) IMU(%.2f,%.2f,%.2f)\n",
        debugOdom.gps_lat, debugOdom.gps_lng,
        debugOdom.imu_accelx, debugOdom.imu_accely, debugOdom.imu_accelz);
    #elif DEBUG_MODE == 2
      teleplotMechanical(&debugMech);
      teleplotOdometry(&debugOdom);
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

  vTaskDelay(pdMS_TO_TICKS(20));
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
  dataFile.print(o->imu_gyroz, 3);
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
