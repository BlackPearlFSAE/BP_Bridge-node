#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include <driver/twai.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <time.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <BP_mobile_util.h>
#include <SD32_util.h>
#include <DS3231_util.h>
#include <syncTime_util.h>
#include <CAN32_util.h>
#include <WIFI32_util.h>

#include <shared_config.h>
#include "helper.h"

/************************* Global Variables ***************************/

int ElectPinArray[9] = {
  I_SENSE_PIN,
  TMP_PIN,
  APPS_PIN,
  BPPS_PIN,
  AMS_OK_PIN,
  IMD_OK_PIN,
  HV_ON_PIN,
  BSPD_OK_PIN,
  STEERING
};

// WebSocket and Network config
const char* ssid = DEFAULT_SSID;
const char* password = DEFAULT_PASSWORD;
const char* serverHost = DEFAULT_SERVER_HOST;
const int   serverPort = DEFAULT_SERVER_PORT;
const char* clientName = DEFAULT_CLIENT_NAME;
WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus*     BPsocketstatus = BPMobile.webSocketstatus;

// Sampling Rates (Hz)
const float MECH_SENSORS_SAMPLING_RATE      = DEFAULT_PUBLISH_RATE;
const float ELECT_SENSORS_SAMPLING_RATE     = DEFAULT_PUBLISH_RATE;
const float ELECT_FAULT_SAMPLING_RATE       = 2.0;
const float ODOM_SENSORS_SAMPLING_RATE      = DEFAULT_PUBLISH_RATE;
const float BAMO_POWER_SAMPLING_RATE        = DEFAULT_PUBLISH_RATE;
const float BAMO_TEMP_SAMPLING_RATE         = 5.0;

// Shared sensor data (protected by dataMutex)
Mechanical myMechData;
Electrical myElectData;
Odometry   myOdometryData;
BAMOCar    myBAMOCar;

// Peripherals
HardwareSerial   gpsSerial(1);
TinyGPSPlus      gps;
Adafruit_BNO055  myimu = Adafruit_BNO055(55, 0x28, &Wire);
RTC_DS3231       rtc;

// Availability flags
bool I2C1_connect  = false;
bool I2C2_connect  = false;
bool sdCardReady   = false;
bool RTCavailable  = false;
bool GPSavailable  = false;
bool IMUavailable  = false;
bool canBusReady   = false;

// Timing Intervals (ms)
#define BAMOCarREQ_INTERVAL 200
const unsigned long LOCAL_SYNC_INTERVAL  = DEFAULT_LOCAL_SYNC_INTERVAL;
const unsigned long REMOTE_SYNC_INTERVAL = DEFAULT_REMOTE_SYNC_INTERVAL;
const unsigned long SD_APPEND_INTERVAL = DEFAULT_SD_LOG_INTERVAL;
const unsigned long SD_FLUSH_INTERVAL  = DEFAULT_SD_FLUSH_INTERVAL;
const unsigned long SD_CLOSE_INTERVAL  = DEFAULT_SD_CLOSE_INTERVAL;
const int           SD_MAX_ROWS        = DEFAULT_SD_ROW_LIMIT;

unsigned long lastSDLog = 0;
unsigned long lastTeleplotDebug = 0;
const unsigned long TELEPLOT_DEBUG_INTERVAL = 200;
int dataPoint = 1;
int sessionNumber = 0;
uint64_t RTC_UNIX_TIME = 0;

// SD Card
char sessionDirPath[48] = {0};
char csvFilename[48]    = {0};
int  fileIndex = 0;
const char* CSV_HEADER =
  "DataPoint,UnixTime,SessionTime,"
  "Wheel_RPM_L,Wheel_RPM_R,Heave_mm,Roll_mm,"
  "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK,Steering,"
  "GPS_Lat,GPS_Lng,GPS_Age,GPS_Course,GPS_Speed,"
  "IMU_AccelX,IMU_AccelY,IMU_AccelZ,"
  "IMU_GyroX,IMU_GyroY,IMU_GyroZ,"
  "IMU_EulerRoll,IMU_EulerPitch,IMU_EulerYaw,"
  "IMU_MagX,IMU_MagY,IMU_MagZ,"
  "IMU_GravX,IMU_GravY,IMU_GravZ,"
  "BAMO_Volt,BAMO_Amp,BAMO_Power,BAMO_MotorTemp,BAMO_ControllerTemp,BAMO_RPM";

/************************* Function Declarations ***************************/

void publishMechData(Mechanical* m);
void publishElectData(Electrical* e);
void publishElectFaultState(Electrical* e);
void publishOdometryData(Odometry* o);
void publishBAMOpower(BAMOCar* b);
void publishBAMOtemp(BAMOCar* b);
void registerClient(const char* clientName);

void showDeviceStatus();

/************************* FreeRTOS ***************************/

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;
TaskHandle_t BPMobileTaskHandle = NULL;
TaskHandle_t timeSyncTaskHandle = NULL;
TaskHandle_t sensorTaskHandle   = NULL;
TaskHandle_t canTaskHandle      = NULL;
TaskHandle_t sdTaskHandle       = NULL;
QueueHandle_t sdQueue = NULL;

struct SDLogEntry {
  int        dataPoint;
  uint64_t   unixTime;
  uint64_t   sessionTime;
  Mechanical mech;
  Electrical elect;
  Odometry   odom;
  BAMOCar    bamo;
};

/************************* SD open write flush, close system ***************************/

static File      _logFile;
static bool      _logFileOpen    = false;
static char      _logFilePath[48] = {0};
static unsigned long _logLastFlush = 0;
static unsigned long _logLastClose = 0;

static bool openLogFile(const char* path) {
  _logFile = SD.open(path, FILE_APPEND);
  if (!_logFile) {
    Serial.printf("[SD] ERROR: Could not open log file: %s\n", path);
    _logFileOpen = false;
    return false;
  }
  strncpy(_logFilePath, path, sizeof(_logFilePath) - 1);
  _logFilePath[sizeof(_logFilePath) - 1] = '\0';
  _logFileOpen  = true;
  _logLastFlush = millis();
  Serial.printf("[SD] Log file opened: %s\n", path);
  return true;
}

static void closeLogFile() {
  if (_logFileOpen && _logFile) {
    _logFile.flush();
    _logFile.close();
    _logFileOpen = false;
    _logFilePath[0] = '\0';
    Serial.println("[SD] Log file closed");
  }
}

static bool isLogFileOpen() { return _logFileOpen; }

static void append_sensors_toCSV(const SDLogEntry& e) {
  char buf[512];
  int n = snprintf(buf, sizeof(buf),
    "%d,%llu,%llu,"
    "%.2f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.2f,"
    "%.4f,%.4f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.1f,%.1f,%.0f\r\n",
    e.dataPoint, e.unixTime, e.sessionTime,
    e.mech.Wheel_RPM_L, e.mech.Wheel_RPM_R, e.mech.STR_Heave_mm, e.mech.STR_Roll_mm,
    e.elect.I_SENSE, e.elect.TMP, e.elect.APPS, e.elect.BPPS,
    e.elect.AMS_OK ? 1 : 0, e.elect.IMD_OK ? 1 : 0, e.elect.HV_ON ? 1 : 0, e.elect.BSPD_OK ? 1 : 0,
    e.elect.steering,
    e.odom.gps_lat, e.odom.gps_lng, e.odom.gps_age, e.odom.gps_course, e.odom.gps_speed,
    e.odom.imu_accelx, e.odom.imu_accely, e.odom.imu_accelz,
    e.odom.imu_gyrox,  e.odom.imu_gyroy,  e.odom.imu_gyroz,
    e.odom.imu_euler_roll, e.odom.imu_euler_pitch, e.odom.imu_euler_yaw,
    e.odom.imu_magx,  e.odom.imu_magy,  e.odom.imu_magz,
    e.odom.imu_gravx, e.odom.imu_gravy, e.odom.imu_gravz,
    e.bamo.canVoltage, e.bamo.canCurrent, e.bamo.power,
    e.bamo.motorTemp2, e.bamo.controllerTemp, e.bamo.rpm
  );
  if (n > 0) _logFile.write((const uint8_t*)buf, n);
}

/************************* Tasks ***************************/

// Core 0: WiFi/WebSocket Task
void BPMobileTask(void* parameter) {
  unsigned long tMech = 0, tElect = 0, tElectFault = 0;
  unsigned long tOdom = 0, tBAMOpow = 0, tBAMOtemp = 0;
  Mechanical localMech;
  Electrical localElect;
  Odometry   localOdom;
  BAMOCar    localBAMO;

  while (true) {
    unsigned long now = millis();
    if (WiFi.status() == WL_CONNECTED) BPwebSocket->loop();

    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        localMech  = myMechData;
        localElect = myElectData;
        localOdom  = myOdometryData;
        localBAMO  = myBAMOCar;
        xSemaphoreGive(dataMutex);
      }

      bool isFront = strcmp(clientName, "front") == 0;
      bool isRear  = strcmp(clientName, "rear")  == 0;

      // mech: both nodes
      if (now - tMech >= (1000.0 / MECH_SENSORS_SAMPLING_RATE))         {publishMechData(&localMech); tMech = now;}

      // elect + faults + bamo: front only
      if (isFront) {
        if (now - tElect      >= (1000.0 / ELECT_SENSORS_SAMPLING_RATE)){ publishElectData(&localElect);       tElect = now; }
        if (now - tElectFault >= (1000.0 / ELECT_FAULT_SAMPLING_RATE))  { publishElectFaultState(&localElect); tElectFault = now; }
        if (now - tBAMOpow    >= (1000.0 / BAMO_POWER_SAMPLING_RATE))   { publishBAMOpower(&localBAMO);        tBAMOpow = now; }
        if (now - tBAMOtemp   >= (1000.0 / BAMO_TEMP_SAMPLING_RATE))    { publishBAMOtemp(&localBAMO);         tBAMOtemp = now; }
      }

      // odom: rear only
      if (isRear) {
        if (now - tOdom >= (1000.0 / ODOM_SENSORS_SAMPLING_RATE)) { publishOdometryData(&localOdom); tOdom = now; }
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
      if (!sdCardReady || !isLogFileOpen()) continue;

      if (localDataPoint > 0 && localDataPoint % SD_MAX_ROWS == 0) {
        closeLogFile();
        fileIndex++;
        SD32_generateFilenameInDir(csvFilename, sessionDirPath, "File", fileIndex);
        SD32_createCSVFile(csvFilename, CSV_HEADER);
        openLogFile(csvFilename);
        Serial.printf("[SD] Row limit reached, rotated to: %s\n", csvFilename);
      }

      append_sensors_toCSV(entry);

      unsigned long now = millis();
      if (SD_FLUSH_INTERVAL == 0 || (now - _logLastFlush >= SD_FLUSH_INTERVAL)) {
        _logFile.flush();
        _logLastFlush = now;
      }
      if (SD_CLOSE_INTERVAL > 0 && (now - _logLastClose >= SD_CLOSE_INTERVAL)) {
        _logFile.flush();
        _logFile.close();
        _logFile = SD.open(_logFilePath, FILE_APPEND);
        if (!_logFile) {
          _logFileOpen = false;
          Serial.println("[SD] ERROR: Could not reopen log file after cycle!");
        }
        _logLastClose = now;
      }

      localDataPoint++;
    }
  }
}

// Core 0: Time Synchronization Task
void timeSyncTask(void* parameter) {
  unsigned long lastLocalSync = 0;
  unsigned long lastRemoteSync = 0;

  while (true) {
    unsigned long now = millis();

    if (now - lastLocalSync >= LOCAL_SYNC_INTERVAL) {
      #if TIME_SRC == 0
      uint64_t t = (uint64_t)RTC_getUnix(rtc, RTCavailable) * 1000ULL;
      #elif TIME_SRC == 1
      uint64_t t = WiFi32_getNTPTime();
      #endif
      if (t > 0) syncTime_setSyncPoint(RTC_UNIX_TIME, t);
      lastLocalSync = now;
    }

    #if WIFI_ENABLED == 1
    if (now - lastRemoteSync >= REMOTE_SYNC_INTERVAL) {
      uint64_t externalTime = WiFi32_getNTPTime();
      if (externalTime > 0 && RTCavailable) {
        if (syncTime_ifDrifted(RTC_UNIX_TIME, externalTime, 1000))
          RTCcalibrate(rtc, RTC_UNIX_TIME / 1000ULL, RTCavailable);
      }
      lastRemoteSync = now;
    }
    #endif

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Core 1: Sensor Reading Task (stroke + electrical + GPS + IMU)
void sensorTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Mechanical localMech;
  Electrical localElect;
  Odometry   localOdom;

  bool isFront = strcmp(clientName, "front") == 0;
  bool isRear  = strcmp(clientName, "rear")  == 0;

  while (true) {
    #if MOCK_FLAG == 0
      StrokesensorUpdate(&localMech, STR_Heave, STR_Roll);
      if (isFront) ElectSensorsUpdate(&localElect, ElectPinArray);
      if (isRear)  { GPSupdate(&localOdom, gpsSerial, gps, GPSavailable); IMUupdate(&localOdom, myimu, IMUavailable); }
    #else
      mockMechanicalData(&localMech);
      if (isFront) mockElectricalData(&localElect);
      if (isRear)  mockOdometryData(&localOdom);
    #endif

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      myMechData     = localMech;
      myElectData    = localElect;
      myOdometryData = localOdom;
      xSemaphoreGive(dataMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // 50Hz
  }
}

// Core 1: CAN Bus Task (BAMOCar polling)
void canTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long lastRequest = 0;
  uint8_t requestSeq = 0;
  bool n100Requested = false;
  twai_message_t txmsg, rxmsg;

  while (true) {
    unsigned long now = millis();
    if (!canBusReady) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    // One-shot N-100% readback to verify scaling matches BAMOCAR_N100_RPM
    if (!n100Requested) {
      request_BamocarN100(&txmsg);
      CAN32_sendCAN(&txmsg);
      Serial.println("[CAN TX] N-100% readback request (reg 0xC8)");
      n100Requested = true;
    }

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

    if (now - lastRequest >= BAMOCarREQ_INTERVAL) {
      switch (requestSeq) {
        case 0: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_MOTOR_TEMP); break;
        case 1: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_CONTROLLER_TEMP); break;
        case 2: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_DC_VOLTAGE); break;
        case 3: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_DC_CURRENT); break;
        case 4: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_SPEED_ACTUAL); break;
        case 5: pack_RequestBamocarMsg(&txmsg, BAMOCAR_REG_N100); break;
      }
      int txResult = CAN32_sendCAN(&txmsg);
      Serial.printf("[CAN TX] ID=0x%03X REG=0x%02X (seq=%d) status=%s\n",
        txmsg.identifier, txmsg.data[1], requestSeq,
        txResult == ESP_OK ? "OK" : "FAIL");
      requestSeq = (requestSeq + 1) % 6;
      lastRequest = now;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      myBAMOCar.power = myBAMOCar.canVoltage * myBAMOCar.canCurrent;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));  // 200Hz
  }
}

/************************* Setup ***************************/

void setup() {
  bool isFront = strcmp(clientName, "front") == 0;
  bool isRear  = strcmp(clientName, "rear")  == 0;

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(UART0_BAUD);
  // --- I2C Init: Wire1 for RTC, Wire for IMU
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  Wire1.setTimeout(2);
  // --- RTC Init
  RTCavailable = RTCinit(rtc, &Wire1);
  if (RTCavailable) {
    #if TIME_SRC == 0
    struct timeval tv = { .tv_sec = (time_t)rtc.now().unixtime(), .tv_usec = 0 };
    #elif TIME_SRC == 1
    struct timeval tv = { .tv_sec = (time_t)WiFi32_getNTPTime(), .tv_usec = 0 };
    #endif
    settimeofday(&tv, NULL);
    Serial.println("[RTC] ESP32 system clock set from DS3231");
  }
  // --- init every Rear sensors 
  if(isRear){
    StrokesensorInit(STR_Heave, STR_Roll);
    // Motion Sensor Init
    I2C2_connect = Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setTimeout(2);
    IMUavailable = IMUinit(&Wire, myimu);
    delay(1000);
    IMUcalibrate(myimu, IMUavailable);
    GPSavailable = GPSinit(gpsSerial, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);
  }
  // --- init every front sensors
  if(isFront){
    // Base Sensor Init
    StrokesensorInit(STR_Heave, STR_Roll);
    ElectSensorsInit(ElectPinArray);
  }
  
  // CAN Bus Init
  canBusReady = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN,
    TWAI_TIMING_CONFIG_500KBITS(), TWAI_FILTER_CONFIG_ACCEPT_ALL());

  // WiFi Init
  #if WIFI_ENABLED == 1
  initWiFi(ssid, password, 10);
  int ntpready;
  if (WiFi.status() == WL_CONNECTED) ntpready = WiFi32_initNTP();
  #else
  int ntpready = 0;
  Serial.println("[WiFi] Disabled (WIFI_ENABLED=0)");
  #endif

  // WebSocket Init
  #if WIFI_ENABLED == 1 && (WS_ENABLED == 1 || WS_ENABLED == 2)
  if (WiFi.status() == WL_CONNECTED) {
    BPMobile.setClientName(clientName);
    BPMobile.setRegisterCallback(registerClient);
    #if WS_ENABLED == 2
    BPMobile.initWebSocketSSL(serverHost, serverPort, clientName, DEFAULT_WS_PATH);
    #else
    BPMobile.initWebSocket(serverHost, serverPort, clientName, DEFAULT_WS_PATH);
    #endif
  }
  #elif WS_ENABLED == 0
  Serial.println("[WS] Disabled (WS_ENABLED=0)");
  #endif
  
  // FreeRTOS
  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  // Core 0
  #if WIFI_ENABLED == 1 && (WS_ENABLED == 1 || WS_ENABLED == 2)
  xTaskCreatePinnedToCore(BPMobileTask, "BPMobileTask", 8192, NULL, 1, &BPMobileTaskHandle, 0);
  Serial.println("[RTOS] BPMobile task on Core 0 (pri 1)");
  #else
  Serial.println("[RTOS] BPMobile task SKIPPED (WIFI_ENABLED=0 or WS_ENABLED=0)");
  #endif
  
  // Core 1 (Sensor, CAN, Time sync)
  xTaskCreatePinnedToCore(timeSyncTask, "TimeSyncTask", 4096, NULL, 3, &timeSyncTaskHandle, 0);
  Serial.println("[RTOS] TimeSync task on Core 0 (pri 3)");
  #if MOCK_FLAG == 0
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 5, &sensorTaskHandle, 1);
  Serial.println("[RTOS] Sensor task on Core 1 (pri 5)");
  if(isFront){
    xTaskCreatePinnedToCore(canTask, "CANTask", 4096, NULL, 5, &canTaskHandle, 1);
    Serial.println("[RTOS] CAN task on Core 1 (pri 5)");
  }
  #else
  Serial.println("[RTOS] Sensor/CAN tasks SKIPPED (MOCK_FLAG=1)");
  #endif
  // Core 1 (SD init, and pinned task)
  #if SD_ENABLED == 1
  SD32_initSDCard(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN, sdCardReady);
  if (sdCardReady) {
    SD32_createSessionDir(sessionNumber, sessionDirPath, "Node");
    SD32_generateFilenameInDir(csvFilename, sessionDirPath, "File", fileIndex);
    SD32_createCSVFile(csvFilename, CSV_HEADER);
    openLogFile(csvFilename);
  }
  sdQueue = xQueueCreate(30, sizeof(SDLogEntry)); // create Queue to share data between 
  xTaskCreatePinnedToCore(sdTask, "SDTask", 4096, NULL, 2, &sdTaskHandle, 1);
  Serial.println("[RTOS] SD Logger task on Core 1 (pri 2)");
  #else
  Serial.println("[SD] Disabled (SD_ENABLED=0)");
  Serial.println("[RTOS] SD Logger task SKIPPED (SD_ENABLED=0)");
  #endif
  // Time calibration
  #if calibrate_RTC == 1
    RTCcalibrate(rtc, (ntpready) ? (WiFi32_getNTPTime() / 1000ULL) : 1000000000000ULL, RTCavailable);
  #endif
  syncTime_setSyncPoint(RTC_UNIX_TIME,
    (RTCavailable) ? (uint64_t)RTC_getUnix(rtc, RTCavailable) * 1000ULL : 1000000000000ULL);
  Serial.println("==================================================");
  Serial.println("BP Bridge Sensor Node - Unified - Ready");
  Serial.println("==================================================");
  Serial.printf("Client: %s\n\n", clientName);
}

/************************* Main Loop ***************************/

void loop() {
  uint64_t SESSION_TIME_MS = millis();
  uint64_t CURRENT_UNIX_TIME_MS = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  // Time calibration debug display
  #if calibrate_RTC == 1
    char timeBuf[32];
    syncTime_formatUnix(timeBuf, CURRENT_UNIX_TIME_MS, 7);
    Serial.println(timeBuf);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  #endif

  // Debug mode load data from semaphores
  #if DEBUG_MODE > 0
  if (SESSION_TIME_MS - lastTeleplotDebug >= TELEPLOT_DEBUG_INTERVAL) {
    Mechanical debugMech;
    Electrical debugElect;
    Odometry   debugOdom;
    BAMOCar    debugBAMO;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      debugMech  = myMechData;
      debugElect = myElectData;
      debugOdom  = myOdometryData;
      debugBAMO  = myBAMOCar;
      xSemaphoreGive(dataMutex);
    }
    #if DEBUG_MODE == 1
      Serial.printf("[DEBUG] Mech Heave=%.1f Roll=%.1f  Elect I=%.2f APPS=%.1f  GPS(%.2f,%.2f)  BAMO V=%.1f I=%.1f\n",
        debugMech.STR_Heave_mm, debugMech.STR_Roll_mm,
        debugElect.I_SENSE, debugElect.APPS,
        debugOdom.gps_lat, debugOdom.gps_lng,
        debugBAMO.canVoltage, debugBAMO.canCurrent);
    #elif DEBUG_MODE == 2
      teleplotMechanical(&debugMech);
      teleplotElectrical(&debugElect);
      teleplotMotion(&debugOdom);
      teleplotBAMOCar(&debugBAMO);
    #endif
    lastTeleplotDebug = SESSION_TIME_MS;
  }
  #endif

  // press ` to read device status , press ~ to exit
  if (Serial.available() && Serial.peek() == '`') {
    Serial.read();
    while (1) {
      showDeviceStatus();
      if (Serial.available() && Serial.read() == '~') break;
      delay(200);
    }
  }

  // Mock
  #if MOCK_FLAG == 1
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      mockMechanicalData(&myMechData);
      if (strcmp(clientName, "front") == 0) {
        mockElectricalData(&myElectData);
        mockBAMOCarData(&myBAMOCar);
      }
      if (strcmp(clientName, "rear") == 0) {
        mockOdometryData(&myOdometryData);
      }
      xSemaphoreGive(dataMutex);
    }
  #endif
  
  // SD queuing
  #if SD_ENABLED == 1
  if (sdCardReady && !SD32_checkSDconnect()) {
    closeLogFile();
    sdCardReady = false;
    Serial.println("[SD] Card removed");
  }

  if (sdCardReady && sdQueue != NULL && (SESSION_TIME_MS - lastSDLog >= SD_APPEND_INTERVAL)) {
    SDLogEntry entry;
    entry.dataPoint   = dataPoint;
    entry.unixTime    = CURRENT_UNIX_TIME_MS;
    entry.sessionTime = SESSION_TIME_MS;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      entry.mech  = myMechData;
      entry.elect = myElectData;
      entry.odom  = myOdometryData;
      entry.bamo  = myBAMOCar;
      xSemaphoreGive(dataMutex);
    }

    if (xQueueSend(sdQueue, &entry, 0) == pdTRUE) {
      dataPoint++;
    } else {
      Serial.println("[SD] Queue full - data dropped");
    }
    lastSDLog = SESSION_TIME_MS;
  }
  #endif

  vTaskDelay(pdMS_TO_TICKS(20));
}

/************************* BPMobile Publishers ***************************/

void publishMechData(Mechanical* m) {
  uint64_t ts = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"type\":\"data\",\"node\":\"%s\",\"group\":\"mech\",\"ts\":%llu,"
    "\"d\":{\"Wheel_RPM_L\":%.4f,\"Wheel_RPM_R\":%.4f,"
    "\"STR_Heave_mm\":%.4f,\"STR_Roll_mm\":%.4f}}",
    clientName, ts,
    m->Wheel_RPM_L, m->Wheel_RPM_R, m->STR_Heave_mm, m->STR_Roll_mm);
  if (n > 0) BPwebSocket->sendTXT(buf, n);
}

void publishElectData(Electrical* e) {
  uint64_t ts = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"type\":\"data\",\"node\":\"%s\",\"group\":\"elect\",\"ts\":%llu,"
    "\"d\":{\"I_SENSE\":%.4f,\"TMP\":%.4f,\"APPS\":%.4f,\"BPPS\":%.4f,\"steering\":%.4f}}",
    clientName, ts,
    e->I_SENSE, e->TMP, e->APPS, e->BPPS, e->steering);
  if (n > 0) BPwebSocket->sendTXT(buf, n);
}

void publishElectFaultState(Electrical* e) {
  uint64_t ts = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"type\":\"data\",\"node\":\"%s\",\"group\":\"faults\",\"ts\":%llu,"
    "\"d\":{\"AMS_OK\":%s,\"IMD_OK\":%s,\"HV_ON\":%s,\"BSPD_OK\":%s}}",
    clientName, ts,
    e->AMS_OK  ? "true" : "false",
    e->IMD_OK  ? "true" : "false",
    e->HV_ON   ? "true" : "false",
    e->BSPD_OK ? "true" : "false");
  if (n > 0) BPwebSocket->sendTXT(buf, n);
}

void publishOdometryData(Odometry* o) {
  uint64_t ts = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  char buf[768];
  int n = snprintf(buf, sizeof(buf),
    "{\"type\":\"data\",\"node\":\"%s\",\"group\":\"odom\",\"ts\":%llu,"
    "\"d\":{"
    "\"gps_lat\":%.6f,\"gps_lng\":%.6f,\"gps_age\":%.2f,"
    "\"gps_course\":%.2f,\"gps_speed\":%.2f,"
    "\"imu_accel_x\":%.4f,\"imu_accel_y\":%.4f,\"imu_accel_z\":%.4f,"
    "\"imu_gyro_x\":%.4f,\"imu_gyro_y\":%.4f,\"imu_gyro_z\":%.4f,"
    "\"imu_euler_roll\":%.4f,\"imu_euler_pitch\":%.4f,\"imu_euler_yaw\":%.4f,"
    "\"imu_mag_x\":%.4f,\"imu_mag_y\":%.4f,\"imu_mag_z\":%.4f,"
    "\"imu_grav_x\":%.4f,\"imu_grav_y\":%.4f,\"imu_grav_z\":%.4f}}",
    clientName, ts,
    o->gps_lat, o->gps_lng, o->gps_age, o->gps_course, o->gps_speed,
    o->imu_accelx, o->imu_accely, o->imu_accelz,
    o->imu_gyrox,  o->imu_gyroy,  o->imu_gyroz,
    o->imu_euler_roll, o->imu_euler_pitch, o->imu_euler_yaw,
    o->imu_magx,  o->imu_magy,  o->imu_magz,
    o->imu_gravx, o->imu_gravy, o->imu_gravz);
  if (n > 0) BPwebSocket->sendTXT(buf, n);
}

void publishBAMOpower(BAMOCar* b) {
  uint64_t ts = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"type\":\"data\",\"group\":\"bamo.power\",\"ts\":%llu,"
    "\"d\":{\"canVoltage\":%.4f,\"canCurrent\":%.4f,\"power\":%.4f,\"rpm\":%.0f}}",
    ts, b->canVoltage, b->canCurrent, b->power, b->rpm);
  if (n > 0) BPwebSocket->sendTXT(buf, n);
}

void publishBAMOtemp(BAMOCar* b) {
  uint64_t ts = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  char buf[192];
  int n = snprintf(buf, sizeof(buf),
    "{\"type\":\"data\",\"group\":\"bamo.temp\",\"ts\":%llu,"
    "\"d\":{\"motorTemp\":%.2f,\"controllerTemp\":%.2f}}",
    ts, b->motorTemp2, b->controllerTemp);
  if (n > 0) BPwebSocket->sendTXT(buf, n);
}

void registerClient(const char* clientName) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["node"] = clientName;
  doc["client_name"] = clientName;

  JsonArray groups = doc["groups"].to<JsonArray>();
  JsonObject g1 = groups.add<JsonObject>(); g1["group"] = "mech";       g1["rate_hz"] = MECH_SENSORS_SAMPLING_RATE;
  JsonObject g2 = groups.add<JsonObject>(); g2["group"] = "elect";      g2["rate_hz"] = ELECT_SENSORS_SAMPLING_RATE;
  JsonObject g3 = groups.add<JsonObject>(); g3["group"] = "faults";     g3["rate_hz"] = ELECT_FAULT_SAMPLING_RATE;
  JsonObject g4 = groups.add<JsonObject>(); g4["group"] = "odom";       g4["rate_hz"] = ODOM_SENSORS_SAMPLING_RATE;
  JsonObject g5 = groups.add<JsonObject>(); g5["group"] = "bamo.power"; g5["rate_hz"] = BAMO_POWER_SAMPLING_RATE;
  JsonObject g6 = groups.add<JsonObject>(); g6["group"] = "bamo.temp";  g6["rate_hz"] = BAMO_TEMP_SAMPLING_RATE;

  JsonArray schema = doc["schema"].to<JsonArray>();
  auto addEntry = [&](const char* key, const char* type, const char* unit, const char* group, float scale = 1, float offset = 0) {
    JsonObject e = schema.add<JsonObject>();
    e["key"] = key; e["type"] = type; e["unit"] = unit; e["scale"] = scale; e["offset"] = offset; e["group"] = group;
  };

  addEntry("mech.Wheel_RPM_L",  "float",  "RPM",  "mech");
  addEntry("mech.Wheel_RPM_R",  "float",  "RPM",  "mech");
  addEntry("mech.STR_Heave_mm", "float",  "mm",   "mech");
  addEntry("mech.STR_Roll_mm",  "float",  "mm",   "mech");
  addEntry("elect.I_SENSE",     "float",  "A",    "elect");
  addEntry("elect.TMP",         "float",  "C",    "elect");
  addEntry("elect.APPS",        "float",  "%",    "elect");
  addEntry("elect.BPPS",        "float",  "%",    "elect");
  addEntry("elect.steering",    "float",  "deg",  "elect");
  addEntry("faults.AMS_OK",     "bool",   "",     "faults");
  addEntry("faults.IMD_OK",     "bool",   "",     "faults");
  addEntry("faults.HV_ON",      "bool",   "",     "faults");
  addEntry("faults.BSPD_OK",    "bool",   "",     "faults");
  addEntry("odom.gps_lat",      "double", "deg",  "odom");
  addEntry("odom.gps_lng",      "double", "deg",  "odom");
  addEntry("odom.gps_age",      "double", "ms",   "odom");
  addEntry("odom.gps_course",   "double", "deg",  "odom");
  addEntry("odom.gps_speed",    "double", "km/h", "odom");
  addEntry("odom.imu_accel_x",  "float",  "m/s2", "odom");
  addEntry("odom.imu_accel_y",  "float",  "m/s2", "odom");
  addEntry("odom.imu_accel_z",  "float",  "m/s2", "odom");
  addEntry("odom.imu_gyro_x",   "float",  "deg/s","odom");
  addEntry("odom.imu_gyro_y",   "float",  "deg/s","odom");
  addEntry("odom.imu_gyro_z",   "float",  "deg/s","odom");
  addEntry("odom.imu_euler_roll",  "float", "deg", "odom");
  addEntry("odom.imu_euler_pitch", "float", "deg", "odom");
  addEntry("odom.imu_euler_yaw",   "float", "deg", "odom");
  addEntry("odom.imu_mag_x",    "float", "uT",    "odom");
  addEntry("odom.imu_mag_y",    "float", "uT",    "odom");
  addEntry("odom.imu_mag_z",    "float", "uT",    "odom");
  addEntry("odom.imu_grav_x",   "float", "m/s2",  "odom");
  addEntry("odom.imu_grav_y",   "float", "m/s2",  "odom");
  addEntry("odom.imu_grav_z",   "float", "m/s2",  "odom");
  addEntry("bamo.power.canVoltage",      "float", "V", "bamo.power");
  addEntry("bamo.power.canCurrent",      "float", "A", "bamo.power");
  addEntry("bamo.power.power",           "float", "W", "bamo.power");
  addEntry("bamo.power.rpm",             "float", "RPM", "bamo.power");
  addEntry("bamo.temp.motorTemp",        "float", "C", "bamo.temp");
  addEntry("bamo.temp.controllerTemp",   "float", "C", "bamo.temp");

  String registration;
  serializeJson(doc, registration);
  Serial.println("[WS] Sending registration...");
  BPwebSocket->sendTXT(registration);
}

/************************* Debug Functions ***************************/

void showDeviceStatus() {
  Serial.println("╔═════════════════════════════════════════════╗");
  Serial.println("║        UNIFIED NODE - SYSTEM STATUS         ║");
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.printf("║ I2C1 (RTC):   %s\n", I2C1_connect ? "OK" : "FAIL");
  Serial.printf("║ I2C2 (IMU):   %s\n", I2C2_connect ? "OK" : "FAIL");
  Serial.printf("║ CAN Bus:      %s\n", canBusReady ? "OK" : "FAIL");
  Serial.printf("║ SD Card:      %s\n", sdCardReady ? "OK" : "FAIL");
  Serial.printf("║ WiFi:         %s (RSSI: %d)\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL", WiFi.RSSI());
  Serial.printf("║ RTC:          %s\n", RTCavailable ? "OK" : "FAIL");
  Serial.printf("║ IMU:          %s\n", IMUavailable ? "OK" : "FAIL");
  Serial.printf("║ GPS:          %s\n", gpsSerial.available() > 0 ? "OK" : "FAIL");
  Serial.printf("║ WebSocket:    %s\n", BPsocketstatus->isConnected ? "OK" : "FAIL");
  Serial.printf("║ Time Sync:    %s\n", syncTime_isSynced() ? "OK" : "FAIL");
  Serial.println("╚═════════════════════════════════════════════╝");
}
