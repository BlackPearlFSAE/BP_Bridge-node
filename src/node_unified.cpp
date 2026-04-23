/* BP Bridge Sensor Node - Unified
 *
 * Single hardwired firmware for Front, Rear, and BAMO boards. Every sensor
 * subsystem (stroke, electrical, GPS, IMU, BAMOCar CAN) is initialized at
 * boot; whichever hardware is physically wired returns real values, the
 * rest stay zero. node_ams is a separate env and source.
 *
 * - Core 0: WiFi/WebSocket (BPMobile), time sync
 * - Core 1: Sensor polling, CAN request/response, SD logging
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
#include <driver/twai.h>
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

/************************* Absorbed: base_sensors ***************************/

typedef struct {
  float Wheel_RPM_L  = 0.0;
  float Wheel_RPM_R  = 0.0;
  float STR_Heave_mm = 0.0;
  float STR_Roll_mm  = 0.0;
} Mechanical;

typedef struct {
  float I_SENSE = 0.0;
  float TMP     = 0.0;
  float APPS    = 0.0;
  float BPPS    = 0.0;
  bool  AMS_OK  = 0;
  bool  IMD_OK  = 0;
  bool  HV_ON   = 0;
  bool  BSPD_OK = 0;
  float steering = 0.0;
} Electrical;

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

static void StrokesensorInit(int Heave, int Roll) {
  pinMode(Heave, INPUT_PULLDOWN);
  pinMode(Roll,  INPUT_PULLDOWN);
}

static void StrokesensorUpdate(Mechanical* m, int Heave, int Roll) {
  analogRead(Heave);
  m->STR_Heave_mm = (float)(analogRead(Heave) * (max_distance1 / pwmres));
  analogRead(Roll);
  m->STR_Roll_mm  = (float)(analogRead(Roll)  * (max_distance2 / pwmres));
}

static void ElectSensorsInit(int* pins) {
  for (int i = 0; i < 9; i++) pinMode(pins[i], INPUT_PULLDOWN);
}

static void ElectSensorsUpdate(Electrical* e, int* pins) {
  uint16_t raw_i_sense  = analogRead(pins[0]);
  uint16_t raw_tmp      = analogRead(pins[1]);
  uint16_t raw_apps     = analogRead(pins[2]);
  uint16_t raw_bpps     = analogRead(pins[3]);
  uint16_t raw_steering = analogRead(pins[8]);

  float volt_i_sense  = (raw_i_sense  / (float)pwmres) * aref;
  float volt_tmp      = (raw_tmp      / (float)pwmres) * aref;
  float volt_bpps     = (raw_bpps     / (float)pwmres) * aref;
  float volt_steering = (raw_steering / (float)pwmres) * aref;

  e->I_SENSE = (volt_i_sense - i_sense_offset) / i_sense_sensitivity;

  if (volt_tmp < aref - 0.01) {
    uint16_t ntc_resistance = (volt_tmp * tmp_series_res) / (aref - volt_tmp);
    e->TMP = (float)ntc_resistance;
  } else {
    e->TMP = 0.0;
  }

  e->APPS = raw_apps;
  e->BPPS = (volt_bpps - bpps_offset_v) / max_volt5;
  e->steering = ((volt_steering / steering_aref) * steering_max__angle) - steering_offset_angle;

  e->AMS_OK  = digitalRead(pins[4]);
  e->IMD_OK  = digitalRead(pins[5]);
  e->HV_ON   = digitalRead(pins[6]);
  e->BSPD_OK = digitalRead(pins[7]);
}

static void mockMechanicalData(Mechanical* m) {
  m->Wheel_RPM_L  = 250.0 + random(-50, 50);
  m->Wheel_RPM_R  = 245.0 + random(-50, 50);
  m->STR_Heave_mm = 35.0 + random(-10, 10);
  m->STR_Roll_mm  = 40.0 + random(-10, 10);
}

static void mockElectricalData(Electrical* e) {
  e->I_SENSE = 5.0 + random(-20, 80) / 10.0;
  e->TMP     = 45.0 + random(-10, 20);
  e->APPS    = 30.0 + random(-15, 30);
  e->BPPS    = 10.0 + random(-5, 15);
  e->AMS_OK  = random(0, 10) > 1;
  e->IMD_OK  = random(0, 10) > 1;
  e->HV_ON   = random(0, 10) > 2;
  e->BSPD_OK = random(0, 10) > 1;
  e->steering = random(0, 10) > 1;
}

static void teleplotMechanical(Mechanical* m) {
  Serial.printf(">Wheel_RPM_L:%.2f\n", m->Wheel_RPM_L);
  Serial.printf(">Wheel_RPM_R:%.2f\n", m->Wheel_RPM_R);
  Serial.printf(">STR_Heave_mm:%.2f\n", m->STR_Heave_mm);
  Serial.printf(">STR_Roll_mm:%.2f\n", m->STR_Roll_mm);
}

static void teleplotElectrical(Electrical* e) {
  Serial.printf(">I_SENSE:%.2f\n", e->I_SENSE);
  Serial.printf(">TMP:%.2f\n", e->TMP);
  Serial.printf(">APPS:%.2f\n", e->APPS);
  Serial.printf(">BPPS:%.2f\n", e->BPPS);
  Serial.printf(">AMS_OK:%d\n", e->AMS_OK ? 1 : 0);
  Serial.printf(">IMD_OK:%d\n", e->IMD_OK ? 1 : 0);
  Serial.printf(">HV_ON:%d\n", e->HV_ON ? 1 : 0);
  Serial.printf(">BSPD_OK:%d\n", e->BSPD_OK ? 1 : 0);
  Serial.printf(">STEERING:%.2f\n", e->steering);
}

/************************* Absorbed: motion_sensors ***************************/

typedef struct {
  double gps_lat    = 0;
  double gps_lng    = 0;
  double gps_age    = 0;
  double gps_course = 0;
  double gps_speed  = 0;
  float imu_accelx = 0;
  float imu_accely = 0;
  float imu_accelz = 0;
  float imu_gyrox  = 0;
  float imu_gyroy  = 0;
  float imu_gyroz  = 0;
  float imu_euler_roll  = 0;
  float imu_euler_pitch = 0;
  float imu_euler_yaw   = 0;
  float imu_magx = 0;
  float imu_magy = 0;
  float imu_magz = 0;
  float imu_gravx = 0;
  float imu_gravy = 0;
  float imu_gravz = 0;
} Odometry;

static bool GPSinit(HardwareSerial& gpsSerial, int tx, int rx, uint32_t baud) {
  gpsSerial.begin(baud, SERIAL_8N1, rx, tx);
  if (!gpsSerial) return false;
  Serial.println("AN251 GPS Module with TinyGPS++");
  Serial.println("Waiting for GPS fix...");
  return true;
}

static void GPSupdate(Odometry* o, HardwareSerial& gpsSerial, TinyGPSPlus& gps, bool& available) {
  if (!available) return;
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        o->gps_lat    = gps.location.lat();
        o->gps_lng    = gps.location.lng();
        o->gps_age    = gps.location.age();
        o->gps_course = gps.course.deg();
        o->gps_speed  = gps.speed.mps();
      }
    }
  }
}

static bool IMUinit([[maybe_unused]] TwoWire* WireIMU, Adafruit_BNO055& bno) {
  if (!bno.begin()) {
    Serial.println(F("BNO055 not detected. Check wiring or I2C ADDR (0x28/0x29)."));
    return false;
  }
  Serial.println(F("BNO055 connected and configured."));
  delay(1000);
  return true;
}

static void IMUcalibrate(Adafruit_BNO055& bno, bool& available) {
  if (!available) return;
  Serial.println("Reading BNO055 calibration status, move sensor through its ranges");
  uint8_t system_s, gyro_s, accel_s, mag_s = 0;
  bno.getCalibration(&system_s, &gyro_s, &accel_s, &mag_s);
  Serial.printf("Calibration: Sys=%u Gyro=%u Accel=%u Mag=%u\n", system_s, gyro_s, accel_s, mag_s);
}

static void IMUupdate(Odometry* o, Adafruit_BNO055& bno, bool& available) {
  if (!available) return;
  sensors_event_t linA, ang, ori, mag, grav;
  bno.getEvent(&linA, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&ang,  Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&ori,  Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&mag,  Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&grav, Adafruit_BNO055::VECTOR_GRAVITY);
  o->imu_accelx = linA.acceleration.x;
  o->imu_accely = linA.acceleration.y;
  o->imu_accelz = linA.acceleration.z;
  o->imu_gyrox  = ang.gyro.x;
  o->imu_gyroy  = ang.gyro.y;
  o->imu_gyroz  = ang.gyro.z;
  // BNO055 NED/left-hand convention: negate all to match right-hand rule.
  // Register order: x=heading(yaw), y=BNO-roll(->vehicle pitch), z=BNO-pitch(->vehicle roll)
  o->imu_euler_yaw   = -ori.orientation.x;
  o->imu_euler_pitch = -ori.orientation.y;
  o->imu_euler_roll  = -ori.orientation.z;
  o->imu_magx = mag.magnetic.x;
  o->imu_magy = mag.magnetic.y;
  o->imu_magz = mag.magnetic.z;
  o->imu_gravx = grav.acceleration.x;
  o->imu_gravy = grav.acceleration.y;
  o->imu_gravz = grav.acceleration.z;
}

static void mockOdometryData(Odometry* o) {
  o->gps_lat = 13.7563 + random(-100, 100) / 10000.0;
  o->gps_lng = 100.5018 + random(-100, 100) / 10000.0;
  o->gps_age = random(0, 1000);
  o->gps_course = random(0, 360);
  o->gps_speed = random(0, 50);
  o->imu_accelx = random(-100, 100) / 50.0;
  o->imu_accely = random(-100, 100) / 50.0;
  o->imu_accelz = 9.81 + random(-50, 50) / 100.0;
  o->imu_gyrox = random(-50, 50) / 10.0;
  o->imu_gyroy = random(-50, 50) / 10.0;
  o->imu_gyroz = random(-50, 50) / 10.0;
  o->imu_euler_roll  = random(-1800, 1800) / 10.0;
  o->imu_euler_pitch = random(-900, 900) / 10.0;
  o->imu_euler_yaw   = random(0, 3600) / 10.0;
  o->imu_magx = random(-500, 500) / 10.0;
  o->imu_magy = random(-500, 500) / 10.0;
  o->imu_magz = random(-500, 500) / 10.0;
  o->imu_gravx = random(-100, 100) / 100.0;
  o->imu_gravy = random(-100, 100) / 100.0;
  o->imu_gravz = 9.81 + random(-50, 50) / 100.0;
}

static void teleplotMotion(Odometry* o) {
  Serial.printf(">GPS_Lat:%.4f\n", o->gps_lat);
  Serial.printf(">GPS_Lng:%.4f\n", o->gps_lng);
  Serial.printf(">GPS_Speed:%.2f\n", o->gps_speed);
  Serial.printf(">GPS_Course:%.2f\n", o->gps_course);
  Serial.printf(">IMU_AccelX:%.2f\n", o->imu_accelx);
  Serial.printf(">IMU_AccelY:%.2f\n", o->imu_accely);
  Serial.printf(">IMU_AccelZ:%.2f\n", o->imu_accelz);
  Serial.printf(">IMU_GyroX:%.2f\n", o->imu_gyrox);
  Serial.printf(">IMU_GyroY:%.2f\n", o->imu_gyroy);
  Serial.printf(">IMU_GyroZ:%.2f\n", o->imu_gyroz);
  Serial.printf(">IMU_EulerRoll:%.2f\n",  o->imu_euler_roll);
  Serial.printf(">IMU_EulerPitch:%.2f\n", o->imu_euler_pitch);
  Serial.printf(">IMU_EulerYaw:%.2f\n",   o->imu_euler_yaw);
  Serial.printf(">IMU_MagX:%.2f\n", o->imu_magx);
  Serial.printf(">IMU_MagY:%.2f\n", o->imu_magy);
  Serial.printf(">IMU_MagZ:%.2f\n", o->imu_magz);
  Serial.printf(">IMU_GravX:%.2f\n", o->imu_gravx);
  Serial.printf(">IMU_GravY:%.2f\n", o->imu_gravy);
  Serial.printf(">IMU_GravZ:%.2f\n", o->imu_gravz);
}

/************************* Absorbed: bamo_helper ***************************/

#define BAMOCAR_REQUEST_ID 0x201
#define BAMOCAR_RESPONSE_ID 0x181
#define BAMOCAR_REG_MOTOR_TEMP 0x49
#define BAMOCAR_REG_CONTROLLER_TEMP 0x4A
#define BAMOCAR_REG_DC_VOLTAGE 0xEB
#define BAMOCAR_REG_DC_CURRENT 0x20

typedef struct {
  float motorTemp1 = 0.0;
  float motorTemp2 = 0.0;
  float controllerTemp = 0.0;
  float canVoltage = 0.0;
  float canCurrent = 0.0;
  float power = 0.0;
  bool motorTempValid = false;
  bool controllerTempValid = false;
  bool canVoltageValid = false;
  bool canCurrentValid = false;
} BAMOCar;

static float convertNTCtoTemp(uint16_t rawValue) {
  const int numPoints = 28;
  const int tempTable[numPoints] = {
    -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
    35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
  };
  const int valueTable[numPoints] = {
    16245, 16308, 16387, 16487, 16609, 16759, 16938, 17151, 17400, 17688, 18017, 18387, 18797, 19247,
    19733, 20250, 20793, 21357, 21933, 22515, 23097, 23671, 24232, 24775, 25296, 25792, 26261, 26702
  };
  if (rawValue <= valueTable[0]) return tempTable[0];
  if (rawValue >= valueTable[numPoints - 1]) return tempTable[numPoints - 1];
  for (int i = 0; i < numPoints - 1; i++) {
    if (rawValue >= valueTable[i] && rawValue <= valueTable[i + 1]) {
      float t1 = tempTable[i];
      float t2 = tempTable[i + 1];
      float v1 = valueTable[i];
      float v2 = valueTable[i + 1];
      return t1 + (rawValue - v1) * (t2 - t1) / (v2 - v1);
    }
  }
  return 25.0;
}

static void pack_RequestBamocarMsg(twai_message_t* msg, uint8_t regAddress) {
  msg->identifier = BAMOCAR_REQUEST_ID;
  msg->extd = 0;
  msg->rtr = 0;
  msg->data_length_code = 3;
  msg->data[0] = 0x3D;
  msg->data[1] = regAddress;
  msg->data[2] = 0x00;
}

static void process_ResponseBamocarMsg(twai_message_t* msg, BAMOCar* bamocar) {
  if (msg->identifier == 0x181 && msg->data_length_code >= 3) {
    uint8_t regAddress = msg->data[0];
    uint16_t rawValue = (msg->data[2] << 8) | msg->data[1];

    if (regAddress == BAMOCAR_REG_MOTOR_TEMP) {
      bamocar->motorTemp1 = (float)msg->data[1] / 10.0;
      bamocar->motorTemp2 = bamocar->motorTemp1 + 8.0;
      bamocar->motorTempValid = true;
      Serial.print("[CAN DECODED] Motor Temp: ");
      Serial.print(bamocar->motorTemp2, 1);
      Serial.print(" °C (raw byte: 0x");
      Serial.print(msg->data[1], HEX);
      Serial.print(" = ");
      Serial.print(msg->data[1]);
      Serial.println(")");
    }
    else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) {
      bamocar->controllerTemp = convertNTCtoTemp(rawValue);
      bamocar->controllerTempValid = true;
      Serial.print("[CAN DECODED] Controller Temp: ");
      Serial.print(bamocar->controllerTemp, 1);
      Serial.print(" °C (resistance: ");
      Serial.print(rawValue);
      Serial.println(" Ω)");
    }
    else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) {
      bamocar->canVoltage = (float)rawValue / 55.1204;
      bamocar->canVoltageValid = true;
      Serial.print("[CAN DECODED] DC Voltage: ");
      Serial.print(bamocar->canVoltage, 2);
      Serial.print(" V (raw: ");
      Serial.print(rawValue);
      Serial.println(")");
    }
    else if (regAddress == BAMOCAR_REG_DC_CURRENT) {
      if (rawValue == 0xFFFF || rawValue > 60000) {
        Serial.println("[CAN DECODED] DC Current: INVALID (0xFFFF) - ignored");
      } else {
        bamocar->canCurrent = (float)rawValue * 0.373832;
        bamocar->canCurrentValid = true;
        Serial.print("[CAN DECODED] DC Current: ");
        Serial.print(bamocar->canCurrent, 2);
        Serial.print(" A (raw: ");
        Serial.print(rawValue);
        Serial.println(")");
      }
    }
    else {
      Serial.print("[CAN] Unknown register 0x");
      Serial.print(regAddress, HEX);
      Serial.print(" with value: ");
      Serial.println(rawValue);
    }
  }
}

static void mockBAMOCarData(BAMOCar* b) {
  b->motorTemp1 = 55.0 + random(-10, 15);
  b->motorTemp2 = b->motorTemp1 + random(5, 10);
  b->motorTempValid = true;
  b->controllerTemp = 50.0 + random(-10, 15);
  b->controllerTempValid = true;
  b->canVoltage = 220.0 + random(-20, 20);
  b->canVoltageValid = true;
  b->canCurrent = 30.0 + random(-10, 50);
  b->canCurrentValid = true;
  b->power = b->canVoltage * b->canCurrent;
}

static void teleplotBAMOCar(BAMOCar* b) {
  Serial.printf(">BAMO_Voltage:%.2f\n", b->canVoltage);
  Serial.printf(">BAMO_Current:%.2f\n", b->canCurrent);
  Serial.printf(">BAMO_Power:%.2f\n", b->power);
  Serial.printf(">BAMO_MotorTemp:%.1f\n", b->motorTemp2);
  Serial.printf(">BAMO_CtrlTemp:%.1f\n", b->controllerTemp);
}

/************************* Global Variables ***************************/

// WebSocket and Network config
const char* ssid = DEFAULT_SSID;
const char* password = DEFAULT_PASSWORD;
const char* serverHost = DEFAULT_SERVER_HOST;
const int   serverPort = DEFAULT_SERVER_PORT;
const char* clientName = "Node";
WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus*     BPsocketstatus = BPMobile.webSocketstatus;

// Sampling Rates (Hz)
const float MECH_SENSORS_SAMPLING_RATE      = DEFAULT_PUBLISH_RATE;
const float ELECT_SENSORS_SAMPLING_RATE     = DEFAULT_PUBLISH_RATE;
const float ELECT_FAULT_STAT_SAMPLING_RATE  = DEFAULT_PUBLISH_RATE;
const float ODOM_SENSORS_SAMPLING_RATE      = DEFAULT_PUBLISH_RATE;
const float BAMO_POWER_SAMPLING_RATE        = DEFAULT_PUBLISH_RATE;
const float BAMO_TEMP_SAMPLING_RATE         = DEFAULT_PUBLISH_RATE;

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
  "Wheel_RPM_Left,Wheel_RPM_Right,Heave_mm,Roll_mm,"
  "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK,Steering angle,"
  "GPS_Lat,GPS_Lng,GPS_Age,GPS_Course,GPS_Speed,"
  "IMU_AccelX,IMU_AccelY,IMU_AccelZ,"
  "IMU_GyroX,IMU_GyroY,IMU_GyroZ,"
  "IMU_EulerRoll,IMU_EulerPitch,IMU_EulerYaw,"
  "IMU_MagX,IMU_MagY,IMU_MagZ,"
  "IMU_GravX,IMU_GravY,IMU_GravZ,"
  "BAMO_Volt,BAMO_Amp,BAMO_Power,BAMO_MotorTemp,BAMO_ControllerTemp";

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

/************************* CSV Row Writer ***************************/

// Single appender: prints an entire SDLogEntry as one CSV row.
// SD32_appendBulkDataPersistent handles comma between appenders and trailing
// println, but with count=1 we emit every comma ourselves except the last.
static void writeWholeRow(File& f, void* data) {
  SDLogEntry* e = static_cast<SDLogEntry*>(data);

  f.print(e->dataPoint);        f.print(',');
  f.print(e->unixTime);         f.print(',');
  f.print(e->sessionTime);      f.print(',');

  f.print(e->mech.Wheel_RPM_L, 2);  f.print(',');
  f.print(e->mech.Wheel_RPM_R, 2);  f.print(',');
  f.print(e->mech.STR_Heave_mm, 2); f.print(',');
  f.print(e->mech.STR_Roll_mm, 2);  f.print(',');

  f.print(e->elect.I_SENSE, 2); f.print(',');
  f.print(e->elect.TMP, 2);     f.print(',');
  f.print(e->elect.APPS, 2);    f.print(',');
  f.print(e->elect.BPPS, 2);    f.print(',');
  f.print(e->elect.AMS_OK  ? 1 : 0); f.print(',');
  f.print(e->elect.IMD_OK  ? 1 : 0); f.print(',');
  f.print(e->elect.HV_ON   ? 1 : 0); f.print(',');
  f.print(e->elect.BSPD_OK ? 1 : 0); f.print(',');
  f.print(e->elect.steering, 2);     f.print(',');

  f.print(e->odom.gps_lat, 4);    f.print(',');
  f.print(e->odom.gps_lng, 4);    f.print(',');
  f.print(e->odom.gps_age, 2);    f.print(',');
  f.print(e->odom.gps_course, 2); f.print(',');
  f.print(e->odom.gps_speed, 2);  f.print(',');
  f.print(e->odom.imu_accelx, 2); f.print(',');
  f.print(e->odom.imu_accely, 2); f.print(',');
  f.print(e->odom.imu_accelz, 2); f.print(',');
  f.print(e->odom.imu_gyrox, 2);  f.print(',');
  f.print(e->odom.imu_gyroy, 2);  f.print(',');
  f.print(e->odom.imu_gyroz, 2);  f.print(',');
  f.print(e->odom.imu_euler_roll, 2);  f.print(',');
  f.print(e->odom.imu_euler_pitch, 2); f.print(',');
  f.print(e->odom.imu_euler_yaw, 2);   f.print(',');
  f.print(e->odom.imu_magx, 2); f.print(',');
  f.print(e->odom.imu_magy, 2); f.print(',');
  f.print(e->odom.imu_magz, 2); f.print(',');
  f.print(e->odom.imu_gravx, 2); f.print(',');
  f.print(e->odom.imu_gravy, 2); f.print(',');
  f.print(e->odom.imu_gravz, 2); f.print(',');

  f.print(e->bamo.canVoltageValid ? e->bamo.canVoltage : 0.0f, 2); f.print(',');
  f.print(e->bamo.canCurrentValid ? e->bamo.canCurrent : 0.0f, 2); f.print(',');
  f.print(e->bamo.power, 2); f.print(',');
  f.print(e->bamo.motorTempValid      ? e->bamo.motorTemp2    : 0.0f, 1); f.print(',');
  f.print(e->bamo.controllerTempValid ? e->bamo.controllerTemp : 0.0f, 1);
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

      if (now - tMech       >= (1000.0 / MECH_SENSORS_SAMPLING_RATE))      { publishMechData(&localMech);          tMech = now; }
      if (now - tElect      >= (1000.0 / ELECT_SENSORS_SAMPLING_RATE))     { publishElectData(&localElect);        tElect = now; }
      if (now - tElectFault >= (1000.0 / ELECT_FAULT_STAT_SAMPLING_RATE))  { publishElectFaultState(&localElect);  tElectFault = now; }
      if (now - tOdom       >= (1000.0 / ODOM_SENSORS_SAMPLING_RATE))      { publishOdometryData(&localOdom);      tOdom = now; }
      if (now - tBAMOpow    >= (1000.0 / BAMO_POWER_SAMPLING_RATE))        { publishBAMOpower(&localBAMO);         tBAMOpow = now; }
      if (now - tBAMOtemp   >= (1000.0 / BAMO_TEMP_SAMPLING_RATE))         { publishBAMOtemp(&localBAMO);          tBAMOtemp = now; }
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

      if (localDataPoint > 0 && localDataPoint % SD_MAX_ROWS == 0) {
        SD32_closePersistentFile();
        fileIndex++;
        SD32_generateFilenameInDir(csvFilename, sessionDirPath, "File", fileIndex);
        SD32_createCSVFile(csvFilename, CSV_HEADER);
        SD32_openPersistentFile(csvFilename);
        Serial.printf("[SD] Row limit reached, rotated to: %s\n", csvFilename);
      }

      AppenderFunc apps[1] = { writeWholeRow };
      void*        data[1] = { &entry };
      SD32_appendBulkDataPersistent(apps, data, 1, SD_FLUSH_INTERVAL, SD_CLOSE_INTERVAL);
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

  while (true) {
    #if MOCK_FLAG == 0
      StrokesensorUpdate(&localMech, STR_Heave, STR_Roll);
      ElectSensorsUpdate(&localElect, ElectPinArray);
      GPSupdate(&localOdom, gpsSerial, gps, GPSavailable);
      IMUupdate(&localOdom, myimu, IMUavailable);
    #else
      mockMechanicalData(&localMech);
      mockElectricalData(&localElect);
      mockOdometryData(&localOdom);
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
  twai_message_t txmsg, rxmsg;

  while (true) {
    unsigned long now = millis();
    if (!canBusReady) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

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
      }
      int txResult = CAN32_sendCAN(&txmsg);
      Serial.printf("[CAN TX] ID=0x%03X REG=0x%02X (seq=%d) status=%s\n",
        txmsg.identifier, txmsg.data[1], requestSeq,
        txResult == ESP_OK ? "OK" : "FAIL");
      requestSeq = (requestSeq + 1) % 4;
      lastRequest = now;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (myBAMOCar.canVoltageValid && myBAMOCar.canCurrentValid) {
        myBAMOCar.power = myBAMOCar.canVoltage * myBAMOCar.canCurrent;
      }
      xSemaphoreGive(dataMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));  // 200Hz
  }
}

/************************* Setup ***************************/

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(UART0_BAUD);

  // I2C Init: Wire1 for RTC, Wire for IMU
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  I2C2_connect = Wire.begin(IMU_SDA, IMU_SCL);
  Wire1.setTimeout(2);
  Wire.setTimeout(2);

  // CAN Bus Init
  canBusReady = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN,
    TWAI_TIMING_CONFIG_500KBITS(), TWAI_FILTER_CONFIG_ACCEPT_ALL());

  // RTC Init
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

  // Motion Sensor Init
  IMUavailable = IMUinit(&Wire, myimu);
  delay(1000);
  IMUcalibrate(myimu, IMUavailable);
  GPSavailable = GPSinit(gpsSerial, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD);

  // Base Sensor Init
  StrokesensorInit(STR_Heave, STR_Roll);
  ElectSensorsInit(ElectPinArray);

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
  #if WIFI_ENABLED == 1 && WS_ENABLED == 1
  if (WiFi.status() == WL_CONNECTED) {
    BPMobile.setClientName(clientName);
    BPMobile.setRegisterCallback(registerClient);
    BPMobile.initWebSocketSSL(serverHost, serverPort, clientName, DEFAULT_WS_PATH);
  }
  #elif WS_ENABLED == 0
  Serial.println("[WS] Disabled (WS_ENABLED=0)");
  #endif

  // SD Card Init
  #if SD_ENABLED == 1
  SD32_initSDCard(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN, sdCardReady);
  if (sdCardReady) {
    SD32_createSessionDir(sessionNumber, sessionDirPath, "Node");
    SD32_generateFilenameInDir(csvFilename, sessionDirPath, "File", fileIndex);
    SD32_createCSVFile(csvFilename, CSV_HEADER);
    SD32_openPersistentFile(csvFilename);
  }
  #else
  Serial.println("[SD] Disabled (SD_ENABLED=0)");
  #endif

  // Time Sync
  #if calibrate_RTC == 1
    RTCcalibrate(rtc, (ntpready) ? (WiFi32_getNTPTime() / 1000ULL) : 1000000000000ULL, RTCavailable);
  #endif
  syncTime_setSyncPoint(RTC_UNIX_TIME,
    (RTCavailable) ? (uint64_t)RTC_getUnix(rtc, RTCavailable) * 1000ULL : 1000000000000ULL);

  Serial.println("==================================================");
  Serial.println("BP Bridge Sensor Node - Unified - Ready");
  Serial.println("==================================================");
  Serial.printf("Client: %s\n\n", clientName);

  // FreeRTOS
  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  #if SD_ENABLED == 1
  sdQueue = xQueueCreate(30, sizeof(SDLogEntry));
  #endif

  // Core 0
  #if WIFI_ENABLED == 1 && WS_ENABLED == 1
  xTaskCreatePinnedToCore(BPMobileTask, "BPMobileTask", 8192, NULL, 1, &BPMobileTaskHandle, 0);
  Serial.println("[RTOS] BPMobile task on Core 0 (pri 1)");
  #else
  Serial.println("[RTOS] BPMobile task SKIPPED (WIFI_ENABLED=0 or WS_ENABLED=0)");
  #endif

  xTaskCreatePinnedToCore(timeSyncTask, "TimeSyncTask", 4096, NULL, 3, &timeSyncTaskHandle, 0);
  Serial.println("[RTOS] TimeSync task on Core 0 (pri 3)");

  // Core 1
  #if MOCK_FLAG == 0
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 5, &sensorTaskHandle, 1);
  Serial.println("[RTOS] Sensor task on Core 1 (pri 5)");
  xTaskCreatePinnedToCore(canTask, "CANTask", 4096, NULL, 5, &canTaskHandle, 1);
  Serial.println("[RTOS] CAN task on Core 1 (pri 5)");
  #else
  Serial.println("[RTOS] Sensor/CAN tasks SKIPPED (MOCK_FLAG=1)");
  #endif

  #if SD_ENABLED == 1
  xTaskCreatePinnedToCore(sdTask, "SDTask", 4096, NULL, 2, &sdTaskHandle, 1);
  Serial.println("[RTOS] SD Logger task on Core 1 (pri 2)");
  #else
  Serial.println("[RTOS] SD Logger task SKIPPED (SD_ENABLED=0)");
  #endif
}

/************************* Main Loop ***************************/

void loop() {
  uint64_t SESSION_TIME_MS = millis();
  uint64_t CURRENT_UNIX_TIME_MS = syncTime_calcRelative_ms(RTC_UNIX_TIME);

  #if calibrate_RTC == 1
    char timeBuf[32];
    syncTime_formatUnix(timeBuf, CURRENT_UNIX_TIME_MS, 7);
    Serial.println(timeBuf);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  #endif

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
      // teleplotMechanical(&debugMech);
      // teleplotElectrical(&debugElect);
      teleplotMotion(&debugOdom);
      // teleplotBAMOCar(&debugBAMO);
    #endif
    lastTeleplotDebug = SESSION_TIME_MS;
  }
  #endif

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
      mockMechanicalData(&myMechData);
      mockElectricalData(&myElectData);
      mockOdometryData(&myOdometryData);
      mockBAMOCarData(&myBAMOCar);
      xSemaphoreGive(dataMutex);
    }
  #endif

  #if SD_ENABLED == 1
  if (sdCardReady && !SD32_checkSDconnect()) {
    SD32_closePersistentFile();
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
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  JsonDocument doc;
  doc["type"]  = "data";
  doc["group"] = "mech";
  doc["ts"]    = timestamp;
  doc["d"]["Wheel_RPM_L"]  = m->Wheel_RPM_L;
  doc["d"]["Wheel_RPM_R"]  = m->Wheel_RPM_R;
  doc["d"]["STR_Heave_mm"] = m->STR_Heave_mm;
  doc["d"]["STR_Roll_mm"]  = m->STR_Roll_mm;
  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishElectData(Electrical* e) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  JsonDocument doc;
  doc["type"]  = "data";
  doc["group"] = "elect";
  doc["ts"]    = timestamp;
  doc["d"]["I_SENSE"] = e->I_SENSE;
  doc["d"]["TMP"]     = e->TMP;
  doc["d"]["APPS"]    = e->APPS;
  doc["d"]["BPPS"]    = e->BPPS;
  doc["d"]["steering"] = e->steering;
  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishElectFaultState(Electrical* e) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  JsonDocument doc;
  doc["type"]  = "data";
  doc["group"] = "faults";
  doc["ts"]    = timestamp;
  doc["d"]["AMS_OK"]  = (bool)e->AMS_OK;
  doc["d"]["IMD_OK"]  = (bool)e->IMD_OK;
  doc["d"]["HV_ON"]   = (bool)e->HV_ON;
  doc["d"]["BSPD_OK"] = (bool)e->BSPD_OK;
  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishOdometryData(Odometry* o) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  JsonDocument doc;
  doc["type"]  = "data";
  doc["group"] = "odom";
  doc["ts"]    = timestamp;
  doc["d"]["gps_lat"]     = o->gps_lat;
  doc["d"]["gps_lng"]     = o->gps_lng;
  doc["d"]["gps_age"]     = o->gps_age;
  doc["d"]["gps_course"]  = o->gps_course;
  doc["d"]["gps_speed"]   = o->gps_speed;
  doc["d"]["imu_accel_x"] = o->imu_accelx;
  doc["d"]["imu_accel_y"] = o->imu_accely;
  doc["d"]["imu_accel_z"] = o->imu_accelz;
  doc["d"]["imu_gyro_x"]  = o->imu_gyrox;
  doc["d"]["imu_gyro_y"]  = o->imu_gyroy;
  doc["d"]["imu_gyro_z"]  = o->imu_gyroz;
  doc["d"]["imu_euler_roll"]  = o->imu_euler_roll;
  doc["d"]["imu_euler_pitch"] = o->imu_euler_pitch;
  doc["d"]["imu_euler_yaw"]   = o->imu_euler_yaw;
  doc["d"]["imu_mag_x"]   = o->imu_magx;
  doc["d"]["imu_mag_y"]   = o->imu_magy;
  doc["d"]["imu_mag_z"]   = o->imu_magz;
  doc["d"]["imu_grav_x"]  = o->imu_gravx;
  doc["d"]["imu_grav_y"]  = o->imu_gravy;
  doc["d"]["imu_grav_z"]  = o->imu_gravz;
  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishBAMOpower(BAMOCar* b) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  JsonDocument doc;
  doc["type"]  = "data";
  doc["group"] = "bamo.power";
  doc["ts"]    = timestamp;
  doc["d"]["canVoltage"]      = b->canVoltage;
  doc["d"]["canCurrent"]      = b->canCurrent;
  doc["d"]["power"]           = b->power;
  doc["d"]["canVoltageValid"] = b->canVoltageValid;
  doc["d"]["canCurrentValid"] = b->canCurrentValid;
  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishBAMOtemp(BAMOCar* b) {
  uint64_t timestamp = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  JsonDocument doc;
  doc["type"]  = "data";
  doc["group"] = "bamo.temp";
  doc["ts"]    = timestamp;
  doc["d"]["motorTemp"]      = b->motorTemp2;
  doc["d"]["controllerTemp"] = b->controllerTemp;
  doc["d"]["motorTempValid"] = b->motorTempValid;
  doc["d"]["ctrlTempValid"]  = b->controllerTempValid;
  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void registerClient(const char* clientName) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = clientName;

  JsonArray groups = doc["groups"].to<JsonArray>();
  JsonObject g1 = groups.add<JsonObject>(); g1["group"] = "mech";       g1["rate_hz"] = MECH_SENSORS_SAMPLING_RATE;
  JsonObject g2 = groups.add<JsonObject>(); g2["group"] = "elect";      g2["rate_hz"] = ELECT_SENSORS_SAMPLING_RATE;
  JsonObject g3 = groups.add<JsonObject>(); g3["group"] = "faults";     g3["rate_hz"] = ELECT_FAULT_STAT_SAMPLING_RATE;
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
  addEntry("bamo.power.canVoltageValid", "bool",  "",  "bamo.power");
  addEntry("bamo.power.canCurrentValid", "bool",  "",  "bamo.power");
  addEntry("bamo.temp.motorTemp",        "float", "C", "bamo.temp");
  addEntry("bamo.temp.controllerTemp",   "float", "C", "bamo.temp");
  addEntry("bamo.temp.motorTempValid",   "bool",  "",  "bamo.temp");
  addEntry("bamo.temp.ctrlTempValid",    "bool",  "",  "bamo.temp");

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
