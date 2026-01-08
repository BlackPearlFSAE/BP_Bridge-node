#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Wire.h>
#include <driver/twai.h>  // ESP32 built-in CAN (TWAI) driver
#include <HardwareSerial.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <MPU6050_light.h>
#include <TinyGPS++.h>

#include <BP_mobile_util.h>
#include <SD32_util.h>
#include <syncTime_util.h> 
#include <CAN32_util.h>
#include <WIFI32_util.h>
#include <motion_sensors.h>
#include <base_sensors.h>
// ============================================================================
// BP MOBILE SERVER CONFIGURATION
// ============================================================================
const char* ssid = "realme C55";
const char* password = "realme1234";
const char* serverHost = "10.10.14.83";
const int serverPort = 3000;
const char* clientName = "ESP32 Rear";  // Unique per sensor node

WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets,&webSocketStatus);
// Alias name for easy use
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

// ============================================================================
// DEVICE BASE CONFIGURATION (WIFI, I2C, UART, ADC, SPI, SDcard)
// ============================================================================

#define WIFI_LED 3
#define WS_LED 4

// ---- I2C
bool I2C1_connect = 0;
bool I2C2_connect = 0;

// ---- SD SPI
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_SCK_PIN 12

// ---- UART
#define UART0_BAUD 115200

bool sdCardReady = false;
unsigned long lastSDLog = 0;
int dataPoint = 0;
int sessionNumber = 0;
const unsigned long SD_LOG_INTERVAL = 500; // 0.5s

char* csvFilename = "Default";  // Will be generated at startup with unique name
// Split CSV headers into modular segments
const char* header_Timestamp = "DataPoint,UnixTime,SessionTime";
const char* header_BAMO = "CAN_Voltage(V),CAN_Current(A),Power(W),MotorTemp(C),ControllerTemp(C)";
const char* header_Mechanical = "Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm";
const char* header_Electrical = "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK";
const char* header_Odometry = "GPS_Lat,GPS_Lng,GPS_Age,GPS_Course,GPS_Speed,IMU_AccelX,IMU_AccelY,IMU_AccelZ,IMU_GyroX,IMU_GyroY,IMU_GyroZ";
char csvHeaderBuffer[512] = "";

void append_Timestamp_toCSVFile(File& dataFile, void* data);
void append_MechData_toCSVFile(File& dataFile, void* data);
void append_ElectData_toCSVFile(File& dataFile, void* data);
void append_OdometryData_toCSVFile(File& dataFile, void* data);
void append_BAMOdata_toCSVFile(File& dataFile, void* data);

// -- CAN Bus config
#define CAN_TX_PIN 48  // GPIO48 for CAN TX
#define CAN_RX_PIN 47  // GPIO47 for CAN RX
  // CANBus Filter Definition (Must define here ,based on use case)

// --- Debugger
void showDeviceStatus();

bool canBusReady;

// ============================================================================
// SENSORS CONFIGURATION
// ============================================================================

// --- Wheel RPM Sensors (left, Right)  ---
#define ENCODER_PINL 41
#define ENCODER_PINR 42
#define ENCODER_N 50 // encoding resolution
unsigned long RPM_CalcInterval = 100; // 100 ms

// --- Stroke Sensors variables (Heave and Roll Distance) ---
#define STR_Roll 6
#define STR_Heave 5 // 5

// PIN Definition
#define I_SENSE_PIN 4
#define TMP_PIN 8
#define APPS_PIN 15
#define BPPS_PIN 7
#define AMS_OK_PIN 37
#define IMD_OK_PIN 38
#define HV_ON_PIN 35
#define BSPD_OK_PIN 36

int ElectPinArray[8] = {
  I_SENSE_PIN,
  TMP_PIN,
  APPS_PIN,
  BPPS_PIN,
  AMS_OK_PIN,
  IMD_OK_PIN,
  HV_ON_PIN,
  BSPD_OK_PIN
};

// --- GPS (TinyGPS++ ANsH51 GNSS) ---
#define GPS_RX_PIN 2  // Connect to GPS TX
#define GPS_TX_PIN 1  // Connect to GPS RX
#define GPS_BAUD 115200
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
bool GPSavailable = 0;

// --- IMU (MPU6050) ---
#define I2C2_SDA 42
#define I2C2_SCL 41
MPU6050 mpu(Wire);
bool IMUavailable = 0;

// --- RTC Time provider (DS3231MZ+) ---
#define I2C1_SDA 18
#define I2C1_SCL 17
RTC_DS3231 rtc;
bool RTCavailable = 0;
bool RTCinit(TwoWire* WireRTC);
void RTCcalibrate(uint64_t unix_time);
void RTCcalibrate();
uint32_t RTC_getUnix();
String   RTC_getISO();

// ============================================================================
// BPMObile SENSOR PUBLISHING
// ============================================================================

// Add RPM and Stroke sampling rates
const float MECH_SENSORS_SAMPLING_RATE = 2.0;    // 2 Hz for wheel RPM publish
const float ELECT_SENSORS_SAMPLING_RATE = 2.0;    // 2 Hz for Elect_SENSOR 
const float ELECT_FAULT_STAT_SAMPLING_RATE = 1.0;   // 1 Hz period for Elect_Faukt stat

// Data publishing functions
void publishMechData(Mechanical* MechSensors);
void publishElectData(Electrical* ElectSensors);
void publishElectFaultState(Electrical* ElectSensors);
void registerClient(const char* clientName);
// Publish timers for different data streams
unsigned long lastMechSend = 0;
unsigned long lastElectSend = 0;
unsigned long lastElectFaultSend = 0;
unsigned long lastOdometrySend = 0;

// Unix timestamp in ms (from ANY source: RTC/NTP/Server)
uint64_t DEVICE_UNIX_TIME = 0;  

hw_timer_t *My_timer = nullptr;

// ============================================================================
// SETUP
// ============================================================================

bool MockFlag = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(UART0_BAUD);

  (Wire1.begin(I2C1_SDA,I2C1_SCL)) ? I2C1_connect=1 : I2C1_connect=0 ;
  (Wire.begin(I2C2_SDA,I2C2_SCL)) ? I2C2_connect =1 : I2C2_connect=0 ;
  Wire1.setTimeout(2); // I2C timeout to be 2 sec
  Wire.setTimeout(2);

  pinMode(WIFI_LED, OUTPUT);  pinMode(WS_LED, OUTPUT);
  digitalWrite(WIFI_LED,0);   digitalWrite(WS_LED,0); 

  // CAN32 Init
  CAN32_initCANBus(CAN_TX_PIN,CAN_RX_PIN, canBusReady,
  TWAI_TIMING_CONFIG_250KBITS(), TWAI_FILTER_CONFIG_ACCEPT_ALL());
  
  // Separated I2C bus
  RTCavailable = RTCinit(&Wire1);
  IMUavailable = IMUinit(&Wire,mpu);
  delay(1000); IMUcalibrate(mpu,IMUavailable); // 1 sec Calibrate
  GPSavailable = GPSinit(gpsSerial,GPS_TX_PIN,GPS_RX_PIN,GPS_BAUD); 

  // Initialize all Mechanical and Electrical sensor
  RPMinit_withExtPullUP(ENCODER_PINL,ENCODER_PINR); 
  RPM_setCalcPeriod(My_timer, 100); 
  StrokesensorInit(STR_Heave,STR_Roll);
  ElectSensorsInit(ElectPinArray);
  
  // Initialize SD Card with SD32_util
  SD32_initSDCard(SD_SCK_PIN,SD_MISO_PIN,SD_MOSI_PIN,SD_CS_PIN,sdCardReady);
  SD32_generateUniqueFilename(sessionNumber,csvFilename); // modify filename to be unique/session
  
  strcat(csvHeaderBuffer,header_Timestamp);  strcat(csvHeaderBuffer,",");
  // Can Comment any one of these out
  strcat(csvHeaderBuffer,header_Mechanical); strcat(csvHeaderBuffer,",");
  strcat(csvHeaderBuffer,header_Electrical); strcat(csvHeaderBuffer,","); 
  strcat(csvHeaderBuffer,header_Odometry);   strcat(csvHeaderBuffer,","); 
  strcat(csvHeaderBuffer,header_BAMO);  
  // create with unique filename and dynamic header
  SD32_createCSVFile(csvFilename,csvHeaderBuffer); 

  // Connect to WiFi and Websocket
  initWiFi(ssid,password, /*Atttempt*/ 10);
  if (WiFi.status() == WL_CONNECTED) {
    // set a register callback into the websocketEventhandler
    BPMobile.setClientName(clientName);
    BPMobile.setRegisterCallback(registerClient);
    BPMobile.initWebSocket(serverHost,serverPort,clientName);
  }

  // Sync device time with RTC on startup (in second scale)
  RTCcalibrate(); // Calibrate with Latest Date (Comment out after use)
  syncTime_setAbsolute(DEVICE_UNIX_TIME,1000000000000ULL /*RTC_getUnix()*/ );
  Serial.print("Device time synced with RTC: "); Serial.println(RTC_getISO());

  Serial.println("==================================================");
  Serial.println("Bridge sensor node - ready to operate");
  Serial.println("==================================================");
  Serial.printf("Client Name: %s\n\n",clientName);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {

  uint32_t SESSION_TIME = millis(); // Now
  uint64_t RELATIVE_UNIX_TIME = syncTime_getRelative(DEVICE_UNIX_TIME); // Now Unix
  // Resync if DeviceTime drifted from CURRENT_TIME by 1 sec
  // if (!syncTime_isSynced()) {
  //   syncTime_resync(DEVICE_UNIX_TIME,RELATIVE_UNIX_TIME,1000);
  // }

  // Init Temp struct per mcu loop
  twai_message_t txmsg; twai_message_t rxmsg;   
  Mechanical myMechData;
  Electrical myElectData;
  Odometry myOdometryData;

  // Handle WebSocket communication 
  if (WiFi.status() == WL_CONNECTED) BPwebSocket->loop();
  // showDeviceStatus(); delay(200);
  // return;

  // Update WiFi LED status
  (WiFi.status() == WL_CONNECTED)? digitalWrite(WIFI_LED,1):digitalWrite(WIFI_LED,0);
  (BPsocketstatus->isConnected == true)? digitalWrite(WS_LED,1):digitalWrite(WS_LED,0);
  
  // Update Each Sensors
  RPMsensorUpdate(&myMechData, RPM_CalcInterval ,ENCODER_N); 
  StrokesensorUpdate(&myMechData,STR_Heave,STR_Roll);
  ElectSensorsUpdate(&myElectData,ElectPinArray);
  GPSupdate(&myOdometryData,gpsSerial,gps); IMUupdate(&myOdometryData,mpu,IMUavailable);
  
  // if (canBusReady) {
  //   // Receive and process BAMOCar d3 400 CAN frame
  //   if(CAN32_receiveCAN(&rxmsg) == ESP_OK) {break;/* Message processor function */}
  //   // else {Serial.println("BAMO RX Buffer = 0");}
  //   }
  // }
  

  // if(MockFlag){
  //   mockMechanicalData(&myMechData);mockElectricalData(&myElectData);
  //   mockOdometryData(&myOdometryData);mockBAMOCarData(&myBAMOCar); 
  // }
  
  // Log to SD card
  if (sdCardReady && (SESSION_TIME - lastSDLog >= SD_LOG_INTERVAL)) {
    int appenderCount = 5; // increase when scale
    AppenderFunc appenders[appenderCount] = {
      append_Timestamp_toCSVFile, // For UNIX_TIME
      append_Timestamp_toCSVFile, // Reuse for SESSION_TIME
      // Comment below to toggle off
      append_MechData_toCSVFile,
      append_ElectData_toCSVFile,
      append_OdometryData_toCSVFile,
    };
    void *structArray[appenderCount] = {
      &RELATIVE_UNIX_TIME,
      &SESSION_TIME,
      // Comment below to toggle off
      &myMechData,
      &myElectData,
      &myOdometryData,
    };

    logDataToSD(csvFilename, appenders, structArray, appenderCount);
    dataPoint++;
    Serial.print("[SD Card] Logged data point #"); Serial.println(dataPoint - 1);
    lastSDLog = SESSION_TIME;
  }

  if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
    // Send Mechanical data
    if (SESSION_TIME - lastMechSend >= (1000.0 / MECH_SENSORS_SAMPLING_RATE)) {
      publishMechData(&myMechData);
      lastMechSend = SESSION_TIME;
    }
    // Send Electrical analog data
    if (SESSION_TIME - lastElectSend >= (1000.0 / ELECT_SENSORS_SAMPLING_RATE)) {
      publishElectData(&myElectData);
      lastElectSend = SESSION_TIME;
    }
    // Send Electrical fault state data
    if (SESSION_TIME - lastElectFaultSend >= (1000.0 / ELECT_FAULT_STAT_SAMPLING_RATE)) {
      publishElectFaultState(&myElectData);
      lastElectFaultSend = SESSION_TIME;
    }
  }
 
}

// ****************************************************************************

// ============================================================================
// WIFI , DEVICE , BPMobile Publishing
// ============================================================================


void showDeviceStatus() {
  
  Serial.println("╔═════════════════════════════════════════════╣");
  Serial.println("║                SYSTEM STATE                 ╣");
  Serial.println("╠═════════════════════════════════════════════╣");

  // CAN Bus status
  Serial.print("║ CAN Bus:              ");
  (canBusReady) ? Serial.println("✓ CONNECTED") : Serial.println("✗ NOT CONNECTED");
  
  // I2C status
  Serial.print("║ I2C1 Bus:             ");
  (I2C1_connect) ? Serial.println("✓ CONNECTED") : Serial.println("✗ NOT CONNECTED");
  Serial.print("║ I2C2 Bus:             ");
  (I2C2_connect) ? Serial.println("✓ CONNECTED") : Serial.println("✗ NOT CONNECTED");

  // SD Status
  Serial.print("║ SD card:              ");
  (sdCardReady) ? Serial.println("✓ CONNECTED") : Serial.println("✗ NOT CONNECTED");

  // Wifi connection
  Serial.print("║ WiFi:                 ");
  (WiFi.status() == WL_CONNECTED) ? 
  Serial.printf("✓ CONNECTED RSS %d dbm\n",WiFi.RSSI()) : Serial.println("✗ DISCONNECTED");

  // WebSocket Status
  Serial.print("║ BPMobile WebSocket:   ");
  if(BPsocketstatus == nullptr) Serial.println("!! OBJECT NOT DECLARED");
  else (BPsocketstatus->isConnected) ? Serial.println("✓ CONNECTED") : Serial.println("✗ NOT CONNECTED");

  // Time Sync Status
  Serial.print("║ DeviceTime Sync:      ");
  (syncTime_isSynced()) ? Serial.println("✓ SYNCED") : Serial.println("✗ NOT SYNCED");

  Serial.println("      ╠**********Peripheral**********╣        ");

  Serial.print("║ RTC:                  ");
  (RTCavailable) ? Serial.println("✓ DISCOVERED (conflict w IMU1)") : Serial.println("✗ NOT AVAILABLE");
  
  Serial.print("║ IMU2:                  ");
  (IMUavailable) ? Serial.println("✓ DISCOVERED") : Serial.println("✗ NOT AVAILABLE");

  Serial.print("║ GPS1:                  ");
  (gpsSerial.available() > 0) ? Serial.println("✓ DISCOVERED") : Serial.println("✗ NOT AVAILABLE");

  Serial.println("╠═════════════════════════════════════════════╣");
}

void publishMechData(Mechanical* MechSensors) {
  unsigned long long unixTimestamp = syncTime_getRelative(DEVICE_UNIX_TIME);
  if (unixTimestamp < 1000000000000ULL) return;

  // Left wheel RPM
  JsonDocument docL;
  docL["type"] = "data";
  docL["topic"] = "wheel/left_rpm";
  docL["data"]["value"] = MechSensors->Wheel_RPM_L;
  docL["data"]["sensor_id"] = "ENC_LEFT";
  docL["timestamp"] = unixTimestamp;
  String msgL;
  serializeJson(docL, msgL);
  BPwebSocket->sendTXT(msgL);

  // Right wheel RPM
  JsonDocument docR;
  docR["type"] = "data";
  docR["topic"] = "wheel/right_rpm";
  docR["data"]["value"] = MechSensors->Wheel_RPM_R;
  docR["data"]["sensor_id"] = "ENC_RIGHT";
  docR["timestamp"] = unixTimestamp;
  String msgR;
  serializeJson(docR, msgR);
  BPwebSocket->sendTXT(msgR);

   // Stroke Heave
  JsonDocument sH;
  sH["type"] = "data";
  sH["topic"] = "sensors/stroke_Heave_distanceMM";
  sH["data"]["value"] = MechSensors->STR_Heave_mm;
  sH["data"]["sensor_id"] = "STR_Heave";
  sH["timestamp"] = unixTimestamp;
  String msg2;
  serializeJson(sH, msg2);
  BPwebSocket->sendTXT(msg2);

  // Stroke 1 (black)
  JsonDocument sR;
  sR["type"] = "data";
  sR["topic"] = "sensors/stroke_Roll_distanceMM";
  sR["data"]["value"] = MechSensors->STR_Roll_mm;
  sR["data"]["sensor_id"] = "STR_Roll";
  sR["timestamp"] = unixTimestamp;
  String msg1;
  serializeJson(sR, msg1);
  BPwebSocket->sendTXT(msg1);

 

  Serial.print("[BP Mobile] RPM L:");
  Serial.print(MechSensors->Wheel_RPM_L, 1);
  Serial.print(" R:");
  Serial.println(MechSensors->Wheel_RPM_R, 1);

  Serial.print("[BP Mobile] Stroke mm STR_Heave:");
  Serial.print(MechSensors->STR_Roll_mm, 2);
  Serial.print(" STR_Roll:");
  Serial.println(MechSensors->STR_Heave_mm, 2);
}

void publishElectData(Electrical* ElectSensors) {
  unsigned long long unixTimestamp = syncTime_getRelative(DEVICE_UNIX_TIME);
  if (unixTimestamp < 1000000000000ULL) return;

  // Current Sense
  JsonDocument iSenseDoc;
  iSenseDoc["type"] = "data";
  iSenseDoc["topic"] = "electrical/current_sense";
  iSenseDoc["data"]["value"] = ElectSensors->I_SENSE;
  iSenseDoc["data"]["sensor_id"] = "I_SENSE";
  iSenseDoc["timestamp"] = unixTimestamp;
  String iSenseMsg;
  serializeJson(iSenseDoc, iSenseMsg);
  BPwebSocket->sendTXT(iSenseMsg);

  // Temperature
  JsonDocument tmpDoc;
  tmpDoc["type"] = "data";
  tmpDoc["topic"] = "electrical/temperature";
  tmpDoc["data"]["value"] = ElectSensors->TMP;
  tmpDoc["data"]["sensor_id"] = "TMP";
  tmpDoc["timestamp"] = unixTimestamp;
  String tmpMsg;
  serializeJson(tmpDoc, tmpMsg);
  BPwebSocket->sendTXT(tmpMsg);

  // Accelerator Pedal Position Sensor
  JsonDocument appsDoc;
  appsDoc["type"] = "data";
  appsDoc["topic"] = "electrical/apps";
  appsDoc["data"]["value"] = ElectSensors->APPS;
  appsDoc["data"]["sensor_id"] = "APPS";
  appsDoc["timestamp"] = unixTimestamp;
  String appsMsg;
  serializeJson(appsDoc, appsMsg);
  BPwebSocket->sendTXT(appsMsg);

  // Brake Pedal Position Sensor
  JsonDocument bppsDoc;
  bppsDoc["type"] = "data";
  bppsDoc["topic"] = "electrical/bpps";
  bppsDoc["data"]["value"] = ElectSensors->BPPS;
  bppsDoc["data"]["sensor_id"] = "BPPS";
  bppsDoc["timestamp"] = unixTimestamp;
  String bppsMsg;
  serializeJson(bppsDoc, bppsMsg);
  BPwebSocket->sendTXT(bppsMsg);

  Serial.print("[BP Mobile] Electrical: I_SENSE=");
  Serial.print(ElectSensors->I_SENSE, 2);
  Serial.print(", TMP=");
  Serial.print(ElectSensors->TMP, 1);
  Serial.print("°C, APPS=");
  Serial.print(ElectSensors->APPS, 2);
  Serial.print(", BPPS=");
  Serial.println(ElectSensors->BPPS, 2);
}

void publishElectFaultState(Electrical* ElectSensors) {
  unsigned long long unixTimestamp = syncTime_getRelative(DEVICE_UNIX_TIME);
  if (unixTimestamp < 1000000000000ULL) return;

  // AMS OK status
  JsonDocument amsDoc;
  amsDoc["type"] = "data";
  amsDoc["topic"] = "electrical/ams_ok";
  amsDoc["data"]["value"] = ElectSensors->AMS_OK;
  amsDoc["data"]["sensor_id"] = "AMS";
  amsDoc["timestamp"] = unixTimestamp;
  String amsMsg;
  serializeJson(amsDoc, amsMsg);
  BPwebSocket->sendTXT(amsMsg);

  // IMD OK status
  JsonDocument imdDoc;
  imdDoc["type"] = "data";
  imdDoc["topic"] = "electrical/imd_ok";
  imdDoc["data"]["value"] = ElectSensors->IMD_OK;
  imdDoc["data"]["sensor_id"] = "IMD";
  imdDoc["timestamp"] = unixTimestamp;
  String imdMsg;
  serializeJson(imdDoc, imdMsg);
  BPwebSocket->sendTXT(imdMsg);

  // HV ON status
  JsonDocument hvDoc;
  hvDoc["type"] = "data";
  hvDoc["topic"] = "electrical/hv_on";
  hvDoc["data"]["value"] = ElectSensors->HV_ON;
  hvDoc["data"]["sensor_id"] = "HV";
  hvDoc["timestamp"] = unixTimestamp;
  String hvMsg;
  serializeJson(hvDoc, hvMsg);
  BPwebSocket->sendTXT(hvMsg);

  // BSPD OK status
  JsonDocument bspdDoc;
  bspdDoc["type"] = "data";
  bspdDoc["topic"] = "electrical/bspd_ok";
  bspdDoc["data"]["value"] = ElectSensors->BSPD_OK;
  bspdDoc["data"]["sensor_id"] = "BSPD";
  bspdDoc["timestamp"] = unixTimestamp;
  String bspdMsg;
  serializeJson(bspdDoc, bspdMsg);
  BPwebSocket->sendTXT(bspdMsg);

  Serial.print("[BP Mobile] Fault States: AMS_OK=");
  Serial.print(ElectSensors->AMS_OK);
  Serial.print(", IMD_OK=");
  Serial.print(ElectSensors->IMD_OK);
  Serial.print(", HV_ON=");
  Serial.print(ElectSensors->HV_ON);
  Serial.print(", BSPD_OK=");
  Serial.println(ElectSensors->BSPD_OK);
}

void registerClient(const char* clientName) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = clientName;
  
  // Define topics
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

  // Define topic metadata
  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();

  // Wheel RPM metadata
  JsonObject leftRpmMeta = metadata["wheel/left_rpm"].to<JsonObject>();
  leftRpmMeta["description"] = "Left wheel speed";
  leftRpmMeta["unit"] = "RPM";
  leftRpmMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

  JsonObject rightRpmMeta = metadata["wheel/right_rpm"].to<JsonObject>();
  rightRpmMeta["description"] = "Right wheel speed";
  rightRpmMeta["unit"] = "RPM";
  rightRpmMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

  // Stroke distance metadata
  JsonObject sHMeta = metadata["sensors/stroke_Heave_distanceMM"].to<JsonObject>();
  sHMeta["description"] = "Stroke Heave";
  sHMeta["unit"] = "mm";
  sHMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

  JsonObject sRMeta = metadata["sensors/stroke_Roll_distanceMM"].to<JsonObject>();
  sRMeta["description"] = "Stroke Roll";
  sRMeta["unit"] = "mm";
  sRMeta["sampling_rate"] = MECH_SENSORS_SAMPLING_RATE;

  // Electrical sensors metadata (analog)
  JsonObject iSenseMeta = metadata["electrical/current_sense"].to<JsonObject>();
  iSenseMeta["description"] = "Current sense";
  iSenseMeta["unit"] = "A";
  iSenseMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  JsonObject tmpMeta = metadata["electrical/temperature"].to<JsonObject>();
  tmpMeta["description"] = "Electrical system temperature";
  tmpMeta["unit"] = "°C";
  tmpMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  JsonObject appsMeta = metadata["electrical/apps"].to<JsonObject>();
  appsMeta["description"] = "Accelerator Pedal Position Sensor";
  appsMeta["unit"] = "%";
  appsMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  JsonObject bppsMeta = metadata["electrical/bpps"].to<JsonObject>();
  bppsMeta["description"] = "Brake Pedal Position Sensor";
  bppsMeta["unit"] = "%";
  bppsMeta["sampling_rate"] = ELECT_SENSORS_SAMPLING_RATE;

  // Electrical fault state metadata (digital)
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

  // Serialize and send
  String registration;
  serializeJson(doc, registration);
  
  Serial.println("[WebSocket] Sending registration...");
  BPwebSocket->sendTXT(registration);
}

// ============================================================================
// SENSORS
// ============================================================================

bool RTCinit(TwoWire* WireRTC){
  if (!rtc.begin(WireRTC)) { // Initialize the I2C connection
    Serial.println("RTC NOT FOUND");
    return false;
  } else {
    Serial.println("RTC FOUND");
    return true;
  } 
}
void RTCcalibrate(uint64_t unix_time){
  if(!RTCavailable){
    Serial.println("RTC Calibrate Failed");
    return;
  }
  rtc.adjust(DateTime(unix_time));
}
void RTCcalibrate(){
  if(!RTCavailable){
    Serial.println("RTC Calibrate Failed");
    return;
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}
uint32_t RTC_getUnix(){
  if(!RTCavailable){
    Serial.println("RTC get Unix Failed");
    return 0;
  }
  uint32_t SESSION_TIME = rtc.now().unixtime(); return SESSION_TIME;
}
String   RTC_getISO(){
  if(!RTCavailable){
    Serial.println("RTC get ISO Failed");
    return "Unknown";
  }
  String SESSION_TIME = rtc.now().timestamp(DateTime::TIMESTAMP_FULL); 
  return SESSION_TIME;
}

// ============================================================================
// CSV Data Appender Functions
// ============================================================================
// Timestamp Appender - writes Unix timestamp, datetime, and data point counter
void append_Timestamp_toCSVFile(File& dataFile, void* data) {
  uint64_t* current_time= (uint64_t*)data;
  
  dataFile.print(dataPoint);
  dataFile.print(",");
  dataFile.print(*current_time);
  dataFile.print(",");
}

// Mechanical Sensors Appender - writes wheel RPM and stroke sensor data
void append_MechData_toCSVFile(File& dataFile, void* data) {
  Mechanical* MechSensors = static_cast<Mechanical*>(data);

  // Write wheel RPMs
    dataFile.print(MechSensors->Wheel_RPM_L, 2);
    dataFile.print(",");
    dataFile.print(MechSensors->Wheel_RPM_R, 2);
    dataFile.print(",");

    // Write stroke distances
    dataFile.print(MechSensors->STR_Heave_mm, 2);
    dataFile.print(",");
    dataFile.print(MechSensors->STR_Roll_mm, 2);
    dataFile.print(",");
}

// Electrical Sensors Appender - writes analog sensors and digital fault status
void append_ElectData_toCSVFile(File& dataFile, void* data) {
  Electrical* ElectSensors = static_cast<Electrical*>(data);

  // Analog sensors (2 decimal places)
  dataFile.print(ElectSensors->I_SENSE, 2);
  dataFile.print(",");
  dataFile.print(ElectSensors->TMP, 2);
  dataFile.print(",");
  dataFile.print(ElectSensors->APPS, 2);
  dataFile.print(",");
  dataFile.print(ElectSensors->BPPS, 2);
  dataFile.print(",");

  // Digital fault status (0/1)
  dataFile.print(ElectSensors->AMS_OK ? 1 : 0);
  dataFile.print(",");
  dataFile.print(ElectSensors->IMD_OK ? 1 : 0);
  dataFile.print(",");
  dataFile.print(ElectSensors->HV_ON ? 1 : 0);
  dataFile.print(",");
  dataFile.print(ElectSensors->BSPD_OK ? 1 : 0);
  dataFile.print(",");
}

// Odometry Sensors Appender - writes GPS and IMU data
void append_OdometryData_toCSVFile(File& dataFile, void* data) {
  Odometry* OdomSensors = static_cast<Odometry*>(data);

  // GPS data (lat/lng: 6 decimals, others: 2 decimals)
  dataFile.print(OdomSensors->gps_lat, 4);
  dataFile.print(",");
  dataFile.print(OdomSensors->gps_lng, 4);
  dataFile.print(",");
  dataFile.print(OdomSensors->gps_age, 2);
  dataFile.print(",");
  dataFile.print(OdomSensors->gps_course, 2);
  dataFile.print(",");
  dataFile.print(OdomSensors->gps_speed, 2);
  dataFile.print(",");

  // IMU accelerometer (3 decimals)
  dataFile.print(OdomSensors->imu_accelx, 3);
  dataFile.print(",");
  dataFile.print(OdomSensors->imu_accely, 3);
  dataFile.print(",");
  dataFile.print(OdomSensors->imu_accelz, 3);
  dataFile.print(",");

  // IMU gyroscope (3 decimals)
  dataFile.print(OdomSensors->imu_gyrox, 3);
  dataFile.print(",");
  dataFile.print(OdomSensors->imu_gyroy, 3);
  dataFile.print(",");
  dataFile.print(OdomSensors->imu_gyroz, 3);
  dataFile.print(",");
}
