#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <driver/twai.h>  // ESP32 built-in CAN (TWAI) driver
#include <Wire.h>
#include <RTClib.h>
#include <MPU6050_light.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
// #include <Ticker.h> // I Think I will just use this for BPMobile publisher

#include <BP_mobile_util.h>
#include <SD32_util.h>
#include <syncTime_util_v2.h>  // V2: Clean time system
#include <CAN32_util.h>
// ============================================================================
// BP MOBILE SERVER CONFIGURATION
// ============================================================================
const char* ssid = "realme C55";
const char* password = "realme1234";
const char* serverHost = "10.136.194.79";
const int serverPort = 3000;
const char* clientName = "ESP32 Rear";  // Unique per sensor node

WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets,&webSocketStatus);
// Alias name for easy use
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

#define WIFI_LED_INDICATOR 3
#define WS_LED_INDICATOR 4
void initWiFi(const char* ssid, const char* password);
void registerClient(const char* clientName);
void showDeviceStatus();
// ============================================================================
// SD CARD CONFIGURATION
// ============================================================================
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 15
#define SD_SCK_PIN 12

char* csvFilename = 0;  // Will be generated at startup with unique name
// Split CSV headers into modular segments
const char* header_Timestamp = "DataPoint,UnixTime";
const char* header_trackinfo = "SessionTime,LapTime,LapCount";
const char* header_BAMO = "CAN_Voltage(V),CAN_Current(A),Power(W),MotorTemp(C),ControllerTemp(C)";
const char* header_Mechanical = "Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm";
const char* header_Electrical = "I_SENSE(A),TMP(C),APPS(%),BPPS(%),AMS_OK,IMD_OK,HV_ON,BSPD_OK";
const char* header_Odometry = "GPS_Lat,GPS_Lng,GPS_Age,GPS_Course,GPS_Speed,IMU_AccelX,IMU_AccelY,IMU_AccelZ,IMU_GyroX,IMU_GyroY,IMU_GyroZ";
char csvHeaderBuffer[512] = "";

void append_Timestamp_toCSVFile(File& dataFile, void* data);
void append_TrackInfo_toCSVFile(File& dataFile, void* data);
void append_MechData_toCSVFile(File& dataFile, void* data);
void append_ElectData_toCSVFile(File& dataFile, void* data);
void append_OdometryData_toCSVFile(File& dataFile, void* data);
void append_BAMOdata_toCSVFile(File& dataFile, void* data);

bool sdCardReady = false;
unsigned long lastSDLog = 0;
int dataPoint = 0;
int sessionNumber = 0;  // For unique filename generation
/* SD card SAMPLING INTERVAL */
const unsigned long SD_LOG_INTERVAL = 500; // Log to SD every 0.5s


// ----- SD LOGGING Compile-time toggles
#define ENABLE_TIMESTAMP_LOG 1

// ============================================================================
// CAN BUS CONFIGURATION
// ============================================================================
#define CAN_TX_PIN 48  // GPIO48 for CAN TX
#define CAN_RX_PIN 47  // GPIO47 for CAN RX

// Bamocar D3 CAN Request IDs 
#define BAMOCAR_BASE_ID 0x200           // Base CAN ID for Bamocar
#define BAMOCAR_REQUEST_ID 0x201        // Request ID (same as reference code)
#define BAMOCAR_RESPONSE_ID 0x181       // Response ID (CORRECTED from 0x181!)

// Bamocar D3 Register Addresses (CORRECTED based on working code)
#define BAMOCAR_REG_MOTOR_TEMP 0x49         // Motor temperature register ✓
#define BAMOCAR_REG_CONTROLLER_TEMP 0x4A    // Controller/IGBT temperature register ✓
#define BAMOCAR_REG_DC_VOLTAGE 0xEB         // DC Link Voltage (CORRECTED from 0xA5!)
#define BAMOCAR_REG_DC_CURRENT 0x20         // DC Current (CORRECTED from 0xC6!)
#define BAMOCAR_REG_SPEED_ACTUAL 0x30       // Actual speed/RPM 

// CAN Bus timing and control variables
unsigned long lastDebugPrint = 0;
const unsigned long CAN_REQUEST_INTERVAL = 200;  // Request every 200ms
bool canBusReady;
unsigned long lastBAMOrequest = 0;
uint8_t CANRequest_sequence = 0;  // 0=motor temp, 1=controller temp, 2=voltage, 3=current

// BAMOCar D3 400 Data Structure
typedef struct{
  // Motor controller measurements (from CAN)
  float motorTemp1 = 0.0;
  float motorTemp2 = 0.0;
  float controllerTemp = 0.0;
  float canVoltage = 0.0;      // DC Link Voltage from CAN
  float canCurrent = 0.0;      // Motor Current from CAN
  float power = 0.0;           // Calculated power (V * I)

  // Data validity flags
  bool motorTempValid = false;
  bool controllerTempValid = false;
  bool canVoltageValid = false;
  bool canCurrentValid = false;
} BAMOCar;

// CAN Bus functions
void pack_RequestBamocarMsg(twai_message_t* msg,uint8_t regAddress);
void process_ResponseBamocarMsg(twai_message_t* msg, BAMOCar* bamocar);
float convertBamocarTemp(uint16_t rawValue);
void decodeBAMO_rawmsg(twai_message_t &message);
void scanBamocarIDs();
void analyzeBamoData(uint8_t reg, uint8_t* data);

// ============================================================================
// MEHCHANICAL SENSORS
// ============================================================================

typedef struct{
  // Analog
  float Wheel_RPM_L =0.0;
  float Wheel_RPM_R =0.0; // Motor temp from NTC thermister or KTY11 
  float STR_Heave_mm =0.0; // in mm scale
  float STR_Roll_mm =0.0;
} Mechanical;

// --- RPM Sensors variable (Wheel_speed Left and Right) ---
#define ENCODER_PINL 41
#define ENCODER_PINR 42
#define ENCODER_N 50 // encoding resolution
hw_timer_t *My_timer = NULL;
float Wheel_RPM_left = 0.0;
float Wheel_RPM_right = 0.0;
unsigned long Period = 100; // 100 ms
// ISR shared variable
volatile int counterL = 0;
volatile int counterR = 0;
volatile bool startCalculate = 0;
void IRAM_ATTR ISR_COUNT_L();
void IRAM_ATTR ISR_COUNT_R();
void IRAM_ATTR ISRreset();
void RPMinit(hw_timer_t* My_timer, int millisec);
void RPMsensorUpdate(Mechanical *MechSensors);

// --- Stroke Sensors variables (Heave and Roll Distance) ---
#define STR_Roll 6
#define STR_Heave 5
float distance_STR_Roll = 0.0;
float distance_STR_Heave = 0.0;
const float aref = 3.3; 
const int pwmres = 4095; // 12 bit ADC resolution
const float max_distance1 = 75.00; // recalibrated with vernier -> Needs to recheck // 52.91
const float max_distance2 = 75.00; // Not_sure needs to check again

// Publish timers for different data streams
unsigned long lastBAMOsend_power = 0;
unsigned long lastBAMOsend_temp = 0;
unsigned long lastMechSend = 0;
unsigned long lastElectSend = 0;
unsigned long lastElectFaultSend = 0;
unsigned long lastOdometrySend = 0;
void StrokesensorInit();
void StrokesensorUpdate(Mechanical *MechSensors);
// ============================================================================
// ELECTRICAL SENSORS
// ============================================================================
typedef struct{
  // Analog
  float I_SENSE =0.0; // Max x A - -A
  float TMP =0.0; // Motor temp from NTC thermister or KTY11 
  float APPS =0.0;
  float BPPS =0.0;
  // Digital (Original signal 12V)
  bool AMS_OK =0; 
  bool IMD_OK =0;
  bool HV_ON =0; // Except this one 24V
  bool BSPD_OK =0; // And this one 5V
} Electrical;

// PIN Definition
#define I_SENSE_PIN 4
#define TMP_PIN 8
#define APPS_PIN 15
#define BPPS_PIN 7

#define AMS_OK_PIN 37
#define IMD_OK_PIN 38
#define HV_ON_PIN 35
#define BSPD_OK_PIN 36

// set of constant to use (Might not be used )
const float max_volt5 =  5.0;
const float max_volt12 =  12.0;
const float max_volt24 =  24.0;  // Fixed: was 5.0, should be 24.0

// Potentiometer stroke sensors (APPS/BPPS) - voltage maps linearly to distance
const float apps_max_dist_mm =  75.00;  // Full stroke distance
const float bpps_max_dist_mm =  75.00;  // Full stroke distance
const float apps_offset_mm = 0.0;       // Mechanical installation offset
const float bpps_offset_mm = 0.0;       // Mechanical installation offset

// Temperature sensor (NTC thermistor in voltage divider)
const uint16_t tmp_series_res = 10000;  // 10k ohm series resistor
// For voltage divider: Vout = Vcc * (R_NTC / (R_series + R_NTC))

// Hall effect current sensor - adjust these based on your actual sensor
const float i_sense_sensitivity = 0.1;  // V/A (example: ACS712-20A = 0.1 V/A)
const float i_sense_offset = 2.5;       // V at 0A (Vcc/2 for bidirectional sensors)

// --- ELECTRICAL SENSOR FUNCTIONS ---
void ElectSensorsInit();
void ElectSensorsUpdate(Electrical *ElectSensors);

// --- THERMISTOR TEMPERATURE CONVERSION FUNCTIONS ---
float convertNTCtoTemp(uint16_t resistance);
float convertKTYtoTemp(uint16_t resistance);

// ============================================================================
// ODOMETRY SENSORS
// ============================================================================

typedef struct{
  // GPS
  double gps_lat =0;
  double gps_lng =0;
  double gps_age =0;
  double gps_course =0;
  double gps_speed  =0;
  // IMU
  float imu_accelx =0;
  float imu_accely =0;
  float imu_accelz =0;
  float imu_gyrox  =0;
  float imu_gyroy  =0;
  float imu_gyroz  =0;
} Odometry;

// --- IMU (MPU6050) ---
#define IMU_SDA 42
#define IMU_SCL 41

MPU6050 mpu(Wire1);
unsigned long imu_timer = 0;
void IMUinit(TwoWire* WireIMU,int sda, int scl);
void IMUcalibrate();
void IMUupdate(Odometry* myimu);

// --- GPS (TinyGPS++ ANsH51 GNSS) ---
#define GPS_RX_PIN 2  // Connect to GPS TX
#define GPS_TX_PIN 1  // Connect to GPS RX
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
#define GPS_BAUD 9600
void GPSinit(HardwareSerial gpsSerial);
void GPSupdate(Odometry* mygps);

// --- RTC Time provider (DS3231MZ+) ---
#define RTC_SDA 42
#define RTC_SCL 41
RTC_DS3231 rtc;
void RTCinit(TwoWire* I2Cbus, int sda, int scl);
void RTCcalibrate(uint64_t unix_time);
void RTCcalibrate();
uint32_t RTC_getUnix();
String   RTC_getISO();

// ============================================================================
// SENSOR PUBLISHING
// ============================================================================
const float BAMO_POWER_SAMPLING_RATE = 2.0;     // 2 Hz for BAMOPower data
const float BAMO_TEMP_SAMPLING_RATE = 1.0;      // 1 Hz for BAMOTemp data to BP Mobile

// Add RPM and Stroke sampling rates
const float MECH_SENSORS_SAMPLING_RATE = 2.0;    // 2 Hz for wheel RPM publish
const float ELECT_SENSORS_SAMPLING_RATE = 2.0;    // 2 Hz for Elect_SENSOR 
const float ELECT_FAULT_STAT_SAMPLING_RATE = 1.0;   // 1 Hz period for Elect_Faukt stat

// Data publishing functions
void publishBAMOpower(BAMOCar* bamocar);
void publishBAMOtemp(BAMOCar* bamocar);
void publishMechData(Mechanical* MechSensors);
void publishElectData(Electrical* ElectSensors);
void publishElectFaultState(Electrical* ElectSensors);

void mockMechanicalData(Mechanical *MechSensors);
void mockElectricalData(Electrical *ElectSensors);
void mockOdometryData(Odometry *OdomSensors);
void mockBAMOCarData(BAMOCar *BamoCar);

// ============================================================================
// DEVICE TIME SYSTEM - Source Agnostic
// ============================================================================
uint64_t DeviceTime_Unix = 0;  // Unix timestamp in ms (from ANY source: RTC/NTP/Server)
// DeviceTime_Relative is calculated on-the-fly via syncTime_getRelative()

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  delay(1000);

    // Setup status LED
  pinMode(WIFI_LED_INDICATOR, OUTPUT);
  digitalWrite(WIFI_LED_INDICATOR, LOW);  // LED off initially

  CAN32_initCANBus(CAN_TX_PIN,CAN_RX_PIN, canBusReady,TWAI_TIMING_CONFIG_250KBITS());
  // CAN32_initCANBus(CAN_TX_PIN,CAN_RX_PIN, canBusReady,
  // TWAI_TIMING_CONFIG_250KBITS(), <Some filter setting>);

  // Separated i2c bus
  // IMUinit(&Wire,IMU_SDA,IMU_SCL);
  // RTCinit(&Wire1,RTC_SDA,RTC_SCL);
  // Calibrate IMU sensor in 1s, please position it well in the vehicle
  delay(1000); 
  // IMUcalibrate();
  // GPSinit(gpsSerial); 

  // Initialize all Mechanical and Electrical sensor
  RPMinit(My_timer, 100);
  StrokesensorInit();
  ElectSensorsInit();
  
  // Initialize SD Card with SD32_util
  // SD32_initSDCard(SD_SCK_PIN,SD_MISO_PIN,SD_MOSI_PIN,SD_CS_PIN,sdCardReady);
  // SD32_generateUniqueFilename(sessionNumber,csvFilename); // modify filename to be unique/session
  // buildCSVHeader(csvHeaderBuffer, sizeof(csvHeaderBuffer)); // Build header based on enabled modules
  strcat(csvHeaderBuffer,header_Timestamp);  strcat(csvHeaderBuffer,",");
  
  // Can Comment any one of these out
  strcat(csvHeaderBuffer,header_Mechanical); strcat(csvHeaderBuffer,",");
  strcat(csvHeaderBuffer,header_Electrical); strcat(csvHeaderBuffer,","); 
  strcat(csvHeaderBuffer,header_Odometry);   strcat(csvHeaderBuffer,","); 
  strcat(csvHeaderBuffer,header_BAMO);  

  // create with unique filename and dynamic header
  // SD32_createCSVFile(csvFilename,csvHeaderBuffer); 

  // Connect to WiFi
  initWiFi(ssid,password);
  // Setup WebSocket
  if (WiFi.status() == WL_CONNECTED) {
    // set a register callback into the websocketEventhandler
    BPMobile.setClientName(clientName);
    BPMobile.setRegisterCallback(registerClient);
    BPMobile.initWebSocket(serverHost,serverPort,clientName);
  }

  // Sync device time with RTC on startup
  // uint64_t current_unixTime_ms = RTC_getUnix()*1000ULL;
  uint64_t current_unixTime_ms = 100;
  syncTime_setAbsolute(DeviceTime_Unix,current_unixTime_ms);
  Serial.print("Device time synced with RTC: ");
  Serial.println(RTC_getISO());

  Serial.println("==================================================");
  Serial.println("Bridge sensor node - ready to operate");
  Serial.println("==================================================");
  Serial.printf("Client Name: %s\n\n",clientName);
  // showDeviceStatus();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long current_deviceTime_ms = millis();
  uint32_t deviceRelativeTime = 0;  // Use getRelativeTime and store it here
  // uint64_t current_unixTime_ms = RTC_getUnix()*1000ULL;
  // syncTime_setAbsolute(DeviceTime_Unix,current_unixTime_ms);

  // Init Temp struct per mcu loop
  twai_message_t txmsg;
  twai_message_t rxmsg;   
  Mechanical myMechData;
  Electrical myElectData;
  Odometry myOdometryData;
  BAMOCar myBAMOCar;

  // Update WiFi LED status
  (WiFi.status() == WL_CONNECTED) ? 
  digitalWrite(WIFI_LED_INDICATOR, HIGH) : digitalWrite(WIFI_LED_INDICATOR, LOW) ;

  (BPsocketstatus->isConnected == true) ?
  digitalWrite(WS_LED_INDICATOR, HIGH) : digitalWrite(WS_LED_INDICATOR, HIGH) ;

  // Handle WebSocket communication
  if (WiFi.status() == WL_CONNECTED) BPwebSocket->loop();
  
  // Update Each Sensors
    // RPMsensorUpdate(&myMechData); StrokesensorUpdate(&myMechData);
    // ElectSensorsUpdate(&myElectData);
    // GPSupdate(&myOdometryData); IMUupdate(&myOdometryData);
  // // Mock Data
  // mockMechanicalData(&myMechData);
  // mockElectricalData(&myElectData);
  // mockOdometryData(&myOdometryData);
  // mockBAMOCarData(&myBAMOCar);
  // // Later there will be Mock Flag
  
  if (canBusReady) {
    // Receive and process BAMOCar d3 400 CAN frame
    if(CAN32_receiveCAN(&rxmsg) == ESP_OK) process_ResponseBamocarMsg(&rxmsg, &myBAMOCar);
    else Serial.println("BAMO RX Buffer = 0");
    // Request CAN data periodically
    // Will cycle: 0 = motor temp, 1 = controller temp, 2 = DC voltage, 3 = DC current
    if (current_deviceTime_ms - lastBAMOrequest >= CAN_REQUEST_INTERVAL) {
      switch (CANRequest_sequence) {
        case 0:
          pack_RequestBamocarMsg(&txmsg,BAMOCAR_REG_MOTOR_TEMP);      // 0x49
          if(CAN32_sendCAN(&txmsg) != ESP_OK); Serial.println("Motor temp request failed"); 
          break;
        case 1:
          pack_RequestBamocarMsg(&txmsg,BAMOCAR_REG_CONTROLLER_TEMP); // 0x4A
          if(CAN32_sendCAN(&txmsg) != ESP_OK); Serial.println("Controller temp  request failed");
          break;
        case 2:
          pack_RequestBamocarMsg(&txmsg,BAMOCAR_REG_DC_VOLTAGE);      // 0xEB (CORRECTED!)
          if(CAN32_sendCAN(&txmsg) != ESP_OK); Serial.println("Bamo temp request failed");
          break;
        case 3:
          pack_RequestBamocarMsg(&txmsg,BAMOCAR_REG_DC_CURRENT);      // 0x20 (CORRECTED!)
          CAN32_sendCAN(&txmsg);
          break;
      }
      CANRequest_sequence++;
      lastBAMOrequest = millis();
    
      if (CANRequest_sequence > 3) 
        CANRequest_sequence = 0;
    }
  }
  // Calculate power
  if (myBAMOCar.canVoltageValid && myBAMOCar.canCurrentValid) {
    myBAMOCar.power = myBAMOCar.canVoltage * myBAMOCar.canCurrent;
  }

  // Log to SD card
  if (sdCardReady && (current_deviceTime_ms - lastSDLog >= SD_LOG_INTERVAL)) {
    int appenderCount = 5; // increase when scale
    AppenderFunc appenders[appenderCount] = {
      append_Timestamp_toCSVFile,
      // append_La,
      // Comment below to toggle off
      append_MechData_toCSVFile,
      append_ElectData_toCSVFile,
      append_OdometryData_toCSVFile,
      append_BAMOdata_toCSVFile
    };
    void *structArray[appenderCount] = {
      &DeviceTime_Unix,
      // Comment below to toggle off
      &myMechData,
      &myElectData,
      &myOdometryData,
      &myBAMOCar
    };

    logDataToSD(csvFilename, appenders, structArray, appenderCount);
    dataPoint++;
    Serial.print("[SD Card] Logged data point #"); Serial.println(dataPoint - 1);
    lastSDLog = current_deviceTime_ms;
  }

  // Send data to BP Mobile server if registered
  if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
    // Publish BAMOcar power data
    if (current_deviceTime_ms - lastBAMOsend_power >= (1000.0 / BAMO_POWER_SAMPLING_RATE)) {
      publishBAMOpower(&myBAMOCar);
      lastBAMOsend_power = current_deviceTime_ms;
    }
    // Publish temperature data
    if (current_deviceTime_ms - lastBAMOsend_temp >= (1000.0 / BAMO_TEMP_SAMPLING_RATE)) {
      publishBAMOtemp(&myBAMOCar);
      lastBAMOsend_temp = current_deviceTime_ms;
    }
    // Send Mechanical data
    if (current_deviceTime_ms - lastMechSend >= (1000.0 / MECH_SENSORS_SAMPLING_RATE)) {
      publishMechData(&myMechData);
      lastMechSend = current_deviceTime_ms;
    }
    // Send Electrical analog data
    if (current_deviceTime_ms - lastElectSend >= (1000.0 / ELECT_SENSORS_SAMPLING_RATE)) {
      publishElectData(&myElectData);
      lastElectSend = current_deviceTime_ms;
    }
    // Send Electrical fault state data
    if (current_deviceTime_ms - lastElectFaultSend >= (1000.0 / ELECT_FAULT_STAT_SAMPLING_RATE)) {
      publishElectFaultState(&myElectData);
      lastElectFaultSend = current_deviceTime_ms;
    }
  }
  delay(10); // Prevent watchdog issues
}

// ****************************************************************************

// ============================================================================
// WIFI , DEVICE , BPMobile Publishing
// ============================================================================

void initWiFi(const char* ssid, const char* password) {
  Serial.println("--- WiFi Initialization ---");
  Serial.printf("Connecting to: %s\n",ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection FAILED!");
    return;
  }
  Serial.printf("WiFi connected! at IP: %d\n",WiFi.localIP());
  Serial.printf("Signal strength:%d dbm\n",WiFi.RSSI());
}

void showDeviceStatus() {
  unsigned long current_deviceTime_ms = millis();
  
  Serial.println("╔══════════════════════════╗");
  Serial.println("║       SYSTME STATE       ║");
  Serial.println("╠══════════════════════════╣");
  
  // Wifi connection
  Serial.print("║ WiFi:            ");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ CONNECTED");
    Serial.print(" (");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm)  ║");
  } else {
    Serial.println("✗ DISCONNECTED     ║");
  }

  // WebSocket Status
  Serial.print("║ BPMobile WebSocket:       ");
  if (BPsocketstatus->isConnected) {
    Serial.println("✓ CONNECTED        ║");
  } else {
    Serial.println("✗ DISCONNECTED     ║");
  }

  // Time Sync Status
  Serial.print("║ Time Sync:       ");
  if (syncTime_isSynced()) {
    Serial.println("✓ SYNCED           ║");
  } else {
    Serial.println("✗ NOT SYNCED       ║");
  }
  Serial.println("╠════════════════════════════════════════╣");

  // CAN Bus status
  Serial.print("║ CAN Bus ready:       ");
  if (canBusReady) {
    Serial.println("✓ SYNCED           ║");
  } else {
    Serial.println("✗ NOT SYNCED       ║");
  }
  Serial.println("╠════════════════════════════════════════╣");
  lastDebugPrint = current_deviceTime_ms;
}

// Publish Voltage, Current , Power consumption
void publishBAMOpower(BAMOCar* bamocar) {
  unsigned long long unixTimestamp = syncTime_getRelative(DeviceTime_Unix);

  if (unixTimestamp < 1000000000000ULL) return;

  // Send CAN voltage
  if (bamocar->canVoltageValid) {
    JsonDocument voltDoc;
    voltDoc["type"] = "data";
    voltDoc["topic"] = "power/can_voltage";
    voltDoc["data"]["value"] = bamocar->canVoltage;
    voltDoc["data"]["sensor_id"] = "BAMOCAR_CAN";
    voltDoc["timestamp"] = unixTimestamp;

    String voltMsg;
    serializeJson(voltDoc, voltMsg);
    BPwebSocket->sendTXT(voltMsg);
  }

  // Send CAN current
  if (bamocar->canCurrentValid) {
    JsonDocument currDoc;
    currDoc["type"] = "data";
    currDoc["topic"] = "power/can_current";
    currDoc["data"]["value"] = bamocar->canCurrent;
    currDoc["data"]["sensor_id"] = "BAMOCAR_CAN";
    currDoc["timestamp"] = unixTimestamp;

    String currMsg;
    serializeJson(currDoc, currMsg);
    BPwebSocket->sendTXT(currMsg);
  }

  // Send calculated power
  JsonDocument powDoc;
  powDoc["type"] = "data";
  powDoc["topic"] = "power/power";
  powDoc["data"]["value"] = bamocar->power;
  powDoc["data"]["sensor_id"] = "CALCULATED";
  powDoc["timestamp"] = unixTimestamp;

  String powMsg;
  serializeJson(powDoc, powMsg);
  BPwebSocket->sendTXT(powMsg);

  Serial.print("[BP Mobile] Power: V=");
  Serial.print(bamocar->canVoltage, 2);
  Serial.print("V, I=");
  Serial.print(bamocar->canCurrent, 3);
  Serial.print("A, P=");
  Serial.print(bamocar->power, 2);
  Serial.println("W");
}
// Publihs controller, and sensed Motor temp
void publishBAMOtemp(BAMOCar* bamocar) {
  unsigned long long unixTimestamp = syncTime_getRelative(DeviceTime_Unix);
  // timeout function
  if (unixTimestamp < 1000000000000ULL) return;

  // Send motor temperature
  if (bamocar->motorTempValid) {
    JsonDocument motorDoc;
    motorDoc["type"] = "data";
    motorDoc["topic"] = "motor/temperature";
    motorDoc["data"]["value"] = bamocar->motorTemp2;
    motorDoc["data"]["sensor_id"] = "BAMOCAR_MOTOR";
    motorDoc["timestamp"] = unixTimestamp;

    String motorMsg;
    serializeJson(motorDoc, motorMsg);
    BPwebSocket->sendTXT(motorMsg);
  }

  // Send controller temperature
  if (bamocar->controllerTempValid) {
    JsonDocument ctrlDoc;
    ctrlDoc["type"] = "data";
    ctrlDoc["topic"] = "motor/controller_temperature";
    ctrlDoc["data"]["value"] = bamocar->controllerTemp;
    ctrlDoc["data"]["sensor_id"] = "BAMOCAR_CTRL";
    ctrlDoc["timestamp"] = unixTimestamp;

    String ctrlMsg;
    serializeJson(ctrlDoc, ctrlMsg);
    BPwebSocket->sendTXT(ctrlMsg);

    Serial.print("[BP Mobile] CAN Temps: Motor=");
    Serial.print(bamocar->motorTemp2, 1);
    Serial.print("°C, Controller=");
    Serial.print(bamocar->controllerTemp, 1);
    Serial.println("°C");
  }
}

void publishMechData(Mechanical* MechSensors) {
  unsigned long long unixTimestamp = syncTime_getRelative(DeviceTime_Unix);
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
  unsigned long long unixTimestamp = syncTime_getRelative(DeviceTime_Unix);
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
  unsigned long long unixTimestamp = syncTime_getRelative(DeviceTime_Unix);
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

  // Define topic metadata
  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();
  
  // CAN Voltage metadata
  JsonObject canVoltMeta = metadata["power/can_voltage"].to<JsonObject>();
  canVoltMeta["description"] = "DC Link Voltage (Bamocar CAN)";
  canVoltMeta["unit"] = "V";
  canVoltMeta["sampling_rate"] = BAMO_TEMP_SAMPLING_RATE;
  
  // CAN Current metadata
  JsonObject canCurrMeta = metadata["power/can_current"].to<JsonObject>();
  canCurrMeta["description"] = "Motor DC Current (Bamocar CAN)";
  canCurrMeta["unit"] = "A";
  canCurrMeta["sampling_rate"] = BAMO_TEMP_SAMPLING_RATE;
  
  // Power metadata
  JsonObject powMeta = metadata["power/power"].to<JsonObject>();
  powMeta["description"] = "Power consumption (calculated)";
  powMeta["unit"] = "W";
  powMeta["sampling_rate"] = BAMO_POWER_SAMPLING_RATE;
  
  // Motor temperature metadata
  JsonObject motorTempMeta = metadata["motor/temperature"].to<JsonObject>();
  motorTempMeta["description"] = "Motor temperature (Bamocar)";
  motorTempMeta["unit"] = "°C";
  motorTempMeta["sampling_rate"] = BAMO_TEMP_SAMPLING_RATE;
  
  // Controller temperature metadata
  JsonObject ctrlTempMeta = metadata["motor/controller_temperature"].to<JsonObject>();
  ctrlTempMeta["description"] = "Motor controller/IGBT temperature (Bamocar)";
  ctrlTempMeta["unit"] = "°C";
  ctrlTempMeta["sampling_rate"] = BAMO_TEMP_SAMPLING_RATE;

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

//  External Interrupt service Routine -- Encoder pulse counter ISR
void RPMinit(hw_timer_t* My_timer, int period) {
  pinMode(ENCODER_PINL, INPUT); // The module has internal Pull up Resistor
  pinMode(ENCODER_PINR, INPUT); // The module has internal Pull up Resistor
  // ------ interrupt
  attachInterrupt(ENCODER_PINL, ISR_COUNT_L, FALLING);
  attachInterrupt(ENCODER_PINR, ISR_COUNT_R, FALLING);

  // Timer interrupt 100 ms (May change to input capture or just a trigger)
  int refresh_calculation_time = period * 1000;
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &ISRreset, true);
  timerAlarmWrite(My_timer, refresh_calculation_time, true);
  timerAlarmEnable(My_timer);
}

void IRAM_ATTR ISR_COUNT_L() {
  counterL++;
}
void IRAM_ATTR ISR_COUNT_R() {
  counterR++;
}
void IRAM_ATTR ISRreset() {
  startCalculate = true;
}
void RPMsensorUpdate(Mechanical *MechSensors){
  // ----- RPM sensors (Wheel speeds)
  if (startCalculate) {
    noInterrupts();
    MechSensors->Wheel_RPM_L = (30000) / Period * ((float)counterL / ENCODER_N);
    MechSensors->Wheel_RPM_R = (30000) / Period * ((float)counterR / ENCODER_N);
    counterL = 0;
    counterR = 0;
    startCalculate = false;
    interrupts();
  }
}

void StrokesensorInit(){
  // The Roll is pin 5 or 6
  pinMode(STR_Roll,INPUT_PULLDOWN);
  pinMode(STR_Heave,INPUT_PULLDOWN);
}
void StrokesensorUpdate(Mechanical *MechSensors){
  // ----- Stroke distances
  // Conversion from ADC value back to its sensor reading voltage
  // Conversion of sensor reading voltage KPM18-50mm distance, total distance is 52.91 mm
  MechSensors->STR_Heave_mm = analogRead(STR_Roll) * (aref / pwmres);
  MechSensors->STR_Roll_mm = analogRead(STR_Heave) * (aref / pwmres);
}


void ElectSensorsInit(){
  pinMode(I_SENSE_PIN,INPUT_PULLDOWN);
  pinMode(TMP_PIN,INPUT_PULLDOWN);
  pinMode(APPS_PIN,INPUT_PULLDOWN);
  pinMode(BPPS_PIN,INPUT_PULLDOWN);

  pinMode(AMS_OK_PIN,INPUT_PULLDOWN);
  pinMode(IMD_OK_PIN,INPUT_PULLDOWN);
  pinMode(HV_ON_PIN,INPUT_PULLDOWN);
  pinMode(BSPD_OK_PIN,INPUT_PULLDOWN);
}
void ElectSensorsUpdate(Electrical *ElectSensors){
  // Read raw ADC values
  uint16_t raw_i_sense = analogRead(I_SENSE_PIN);
  uint16_t raw_tmp = analogRead(TMP_PIN);
  uint16_t raw_apps = analogRead(APPS_PIN);
  uint16_t raw_bpps = analogRead(BPPS_PIN);

  // Convert ADC to voltage (0-3.3V mapped to 0-4095)
  float volt_i_sense = (raw_i_sense / (float)pwmres) * aref;
  float volt_tmp = (raw_tmp / (float)pwmres) * aref;
  float volt_apps = (raw_apps / (float)pwmres) * aref;
  float volt_bpps = (raw_bpps / (float)pwmres) * aref;

  // --- Hall Effect Current Sensor ---
  // Formula: I = (V_sensor - V_offset) / Sensitivity
  ElectSensors->I_SENSE = (volt_i_sense - i_sense_offset) / i_sense_sensitivity;

  // --- Temperature Sensor (NTC Thermistor in voltage divider) ---
  // Calculate NTC resistance from voltage divider
  // Vout = Vcc * (R_NTC / (R_series + R_NTC))
  // Solving for R_NTC: R_NTC = (Vout * R_series) / (Vcc - Vout)
  if (volt_tmp < aref - 0.01) {  // Avoid division by zero
    uint16_t ntc_resistance = (volt_tmp * tmp_series_res) / (aref - volt_tmp);
    ElectSensors->TMP = convertNTCtoTemp(ntc_resistance);
  } else {
    ElectSensors->TMP = 0.0;  // For iInvalid reading
  }

  // --- APPS (Accelerator Position) - Linear potentiometer ---
  // Voltage maps linearly to distance (0V = 0mm, 3.3V = 75mm)
  ElectSensors->APPS = (volt_apps / aref) * apps_max_dist_mm + apps_offset_mm;

  // --- BPPS (Brake Position) - Linear potentiometer ---
  // Voltage maps linearly to distance (0V = 0mm, 3.3V = 75mm)
  ElectSensors->BPPS = (volt_bpps / aref) * bpps_max_dist_mm + bpps_offset_mm;

  // --- Digital Fault Status Signals ---
  ElectSensors->AMS_OK = digitalRead(AMS_OK_PIN);
  ElectSensors->IMD_OK = digitalRead(IMD_OK_PIN);
  ElectSensors->HV_ON = digitalRead(HV_ON_PIN);
  ElectSensors->BSPD_OK = digitalRead(BSPD_OK_PIN);
}

/* THERMISTOR TEMPERATURE CONVERSION FUNCTIONS */
  float convertBamocarTemp(uint16_t rawValue) {
    // Bamocar temperature conversion formula
    // Common conversion: Temperature in 0.1°C steps
    // Example: rawValue = 250 means 25.0°C
    float tempC = (float)rawValue / 10.0;
    return tempC;
  }
  // KTY Sensor lookup table for Motor Temperature
  // Resistance (Ω) vs Temperature (°C)
  float convertKTYtoTemp(uint16_t resistance) {
    // KTY lookup table from your image
    const int numPoints = 15;
    const int tempTable[numPoints] = {-40, -20, 0, 20, 25, 40, 60, 80, 100, 120, 140, 160, 180, 200};
    const int resistTable[numPoints] = {688, 813, 1000, 1079, 1115, 1203, 1300, 1397, 1494, 1591, 1688, 1785, 1882, 1979};
    
    // If resistance is out of range, return boundary values
    if (resistance <= resistTable[0]) return tempTable[0];  // Too cold
    if (resistance >= resistTable[numPoints-1]) return tempTable[numPoints-1];  // Too hot
    
    // Linear interpolation between points
    for (int i = 0; i < numPoints - 1; i++) {
      if (resistance >= resistTable[i] && resistance <= resistTable[i + 1]) {
        // Interpolate
        float t1 = tempTable[i];
        float t2 = tempTable[i + 1];
        float r1 = resistTable[i];
        float r2 = resistTable[i + 1];
        
        float temperature = t1 + (resistance - r1) * (t2 - t1) / (r2 - r1);
        return temperature;
      }
    }
    
    return 25.0; // Default fallback
  }
  // NTC Sensor lookup table for Controller Temperature
  // Resistance (Ω) vs Temperature (°C)
  float convertNTCtoTemp(uint16_t rawValue) {
    // NTC lookup table - Column 3 values [n] vs Temperature [°C]
    // This matches the actual Bamocar controller sensor table
    const int numPoints = 28;
    const int tempTable[numPoints] = {
      -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 
      35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
    };
    const int valueTable[numPoints] = {
      16245, 16308, 16387, 16487, 16609, 16759, 16938, 17151, 17400, 17688, 18017, 18387, 18797, 19247,
      19733, 20250, 20793, 21357, 21933, 22515, 23097, 23671, 24232, 24775, 25296, 25792, 26261, 26702
    };
    
    // If value is out of range, return boundary values
    if (rawValue <= valueTable[0]) return tempTable[0];  // Too cold
    if (rawValue >= valueTable[numPoints-1]) return tempTable[numPoints-1];  // Too hot
    
    // Linear interpolation between points
    for (int i = 0; i < numPoints - 1; i++) {
      if (rawValue >= valueTable[i] && rawValue <= valueTable[i + 1]) {
        // Interpolate
        float t1 = tempTable[i];
        float t2 = tempTable[i + 1];
        float v1 = valueTable[i];
        float v2 = valueTable[i + 1];
        
        float temperature = t1 + (rawValue - v1) * (t2 - t1) / (v2 - v1);
        return temperature;
      }
    }
    
    return 25.0; // Default fallback
  }

void IMUinit(TwoWire* WireIMU, int sda , int scl){
  WireIMU->begin(sda,scl);
  byte status = mpu.begin();
  Serial.printf("MPU6050 status: %c\n", status);
}
void IMUcalibrate(){
  Serial.println("Calculating offsets, do not move MPU6050");
  // delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}
void IMUupdate(Odometry* myimu){
  // --- IMU update ---
  mpu.update();
  myimu->imu_accelx = mpu.getAccX();
  myimu->imu_accely = mpu.getAccY();
  myimu->imu_accely = mpu.getAccZ();
  myimu->imu_gyroz = mpu.getGyroX();
  myimu->imu_gyroy = mpu.getGyroY();
  myimu->imu_gyroz = mpu.getGyroZ();
}

void GPSinit(HardwareSerial gpsSerial){
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  Serial.println(F("AN251 GPS Module with TinyGPS++"));
  Serial.println(F("Waiting for GPS fix..."));
}
void GPSupdate(Odometry *mygps) {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      Serial.println(F("\n===== GPS Lat lng ====="));
      if (gps.location.isValid()) {
        mygps->gps_lat= gps.location.lat();
        mygps->gps_lng= gps.location.lng();
        mygps->gps_age= gps.location.age();
        mygps->gps_course= gps.course.deg(); //deg
        mygps->gps_speed= gps.speed.mps(); // mps
      } else {
        Serial.println(F("INVALID"));
      }
    }
  }  
}

// ============================================================================
// Time source
// ============================================================================
// Time source provider 
void RTCinit(TwoWire* I2Cbus,int sda, int scl){
  I2Cbus->begin(sda,scl);
  if (!rtc.begin(I2Cbus)) { // Initialize the I2C connection
    Serial.println("Couldn't find RTC");
    while (1); // Stop if RTC is not found
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // set at compile time
  }
}
void RTCcalibrate(uint64_t unix_time){
  rtc.adjust(DateTime(unix_time));
}
void RTCcalibrate(){
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}
uint32_t RTC_getUnix(){
  uint32_t current_deviceTime_ms = rtc.now().unixtime(); return current_deviceTime_ms;
}
String   RTC_getISO(){
  String current_deviceTime_ms = rtc.now().timestamp(DateTime::TIMESTAMP_FULL); 
  return current_deviceTime_ms;
}

// ============================================================================
// CSV Data Appender Functions
// ============================================================================
// Timestamp Appender - writes Unix timestamp, datetime, and data point counter
void append_Timestamp_toCSVFile(File& dataFile, void* data) {
  uint64_t* newUnixTime= (uint64_t*)data;
  
  // tries resync one time
  if (!syncTime_isSynced()) {
    syncTime_resync(DeviceTime_Unix,*newUnixTime);
  }
  dataFile.print(dataPoint);
  dataFile.print(",");
  dataFile.print(*newUnixTime);
  dataFile.print(",");
}

void append_TrackInfo_toCSVFile(File& dataFile, void* data){
  (void)data;  // Unused - access globals directly

  uint64_t sessionTime = syncTime_getRelative(DeviceTime_Unix);
  // uint64_t 
  uint64_t timestamp;
  char dateTimeStr[32] = "";

  if (syncTime_isSynced()) {
    timestamp = (sessionTime / 1000ULL);  // Convert ms to seconds
    syncTime_formatUnix(dateTimeStr, sessionTime,7);
  } else {
    timestamp = millis() / 1000;
    strcpy(dateTimeStr, "NOT_SYNCED");
  }

  dataFile.print(timestamp);
  dataFile.print(",");
  dataFile.print(dateTimeStr);
  dataFile.print(",");
  dataFile.print(dataPoint);
  dataFile.print(",");

}
// BAMOCar Appender - writes motor controller data from CAN bus
void append_BAMOdata_toCSVFile(File& dataFile, void* data) {
  BAMOCar* bamocar = static_cast<BAMOCar*>(data);

  // Write CAN voltage
    dataFile.print((bamocar->canVoltageValid) ? bamocar->canVoltage : 0.0, 2);
    dataFile.print(",");

    // Write CAN current
    dataFile.print((bamocar->canCurrentValid) ? bamocar->canCurrent : 0.0, 2);
    dataFile.print(",");

    // Write power
    dataFile.print(bamocar->power, 2);
    dataFile.print(",");

    // Write motor temperature
    dataFile.print((bamocar->motorTempValid) ? bamocar->motorTemp2 : 0.0, 1);
    dataFile.print(",");

    // Write controller temperature
    dataFile.print((bamocar->controllerTempValid) ? bamocar->controllerTemp : 0.0, 1);
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

// ============================================================================
// BAMOCar CAN Bus function
// ============================================================================
// Fixed ID 0x201 is request
void pack_RequestBamocarMsg(twai_message_t* msg,uint8_t regAddress) {
  
  // Set Request ID
  msg->identifier = BAMOCAR_REQUEST_ID;
  msg->extd = 0;  // Standard frame
  msg->rtr = 0;   // Data frame
  msg->data_length_code = 3;
  
  // Bamocar request format: [0x3D, register_address, 0x00]
  msg->data[0] = 0x3D;  // Read command
  msg->data[1] = regAddress;
  msg->data[2] = 0x00; // Command send once, since we will do request response altogether
  
    // Print register name
    if (regAddress == BAMOCAR_REG_MOTOR_TEMP) Serial.print("Motor Temp");
    else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) Serial.print("Controller Temp");
    else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) Serial.print("DC Voltage");
    else if (regAddress == BAMOCAR_REG_DC_CURRENT) Serial.print("DC Current");
    else Serial.print("Unkcurrent_deviceTime_msn");
    Serial.println(")");
}

// Non Fixed is not request
void process_ResponseBamocarMsg(twai_message_t* msg, BAMOCar* bamocar) {

  // decodeBAMO_rawmsg(msg);

    // Check if response from Bamocar (ID 0x181[Response ID])
    if (msg->identifier == 0x181 && msg->data_length_code >= 3) {
      uint8_t regAddress = msg->data[0];

      // Get raw 16-bit value (little-endian: low byte first, high byte second)
      uint16_t rawValue = (msg->data[2] << 8) | msg->data[1];


      // === MOTOR TEMPERATURE (Register 0x49) ===
      if (regAddress == BAMOCAR_REG_MOTOR_TEMP) {
        // Try single byte interpretation: byte[1] / 10
        bamocar->motorTemp1 = (float)msg->data[1] / 10.0;
        bamocar->motorTemp2 = bamocar->motorTemp1 + 8.0; // Assume second sensor is +8°C
        bamocar->motorTempValid = true;

        Serial.print("[CAN DECODED] Motor Temp: ");
        Serial.print(bamocar->motorTemp2, 1);
        Serial.print(" °C (raw byte: 0x");
        Serial.print(msg->data[1], HEX);
        Serial.print(" = ");
        Serial.print(msg->data[1]);
        Serial.println(")");
      }

      // === CONTROLLER TEMPERATURE (Register 0x4A, NTC Sensor) ===
      else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) {
        // Value is resistance in Ohms
        bamocar->controllerTemp = convertNTCtoTemp(rawValue);
        bamocar->controllerTempValid = true;

        Serial.print("[CAN DECODED] Controller Temp: ");
        Serial.print(bamocar->controllerTemp, 1);
        Serial.print(" °C (resistance: ");
        Serial.print(rawValue);
        Serial.println(" Ω)");
      }

      // === DC VOLTAGE (Register 0xEB) ===
      else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) {
        // Use formula from reference code: rawValue / 55.1204
        // Raw value ~12520 → 227.2V (matches your multimeter!)
        bamocar->canVoltage = (float)rawValue / 55.1204;
        bamocar->canVoltageValid = true;

        Serial.print("[CAN DECODED] DC Voltage: ");
        Serial.print(bamocar->canVoltage, 2);
        Serial.print(" V (raw: ");
        Serial.print(rawValue);
        Serial.println(")");
      }

      // === DC CURRENT (Register 0x20) ===
      else if (regAddress == BAMOCAR_REG_DC_CURRENT) {
        // Special handling: 0xFFFF (-1 signed) means "invalid/not available"
        if (rawValue == 0xFFFF) {
          // Invalid reading - keep previous value or set to 0
          Serial.println("[CAN DECODED] DC Current: INVALID (0xFFFF)");
        } else {
          // Use formula from reference code: rawValue × 0.373832
          bamocar->canCurrent = (float)rawValue * 0.373832;
          bamocar->canCurrentValid = true;

          Serial.print("[CAN DECODED] DC Current: ");
          Serial.print(bamocar->canCurrent, 2);
          Serial.print(" A (raw: ");
          Serial.print(rawValue);
          Serial.println(")");
        }
      }
      // === DC CURRENT (Register 0x20) - Duplicate case, keeping for backward compatibility ===
      else if (regAddress == BAMOCAR_REG_DC_CURRENT) {
        // Special handling: 0xFFFF (-1 signed) means "invalid/not available"
        if (rawValue == 0xFFFF || rawValue > 60000) {
          // Invalid reading - ignore it, keep previous valid value
          Serial.println("[CAN DECODED] DC Current: INVALID (0xFFFF) - ignored");
          // Don't update bamocar->canCurrent or bamocar->canCurrentValid
        } else {
          // Use formula from reference code: rawValue × 0.373832
          bamocar->canCurrent = (float)rawValue * 0.373832;
          bamocar->canCurrentValid = true;

          Serial.print("[CAN DECODED] DC Current: ");
          Serial.print(bamocar->canCurrent, 2);
          Serial.print(" A (raw: ");
          Serial.print(rawValue);
          Serial.println(")");
        }
      }

      // === UNKcurrent_deviceTime_msN REGISTER ===
      else {
        Serial.print("[CAN] Unkcurrent_deviceTime_msn register 0x");
        Serial.print(regAddress, HEX);
        Serial.print(" with value: ");
        Serial.println(rawValue);
      }
    }

}

// ============================================================================
// BAMOCar CAN Helper function - SHOWS ALL POSSIBLE INTERPRETATIONS
// ============================================================================

// Should be written to analyze and interpret all value into physica
void analyzeBamoData(uint8_t reg, uint8_t* data) {
  Serial.print("\n▼ Analyzing Register 0x");
  Serial.print(reg, HEX);
  Serial.print(" (");
  
  // Print register name
  if (reg == BAMOCAR_REG_MOTOR_TEMP) Serial.print("Motor Temp");
  else if (reg == BAMOCAR_REG_CONTROLLER_TEMP) Serial.print("Controller Temp");
  else if (reg == BAMOCAR_REG_DC_VOLTAGE) Serial.print("DC Voltage");
  else if (reg == BAMOCAR_REG_DC_CURRENT) Serial.print("DC Current");
  else Serial.print("Unkcurrent_deviceTime_msn");
  
  Serial.println(")");
  Serial.println("  Raw bytes: 0x" + String(data[1], HEX) + " 0x" + String(data[2], HEX) + " 0x" + String(data[3], HEX));
  
  // Single byte interpretations
  Serial.println("  ┌─ Single Byte Options:");
  Serial.print("  │  Byte[1] = ");
  Serial.print(data[1]);
  Serial.print(" → ");
  Serial.print(data[1] / 10.0, 1);
  Serial.print(" (÷10) or ");
  Serial.print(data[1] / 100.0, 2);
  Serial.println(" (÷100)");
  
  // Two byte interpretations (little-endian)
  uint16_t val_LE = (data[2] << 8) | data[1];
  Serial.println("  ├─ Two Bytes (Little-Endian):");
  Serial.print("  │  Value = ");
  Serial.print(val_LE);
  Serial.print(" → ");
  Serial.print(val_LE / 10.0, 1);
  Serial.print(" (÷10) or ");
  Serial.print(val_LE / 100.0, 2);
  Serial.println(" (÷100)");
  
  // Two byte interpretations (big-endian)
  uint16_t val_BE = (data[1] << 8) | data[2];
  Serial.println("  ├─ Two Bytes (Big-Endian):");
  Serial.print("  │  Value = ");
  Serial.print(val_BE);
  Serial.print(" → ");
  Serial.print(val_BE / 10.0, 1);
  Serial.print(" (÷10) or ");
  Serial.print(val_BE / 100.0, 2);
  Serial.println(" (÷100)");
  
  // Signed interpretations
  int16_t signed_LE = (int16_t)val_LE;
  Serial.println("  └─ Signed (Little-Endian):");
  Serial.print("     Value = ");
  Serial.print(signed_LE);
  Serial.print(" → ");
  Serial.print(signed_LE / 10.0, 1);
  Serial.print(" (÷10) or ");
  Serial.print(signed_LE / 100.0, 2);
  Serial.println(" (÷100)");
  Serial.println();

}

void decodeBAMO_rawmsg(twai_message_t &message) {
    Serial.print("[CAN RAW] ID: 0x");
    Serial.print(message.identifier, HEX);
    Serial.print(" | DLC: ");
    Serial.print(message.data_length_code);
    Serial.print(" | Data: ");
    
    for (int i = 0; i < message.data_length_code; i++) {
      if (message.data[i] < 0x10) Serial.print("0");
      Serial.print(message.data[i], HEX);
      Serial.print(" ");
    }
    
    // Decode based on actual Bamocar format (DLC=4)
    if (message.identifier == 0x181 && message.data_length_code == 4) {
      uint8_t reg = message.data[0];
      uint16_t value = (message.data[2] << 8) | message.data[1];
      
      Serial.print("| REG: 0x");
      Serial.print(reg, HEX);
      Serial.print(" VAL: ");
      Serial.print(value);
      Serial.print(" (");
      Serial.print(value / 10.0, 1);
      Serial.print(")");
    }
    Serial.println();
}

void scanBamocarIDs() {
  Serial.println("╔═=══════════════════════════════════════╗");
  Serial.println("║     DISCOVER TOOL BAMOCAR CAN ID       ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  // Common Bamocar request IDs to try
  uint16_t requestIDs[] = {0x201, 0x210, 0x220, 0x200};
  uint8_t testReg = BAMOCAR_REG_MOTOR_TEMP;
  
  for (int i = 0; i < 4; i++) {
    Serial.print("║ Testing Request ID: 0x");
    Serial.print(requestIDs[i], HEX);
    Serial.println("              ║");
    
    twai_message_t message;
    message.identifier = requestIDs[i];
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 3;
    message.data[0] = 0x3D;
    message.data[1] = testReg;
    message.data[2] = 0x00;
    
    if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
      Serial.println("║   Request sent, waiting for response...║");
      delay(200);  // Wait for response
      
      // Check for any message
      twai_message_t response;
      if (twai_receive(&response, pdMS_TO_TICKS(100)) == ESP_OK) {
        Serial.print("║   ✓ Got response from ID: 0x");
        Serial.print(response.identifier, HEX);
        Serial.println("       ║");
        decodeBAMO_rawmsg(response);
      } else {
        Serial.println("║   ✗ No response                        ║");
      }
    }
    Serial.println("╠════════════════════════════════════════╣");
    delay(500);
  }
  Serial.println("╚════════════════════════════════════════╝\n");
}

// ============================================================================
// MOCK DATA FUNCTIONS FOR TESTING
// ============================================================================
// --- MECHANICAL SENSORS MOCK DATA ---
void mockMechanicalData(Mechanical *MechSensors) {
  // Simulate wheel RPM (0-500 RPM range with some variation)
  MechSensors->Wheel_RPM_L = 250.0 + random(-50, 50);
  MechSensors->Wheel_RPM_R = 245.0 + random(-50, 50);

  // Simulate stroke sensors (0-75mm range)
  MechSensors->STR_Heave_mm = 35.0 + random(-10, 10);
  MechSensors->STR_Roll_mm = 40.0 + random(-10, 10);
}

// --- ELECTRICAL SENSORS MOCK DATA ---
void mockElectricalData(Electrical *ElectSensors) {
  // Simulate current sensor (-20A to +20A range)
  ElectSensors->I_SENSE = 5.0 + random(-20, 80) / 10.0;

  // Simulate temperature (20-80°C range)
  ElectSensors->TMP = 45.0 + random(-10, 20);

  // Simulate APPS and BPPS positions (0-75mm range)
  ElectSensors->APPS = 30.0 + random(-15, 30);
  ElectSensors->BPPS = 10.0 + random(-5, 15);

  // Simulate digital fault status (mostly OK, occasional faults)
  ElectSensors->AMS_OK = random(0, 10) > 1;    // 90% OK
  ElectSensors->IMD_OK = random(0, 10) > 1;    // 90% OK
  ElectSensors->HV_ON = random(0, 10) > 2;     // 80% ON
  ElectSensors->BSPD_OK = random(0, 10) > 1;   // 90% OK
}

// --- ODOMETRY SENSORS MOCK DATA ---
void mockOdometryData(Odometry *OdomSensors) {
  // GPS Mock Data (Bangkok area coordinates)
  OdomSensors->gps_lat = 13.7563 + random(-100, 100) / 10000.0;
  OdomSensors->gps_lng = 100.5018 + random(-100, 100) / 10000.0;
  OdomSensors->gps_age = random(0, 1000);
  OdomSensors->gps_course = random(0, 360);
  OdomSensors->gps_speed = random(0, 50);  // 0-50 m/s

  // IMU Mock Data (accelerometer in m/s^2)
  OdomSensors->imu_accelx = random(-100, 100) / 50.0;
  OdomSensors->imu_accely = random(-100, 100) / 50.0;
  OdomSensors->imu_accelz = 9.81 + random(-50, 50) / 100.0;

  // IMU Mock Data (gyroscope in deg/s)
  OdomSensors->imu_gyrox = random(-50, 50) / 10.0;
  OdomSensors->imu_gyroy = random(-50, 50) / 10.0;
  OdomSensors->imu_gyroz = random(-50, 50) / 10.0;
}

// --- BAMOCAR D3 400 MOCK DATA ---
void mockBAMOCarData(BAMOCar *BamoCar) {
  // Motor temperatures (30-80°C range)
  BamoCar->motorTemp1 = 55.0 + random(-10, 15);
  BamoCar->motorTemp2 = BamoCar->motorTemp1 + random(5, 10);
  BamoCar->motorTempValid = true;

  // Controller temperature (30-70°C range)
  BamoCar->controllerTemp = 50.0 + random(-10, 15);
  BamoCar->controllerTempValid = true;

  // DC Link Voltage (200-240V range for typical EV)
  BamoCar->canVoltage = 220.0 + random(-20, 20);
  BamoCar->canVoltageValid = true;

  // DC Current (0-100A range)
  BamoCar->canCurrent = 30.0 + random(-10, 50);
  BamoCar->canCurrentValid = true;

  // Calculated power (V * I)
  BamoCar->power = BamoCar->canVoltage * BamoCar->canCurrent;
}
