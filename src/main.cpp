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

#include <BP_mobile_util.h>
#include <SD32_util.h>
#include <syncTime_util.h>
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
BPMobileConfig BPMobile(&webSockets);
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
const char* csvHeader = "Timestamp,DateTime,DataPoint,CAN_Voltage(V),CAN_Current(A),Power(W),MotorTemp(C)"
                        ",ControllerTemp(C),Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm";
bool sdCardReady = false;
unsigned long lastSDLog = 0;
int dataPoint = 0;
int sessionNumber = 0;  // For unique filename generation
/* SD card SAMPLING INTERVAL */
const unsigned long SD_LOG_INTERVAL = 500; // Log to SD every 0.5s
void logDataToSD(char* csvFilename);

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

unsigned long lastDebugPrint = 0;// DC Current (0.1A units, signed)
const unsigned long CAN_REQUEST_INTERVAL = 200;  // Request every 100ms (May up it to 200ms)

bool canBusReady;
unsigned long lastCANRequest = 0;
// CANRequest_sequence
uint8_t CANRequest_sequence = 0;  // 0=motor temp, 1=controller temp, 2=voltage, 3=current
unsigned long lastPowerSend = 0;
unsigned long lastCANSend = 0;
// New timers for RPM and stroke publishes
unsigned long lastRPMSend = 0;
unsigned long lastStrokeSend = 0;
/* Motor/Motor controller's parameters (CAN data, temp, Power) */
float power = 0.0;

// Bamocar CAN data
float motorTemp1 = 0.0;
float motorTemp2 = 0.0;
float controllerTemp = 0.0;
float canVoltage = 0.0;      // DC Link Voltage from CAN
float canCurrent = 0.0;      // Motor Current from CAN
bool motorTempValid = false;
bool controllerTempValid = false;
bool canVoltageValid = false;
bool canCurrentValid = false;
// CAN Bus functions
void pack_RequestBamocarMsg(twai_message_t* msg,uint8_t regAddress);
void process_ResponseBamocarMsg(twai_message_t* msg);
float convertBamocarTemp(uint16_t rawValue);
void decodeBAMO_rawmsg(twai_message_t &message);
void scanBamocarIDs();
void analyzeCANValue(uint8_t reg, uint8_t* data);

// ============================================================================
// MEHCHANICAL SENSORS
// ============================================================================
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
void RPMsensorUpdate();

// --- Stroke Sensors variables (Heave and Roll Distance) ---
#define stroke1_black 6
#define stroke2_green 5
float distance_stroke1_black = 0.0;
float distance_stroke2_green = 0.0;
float potvolts1_black = 0.0;
float potvolts2_green = 0.0;
const float aref = 3.3; 
const int pwmres = 4095; // 12 bit ADC resolution
const float max_distance1 = 52.91; // recalibrated with vernier -> Needs to recheck
const float max_distance2 = 75.00; // Not_sure needs to check again
void StrokesensorInit();
void StrokesensorUpdate();


// ============================================================================
// ELECTRICAL SENSORS
// ============================================================================

// --- THERMISTOR TEMPERATURE CONVERSION FUNCTIONS ---
float convertNTCtoTemp(uint16_t resistance);
float convertKTYtoTemp(uint16_t resistance);

// ============================================================================
// ODOMETRY SENSORS
// ============================================================================

// There I think I should unite odometry as one, but for now ... 
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
}Odometry; // Maybe this should be after fusion?

// --- IMU (MPU6050) ---
#define IMU_SDA 42
#define IMU_SCL 41

MPU6050 mpu(Wire1);
unsigned long imu_timer = 0;
void IMUinit(TwoWire* WireIMU,int sda, int scl);
void IMUcalibrate();
void IMUupdate(Odometry* myimu);

// --- GPS (TinyGPS++ ANS251 GNSS) ---
#define GPS_RX_PIN 2  // Connect to GPS TX
#define GPS_TX_PIN 1  // Connect to GPS RX
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
#define GPS_BAUD 9600
void GPSinit(HardwareSerial gpsSerial);
void GPSlocation();
void GPS_ISOdatetime();
void GPSext_info();
void GPSupdate(Odometry* mygps);

// --- RTC Time provider (DS3231MZ+) ---
#define RTC_SDA 42
#define RTC_SCL 41
RTC_DS3231 rtc;
void RTCinit(TwoWire* I2Cbus, int sda, int scl);
void RTCcalibrate(uint64_t unix_time);
void RTCcalibrate();
uint64_t RTC_getUnix();
String   RTC_getISO();

// ============================================================================
// SENSOR PUBLISHING
// ============================================================================
const float POWER_SAMPLING_RATE = 2.0;     // 2 Hz for power data
const float CAN_SAMPLING_RATE = 1.0;       // 1 Hz for CAN data to BP Mobile

// Add RPM and Stroke sampling rates
const float RPM_SAMPLING_RATE = 2.0;       // 2 Hz for wheel RPM publish
const float STROKE_SAMPLING_RATE = 2.0;    // 2 Hz for stroke distance publish

// Data publishing functions
void sendPowerData();
void sendCANData();
void sendRPMData();
void sendStrokeData();


// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Wire1.be
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  delay(1000);

    // Setup status LED
  pinMode(WIFI_LED_INDICATOR, OUTPUT);
  digitalWrite(WIFI_LED_INDICATOR, LOW);  // LED off initially

  Serial.println("==================================================");
  Serial.println("Bridge sensor node - Rear side is ready to operate");
  Serial.println("==================================================");
  Serial.printf("Client Name: %d\n\n",clientName);
  
  // Initialize CAN Bus with CAN32_util
  CAN32_initCANBus(CAN_TX_PIN,CAN_RX_PIN, canBusReady,TWAI_TIMING_CONFIG_250KBITS());
  // CAN32_initCANBus(CAN_TX_PIN,CAN_RX_PIN, canBusReady,
  // TWAI_TIMING_CONFIG_250KBITS(), <Some filter setting>);

  // Initialize SD Card with SD32_util
  SD32_initSDCard(SD_SCK_PIN,SD_MISO_PIN,SD_MOSI_PIN,SD_CS_PIN,sdCardReady);
  SD32_generateUniqueFilename(sessionNumber,csvFilename); // modify filename to be unique/session
  SD32_createCSVFile(csvFilename,csvHeader); // create with unique filename, but fixed header
  
  // Separated i2c bus
  IMUinit(&Wire,IMU_SDA,IMU_SCL);
  RTCinit(&Wire1,RTC_SDA,RTC_SCL);
  // Calibrate IMU sensor in 1s, please position it well in the vehicle
  delay(1000); 
  IMUcalibrate();
  // uart 1 line
  GPSinit(gpsSerial);

  // Initialize all Mechanical and Electrical sensor
  RPMinit(My_timer, 100);
  StrokesensorInit();
  
  // Connect to WiFi
  initWiFi(ssid,password);
  // Setup WebSocket
  if (WiFi.status() == WL_CONNECTED) {
    // set a register callback into the websocketEventhandler
    BPMobile.setClientName(clientName);
    BPMobile.setRegisterCallback(registerClient);
    BPMobile.initWebSocket(serverHost,serverPort,clientName);
  }
  

  // inspect Device status one time
  // showDeviceStatus();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

// Need a data struct for BAMO -> May Encapsulated in .h
// Need a data struct for Rear Node -> May encapsulated in .h
//  For now use the case that work for both

void loop() {
  
  // Temp data structure per mcu loop
  twai_message_t txmsg;
  twai_message_t rxmsg;
  
  // init mech , Elect, and Odometry strcut or can be vehicle class of those 3 structure?
  Odometry myodometry;
  
  // Update WiFi LED status
  (WiFi.status() == WL_CONNECTED) ? 
  digitalWrite(WIFI_LED_INDICATOR, HIGH) : digitalWrite(WIFI_LED_INDICATOR, LOW) ;

  (BPsocketstatus->isConnected == true) ?
  digitalWrite(WS_LED_INDICATOR, HIGH) : digitalWrite(WS_LED_INDICATOR, HIGH) ;

  // Handle WebSocket communication
  if (WiFi.status() == WL_CONNECTED) BPwebSocket->loop();
  
  // Mechanical sensor update
  RPMsensorUpdate();
  StrokesensorUpdate();
  // Electrical sensor update
    // current sensor
    // All digital fault status
    // CAN Data from BMU internal bus

  // Odometry sensor update
  GPSupdate(&myodometry);
  IMUupdate(&myodometry);
  
  unsigned long now = millis();

  if (canBusReady) {
    if(CAN32_receiveCAN(&rxmsg) == ESP_OK) process_ResponseBamocarMsg(&rxmsg);
    else Serial.println("RX Buffer = 0");
    // Request CAN data periodically
    // Will cycle: 0 = motor temp, 1 = controller temp, 2 = DC voltage, 3 = DC current
    // In loop() function, around line 310-330:
    if (now - lastCANRequest >= CAN_REQUEST_INTERVAL) {
      switch (CANRequest_sequence) {
        case 0:
          // The will be passed through the sedn and requece
          pack_RequestBamocarMsg(&txmsg,BAMOCAR_REG_MOTOR_TEMP);      // 0x49
          if(CAN32_sendCAN(&txmsg) != ESP_OK); Serial.println("Motor temp request failed"); 
          // เดะดูก่อนว่าจะใส่ return มาดีมั้ย เผื่อเราไม่อยากให้มันข้าม BPMobile publisher
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
      lastCANRequest = millis();
    
      if (CANRequest_sequence > 3) 
        CANRequest_sequence = 0;
      
    }
  }

  // Calculate power 
  if (canVoltageValid && canCurrentValid) {
    power = canVoltage * canCurrent;
  }

  
  // Log to SD card
  if (sdCardReady && (now - lastSDLog >= SD_LOG_INTERVAL)) {
    logDataToSD(csvFilename);
    lastSDLog = now;
  }

  
  // Send data to BP Mobile server if registered
  if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
    
    // Send power data
    if (now - lastPowerSend >= (1000.0 / POWER_SAMPLING_RATE)) {
      sendPowerData();
      lastPowerSend = now;
    }   
    // Send CAN (Other packages??) data
    if (now - lastCANSend >= (1000.0 / CAN_SAMPLING_RATE)) {
      sendCANData();
      lastCANSend = now;
    }
    // Send RPM data
    if (now - lastRPMSend >= (1000.0 / RPM_SAMPLING_RATE)) {
      sendRPMData();
      lastRPMSend = now;
    }
    // Send stroke distance data
    if (now - lastStrokeSend >= (1000.0 / STROKE_SAMPLING_RATE)) {
      sendStrokeData();
      lastStrokeSend = now;
    }
  }
   
  
  delay(10); // Prevent watchdog issues
}

// ============================================================================
// WIFI , DEVICE , BPMobile
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
  unsigned long now = millis();
  
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
  if (timeIsSynchronized) {
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
  lastDebugPrint = now;
}


// ============================================================================
// DATA PUBLISHING TO BP MOBILE
// ============================================================================

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
  topics.add("sensors/stroke1_distance");
  topics.add("sensors/stroke2_distance");

  // Define topic metadata
  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();
  
  // CAN Voltage metadata
  JsonObject canVoltMeta = metadata["power/can_voltage"].to<JsonObject>();
  canVoltMeta["description"] = "DC Link Voltage (Bamocar CAN)";
  canVoltMeta["unit"] = "V";
  canVoltMeta["sampling_rate"] = CAN_SAMPLING_RATE;
  
  // CAN Current metadata
  JsonObject canCurrMeta = metadata["power/can_current"].to<JsonObject>();
  canCurrMeta["description"] = "Motor DC Current (Bamocar CAN)";
  canCurrMeta["unit"] = "A";
  canCurrMeta["sampling_rate"] = CAN_SAMPLING_RATE;
  
  // Power metadata
  JsonObject powMeta = metadata["power/power"].to<JsonObject>();
  powMeta["description"] = "Power consumption (calculated)";
  powMeta["unit"] = "W";
  powMeta["sampling_rate"] = POWER_SAMPLING_RATE;
  
  // Motor temperature metadata
  JsonObject motorTempMeta = metadata["motor/temperature"].to<JsonObject>();
  motorTempMeta["description"] = "Motor temperature (Bamocar)";
  motorTempMeta["unit"] = "°C";
  motorTempMeta["sampling_rate"] = CAN_SAMPLING_RATE;
  
  // Controller temperature metadata
  JsonObject ctrlTempMeta = metadata["motor/controller_temperature"].to<JsonObject>();
  ctrlTempMeta["description"] = "Motor controller/IGBT temperature (Bamocar)";
  ctrlTempMeta["unit"] = "°C";
  ctrlTempMeta["sampling_rate"] = CAN_SAMPLING_RATE;

  // Wheel RPM metadata
  JsonObject leftRpmMeta = metadata["wheel/left_rpm"].to<JsonObject>();
  leftRpmMeta["description"] = "Left wheel speed";
  leftRpmMeta["unit"] = "RPM";
  leftRpmMeta["sampling_rate"] = RPM_SAMPLING_RATE;

  JsonObject rightRpmMeta = metadata["wheel/right_rpm"].to<JsonObject>();
  rightRpmMeta["description"] = "Right wheel speed";
  rightRpmMeta["unit"] = "RPM";
  rightRpmMeta["sampling_rate"] = RPM_SAMPLING_RATE;

  // Stroke distance metadata
  JsonObject s1Meta = metadata["sensors/stroke1_distance"].to<JsonObject>();
  s1Meta["description"] = "Stroke 1 distance (black)";
  s1Meta["unit"] = "mm";
  s1Meta["sampling_rate"] = STROKE_SAMPLING_RATE;

  JsonObject s2Meta = metadata["sensors/stroke2_distance"].to<JsonObject>();
  s2Meta["description"] = "Stroke 2 distance (green)";
  s2Meta["unit"] = "mm";
  s2Meta["sampling_rate"] = STROKE_SAMPLING_RATE;

  // Serialize and send
  String registration;
  serializeJson(doc, registration);
  
  Serial.println("[WebSocket] Sending registration...");
  BPwebSocket->sendTXT(registration);
}

void sendPowerData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  
  if (unixTimestamp < 1000000000000ULL) return;
  
  // Send CAN voltage
  if (canVoltageValid) {
    // StaticJsonDocument<256> voltDoc;
    JsonDocument voltDoc;
    voltDoc["type"] = "data";
    voltDoc["topic"] = "power/can_voltage";
    voltDoc["data"]["value"] = canVoltage;
    voltDoc["data"]["sensor_id"] = "BAMOCAR_CAN";
    voltDoc["timestamp"] = unixTimestamp;
    
    String voltMsg;
    serializeJson(voltDoc, voltMsg);
    BPwebSocket->sendTXT(voltMsg);
  }
  
  // Send CAN current
  if (canCurrentValid) {
    // StaticJsonDocument<256> currDoc;
    JsonDocument currDoc;
    currDoc["type"] = "data";
    currDoc["topic"] = "power/can_current";
    currDoc["data"]["value"] = canCurrent;
    currDoc["data"]["sensor_id"] = "BAMOCAR_CAN";
    currDoc["timestamp"] = unixTimestamp;
    
    String currMsg;
    serializeJson(currDoc, currMsg);
    BPwebSocket->sendTXT(currMsg);
  }
  
  // Send calculated power
  // StaticJsonDocument<256> powDoc;
  JsonDocument powDoc;
  powDoc["type"] = "data";
  powDoc["topic"] = "power/power";
  powDoc["data"]["value"] = power;
  powDoc["data"]["sensor_id"] = "CALCULATED";
  powDoc["timestamp"] = unixTimestamp;
  
  String powMsg;
  serializeJson(powDoc, powMsg);
  BPwebSocket->sendTXT(powMsg);
  
  Serial.print("[BP Mobile] Power: V=");
  Serial.print(canVoltage, 2);
  Serial.print("V, I=");
  Serial.print(canCurrent, 3);
  Serial.print("A, P=");
  Serial.print(power, 2);
  Serial.println("W");
}

// What does this CANData mean , I have to inspect
void sendCANData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  // timeout function
  if (unixTimestamp < 1000000000000ULL) return;
  
  // Send motor temperature
  if (motorTempValid) {
    JsonDocument motorDoc;
    motorDoc["type"] = "data";
    motorDoc["topic"] = "motor/temperature";
    motorDoc["data"]["value"] = motorTemp2;
    motorDoc["data"]["sensor_id"] = "BAMOCAR_MOTOR";
    motorDoc["timestamp"] = unixTimestamp;
    
    String motorMsg;
    serializeJson(motorDoc, motorMsg);
    BPwebSocket->sendTXT(motorMsg);
  }
  
  // Send controller temperature
  if (controllerTempValid) {
    JsonDocument ctrlDoc;
    ctrlDoc["type"] = "data";
    ctrlDoc["topic"] = "motor/controller_temperature";
    ctrlDoc["data"]["value"] = controllerTemp;
    ctrlDoc["data"]["sensor_id"] = "BAMOCAR_CTRL";
    ctrlDoc["timestamp"] = unixTimestamp;
    
    String ctrlMsg;
    serializeJson(ctrlDoc, ctrlMsg);
    BPwebSocket->sendTXT(ctrlMsg);
    
    Serial.print("[BP Mobile] CAN Temps: Motor=");
    Serial.print(motorTemp2, 1);
    Serial.print("°C, Controller=");
    Serial.print(controllerTemp, 1);
    Serial.println("°C");
  }
}

void sendRPMData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  if (unixTimestamp < 1000000000000ULL) return;

  // Left wheel RPM
  // StaticJsonDocument<192> docL;
  JsonDocument docL;
  docL["type"] = "data";
  docL["topic"] = "wheel/left_rpm";
  docL["data"]["value"] = Wheel_RPM_left;
  docL["data"]["sensor_id"] = "ENC_LEFT";
  docL["timestamp"] = unixTimestamp;
  String msgL;
  serializeJson(docL, msgL);
  BPwebSocket->sendTXT(msgL);

  // Right wheel RPM
  // StaticJsonDocument<192> docR;
  JsonDocument docR;
  docR["type"] = "data";
  docR["topic"] = "wheel/right_rpm";
  docR["data"]["value"] = Wheel_RPM_right;
  docR["data"]["sensor_id"] = "ENC_RIGHT";
  docR["timestamp"] = unixTimestamp;
  String msgR;
  serializeJson(docR, msgR);
  BPwebSocket->sendTXT(msgR);

  Serial.print("[BP Mobile] RPM L:");
  Serial.print(Wheel_RPM_left, 1);
  Serial.print(" R:");
  Serial.println(Wheel_RPM_right, 1);
}

void sendStrokeData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  if (unixTimestamp < 1000000000000ULL) return;

  // Stroke 1 (black)
  JsonDocument s1;
  s1["type"] = "data";
  s1["topic"] = "sensors/stroke1_distance";
  s1["data"]["value"] = distance_stroke1_black;
  s1["data"]["sensor_id"] = "STROKE1_BLACK";
  s1["timestamp"] = unixTimestamp;
  String msg1;
  serializeJson(s1, msg1);
  BPwebSocket->sendTXT(msg1);

  // Stroke 2 (green)
  // StaticJsonDocument<192> s2;
  JsonDocument s2;
  s2["type"] = "data";
  s2["topic"] = "sensors/stroke2_distance";
  s2["data"]["value"] = distance_stroke2_green;
  s2["data"]["sensor_id"] = "STROKE2_GREEN";
  s2["timestamp"] = unixTimestamp;
  String msg2;
  serializeJson(s2, msg2);
  BPwebSocket->sendTXT(msg2);

  Serial.print("[BP Mobile] Stroke mm S1:");
  Serial.print(distance_stroke1_black, 2);
  Serial.print(" S2:");
  Serial.println(distance_stroke2_green, 2);
}

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
void RPMsensorUpdate(){
  // ----- RPM sensors (Wheel speeds)
  if (startCalculate) {
    noInterrupts();
    Wheel_RPM_left = (30000) / Period * ((float)counterL / ENCODER_N);
    Wheel_RPM_right = (30000) / Period * ((float)counterR / ENCODER_N);
    counterL = 0;
    counterR = 0;
    startCalculate = false;
    interrupts();
  }

}


void StrokesensorInit(){
  pinMode(stroke1_black,INPUT);
  pinMode(stroke2_green,INPUT);
}
void StrokesensorUpdate(){
  // ----- Stroke distances
  // Conversion from ADC value back to its sensor reading voltage
  potvolts1_black = analogRead(stroke1_black) * (aref / pwmres);
  potvolts2_green = analogRead(stroke2_green) * (aref / pwmres);
  // Conversion of sensor reading voltage KPM18-50mm distance, total distance is 52.91 mm
  distance_stroke1_black = potvolts1_black * (max_distance1 / aref);
  distance_stroke2_green = potvolts2_green * (max_distance2 / aref);

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

// Will passon odemetry data structure (Will definitely make a vehicle struct (Odometry, Mechanic, Electric))
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
// SD CARD LOGGING & Device RTC (DS3231MZ+)
// ============================================================================

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
uint64_t RTC_getUnix(){
  uint64_t now = rtc.now().unixtime(); return now;
}
String   RTC_getISO(){
  String now = rtc.now().timestamp(DateTime::TIMESTAMP_FULL); 
  return now;
}


void logDataToSD(char* csvFilename) {
  File dataFile = SD.open((const char*)csvFilename, FILE_APPEND);
  
  if (dataFile) {
    // THE SD Logging function here must depends on RTC version (Local time , not the one on)
    uint64_t syncTime = getSynchronizedTime();
    uint64_t timestamp;
    const char* dateTimeStr = "";
    
    if (timeIsSynchronized) {
      // Use synchronized timestamp
      timestamp = (syncTime / 1000ULL);
      formatDateTimeBangkok((char*)dateTimeStr,syncTime);  // Bangkok time (UTC+7)
    } else {
      // Fallback to millis() if never synced
      timestamp = millis() / 1000;
      dateTimeStr = "NOT_SYNCED";
    }
    
    // Write Unix timestamp (seconds)
    dataFile.print(timestamp);
    dataFile.print(",");
    
    // Write human-readable datetime (Bangkok timezone)
    dataFile.print(dateTimeStr);
    dataFile.print(",");
    
    // Write data point number
    dataFile.print(dataPoint);
    dataFile.print(",");
    
    // Write CAN voltage
    dataFile.print(canVoltageValid ? canVoltage : 0.0, 2);
    dataFile.print(",");
    
    // Write CAN current
    dataFile.print(canCurrentValid ? canCurrent : 0.0, 3);
    dataFile.print(",");
    
    // Write power
    dataFile.print(power, 2);
    dataFile.print(",");
    
    // Write motor temperature
    dataFile.print(motorTempValid ? motorTemp2 : 0.0, 1);
    dataFile.print(",");
    
    // Write controller temperature
    dataFile.print(controllerTempValid ? controllerTemp : 0.0, 1);
    dataFile.print(",");
    
    // Write wheel RPMs
    dataFile.print(Wheel_RPM_left, 2);
    dataFile.print(",");
    dataFile.print(Wheel_RPM_right, 2);
    dataFile.print(",");
    
    // Write stroke distances
    dataFile.print(distance_stroke1_black, 3);
    dataFile.print(",");
    dataFile.println(distance_stroke2_green, 3);
    
    dataFile.flush();
    dataFile.close();
    
    // Updated debug print
    Serial.print("[SD Card] Logged #");
    Serial.print(dataPoint);
    Serial.print(" | Time: ");
    Serial.print(dateTimeStr);
    Serial.print(" | V:");
    Serial.print(canVoltage, 2);
    Serial.print("V I:");
    Serial.print(canCurrent, 3);
    Serial.print("A P:");
    Serial.print(power, 2);
    Serial.print("W MT:");
    Serial.print(motorTemp2, 1);
    Serial.print("°C CT:");
    Serial.print(controllerTemp, 1);
    Serial.print("°C RPML:");
    Serial.print(Wheel_RPM_left, 2);
    Serial.print(" RPMR:");
    Serial.print(Wheel_RPM_right, 2);
    Serial.print(" S1:");
    Serial.print(distance_stroke1_black, 3);
    Serial.print("mm S2:");
    Serial.print(distance_stroke2_green, 3);
    Serial.println("mm");
    
    dataPoint++;
  } else {
    Serial.println("[SD Card] ERROR: Could not open file!");
  }
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
    else Serial.print("Unknown");
    Serial.println(")");
}

// Non Fixed is not request
void process_ResponseBamocarMsg(twai_message_t* msg) {
  
  // decodeBAMO_rawmsg(msg);
    
    // Check if response from Bamocar (ID 0x181[Response ID])
    if (msg->identifier == 0x181 && msg->data_length_code >= 3) {
      uint8_t regAddress = msg->data[0];
      
      // Get raw 16-bit value (little-endian: low byte first, high byte second)
      uint16_t rawValue = (msg->data[2] << 8) | msg->data[1];
      

      // === MOTOR TEMPERATURE (Register 0x49) ===
      if (regAddress == BAMOCAR_REG_MOTOR_TEMP) {
        // Try single byte interpretation: byte[1] / 10
        motorTemp1 = (float)msg->data[1] / 10.0;
        motorTemp2 = motorTemp1 + 8.0; // Assume second sensor is +8°C
        motorTempValid = true;
        
        Serial.print("[CAN DECODED] Motor Temp: ");
        Serial.print(motorTemp2, 1);
        Serial.print(" °C (raw byte: 0x");
        Serial.print(msg->data[1], HEX);
        Serial.print(" = ");
        Serial.print(msg->data[1]);
        Serial.println(")");
      }
      
      // === CONTROLLER TEMPERATURE (Register 0x4A, NTC Sensor) ===
      else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) {
        // Value is resistance in Ohms
        controllerTemp = convertNTCtoTemp(rawValue);
        controllerTempValid = true;
        
        Serial.print("[CAN DECODED] Controller Temp: ");
        Serial.print(controllerTemp, 1);
        Serial.print(" °C (resistance: ");
        Serial.print(rawValue);
        Serial.println(" Ω)");
      }
      
      // === DC VOLTAGE (Register 0xEB) ===
      else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) {
        // Use formula from reference code: rawValue / 55.1204
        // Raw value ~12520 → 227.2V (matches your multimeter!)
        canVoltage = (float)rawValue / 55.1204;
        canVoltageValid = true;
        
        Serial.print("[CAN DECODED] DC Voltage: ");
        Serial.print(canVoltage, 2);
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
          canCurrent = (float)rawValue * 0.373832;
          canCurrentValid = true;
          
          Serial.print("[CAN DECODED] DC Current: ");
          Serial.print(canCurrent, 2);
          Serial.print(" A (raw: ");
          Serial.print(rawValue);
          Serial.println(")");
        }
      }
      // === DC CURRENT (Register 0x20) ===
      else if (regAddress == BAMOCAR_REG_DC_CURRENT) {
        // Special handling: 0xFFFF (-1 signed) means "invalid/not available"
        if (rawValue == 0xFFFF || rawValue > 60000) {
          // Invalid reading - ignore it, keep previous valid value
          Serial.println("[CAN DECODED] DC Current: INVALID (0xFFFF) - ignored");
          // Don't update canCurrent or canCurrentValid
        } else {
          // Use formula from reference code: rawValue × 0.373832
          canCurrent = (float)rawValue * 0.373832;
          canCurrentValid = true;
          
          Serial.print("[CAN DECODED] DC Current: ");
          Serial.print(canCurrent, 2);
          Serial.print(" A (raw: ");
          Serial.print(rawValue);
          Serial.println(")");
        }
      }
      
      // === UNKNOWN REGISTER ===
      else {
        Serial.print("[CAN] Unknown register 0x");
        Serial.print(regAddress, HEX);
        Serial.print(" with value: ");
        Serial.println(rawValue);
      }
    }
  
}

// ============================================================================
// BAMOCar CAN Helper function - SHOWS ALL POSSIBLE INTERPRETATIONS
// ============================================================================

// Analyze CAN Request Data Frame
// Analyze CAN Request ID Frame

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
  else Serial.print("Unknown");
  
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