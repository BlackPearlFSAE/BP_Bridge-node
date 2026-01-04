
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <driver/twai.h>  // ESP32 built-in CAN (TWAI) driver
// #include <SD_utility.h>
// --- Integrated IMU & GPS Includes ---
#include <Wire.h>
#include <MPU6050_light.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================
// const char* ssid = "jackienyyy";
// const char* password = "jackjack";
const char* ssid = "realme C55";
const char* password = "realme1234";
//5G password 01408638
// ============================================================================
// BP MOBILE SERVER CONFIGURATION
// ============================================================================
const char* serverHost = "10.136.194.79";
const int serverPort = 3000;
const char* clientName = "ESP32 Rear";  // Must be unique!


#define WIFI_LED_PIN 2
// ============================================================================
// SD CARD CONFIGURATION
// ============================================================================
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 15
#define SD_SCK_PIN 12
String csvFilename = "";  // Will be generated at startup with unique name

// ============================================================================
// CAN BUS CONFIGURATION
// ============================================================================
#define CAN_DEBUG_ENABLED true  // Set to false to disable CAN message printing
#define CAN_TX_PIN 14  // GPIO14 for CAN TX
#define CAN_RX_PIN 13  // GPIO13 for CAN RX

// =============================================================
// TIME SYNCHRONIZATION VARIABLES
// ============================================================================
bool timeIsSynchronized = false;           // Flag: have we synced time at least once?
unsigned long long baseTimestamp = 0;      // The server timestamp when we synced
unsigned long syncMillis = 0;              // The millis() value when we synced

// Bamocar CAN IDs and Registers
#define BAMOCAR_BASE_ID 0x200           // Base CAN ID for Bamocar
#define BAMOCAR_REQUEST_ID 0x201        // Request ID (same as reference code)
#define BAMOCAR_RESPONSE_ID 0x181       // Response ID (CORRECTED from 0x181!)

// Bamocar D3 Register Addresses (CORRECTED based on working code)
#define BAMOCAR_REG_MOTOR_TEMP 0x49         // Motor temperature register ✓
#define BAMOCAR_REG_CONTROLLER_TEMP 0x4A    // Controller/IGBT temperature register ✓
#define BAMOCAR_REG_DC_VOLTAGE 0xEB         // DC Link Voltage (CORRECTED from 0xA5!)
#define BAMOCAR_REG_DC_CURRENT 0x20         // DC Current (CORRECTED from 0xC6!)
#define BAMOCAR_REG_SPEED_ACTUAL 0x30       // Actual speed/RPM 

#define DEBUG_CAN_MONITOR_ENABLED true  // Set to false to disable
const unsigned long DEBUG_PRINT_INTERVAL = 2000;  // Print every 2 seconds
unsigned long lastDebugPrint = 0;// DC Current (0.1A units, signed)

// CAN Request timing
const unsigned long CAN_REQUEST_INTERVAL = 100;  // Request every 100ms

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
/* SD card SAMPLING INTERVAL */
const unsigned long SD_LOG_INTERVAL = 600; // Log to SD every 1 second

/* SENSOR SAMPLING RATE FOR WEBSOCKET TOPICS */
const float POWER_SAMPLING_RATE = 2.0;     // 2 Hz for power data
const float CAN_SAMPLING_RATE = 1.0;       // 1 Hz for CAN data to BP Mobile

// Add RPM and Stroke sampling rates
const float RPM_SAMPLING_RATE = 2.0;       // 2 Hz for wheel RPM publish
const float STROKE_SAMPLING_RATE = 2.0;    // 2 Hz for stroke distance publish



// ============================================================================
// GLOBAL VARIABLES - WebSocket
// ============================================================================
WebSocketsClient webSocket;
bool isRegistered = false;
bool isConnected = false;
unsigned long lastPingReceived = 0;
unsigned long lastPowerSend = 0;
unsigned long lastCANSend = 0;
unsigned long connectionStartTime = 0;

// New timers for RPM and stroke publishes
unsigned long lastRPMSend = 0;
unsigned long lastStrokeSend = 0;

// ============================================================================
// GLOBAL VARIABLES - SD Card
// ============================================================================
bool sdCardReady = false;
unsigned long lastSDLog = 0;
int dataPoint = 0;
int sessionNumber = 0;  // For unique filename generation

// ============================================================================
// GLOBAL VARIABLES - CAN Bus
// ============================================================================
bool canBusReady = false;
unsigned long lastCANRequest = 0;
uint8_t canRequestState = 0;  // 0=motor temp, 1=controller temp, 2=voltage, 3=current

// ============================================================================
// GLOBAL VARIABLES - Sensor Data
// ============================================================================

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



/* PIN DEF*/
// GPIO 5,6  ADC reading
#define STR_Roll 6
#define STR_Heave 5

// Camshaft position sensor Left, Right
#define ENCODER_PINL 41
#define ENCODER_PINR 42
#define ENCODER_N 50 // encoding resolution

/* Stroke Sensors variables (Stroke_distance)*/
float distance_STR_Roll = 0.0;
float distance_STR_Heave = 0.0;
float potvolts1_black = 0.0;
float potvolts2_green = 0.0;
// constant 
const float aref = 3.3; 
const int pwmres = 4095; // 12 bit ADC resolution
const float max_distance1 = 52.91; // recalibrated with vernier -> Needs to recheck
const float max_distance2 = 75.00; // Not_sure needs to check again

/* RPM Sensors variable (Wheel_speed)*/
// Timer interrupt setter
hw_timer_t *My_timer = NULL;
// RPM measurement Variables
float Wheel_RPM_left = 0.0;
float Wheel_RPM_right = 0.0;

unsigned long Period = 100; // 100 ms
// ISR shared variable
volatile int counterL = 0;
volatile int counterR = 0;
volatile bool startCalculate = 0;

// ============================================================================
// --- IMU (MPU6050) globals ---
// ============================================================================
#define I2C_SDA 42
#define I2C_SCL 41
TwoWire WireIMU = TwoWire(1);
MPU6050 mpu(WireIMU);
unsigned long imu_timer = 0;

// ============================================================================
// --- GPS (TinyGPS++) globals ---
// ============================================================================
#define GPS_RX_PIN 2  // Connect to GPS TX
#define GPS_TX_PIN 1  // Connect to GPS RX
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
#define GPS_BAUD 115200

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================


unsigned long long getSynchronizedTime();
void synchronizeTime(unsigned long long serverTimeMs);
String getFormattedDateTime(unsigned long long timestampMs);
String getFormattedDateTimeBangkok(unsigned long long timestampMs);
void printCANDebugMonitor();



/*-----------------------ISR*/
// External Interrupt service Routine -- Encoder pulse counter ISR
void IRAM_ATTR ISR_COUNT_L() {
  counterL++;
}
void IRAM_ATTR ISR_COUNT_R() {
  counterR++;
}
void IRAM_ATTR ISRreset() {
  startCalculate = true;
}

/* =================================================
==========================================
*/



// CAN Bus functions
void requestBamocarData(uint8_t regAddress);
void processCANMessages();
float convertBamocarTemp(uint16_t rawValue);
void printCANRawMessage(twai_message_t &message);
void scanBamocarIDs();
void analyzeCANValue(uint8_t reg, uint8_t* data);



// ============================================================================
// CAN BUS Universal
// ============================================================================

void initCANBus();
void initCANBus() {
  Serial.println("--- CAN Bus Initialization ---");
  Serial.print("Initializing CAN bus...");
  
  // Configure CAN timing for 250 kbps (as per your code)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, 
                                                                (gpio_num_t)CAN_RX_PIN, 
                                                                TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println(" Driver installed");
  } else {
    Serial.println(" FAILED to install driver!");
    canBusReady = false;
    return;
  }
  
  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("CAN bus started successfully!");
    Serial.print("CAN TX Pin: GPIO");
    Serial.println(CAN_TX_PIN);
    Serial.print("CAN RX Pin: GPIO");
    Serial.println(CAN_RX_PIN);
    Serial.println("CAN Speed: 250 kbps");
    canBusReady = true;
    delay(500);
  } else {
    Serial.println("FAILED to start CAN bus!");
    canBusReady = false;
  }
  Serial.println();
}



// ============================================================================
// BP Mobile helper
// ============================================================================
// WebSocket functions
void initWiFi();
void initWebSocket();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void handleMessage(String message);

/* WEBSOCKET EVENT HANDLER */
void initWiFi() {
  Serial.println("--- WiFi Initialization ---");
  Serial.print("Connecting to: ");
  Serial.println(ssid);
  
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
    Serial.println("BP Mobile streaming disabled.");
    Serial.println("SD card logging will continue...");
    return;
  }
  
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.println();
}

void initWebSocket() {
  Serial.println("--- WebSocket Initialization ---");
  Serial.print("Connecting to: ws://");
  Serial.print(serverHost);
  Serial.print(":");
  Serial.println(serverPort);
  
  webSocket.begin(serverHost, serverPort, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  connectionStartTime = millis();
  Serial.println();
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WebSocket] Disconnected");
      isConnected = false;
      isRegistered = false;
      break;
      
    case WStype_CONNECTED:
      Serial.println("[WebSocket] Connected!");
      isConnected = true;
      registerClient();
      break;
      
    case WStype_TEXT:
      handleMessage((char*)payload);
      break;
      
    case WStype_ERROR:
      Serial.print("[WebSocket] Error: ");
      Serial.println((char*)payload);
      break;
      
    default:
      break;
  }
}

void handleMessage(String message) {
  // StaticJsonDocument<512> doc;
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("[WebSocket] JSON error: ");
    Serial.println(error.c_str());
    return;
  }
  
  String type = doc["type"] | "";
  
  if (type == "registration_response") {
    String status = doc["status"] | "";
    
    if (status == "accepted") {
      Serial.println("[WebSocket] ✓ Registration ACCEPTED!");
      isRegistered = true;
      
      // Sync time - ONE TIME SYNCHRONIZATION (NEW METHOD ONLY)
      // if (doc.containsKey("system_time")) {
      if (doc["system_time"].is<unsigned long long>()) {
        JsonObject sysTime = doc["system_time"];
        // if (sysTime.containsKey("timestamp_ms")) {
        if (sysTime["timestamp_ms"].is<unsigned long long>()) {
          unsigned long long serverTime = sysTime["timestamp_ms"];
          
          // Call our new sync function (only syncs once)
          if (!timeIsSynchronized) {
            synchronizeTime(serverTime);
          }
        }
      }
      
      Serial.println("[WebSocket] Ready to stream data!");
      
    } else if (status == "rejected") {
      Serial.println("[WebSocket] ✗ Registration REJECTED!");
      String msg = doc["message"] | "Unknown error";
      Serial.print("  Reason: ");
      Serial.println(msg);
      isRegistered = false;
      
      if (msg.indexOf("already exists") >= 0) {
        Serial.println("\n*** Change 'clientName' to a unique value! ***");
      }
    }
    
  } else if (type == "ping") {
    String pingId = doc["ping_id"] | "";
    lastPingReceived = millis();
    
    // StaticJsonDocument<128> pongDoc;
    JsonDocument pongDoc;
    pongDoc["type"] = "pong";
    pongDoc["ping_id"] = pingId;
    pongDoc["timestamp"] = millis();
    
    String pong;
    serializeJson(pongDoc, pong);
    webSocket.sendTXT(pong);
    
    Serial.print("[WebSocket] Ping/Pong (ID: ");
    Serial.print(pingId);
    Serial.println(")");
  }
}




// ============================================================================
// TIME FORMATTING and SYNC TIME HELPER FUNCTIONS
// ============================================================================

unsigned long long getSynchronizedTime() {
  if (timeIsSynchronized) {
    // We have synced at least once - calculate time based on millis() offset
    unsigned long currentMillis = millis();
    unsigned long elapsedMillis = currentMillis - syncMillis;
    
    // Return base timestamp + elapsed time
    return baseTimestamp + (unsigned long long)elapsedMillis;
    
  } else {
    // Not synced yet - return 0 or a flag value
    return 0ULL;
  }
}

void synchronizeTime(unsigned long long serverTimeMs) {
  baseTimestamp = serverTimeMs;
  syncMillis = millis();
  timeIsSynchronized = true;
  
  // Show synchronized time in both formats
  String syncedTime = getFormattedDateTimeBangkok(serverTimeMs);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║      TIME SYNCHRONIZED SUCCESS!        ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.print("║ Unix Time:   ");
  Serial.print(serverTimeMs / 1000ULL);
  Serial.println(" sec       ║");
  Serial.print("║ Bangkok Time: ");
  Serial.print(syncedTime);
  Serial.println("   ║");
  Serial.print("║ Sync Millis:  ");
  Serial.print(syncMillis);
  Serial.println(" ms            ║");
  Serial.println("║ Time will now run independently!       ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

String getFormattedDateTime(unsigned long long timestampMs) {
  // Convert milliseconds to seconds
  unsigned long long timestampSec = timestampMs / 1000ULL;
  
  // This gives UTC time
  time_t rawtime = (time_t)timestampSec;
  struct tm * timeinfo;
  timeinfo = gmtime(&rawtime);
  
  char buffer[32];
  // Format: YYYY-MM-DD HH:MM:SS
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
  
  return String(buffer);
}

String getFormattedDateTimeBangkok(unsigned long long timestampMs) {
  // Convert milliseconds to seconds
  unsigned long long timestampSec = timestampMs / 1000ULL;
  
  // Add 7 hours for Bangkok timezone (UTC+7)
  timestampSec += (7 * 3600);
  
  time_t rawtime = (time_t)timestampSec;
  struct tm * timeinfo;
  timeinfo = gmtime(&rawtime);
  
  char buffer[32];
  // Format: YYYY-MM-DD HH:MM:SS
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
  
  return String(buffer);
}







// ********************************************************************** KEEEP 




// ============================================================================
// DATA PUBLISHING TO BP MOBILE
// ============================================================================
// This is the function of BP mobile --> I can make it a fr

/* CLIENT REGISTRATION */
// this can be done locally in here (register client)


void registerClient();
// Data publishing functions
void sendPowerData();
void sendCANData();
void sendRPMData();
void sendStrokeData();

void registerClient() {
  // StaticJsonDocument<2048> doc;
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
  webSocket.sendTXT(registration);
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
    webSocket.sendTXT(voltMsg);
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
    webSocket.sendTXT(currMsg);
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
  webSocket.sendTXT(powMsg);
  
  Serial.print("[BP Mobile] Power: V=");
  Serial.print(canVoltage, 2);
  Serial.print("V, I=");
  Serial.print(canCurrent, 3);
  Serial.print("A, P=");
  Serial.print(power, 2);
  Serial.println("W");
}

void sendCANData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  // timeout function
  if (unixTimestamp < 1000000000000ULL) return;
  
  // Send motor temperature
  if (motorTempValid) {
    // StaticJsonDocument<256> motorDoc;
    JsonDocument motorDoc;
    motorDoc["type"] = "data";
    motorDoc["topic"] = "motor/temperature";
    motorDoc["data"]["value"] = motorTemp2;
    motorDoc["data"]["sensor_id"] = "BAMOCAR_MOTOR";
    motorDoc["timestamp"] = unixTimestamp;
    
    String motorMsg;
    serializeJson(motorDoc, motorMsg);
    webSocket.sendTXT(motorMsg);
  }
  
  // Send controller temperature
  if (controllerTempValid) {
    // StaticJsonDocument<256> ctrlDoc;
    JsonDocument ctrlDoc;
    ctrlDoc["type"] = "data";
    ctrlDoc["topic"] = "motor/controller_temperature";
    ctrlDoc["data"]["value"] = controllerTemp;
    ctrlDoc["data"]["sensor_id"] = "BAMOCAR_CTRL";
    ctrlDoc["timestamp"] = unixTimestamp;
    
    String ctrlMsg;
    serializeJson(ctrlDoc, ctrlMsg);
    webSocket.sendTXT(ctrlMsg);
    
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
  webSocket.sendTXT(msgL);

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
  webSocket.sendTXT(msgR);

  Serial.print("[BP Mobile] RPM L:");
  Serial.print(Wheel_RPM_left, 1);
  Serial.print(" R:");
  Serial.println(Wheel_RPM_right, 1);
}

void sendStrokeData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  if (unixTimestamp < 1000000000000ULL) return;

  // Stroke 1 (black)
  // StaticJsonDocument<192> s1;
  JsonDocument s1;
  s1["type"] = "data";
  s1["topic"] = "sensors/stroke1_distance";
  s1["data"]["value"] = distance_STR_Roll;
  s1["data"]["sensor_id"] = "STR_Roll";
  s1["timestamp"] = unixTimestamp;
  String msg1;
  serializeJson(s1, msg1);
  webSocket.sendTXT(msg1);

  // Stroke 2 (green)
  // StaticJsonDocument<192> s2;
  JsonDocument s2;
  s2["type"] = "data";
  s2["topic"] = "sensors/stroke2_distance";
  s2["data"]["value"] = distance_STR_Heave;
  s2["data"]["sensor_id"] = "STR_Heave";
  s2["timestamp"] = unixTimestamp;
  String msg2;
  serializeJson(s2, msg2);
  webSocket.sendTXT(msg2);

  Serial.print("[BP Mobile] Stroke mm S1:");
  Serial.print(distance_STR_Roll, 2);
  Serial.print(" S2:");
  Serial.println(distance_STR_Heave, 2);
}



// ============================================================================
// SD CARD LOGGING
// ============================================================================
// SD Card functions

void initSDCard();
void generateUniqueFilename();
void createCSVFile();
void logDataToSD();

void generateUniqueFilename() {
  // Find the next available session number
  sessionNumber = 0;
  
  // Check existing files and find the highest session number
  File root = SD.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      String filename = entry.name();
      // Look for files matching pattern: datalog_NNN.csv
      if (filename.startsWith("datalog_") && filename.endsWith(".csv")) {
        // Extract the number
        int startIdx = 8;  // Length of "datalog_"
        int endIdx = filename.indexOf(".csv");
        if (endIdx > startIdx) {
          String numStr = filename.substring(startIdx, endIdx);
          int fileNum = numStr.toInt();
          if (fileNum >= sessionNumber) {
            sessionNumber = fileNum + 1;
          }
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }
  
  // Generate filename with 3-digit session number
  char buffer[30];
  sprintf(buffer, "/datalog_%03d.csv", sessionNumber);
  csvFilename = String(buffer);
  
  Serial.print("Generated unique filename: ");
  Serial.println(csvFilename);
}

void initSDCard() {
  Serial.println("--- SD Card Initialization ---");
  Serial.print("Initializing SD card...");
  
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(" FAILED!");
    Serial.println("SD card logging disabled.");
    Serial.println("Check:");
    Serial.println("  - SD card is inserted");
    Serial.println("  - Connections are correct");
    Serial.println("  - SD card is formatted (FAT32)");
    sdCardReady = false;
    return;
  }
  
  Serial.println(" SUCCESS!");
  sdCardReady = true;
  
  // Display card info
  uint8_t cardType = SD.cardType();
  Serial.print("Card Type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card Size: %lluMB\n", cardSize);
  
  // Generate unique filename for this session
  generateUniqueFilename();
  
  // Create new CSV file
  Serial.println("Creating new CSV file...");
  createCSVFile();
  
  Serial.println("SD card ready for logging!");
  Serial.println();
}

void createCSVFile() {
  File dataFile = SD.open(csvFilename.c_str(), FILE_WRITE);
  
  if (dataFile) {
    // Updated CSV header with DateTime column
    dataFile.println("Timestamp,DateTime,DataPoint,CAN_Voltage(V),CAN_Current(A),Power(W),MotorTemp(C),ControllerTemp(C),Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm");
    dataFile.flush();
    dataFile.close();
    Serial.println("[SD Card] CSV header created with DateTime column");
  } else {
    Serial.println("[SD Card] ERROR: Could not create CSV file!");
  }
}

void logDataToSD() {
  File dataFile = SD.open(csvFilename.c_str(), FILE_APPEND);
  
  if (dataFile) {
    unsigned long long syncTime = getSynchronizedTime();
    unsigned long timestamp;
    String dateTimeStr = "";
    
    if (timeIsSynchronized) {
      // Use synchronized timestamp
      timestamp = (unsigned long)(syncTime / 1000ULL);
      dateTimeStr = getFormattedDateTimeBangkok(syncTime);  // Bangkok time (UTC+7)
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
    dataFile.print(distance_STR_Roll, 3);
    dataFile.print(",");
    dataFile.println(distance_STR_Heave, 3);
    
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
    Serial.print(distance_STR_Roll, 3);
    Serial.print("mm S2:");
    Serial.print(distance_STR_Heave, 3);
    Serial.println("mm");
    
    dataPoint++;
  } else {
    Serial.println("[SD Card] ERROR: Could not open file!");
  }
}



// ============================================================================
// CAN BUS FUNCTIONS FOr BAMO car
// ============================================================================


void requestBamocarData(uint8_t regAddress) {
  if (!canBusReady) return;
  
  twai_message_t message;
  message.identifier = BAMOCAR_REQUEST_ID;
  message.extd = 0;  // Standard frame
  message.rtr = 0;   // Data frame
  message.data_length_code = 3;
  
  // Bamocar request format: [0x3D, register_address, 0x00]
  message.data[0] = 0x3D;  // Read command
  message.data[1] = regAddress;
  message.data[2] = 0x00;
  
  if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    // Print what we're requesting
    Serial.print("[CAN TX] Request sent to ID 0x");
    Serial.print(BAMOCAR_REQUEST_ID, HEX);
    Serial.print(" for Register 0x");
    Serial.print(regAddress, HEX);
    Serial.print(" (");
    
    // Print register name
    if (regAddress == BAMOCAR_REG_MOTOR_TEMP) Serial.print("Motor Temp");
    else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) Serial.print("Controller Temp");
    else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) Serial.print("DC Voltage");
    else if (regAddress == BAMOCAR_REG_DC_CURRENT) Serial.print("DC Current");
    else Serial.print("Unknown");
    
    Serial.println(")");
  } else {
    Serial.print("[CAN ERROR] Failed to send request for Register 0x");
    Serial.println(regAddress, HEX);
  }
}

void processCANMessages() {
  if (!canBusReady) return;
  
  twai_message_t message;
  
  if (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
    
    printCANRawMessage(message);
    
    // Check if response from Bamocar (ID 0x181 - YOUR ORIGINAL WAS CORRECT!)
    if (message.identifier == 0x181 && message.data_length_code >= 3) {
      uint8_t regAddress = message.data[0];
      
      // Get raw 16-bit value (little-endian: low byte first, high byte second)
      uint16_t rawValue = (message.data[2] << 8) | message.data[1];
      
      // === MOTOR TEMPERATURE (Register 0x49) ===
      if (regAddress == BAMOCAR_REG_MOTOR_TEMP) {
        // Try single byte interpretation: byte[1] / 10
        motorTemp1 = (float)message.data[1] / 10.0;
        motorTemp2 = motorTemp1 + 8.0; // Assume second sensor is +8°C
        motorTempValid = true;
        
        Serial.print("[CAN DECODED] Motor Temp: ");
        Serial.print(motorTemp2, 1);
        Serial.print(" °C (raw byte: 0x");
        Serial.print(message.data[1], HEX);
        Serial.print(" = ");
        Serial.print(message.data[1]);
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
}




// ============================================================================
// CAN DATA ANALYZER - SHOWS ALL POSSIBLE INTERPRETATIONS
// ============================================================================

void analyzeCANValue(uint8_t reg, uint8_t* data) {
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

void printCANRawMessage(twai_message_t &message) {
  #if CAN_DEBUG_ENABLED
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
  #endif
}

void printCANDebugMonitor() {
    unsigned long now = millis();
    
    // Only print at specified interval
    if (now - lastDebugPrint < DEBUG_PRINT_INTERVAL) {
      return;
    }
    lastDebugPrint = now;
    
    // Print header
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║       CAN MONITOR - DEBUG VIEW         ║");
    Serial.println("╠════════════════════════════════════════╣");
    
    Serial.print("║ WiFi:            ");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("✓ CONNECTED");
      Serial.print(" (");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm)  ║");
    } else {
      Serial.println("✗ DISCONNECTED     ║");
    }


    // WebSocket Status (NEW!)
    Serial.print("║ WebSocket:       ");
    if (isConnected) {
      Serial.println("✓ CONNECTED        ║");
    } else {
      Serial.println("✗ DISCONNECTED     ║");
    }

    
    // Time Sync Status (NEW!)
    Serial.print("║ Time Sync:       ");
    if (timeIsSynchronized) {
      Serial.println("✓ SYNCED           ║");
    } else {
      Serial.println("✗ NOT SYNCED       ║");
    }
    
    Serial.println("╠════════════════════════════════════════╣");

    // Voltage
    Serial.print("║ DC Voltage:      ");
    if (canVoltageValid) {
      Serial.print(canVoltage, 2);
      Serial.println(" V          ║");
    } else {
      Serial.println("NO DATA          ║");
    }

    // Current
    Serial.print("║ DC Current:      ");
    if (canCurrentValid) {
      Serial.print(canCurrent, 2);
      Serial.println(" A          ║");
    } else {
      Serial.println("NO DATA          ║");
    }
    
    // Motor Temperature
    Serial.print("║ Motor Temp:      ");
    if (motorTempValid) {
      Serial.print(motorTemp2, 1);
      Serial.println(" °C        ║");
    } else {
      Serial.println("NO DATA          ║");
    }
    
    // Controller Temperature
    Serial.print("║ Controller Temp: ");
    if (controllerTempValid) {
      Serial.print(controllerTemp, 1);
      Serial.println(" °C        ║");
    } else {
      Serial.println("NO DATA          ║");
    }
    
    // Calculate and show power if both voltage and current valid
    if (canVoltageValid && canCurrentValid) {
      Serial.print("║ Power:           ");
      Serial.print(power, 2);
      Serial.println(" W         ║");
    }
    Serial.println("╚════════════════════════════════════════╝\n");
}

void scanBamocarIDs() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     CAN ID DISCOVERY - TESTING         ║");
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
        printCANRawMessage(response);
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
// SENSOR READING
// ============================================================================

void initRPM(hw_timer_t* My_timer, int millisec) {
  pinMode(ENCODER_PINL, INPUT); // The module has internal Pull up Resistor
  pinMode(ENCODER_PINR, INPUT); // The module has internal Pull up Resistor
  
  // ------ interrupt
  attachInterrupt(ENCODER_PINL, ISR_COUNT_L, FALLING);
  attachInterrupt(ENCODER_PINR, ISR_COUNT_R, FALLING);

  // Timer interrupt 100 ms
  int refresh_calculation_time = millisec * 1000;
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &ISRreset, true);
  timerAlarmWrite(My_timer, refresh_calculation_time, true);
  timerAlarmEnable(My_timer);
}

void updateMSensorReadings() {
  // Calculate power from CAN voltage and current
  if (canVoltageValid && canCurrentValid) {
    power = canVoltage * canCurrent;
  }
  
  // ----- Stroke distances
  // Conversion from ADC value back to its sensor reading voltage
  potvolts1_black = analogRead(STR_Roll) * (aref / pwmres);
  potvolts2_green = analogRead(STR_Heave) * (aref / pwmres);
  // Conversion of sensor reading voltage KPM18-50mm distance, total distance is 52.91 mm
  distance_STR_Roll = potvolts1_black * (max_distance1 / aref);
  distance_STR_Heave = potvolts2_green * (max_distance2 / aref);

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

// ============================================================================
// THERMISTOR TEMPERATURE CONVERSION FUNCTIONS
// ============================================================================

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

float convertBamocarTemp(uint16_t rawValue) {
  // Bamocar temperature conversion formula
  // Common conversion: Temperature in 0.1°C steps
  // Example: rawValue = 250 means 25.0°C
  float tempC = (float)rawValue / 10.0;
  
  return tempC;
}

// IMU/GPS helper prototypes
float convertNTCtoTemp(uint16_t resistance);
float convertKTYtoTemp(uint16_t resistance);
void initRPM(hw_timer_t* My_timer, int millisec);



void IMUinitwrapper();
void IMUcalibrate();
void GPSlocation();
void GPS_ISOdatetime();
void GPSext_info();

// Sensor functions
void updateSensorReadings();

void IMUinitwrapper(){
  WireIMU.begin(I2C_SDA,I2C_SCL);
  byte status = mpu.begin();
  Serial.printf("MPU6050 status: %c\n", status);
}

void IMUcalibrate(){
  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}

void GPSlocation() {
  Serial.println(F("\n===== GPS Lat lng ====="));
  
  // Location
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {

    // Serial debugging
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.println(gps.location.lng(), 6);
    Serial.print(F("Age: "));
    Serial.print(gps.location.age());
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("INVALID"));
  }  
}

void GPS_ISOdatetime(){
    // GET ISO Date & Time
  Serial.print(F("Date/Time: "));
  if (gps.date.isValid() && gps.time.isValid()) {
    // Serial debugging
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    Serial.print(F(" "));
    
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.println(gps.time.second());
  } else {
    Serial.println(F("INVALID"));
  }
}

void GPSext_info(){
  // Altitude
  Serial.print(F("Altitude: "));
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters());
    Serial.print(F(" m ("));
    Serial.print(gps.altitude.feet());
    Serial.println(F(" ft)"));
  } else {
    Serial.println(F("INVALID"));
  }
  
  // Speed
  Serial.print(F("Speed: "));
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph());
    Serial.print(F(" km/h ("));
    Serial.print(gps.speed.mph());
    Serial.print(F(" mph, "));
    Serial.print(gps.speed.knots());
    Serial.println(F(" knots)"));
  } else {
    Serial.println(F("INVALID"));
  }
  
  // Course/Heading
  Serial.print(F("Course: "));
  if (gps.course.isValid()) {
    Serial.print(gps.course.deg());
    Serial.print(F("° ("));
    Serial.print(TinyGPSPlus::cardinal(gps.course.deg()));
    Serial.println(F(")"));
  } else {
    Serial.println(F("INVALID"));
  }
}

// End of file



// ============================================================================
// SETUP
// ============================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  delay(1000);
    // Setup status LED
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);  // LED off initially

  Serial.println("\n========================================");
  Serial.println("  BP Mobile + SD Card + CAN Logger");
  Serial.println("========================================");
  Serial.print("Client Name: ");
  Serial.println(clientName);
  Serial.println();
  
  // Initialize CAN Bus first
  initCANBus();
  // if (canBusReady) {
  //   delay(2000);  // Wait for Bamocar to be ready
  //   scanBamocarIDs();
  // }
  if (canBusReady) {
    delay(2000);  // ✓ Give Bamocar time to boot
    // scanBamocarIDs();  // Optional: uncomment to debug
  }
  // Initialize SD Card
  initSDCard();
  
  // Connect to WiFi
  initWiFi();
  
  // Initialize RPM sensors
  initRPM(My_timer, 100);

  // --- Initialize IMU ---
  IMUinitwrapper();
  IMUcalibrate();

  // --- Initialize GPS serial ---
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started");
  
  // Setup WebSocket
  if (WiFi.status() == WL_CONNECTED) {
    initWebSocket();
  }
  
  Serial.println("\n========================================");
  Serial.println("System Ready!");
  Serial.println("========================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    // Update WiFi LED status
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(WIFI_LED_PIN, HIGH);  // LED ON = WiFi connected
  } else {
    digitalWrite(WIFI_LED_PIN, LOW);   // LED OFF = WiFi disconnected
  }
  // Handle WebSocket communication
  if (WiFi.status() == WL_CONNECTED) {
    webSocket.loop();
  }
  
  unsigned long now = millis();
  
  // Process incoming CAN messages
  if (canBusReady) {
    // checkCANBusStatus();
    processCANMessages();
    
    // Request CAN data periodically
    // Will cycle: 0 = motor temp, 1 = controller temp, 2 = DC voltage, 3 = DC current
    // In loop() function, around line 310-330:
    if (now - lastCANRequest >= CAN_REQUEST_INTERVAL) {
      switch (canRequestState) {
        case 0:
          requestBamocarData(BAMOCAR_REG_MOTOR_TEMP);      // 0x49
          break;
        case 1:
          requestBamocarData(BAMOCAR_REG_CONTROLLER_TEMP); // 0x4A
          break;
        case 2:
          requestBamocarData(BAMOCAR_REG_DC_VOLTAGE);      // 0xEB (CORRECTED!)
          break;
        case 3:
          requestBamocarData(BAMOCAR_REG_DC_CURRENT);      // 0x20 (CORRECTED!)
          break;
      }
      canRequestState++;
      if (canRequestState > 3) canRequestState = 0;
      lastCANRequest = now;
    }
  }

  // --- IMU update ---
  mpu.update();

  // --- GPS data ingestion ---
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      GPSlocation();
    }
  }
  
  // Update other sensor readings
  // should separate, each as its own function , not object of more than that
  updateSensorReadings();
  
  // printCANDebugMonitor();
  
  
  // Send data to BP Mobile server if registered
  // if (isRegistered && isConnected) {
  //   // Send power data
  //   if (now - lastPowerSend >= (1000.0 / POWER_SAMPLING_RATE)) {
  //     sendPowerData();
  //     lastPowerSend = now;
  //   }
    
  //   // Send CAN data
  //   if (now - lastCANSend >= (1000.0 / CAN_SAMPLING_RATE)) {
  //     sendCANData();
  //     lastCANSend = now;
  //   }

  //   // Send RPM data
  //   if (now - lastRPMSend >= (1000.0 / RPM_SAMPLING_RATE)) {
  //     sendRPMData();
  //     lastRPMSend = now;
  //   }

  //   // Send stroke distance data
  //   if (now - lastStrokeSend >= (1000.0 / STROKE_SAMPLING_RATE)) {
  //     sendStrokeData();
  //     lastStrokeSend = now;
  //   }
  // }
  
  // // Log to SD card
  // if (sdCardReady && (now - lastSDLog >= SD_LOG_INTERVAL)) {
  //   logDataToSD();
  //   lastSDLog = now;
  // }
  
  delay(10); // Prevent watchdog issues
}
