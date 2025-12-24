/*
 * Integrated BP Mobile + SD Card + CAN Bus Data Logger
 * 
 * This sketch combines:
 * - BP Mobile WebSocket client for real-time data streaming
 * - SD Card logging for local data storage with UNIQUE filenames
 * - CAN Bus communication for Bamocar motor controller data
 * - Wheel RPM sensors and stroke distance sensors
 * 
 * Required Libraries:
 * - WebSockets by Markus Sattler
 * - ArduinoJson by Benoit Blanchon
 * - SD (built-in)
 * - SPI (built-in)
 * - ESP32-TWAI-CAN by miwagner (or CAN library for ESP32)
 */
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <driver/twai.h>  // ESP32 built-in CAN (TWAI) driver

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================
const char* ssid = "realme C55";
const char* password = "realme1234";
//5G password 01408638
// ============================================================================
// BP MOBILE SERVER CONFIGURATION
// ============================================================================
const char* serverHost = "10.32.159.125";
const int serverPort = 3000;
const char* clientName = "ESP32_Rear";  // Must be unique!

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
#define CAN_TX_PIN 14  // GPIO14 for CAN TX
#define CAN_RX_PIN 13  // GPIO13 for CAN RX

// Bamocar CAN IDs and Registers
#define BAMOCAR_BASE_ID 0x200           // Base CAN ID for Bamocar
#define BAMOCAR_REQUEST_ID 0x201        // Request ID (typically base + 1)
#define BAMOCAR_RESPONSE_ID 0x181       // Response ID from Bamocar

// Bamocar D3 Register Addresses
#define BAMOCAR_REG_MOTOR_TEMP 0x49         // Motor temperature register
#define BAMOCAR_REG_CONTROLLER_TEMP 0x4A    // Controller/IGBT temperature register
#define BAMOCAR_REG_CAPACITOR_TEMP 0x4B     // Capacitor temperature register
#define BAMOCAR_REG_DC_VOLTAGE 0xA5         // DC Link Voltage (0.1V units)
#define BAMOCAR_REG_DC_CURRENT 0xC6         // DC Current (0.1A units, signed)

// CAN Request timing
const unsigned long CAN_REQUEST_INTERVAL = 100;  // Request every 100ms

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
/* SD card SAMPLING INTERVAL */
const unsigned long SD_LOG_INTERVAL = 1000; // Log to SD every 1 second

/* SENSOR SAMPLING RATE FOR WEBSOCKET TOPICS */
const float POWER_SAMPLING_RATE = 2.0;     // 2 Hz for power data
const float CAN_SAMPLING_RATE = 1.0;       // 1 Hz for CAN data to BP Mobile

// Add RPM and Stroke sampling rates
const float RPM_SAMPLING_RATE = 2.0;       // 2 Hz for wheel RPM publish
const float STROKE_SAMPLING_RATE = 2.0;    // 2 Hz for stroke distance publish

/* PIN DEF*/
// GPIO 5,6  ADC reading
#define stroke1_black 5
#define stroke2_green 6

// Camshaft position sensor Left, Right
#define ENCODER_PINL 42
#define ENCODER_PINR 41
#define ENCODER_N 50 // encoding resolution

// ============================================================================
// GLOBAL VARIABLES - WebSocket
// ============================================================================
WebSocketsClient webSocket;
bool isRegistered = false;
bool isConnected = false;
unsigned long lastPingReceived = 0;
unsigned long lastPowerSend = 0;
unsigned long lastCANSend = 0;
unsigned long long serverTimeOffset = 0;
unsigned long bootTimeMillis = 0;
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
float motorTemp = 0.0;
float controllerTemp = 0.0;
float canVoltage = 0.0;      // DC Link Voltage from CAN
float canCurrent = 0.0;      // Motor Current from CAN
bool motorTempValid = false;
bool controllerTempValid = false;
bool canVoltageValid = false;
bool canCurrentValid = false;

/* Stroke Sensors variables (Stroke_distance)*/
float distance_stroke1_black = 0.0;
float distance_stroke2_green = 0.0;
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
// FUNCTION DECLARATIONS
// ============================================================================

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

// Initialization functions
void initSDCard();
void initWiFi();
void initWebSocket();
void initCANBus();
void initRPM(hw_timer_t* My_timer, int millisec);

// WebSocket functions
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void registerClient();
void handleMessage(String message);

// Data publishing functions
void sendPowerData();
void sendCANData();
void sendRPMData();
void sendStrokeData();

// SD Card functions
void generateUniqueFilename();
void createCSVFile();
void logDataToSD();

// CAN Bus functions
void requestBamocarData(uint8_t regAddress);
void processCANMessages();
float convertBamocarTemp(uint16_t rawValue);

// Sensor functions
void updateSensorReadings();

// Helper functions
unsigned long long getSynchronizedTime();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("  BP Mobile + SD Card + CAN Logger");
  Serial.println("========================================");
  Serial.print("Client Name: ");
  Serial.println(clientName);
  Serial.println();
  
  // Initialize CAN Bus first
  initCANBus();
  
  // Initialize SD Card
  initSDCard();
  
  // Connect to WiFi
  initWiFi();
  
  // Initialize RPM sensors
  initRPM(My_timer, 100);
  
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
  // Handle WebSocket communication
  if (WiFi.status() == WL_CONNECTED) {
    webSocket.loop();
  }
  
  unsigned long now = millis();
  
  // Process incoming CAN messages
  if (canBusReady) {
    processCANMessages();
    
    // Request CAN data periodically
    // Will cycle: 0 = motor temp, 1 = controller temp, 2 = DC voltage, 3 = DC current
    if (now - lastCANRequest >= CAN_REQUEST_INTERVAL) {
      switch (canRequestState) {
        case 0:
          requestBamocarData(BAMOCAR_REG_MOTOR_TEMP);
          break;
        case 1:
          requestBamocarData(BAMOCAR_REG_CONTROLLER_TEMP);
          break;
        case 2:
          requestBamocarData(BAMOCAR_REG_DC_VOLTAGE);
          break;
        case 3:
          requestBamocarData(BAMOCAR_REG_DC_CURRENT);
          break;
      }
      canRequestState++;
      if (canRequestState > 3) canRequestState = 0;
      lastCANRequest = now;
    }
  }
  
  // Update sensor readings
  updateSensorReadings();
  
  // Send data to BP Mobile server if registered
  if (isRegistered && isConnected) {
    // Send power data
    if (now - lastPowerSend >= (1000.0 / POWER_SAMPLING_RATE)) {
      sendPowerData();
      lastPowerSend = now;
    }
    
    // Send CAN data
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
  
  // Log to SD card
  if (sdCardReady && (now - lastSDLog >= SD_LOG_INTERVAL)) {
    logDataToSD();
    lastSDLog = now;
  }
  
  delay(10); // Prevent watchdog issues
}

// ============================================================================
// INITIALIZATION FUNCTIONS
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

void initCANBus() {
  Serial.println("--- CAN Bus Initialization ---");
  Serial.print("Initializing CAN bus...");
  
  // Configure CAN timing for 250 kbps (as per your code)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, 
                                                                (gpio_num_t)CAN_RX_PIN, 
                                                                TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
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
  } else {
    Serial.println("FAILED to start CAN bus!");
    canBusReady = false;
  }
  Serial.println();
}

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

// ============================================================================
// WEBSOCKET EVENT HANDLER
// ============================================================================

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

// ============================================================================
// CLIENT REGISTRATION
// ============================================================================

void registerClient() {
  StaticJsonDocument<2048> doc;
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

// ============================================================================
// MESSAGE HANDLER
// ============================================================================

void handleMessage(String message) {
  StaticJsonDocument<512> doc;
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
      
      // Sync time
      if (doc.containsKey("system_time")) {
        JsonObject sysTime = doc["system_time"];
        if (sysTime.containsKey("timestamp_ms")) {
          unsigned long long serverTime = sysTime["timestamp_ms"];
          unsigned long clientTime = millis();
          
          bootTimeMillis = clientTime;
          serverTimeOffset = serverTime - (unsigned long long)clientTime;
          
          Serial.println("[WebSocket] Time synchronized!");
          Serial.print("  Server time: ");
          Serial.print(serverTime / 1000ULL);
          Serial.println(" sec since epoch");
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
    
    StaticJsonDocument<128> pongDoc;
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
// CAN BUS FUNCTIONS
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
    // Success - no print to avoid spam
  } else {
    Serial.println("[CAN] Failed to send request");
  }
}

void processCANMessages() {
  if (!canBusReady) return;
  
  twai_message_t message;
  
  // Check for messages (non-blocking)
  if (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
    // Check if this is a response from Bamocar
    if (message.identifier == BAMOCAR_RESPONSE_ID && message.data_length_code >= 6) {
      uint8_t regAddress = message.data[1];
      
      // Extract 16-bit value (little-endian)
      uint16_t rawValue = (message.data[3] << 8) | message.data[2];
      
      if (regAddress == BAMOCAR_REG_MOTOR_TEMP) {
        motorTemp = convertBamocarTemp(rawValue);
        motorTempValid = true;
        Serial.print("[CAN] Motor Temp: ");
        Serial.print(motorTemp, 1);
        Serial.println(" °C");
      } 
      else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) {
        controllerTemp = convertBamocarTemp(rawValue);
        controllerTempValid = true;
        Serial.print("[CAN] Controller Temp: ");
        Serial.print(controllerTemp, 1);
        Serial.println(" °C");
      }
      else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) {
        // Register 0xA5: DC Link Voltage in 0.1V units
        canVoltage = (float)rawValue / 10.0;
        canVoltageValid = true;
        Serial.print("[CAN] DC Voltage: ");
        Serial.print(canVoltage, 1);
        Serial.println(" V");
      }
      else if (regAddress == BAMOCAR_REG_DC_CURRENT) {
        // Register 0xC6: DC Current in 0.1A units (signed for regen)
        int16_t signedCurrent = (int16_t)rawValue;
        canCurrent = (float)signedCurrent / 10.0;
        canCurrentValid = true;
        Serial.print("[CAN] DC Current: ");
        Serial.print(canCurrent, 1);
        Serial.println(" A");
      }
    }
  }
}

float convertBamocarTemp(uint16_t rawValue) {
  // Bamocar temperature conversion formula
  // Common conversion: Temperature in 0.1°C steps
  // Example: rawValue = 250 means 25.0°C
  float tempC = (float)rawValue / 10.0;
  
  return tempC;
}

// ============================================================================
// DATA PUBLISHING TO BP MOBILE
// ============================================================================

void sendPowerData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  
  if (unixTimestamp < 1000000000000ULL) return;
  
  // Send CAN voltage
  if (canVoltageValid) {
    StaticJsonDocument<256> voltDoc;
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
    StaticJsonDocument<256> currDoc;
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
  StaticJsonDocument<256> powDoc;
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
  
  if (unixTimestamp < 1000000000000ULL) return;
  
  // Send motor temperature
  if (motorTempValid) {
    StaticJsonDocument<256> motorDoc;
    motorDoc["type"] = "data";
    motorDoc["topic"] = "motor/temperature";
    motorDoc["data"]["value"] = motorTemp;
    motorDoc["data"]["sensor_id"] = "BAMOCAR_MOTOR";
    motorDoc["timestamp"] = unixTimestamp;
    
    String motorMsg;
    serializeJson(motorDoc, motorMsg);
    webSocket.sendTXT(motorMsg);
  }
  
  // Send controller temperature
  if (controllerTempValid) {
    StaticJsonDocument<256> ctrlDoc;
    ctrlDoc["type"] = "data";
    ctrlDoc["topic"] = "motor/controller_temperature";
    ctrlDoc["data"]["value"] = controllerTemp;
    ctrlDoc["data"]["sensor_id"] = "BAMOCAR_CTRL";
    ctrlDoc["timestamp"] = unixTimestamp;
    
    String ctrlMsg;
    serializeJson(ctrlDoc, ctrlMsg);
    webSocket.sendTXT(ctrlMsg);
    
    Serial.print("[BP Mobile] CAN Temps: Motor=");
    Serial.print(motorTemp, 1);
    Serial.print("°C, Controller=");
    Serial.print(controllerTemp, 1);
    Serial.println("°C");
  }
}

void sendRPMData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  if (unixTimestamp < 1000000000000ULL) return;

  // Left wheel RPM
  StaticJsonDocument<192> docL;
  docL["type"] = "data";
  docL["topic"] = "wheel/left_rpm";
  docL["data"]["value"] = Wheel_RPM_left;
  docL["data"]["sensor_id"] = "ENC_LEFT";
  docL["timestamp"] = unixTimestamp;
  String msgL;
  serializeJson(docL, msgL);
  webSocket.sendTXT(msgL);

  // Right wheel RPM
  StaticJsonDocument<192> docR;
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
  StaticJsonDocument<192> s1;
  s1["type"] = "data";
  s1["topic"] = "sensors/stroke1_distance";
  s1["data"]["value"] = distance_stroke1_black;
  s1["data"]["sensor_id"] = "STROKE1_BLACK";
  s1["timestamp"] = unixTimestamp;
  String msg1;
  serializeJson(s1, msg1);
  webSocket.sendTXT(msg1);

  // Stroke 2 (green)
  StaticJsonDocument<192> s2;
  s2["type"] = "data";
  s2["topic"] = "sensors/stroke2_distance";
  s2["data"]["value"] = distance_stroke2_green;
  s2["data"]["sensor_id"] = "STROKE2_GREEN";
  s2["timestamp"] = unixTimestamp;
  String msg2;
  serializeJson(s2, msg2);
  webSocket.sendTXT(msg2);

  Serial.print("[BP Mobile] Stroke mm S1:");
  Serial.print(distance_stroke1_black, 2);
  Serial.print(" S2:");
  Serial.println(distance_stroke2_green, 2);
}

// ============================================================================
// SD CARD LOGGING
// ============================================================================

void createCSVFile() {
  File dataFile = SD.open(csvFilename.c_str(), FILE_WRITE);
  
  if (dataFile) {
    // Updated CSV header with CAN voltage/current, removed temperature column
    dataFile.println("Timestamp,DataPoint,CAN_Voltage(V),CAN_Current(A),Power(W),MotorTemp(C),ControllerTemp(C),Wheel_RPM_Left,Wheel_RPM_Right,Stroke1_mm,Stroke2_mm");
    dataFile.flush();
    dataFile.close();
    Serial.println("[SD Card] CSV header created");
  } else {
    Serial.println("[SD Card] ERROR: Could not create CSV file!");
  }
}

void logDataToSD() {
  File dataFile = SD.open(csvFilename.c_str(), FILE_APPEND);
  
  if (dataFile) {
    // Use synchronized time if available, fall back to millis()
    unsigned long timestamp = (unsigned long)(getSynchronizedTime() / 1000ULL);
    
    dataFile.print(timestamp);
    dataFile.print(",");
    dataFile.print(dataPoint);
    dataFile.print(",");
    dataFile.print(canVoltageValid ? canVoltage : 0.0, 2);
    dataFile.print(",");
    dataFile.print(canCurrentValid ? canCurrent : 0.0, 3);
    dataFile.print(",");
    dataFile.print(power, 2);
    dataFile.print(",");
    dataFile.print(motorTempValid ? motorTemp : 0.0, 1);
    dataFile.print(",");
    dataFile.print(controllerTempValid ? controllerTemp : 0.0, 1);
    dataFile.print(",");
    dataFile.print(Wheel_RPM_left, 2);
    dataFile.print(",");
    dataFile.print(Wheel_RPM_right, 2);
    dataFile.print(",");
    dataFile.print(distance_stroke1_black, 3);
    dataFile.print(",");
    dataFile.println(distance_stroke2_green, 3);
    
    dataFile.flush();
    dataFile.close();
    
    Serial.print("[SD Card] Logged #");
    Serial.print(dataPoint);
    Serial.print(" | V:");
    Serial.print(canVoltage, 2);
    Serial.print("V I:");
    Serial.print(canCurrent, 3);
    Serial.print("A P:");
    Serial.print(power, 2);
    Serial.print("W MT:");
    Serial.print(motorTemp, 1);
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
// SENSOR READING
// ============================================================================

void updateSensorReadings() {
  // Calculate power from CAN voltage and current
  if (canVoltageValid && canCurrentValid) {
    power = canVoltage * canCurrent;
  }
  
  // ----- Stroke distances
  // Conversion from ADC value back to its sensor reading voltage
  potvolts1_black = analogRead(stroke1_black) * (aref / pwmres);
  potvolts2_green = analogRead(stroke2_green) * (aref / pwmres);
  // Conversion of sensor reading voltage KPM18-50mm distance, total distance is 52.91 mm
  distance_stroke1_black = potvolts1_black * (max_distance1 / aref);
  distance_stroke2_green = potvolts2_green * (max_distance2 / aref);

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
// HELPER FUNCTIONS
// ============================================================================
unsigned long long getSynchronizedTime() {
  return (unsigned long long)millis() + serverTimeOffset;
}
