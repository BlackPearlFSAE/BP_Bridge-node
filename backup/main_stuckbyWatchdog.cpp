/*
 * Integrated BP Mobile + SD Card + CAN Bus Data Logger
 * 
 * This sketch combines:
 * - BP Mobile WebSocket client for real-time data streaming
 * - SD Card logging for local data storage with UNIQUE filenames
 * - CAN Bus communication for Bamocar motor controller data
 * - Mock sensor data generation (temperature, humidity, voltage, current)
 * - RPM sensors (2x 5V sensors, 2x 3.3V 4-stroke sensors)
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
const char* serverHost = "10.246.182.168";
const int serverPort = 3000;
const char* clientName = "wad2";  // Must be unique!

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
#define CAN_TX_PIN 14  // GPIO5 for CAN TX
#define CAN_RX_PIN 13    // GPIO4 for CAN RX

// Bamocar CAN IDs and Registers
#define BAMOCAR_BASE_ID 0x200           // Base CAN ID for Bamocar
#define BAMOCAR_REQUEST_ID 0x201        // Request ID (typically base + 1)
#define BAMOCAR_RESPONSE_ID 0x181       // Response ID from Bamocar

// Bamocar Register Addresses
#define BAMOCAR_REG_MOTOR_TEMP 0x49     // Motor temperature register
#define BAMOCAR_REG_CONTROLLER_TEMP 0x4A // Controller/IGBT temperature register
#define BAMOCAR_REG_CAPACITOR_TEMP 0x4B  // Capacitor temperature register (if available)

// CAN Request timing
const unsigned long CAN_REQUEST_INTERVAL = 100;  // Request every 100ms

// ============================================================================
// RPM SENSOR CONFIGURATION (4x 5V RPM sensors)
// ============================================================================
// Define GPIO pins for RPM sensors - ADJUST TO YOUR ACTUAL PINS
#define RPM_5V_1_PIN 32      // First 5V RPM sensor
#define RPM_5V_2_PIN 33      // Second 5V RPM sensor
#define RPM_5V_3_PIN 25      // Third 5V RPM sensor
#define RPM_5V_4_PIN 26      // Fourth 5V RPM sensor

// RPM calculation variables
volatile unsigned long pulseCount_rpm_1 = 0;
volatile unsigned long pulseCount_rpm_2 = 0;
volatile unsigned long pulseCount_rpm_3 = 0;
volatile unsigned long pulseCount_rpm_4 = 0;

volatile unsigned long lastPulseTime_rpm_1 = 0;
volatile unsigned long lastPulseTime_rpm_2 = 0;
volatile unsigned long lastPulseTime_rpm_3 = 0;
volatile unsigned long lastPulseTime_rpm_4 = 0;

unsigned long lastRPMCalc = 0;
const unsigned long RPM_CALC_INTERVAL = 1000;  // Calculate RPM every 1 second

// ============================================================================
// POTENTIOMETER CONFIGURATION (4x 3.3V Linear Potentiometers)
// ============================================================================
// Define analog input pins for potentiometers - ADJUST TO YOUR ACTUAL PINS
#define POT_1_PIN 34         // First potentiometer (ADC1_CH6)
#define POT_2_PIN 35         // Second potentiometer (ADC1_CH7)
#define POT_3_PIN 36         // Third potentiometer (ADC1_CH0)
#define POT_4_PIN 39         // Fourth potentiometer (ADC1_CH3)

// Potentiometer reading variables
const int ADC_RESOLUTION = 4095;     // 12-bit ADC (0-4095)
const float ADC_VREF = 3.3;          // Reference voltage
unsigned long lastPotRead = 0;
const unsigned long POT_READ_INTERVAL = 100;  // Read potentiometers every 100ms

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
const float TEMP_SAMPLING_RATE = 1.0;      // 1 Hz for temperature
const float HUMIDITY_SAMPLING_RATE = 1.0;  // 1 Hz for humidity
const float POWER_SAMPLING_RATE = 2.0;     // 2 Hz for power data
const float CAN_SAMPLING_RATE = 1.0;       // 1 Hz for CAN data to BP Mobile
const unsigned long SD_LOG_INTERVAL = 1000; // Log to SD every 1 second

// ============================================================================
// GLOBAL VARIABLES - WebSocket
// ============================================================================
WebSocketsClient webSocket;
bool isRegistered = false;
bool isConnected = false;
unsigned long lastPingReceived = 0;
unsigned long lastTempSend = 0;
unsigned long lastHumiditySend = 0;
unsigned long lastPowerSend = 0;
unsigned long lastCANSend = 0;
unsigned long long serverTimeOffset = 0;
unsigned long bootTimeMillis = 0;
unsigned long connectionStartTime = 0;

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
uint8_t canRequestState = 0;  // 0=motor temp, 1=controller temp

// ============================================================================
// GLOBAL VARIABLES - Sensor Data
// ============================================================================
float temperature = 25.5;
float humidity = 60.0;
float voltage = 3.7;
float current = 0.5;
float power = 0.0;

// Bamocar temperatures
float motorTemp = 0.0;
float controllerTemp = 0.0;
bool motorTempValid = false;
bool controllerTempValid = false;

// RPM sensors (4x 5V sensors)
float rpm_1 = 0.0;
float rpm_2 = 0.0;
float rpm_3 = 0.0;
float rpm_4 = 0.0;

// Potentiometers (4x 3.3V linear pots)
float pot_1_voltage = 0.0;
float pot_2_voltage = 0.0;
float pot_3_voltage = 0.0;
float pot_4_voltage = 0.0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization functions
void initSDCard();
void initWiFi();
void initWebSocket();
void initCANBus();
void initRPMSensors();

// WebSocket functions
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void registerClient();
void handleMessage(String message);

// Data publishing functions
void sendTemperatureData();
void sendHumidityData();
void sendPowerData();
void sendCANData();

// SD Card functions
void generateUniqueFilename();
void createCSVFile();
void logDataToSD();

// CAN Bus functions
void requestBamocarData(uint8_t regAddress);
void processCANMessages();
float convertBamocarTemp(uint16_t rawValue);

// RPM Sensor functions
void IRAM_ATTR rpmISR_1();
void IRAM_ATTR rpmISR_2();
void IRAM_ATTR rpmISR_3();
void IRAM_ATTR rpmISR_4();
void calculateRPM();

// Potentiometer functions
void readPotentiometers();

// Sensor functions
void updateSensorReadings();

// Helper functions
unsigned long long getSynchronizedTime();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable detector
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("  BP Mobile + SD Card + CAN Logger");
  Serial.println("========================================");
  Serial.print("Client Name: ");
  Serial.println(clientName);
  Serial.println();
  
  // Initialize RPM sensors
  initRPMSensors();
  
  // Initialize CAN Bus first
  initCANBus();
  
  // Initialize SD Card
  initSDCard();
  
  // Connect to WiFi
  initWiFi();
  
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
  
  // Read potentiometers periodically
  if (now - lastPotRead >= POT_READ_INTERVAL) {
    readPotentiometers();
    lastPotRead = now;
  }
  
  // Calculate RPM periodically
  if (now - lastRPMCalc >= RPM_CALC_INTERVAL) {
    calculateRPM();
    lastRPMCalc = now;
  }
  
  // Process incoming CAN messages
  if (canBusReady) {
    processCANMessages();
    
    // Request CAN data periodically
    if (now - lastCANRequest >= CAN_REQUEST_INTERVAL) {
      if (canRequestState == 0) {
        requestBamocarData(BAMOCAR_REG_MOTOR_TEMP);
        canRequestState = 1;
      } else {
        requestBamocarData(BAMOCAR_REG_CONTROLLER_TEMP);
        canRequestState = 0;
      }
      lastCANRequest = now;
    }
  }
  
  // Process incoming CAN messages
  if (isRegistered && isConnected) {
    // Send temperature
    if (now - lastTempSend >= (1000.0 / TEMP_SAMPLING_RATE)) {
      sendTemperatureData();
      lastTempSend = now;
    }
    
    // Send humidity
    // if (now - lastHumiditySend >= (1000.0 / HUMIDITY_SAMPLING_RATE)) {
    //   sendHumidityData();
    //   lastHumiditySend = now;
    // }
    
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

void initRPMSensors() {
  Serial.println("--- RPM Sensor Initialization ---");
  
  // Configure input pins for RPM sensors (digital inputs)
  pinMode(RPM_5V_1_PIN, INPUT);
  pinMode(RPM_5V_2_PIN, INPUT);
  pinMode(RPM_5V_3_PIN, INPUT);
  pinMode(RPM_5V_4_PIN, INPUT);
  
  // Attach interrupts for pulse counting (RISING edge detection)
  attachInterrupt(digitalPinToInterrupt(RPM_5V_1_PIN), rpmISR_1, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_5V_2_PIN), rpmISR_2, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_5V_3_PIN), rpmISR_3, RISING);
  attachInterrupt(digitalPinToInterrupt(RPM_5V_4_PIN), rpmISR_4, RISING);
  
  Serial.println("RPM sensors initialized:");
  Serial.print("  RPM Sensor 1: GPIO");
  Serial.println(RPM_5V_1_PIN);
  Serial.print("  RPM Sensor 2: GPIO");
  Serial.println(RPM_5V_2_PIN);
  Serial.print("  RPM Sensor 3: GPIO");
  Serial.println(RPM_5V_3_PIN);
  Serial.print("  RPM Sensor 4: GPIO");
  Serial.println(RPM_5V_4_PIN);
  
  Serial.println("\n--- Potentiometer Initialization ---");
  
  // Configure analog input pins for potentiometers
  pinMode(POT_1_PIN, INPUT);
  pinMode(POT_2_PIN, INPUT);
  pinMode(POT_3_PIN, INPUT);
  pinMode(POT_4_PIN, INPUT);
  
  // Set ADC resolution (ESP32 supports 9-12 bits)
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  
  Serial.println("Potentiometers initialized:");
  Serial.print("  Potentiometer 1: GPIO");
  Serial.println(POT_1_PIN);
  Serial.print("  Potentiometer 2: GPIO");
  Serial.println(POT_2_PIN);
  Serial.print("  Potentiometer 3: GPIO");
  Serial.println(POT_3_PIN);
  Serial.print("  Potentiometer 4: GPIO");
  Serial.println(POT_4_PIN);
  Serial.println();
}

void initCANBus() {
  Serial.println("--- CAN Bus Initialization ---");
  Serial.print("Initializing CAN bus...");
  
  // Configure CAN timing for 500 kbps (common for Bamocar)
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
    Serial.println("CAN Speed: 500 kbps");
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
// RPM INTERRUPT SERVICE ROUTINES
// ============================================================================

void IRAM_ATTR rpmISR_1() {
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime_rpm_1 > 1000) {  // Debounce: 1ms
    pulseCount_rpm_1++;
    lastPulseTime_rpm_1 = currentTime;
  }
}

void IRAM_ATTR rpmISR_2() {
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime_rpm_2 > 1000) {
    pulseCount_rpm_2++;
    lastPulseTime_rpm_2 = currentTime;
  }
}

void IRAM_ATTR rpmISR_3() {
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime_rpm_3 > 1000) {
    pulseCount_rpm_3++;
    lastPulseTime_rpm_3 = currentTime;
  }
}

void IRAM_ATTR rpmISR_4() {
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime_rpm_4 > 1000) {
    pulseCount_rpm_4++;
    lastPulseTime_rpm_4 = currentTime;
  }
}

// ============================================================================
// RPM CALCULATION
// ============================================================================

void calculateRPM() {
  // For 4-stroke engines: 2 revolutions = 1 complete cycle
  // RPM = (pulses_per_second * 60) / pulses_per_revolution
  // For a typical 4-stroke sensor: 1 pulse per 2 revolutions
  // So: RPM = pulses_per_second * 60 * 2
  
  // Calculate RPM for each sensor
  // Note: Adjust the multiplication factor based on your sensor configuration
  // If sensor gives 1 pulse per revolution: multiply by 60
  // If sensor gives 1 pulse per 2 revolutions (4-stroke): multiply by 120
  
  noInterrupts();  // Disable interrupts while reading counters
  unsigned long count_rpm_1 = pulseCount_rpm_1;
  unsigned long count_rpm_2 = pulseCount_rpm_2;
  unsigned long count_rpm_3 = pulseCount_rpm_3;
  unsigned long count_rpm_4 = pulseCount_rpm_4;
  
  // Reset counters
  pulseCount_rpm_1 = 0;
  pulseCount_rpm_2 = 0;
  pulseCount_rpm_3 = 0;
  pulseCount_rpm_4 = 0;
  interrupts();  // Re-enable interrupts
  
  // Calculate RPM (assuming 1 pulse per 2 revolutions for 4-stroke)
  // Adjust multiplier (120) based on your sensor configuration:
  // - 1 pulse/revolution -> multiply by 60
  // - 1 pulse/2 revolutions -> multiply by 120
  // - 2 pulses/revolution -> multiply by 30
  rpm_1 = count_rpm_1 * 120.0;    // For 4-stroke: 1 pulse per 2 revs
  rpm_2 = count_rpm_2 * 120.0;
  rpm_3 = count_rpm_3 * 120.0;
  rpm_4 = count_rpm_4 * 120.0;
  
  // Print RPM values for debugging
  Serial.print("[RPM] 1:");
  Serial.print(rpm_1, 0);
  Serial.print(" 2:");
  Serial.print(rpm_2, 0);
  Serial.print(" 3:");
  Serial.print(rpm_3, 0);
  Serial.print(" 4:");
  Serial.println(rpm_4, 0);
}

// ============================================================================
// POTENTIOMETER READING
// ============================================================================

void readPotentiometers() {
  // Read all 4 potentiometers (12-bit ADC: 0-4095)
  int raw_1 = analogRead(POT_1_PIN);
  int raw_2 = analogRead(POT_2_PIN);
  int raw_3 = analogRead(POT_3_PIN);
  int raw_4 = analogRead(POT_4_PIN);
  
  // Convert ADC reading to voltage (0-3.3V)
  pot_1_voltage = (raw_1 / (float)ADC_RESOLUTION) * ADC_VREF;
  pot_2_voltage = (raw_2 / (float)ADC_RESOLUTION) * ADC_VREF;
  pot_3_voltage = (raw_3 / (float)ADC_RESOLUTION) * ADC_VREF;
  pot_4_voltage = (raw_4 / (float)ADC_RESOLUTION) * ADC_VREF;
  
  // Print potentiometer readings for debugging
  Serial.print("[POT] 1:");
  Serial.print(pot_1_voltage, 2);
  Serial.print("V 2:");
  Serial.print(pot_2_voltage, 2);
  Serial.print("V 3:");
  Serial.print(pot_3_voltage, 2);
  Serial.print("V 4:");
  Serial.print(pot_4_voltage, 2);
  Serial.println("V");
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
  topics.add("sensors/temperature");
  // topics.add("sensors/humidity");
  topics.add("power/voltage");
  topics.add("power/current");
  topics.add("power/power");
  topics.add("motor/temperature");
  topics.add("motor/controller_temperature");
  
  // Define topic metadata
  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();
  
  // Temperature metadata
  JsonObject tempMeta = metadata["sensors/temperature"].to<JsonObject>();
  tempMeta["description"] = "Ambient temperature sensor";
  tempMeta["unit"] = "°C";
  tempMeta["sampling_rate"] = TEMP_SAMPLING_RATE;
  
  // Humidity metadata
  // JsonObject humMeta = metadata["sensors/humidity"].to<JsonObject>();
  // humMeta["description"] = "Humidity sensor reading";
  // humMeta["unit"] = "%";
  // humMeta["sampling_rate"] = HUMIDITY_SAMPLING_RATE;
  
  // Voltage metadata
  JsonObject voltMeta = metadata["power/voltage"].to<JsonObject>();
  voltMeta["description"] = "Battery/supply voltage";
  voltMeta["unit"] = "V";
  voltMeta["sampling_rate"] = POWER_SAMPLING_RATE;
  
  // Current metadata
  JsonObject currMeta = metadata["power/current"].to<JsonObject>();
  currMeta["description"] = "Current consumption";
  currMeta["unit"] = "A";
  currMeta["sampling_rate"] = POWER_SAMPLING_RATE;
  
  // Power metadata
  JsonObject powMeta = metadata["power/power"].to<JsonObject>();
  powMeta["description"] = "Power consumption";
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
    }
  }
}

float convertBamocarTemp(uint16_t rawValue) {
  // Bamocar temperature conversion formula
  // This is based on typical NTC thermistor lookup tables
  // The exact formula depends on your Bamocar configuration
  
  // Common conversion: Temperature in 0.1°C steps
  // Example: rawValue = 250 means 25.0°C
  float tempC = (float)rawValue / 10.0;
  
  // Alternative: If using raw ADC values, you may need lookup table
  // This is a simplified linear approximation
  // Adjust based on your actual Bamocar manual/configuration
  
  return tempC;
}

// ============================================================================
// DATA PUBLISHING TO BP MOBILE
// ============================================================================

void sendTemperatureData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  
  if (unixTimestamp < 1000000000000ULL) return;
  
  StaticJsonDocument<256> doc;
  doc["type"] = "data";
  doc["topic"] = "sensors/temperature";
  doc["data"]["value"] = temperature;
  doc["data"]["sensor_id"] = "TEMP_001";
  doc["timestamp"] = unixTimestamp;
  
  String message;
  serializeJson(doc, message);
  webSocket.sendTXT(message);
  
  Serial.print("[BP Mobile] Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" °C");
}

void sendHumidityData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  
  if (unixTimestamp < 1000000000000ULL) return;
  
  StaticJsonDocument<256> doc;
  doc["type"] = "data";
  doc["topic"] = "sensors/humidity";
  doc["data"]["value"] = humidity;
  doc["data"]["sensor_id"] = "HUM_001";
  doc["timestamp"] = unixTimestamp;
  
  String message;
  serializeJson(doc, message);
  webSocket.sendTXT(message);
  
  Serial.print("[BP Mobile] Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");
}

void sendPowerData() {
  unsigned long long unixTimestamp = getSynchronizedTime();
  
  if (unixTimestamp < 1000000000000ULL) return;
  
  // Send voltage
  StaticJsonDocument<256> voltDoc;
  voltDoc["type"] = "data";
  voltDoc["topic"] = "power/voltage";
  voltDoc["data"]["value"] = voltage;
  voltDoc["data"]["sensor_id"] = "INA260_001";
  voltDoc["timestamp"] = unixTimestamp;
  
  String voltMsg;
  serializeJson(voltDoc, voltMsg);
  webSocket.sendTXT(voltMsg);
  
  // Send current
  StaticJsonDocument<256> currDoc;
  currDoc["type"] = "data";
  currDoc["topic"] = "power/current";
  currDoc["data"]["value"] = current;
  currDoc["data"]["sensor_id"] = "INA260_001";
  currDoc["timestamp"] = unixTimestamp;
  
  String currMsg;
  serializeJson(currDoc, currMsg);
  webSocket.sendTXT(currMsg);
  
  // Send power
  StaticJsonDocument<256> powDoc;
  powDoc["type"] = "data";
  powDoc["topic"] = "power/power";
  powDoc["data"]["value"] = power;
  powDoc["data"]["sensor_id"] = "INA260_001";
  powDoc["timestamp"] = unixTimestamp;
  
  String powMsg;
  serializeJson(powDoc, powMsg);
  webSocket.sendTXT(powMsg);
  
  Serial.print("[BP Mobile] Power: V=");
  Serial.print(voltage, 2);
  Serial.print("V, I=");
  Serial.print(current, 3);
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
    
    Serial.print("[BP Mobile] CAN: Motor=");
    Serial.print(motorTemp, 1);
    Serial.print("°C, Controller=");
    Serial.print(controllerTemp, 1);
    Serial.println("°C");
  }
}

// ============================================================================
// SD CARD LOGGING
// ============================================================================

void createCSVFile() {
  File dataFile = SD.open(csvFilename.c_str(), FILE_WRITE);
  
  if (dataFile) {
    dataFile.println("Timestamp,DataPoint,Voltage(V),Current(A),Temperature(C),Power(W),MotorTemp(C),ControllerTemp(C),RPM_1,RPM_2,RPM_3,RPM_4,Pot_1(V),Pot_2(V),Pot_3(V),Pot_4(V)");
    dataFile.close();
    Serial.println("[SD Card] CSV header created");
  } else {
    Serial.println("[SD Card] ERROR: Could not create CSV file!");
  }
}

void logDataToSD() {
  File dataFile = SD.open(csvFilename.c_str(), FILE_APPEND);
  
  if (dataFile) {
    unsigned long timestamp = millis() / 1000;
    
    dataFile.print(timestamp);
    dataFile.print(",");
    dataFile.print(dataPoint);
    dataFile.print(",");
    dataFile.print(voltage, 2);
    dataFile.print(",");
    dataFile.print(current, 3);
    dataFile.print(",");
    dataFile.print(temperature, 1);
    dataFile.print(",");
    dataFile.print(power, 2);
    dataFile.print(",");
    dataFile.print(motorTempValid ? motorTemp : 0.0, 1);
    dataFile.print(",");
    dataFile.print(controllerTempValid ? controllerTemp : 0.0, 1);
    dataFile.print(",");
    dataFile.print(rpm_1, 0);
    dataFile.print(",");
    dataFile.print(rpm_2, 0);
    dataFile.print(",");
    dataFile.print(rpm_3, 0);
    dataFile.print(",");
    dataFile.print(rpm_4, 0);
    dataFile.print(",");
    dataFile.print(pot_1_voltage, 2);
    dataFile.print(",");
    dataFile.print(pot_2_voltage, 2);
    dataFile.print(",");
    dataFile.print(pot_3_voltage, 2);
    dataFile.print(",");
    dataFile.println(pot_4_voltage, 2);
    
    dataFile.close();
    
    Serial.print("[SD Card] Logged point #");
    Serial.print(dataPoint);
    Serial.print(" | V:");
    Serial.print(voltage, 2);
    Serial.print("V I:");
    Serial.print(current, 3);
    Serial.print("A T:");
    Serial.print(temperature, 1);
    Serial.print("°C P:");
    Serial.print(power, 2);
    Serial.print("W MT:");
    Serial.print(motorTemp, 1);
    Serial.print("°C CT:");
    Serial.print(controllerTemp, 1);
    Serial.print("°C RPM:");
    Serial.print(rpm_1, 0);
    Serial.print("/");
    Serial.print(rpm_2, 0);
    Serial.print("/");
    Serial.print(rpm_3, 0);
    Serial.print("/");
    Serial.print(rpm_4, 0);
    Serial.print(" POT:");
    Serial.print(pot_1_voltage, 2);
    Serial.print("/");
    Serial.print(pot_2_voltage, 2);
    Serial.print("/");
    Serial.print(pot_3_voltage, 2);
    Serial.print("/");
    Serial.println(pot_4_voltage, 2);
    
    dataPoint++;
  } else {
    Serial.println("[SD Card] ERROR: Could not open file!");
  }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

unsigned long long getSynchronizedTime() {
  return (unsigned long long)millis() + serverTimeOffset;
}
