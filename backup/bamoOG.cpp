#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/twai.h>
#include <WiFi.h>
#include <time.h>

// WiFi Configuration - UPDATE THESE WITH YOUR CREDENTIALS
const char* WIFI_SSID = "jackienyyy";
const char* WIFI_PASSWORD = "jackjack";

// NTP Configuration
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 7 * 3600;  // GMT+7 for Bangkok
const int DAYLIGHT_OFFSET_SEC = 0;

// BAMOCAR Register IDs (from your working code)
#define REGID_SPEED_ACTUAL 0x30 
#define REGID_VOLTAGE 0xEB
#define REGID_CURRENT 0x5f
#define REGID_MOTOR_TEMP 0x49
#define REGID_CONTROLLER_TEMP 0x4A

// CAN IDs (confirmed working)
#define CAN_ID_REQUEST 0x201
#define CAN_ID_RESPONSE 0x181  // CONFIRMED: 0x181 works, not 0x180

// Pin definitions for ESP32-S3-WROOM-1 TWAI CAN
#define CAN_TX_PIN 14   // TWAI TX pin → SN65HVD230 TX
#define CAN_RX_PIN 13   // TWAI RX pin → SN65HVD230 RX

// SD card pins
#define SD_CS_PIN 10
#define SD_SCK_PIN 12
#define SD_MISO_PIN 11
#define SD_MOSI_PIN 9

// Thermistor removed - not used

// Logging configuration
#define LOG_INTERVAL_MS 200   // Slower for more reliable readings
#define BUFFER_SIZE 10

// Communication state tracking
bool dataReceived = false;

// System state
bool isLogging = false;
bool isTransmitting = false;  // Added this from your working code
bool useMockData = false;     // Default to real CAN data
bool canInitialized = false;
bool sdInitialized = false;
bool wifiConnected = false;
bool timeSync = false;
int fileNumber = 1;
String currentFileName = "";
unsigned long sessionStartTime = 0;

// Current sensor data
float voltage = 0.0;
float current = 0.0;
float power = 0.0;
float motorTemp = 0.0;
float controllerTemp = 0.0;
float bamoTemp = 25.0;  // Default ambient temperature
int rpm = 0;

// Data buffer for efficient SD writing
struct DataPoint {
  unsigned long timestamp;
  String isoTimestamp;
  float voltage;
  float current;
  float power;
  float motorTemp;
  float bamoTemp;
  int rpm;
};

DataPoint dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastLogTime = 0;

// TWAI message structures
twai_message_t tx_message;
twai_message_t rx_message;

// =======================
// FUNCTION DECLARATIONS
// =======================
void initializeSystem();
void initSD();
void initTWAI();
void initWiFi();
void syncTime();
String getCurrentISOTime();
void findNextFileNumber();
void generateMockData();
void sendCANRequest(uint8_t regID);
void stopSpeedTransmission();
void readCANResponse();
void clearCANBuffer();
void startLogging();
void stopLogging();
void addDataToBuffer();
void flushBufferToSD();
void handleCommands();
void showStatus();
void listFiles();
void deleteLogFiles();
void showSDInfo();
void showCANStatus();
void showWiFiStatus();
void displayValues();
void extendedCANDiagnostics();
void testMotorControl();
void sendControlCommand(uint8_t regID, int16_t value);

// =======================
// MAIN PROGRAM
// =======================

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  while (!Serial && millis() < 5000) {
    delay(100);
  }
  
  Serial.println("===============================================");
  Serial.println("ESP32-S3 BAMOCAR Data Logger v2.3 (Fixed CAN)");
  Serial.println("Based on working BAMOCAR communication protocol");
  Serial.println("===============================================");
  
  initializeSystem();
  
  Serial.println("\n=== BAMOCAR LOGGER COMMANDS ===");
  Serial.println("'1' - Start transmission and logging");
  Serial.println("'0' - Stop transmission and logging");
  Serial.println("'s' - Show system status");
  Serial.println("'m' - Toggle mock/real CAN data");
  Serial.println("'l' - List log files on SD card");
  Serial.println("'d' - Delete all log files");
  Serial.println("'f' - Manual buffer flush");
  Serial.println("'i' - Show SD card info");
  Serial.println("'c' - Show CAN status");
  Serial.println("'w' - Show WiFi status");
  Serial.println("'r' - Reconnect WiFi");
  Serial.println("'n' - Sync time from NTP");
  Serial.println("'x' - Extended CAN diagnostics");
  Serial.println("'p' - Motor control test");
  Serial.println("'t' - Quick SD test");
  Serial.println("===============================");
  Serial.println("🚀 System ready! Type a command...");
}

void loop() {
  // Handle user commands
  handleCommands();
  
  // Data acquisition at specified interval (like your working code)
  if (millis() - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = millis();
    
    // Get data (mock or real CAN) with improved timing
    if (useMockData) {
      generateMockData();
    } else {
      // Send CAN requests with proper delays for reliable data
      if (isTransmitting) {
        sendCANRequest(REGID_VOLTAGE);
        delay(50);  // Wait for response
        readCANResponse();
        
        sendCANRequest(REGID_CURRENT);
        delay(50);  // Wait for response
        readCANResponse();
        
        sendCANRequest(REGID_MOTOR_TEMP);
        delay(50);  // Wait for response
        readCANResponse();
        
        sendCANRequest(REGID_SPEED_ACTUAL);
        delay(50);  // Wait for response
        readCANResponse();
      } else {
        // Still read any incoming messages even when not transmitting
        readCANResponse();
      }
      
      // Calculate power
      power = voltage * current;
    }
    
    // Display values
    displayValues();
    
    // Add to buffer if logging
    if (isLogging) {
      addDataToBuffer();
    }
    
    // Clear CAN buffer (from your working code)
    clearCANBuffer();
  }
  
  delay(10);
}

// =======================
// INITIALIZATION FUNCTIONS
// =======================

void initializeSystem() {
  Serial.println("Initializing system components...");
  
  // Initialize WiFi and time sync first
  initWiFi();
  
  // Initialize SD card
  initSD();
  
  // Initialize TWAI CAN system
  initTWAI();
  
  Serial.println("✅ System initialization complete!");
  Serial.print("📊 Sampling rate: ");
  Serial.print(1000.0 / LOG_INTERVAL_MS);
  Serial.println(" Hz");
}

void initWiFi() {
  Serial.println("🔧 Initializing WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("✅ WiFi connected!");
    Serial.print("📶 IP address: ");
    Serial.println(WiFi.localIP());
    syncTime();
  } else {
    wifiConnected = false;
    Serial.println("❌ WiFi connection failed!");
  }
}

void syncTime() {
  if (!wifiConnected) return;
  
  Serial.println("🕐 Syncing time with NTP server...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  int attempts = 0;
  while (!time(nullptr) && attempts < 10) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  time_t now = time(nullptr);
  if (now > 1000000000) {
    timeSync = true;
    Serial.println("✅ Time synchronized!");
    Serial.print("🕐 Current time: ");
    Serial.println(getCurrentISOTime());
  } else {
    timeSync = false;
    Serial.println("❌ Time sync failed!");
  }
}

String getCurrentISOTime() {
  if (!timeSync) return "NO_TIME_SYNC";
  
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", timeinfo);
  return String(buffer);
}

void initSD() {
  Serial.println("🔧 Initializing SD Card...");
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  if (SD.begin(SD_CS_PIN, SPI, 4000000)) {
    sdInitialized = true;
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print("✅ SD Card ready (");
    Serial.print(cardSize);
    Serial.println(" MB)");
    findNextFileNumber();
  } else {
    Serial.println("❌ SD Card initialization failed!");
    sdInitialized = false;
  }
}

void initTWAI() {
  Serial.println("🔧 Initializing ESP32-S3 TWAI CAN...");
  
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
  if (result == ESP_OK) {
    result = twai_start();
    if (result == ESP_OK) {
      canInitialized = true;
      Serial.println("✅ TWAI CAN controller ready (500kbps)");
    } else {
      Serial.print("❌ TWAI start failed: ");
      Serial.println(esp_err_to_name(result));
      canInitialized = false;
    }
  } else {
    Serial.print("❌ TWAI driver install failed: ");
    Serial.println(esp_err_to_name(result));
    canInitialized = false;
  }
}

void findNextFileNumber() {
  while (SD.exists("/bamocar_" + String(fileNumber) + ".csv")) {
    fileNumber++;
    if (fileNumber > 9999) fileNumber = 1;
  }
}

// =======================
// CAN COMMUNICATION (Fixed based on working code)
// =======================

void sendCANRequest(uint8_t regID) {
  if (!canInitialized) return;
  
  // Use exact format from your working code with improved timing
  tx_message.identifier = CAN_ID_REQUEST;
  tx_message.extd = 0;
  tx_message.rtr = 0;
  tx_message.data_length_code = 3;
  tx_message.data[0] = 0x3D;  // BAMOCAR read command
  tx_message.data[1] = regID;
  tx_message.data[2] = 0x64;  // Interval
  
  esp_err_t result = twai_transmit(&tx_message, pdMS_TO_TICKS(100));
  if (result != ESP_OK) {
    Serial.print("❌ TX failed for 0x");
    Serial.print(regID, HEX);
    Serial.print(": ");
    Serial.println(esp_err_to_name(result));
  }
}

void stopSpeedTransmission() {
  if (!canInitialized) return;
  
  // Use exact format from your working code
  tx_message.identifier = CAN_ID_REQUEST;
  tx_message.extd = 0;
  tx_message.rtr = 0;
  tx_message.data_length_code = 3;
  tx_message.data[0] = 0x3D;
  tx_message.data[1] = REGID_SPEED_ACTUAL;
  tx_message.data[2] = 0xFF;  // Stop transmission
  
  twai_transmit(&tx_message, pdMS_TO_TICKS(100));
}

void readCANResponse() {
  if (!canInitialized) return;
  
  int messagesRead = 0;
  while (messagesRead < 10) {
    esp_err_t result = twai_receive(&rx_message, pdMS_TO_TICKS(10));
    
    if (result == ESP_OK) {
      messagesRead++;
      
      // Use the EXACT parsing logic from your working code
      if (rx_message.data_length_code >= 3) {
        uint8_t regID = rx_message.data[0];  // REGID is in byte 0 (not byte 1!)
        
        if (regID == REGID_VOLTAGE) {
          uint16_t rawVoltage = (rx_message.data[2] << 8) | rx_message.data[1]; // Little-Endian
          voltage = rawVoltage / 55.1204; // Use your working conversion factor
        } else if (regID == REGID_CURRENT) {
          uint16_t rawCurrent = (rx_message.data[2] << 8) | rx_message.data[1]; // Little-Endian
          current = rawCurrent * 0.373832; // Use your working conversion factor
        } else if (regID == REGID_MOTOR_TEMP) {
          uint16_t rawTemp = (rx_message.data[2] << 8) | rx_message.data[1]; // Little-Endian
          motorTemp = rawTemp; // Direct value
        } else if (regID == REGID_SPEED_ACTUAL) {
          uint16_t rawRPM = (rx_message.data[2] << 8) | rx_message.data[1];
          rpm = rawRPM;
          
          // Debug: Show if RPM changed significantly
          static int lastRPM = 0;
          if (abs(rpm - lastRPM) > 50) {
            Serial.print("🔄 RPM changed: ");
            Serial.print(lastRPM);
            Serial.print(" → ");
            Serial.print(rpm);
            Serial.println(" (Check if motor actually moved!)");
            lastRPM = rpm;
          }
        }
      }
    } else if (result == ESP_ERR_TIMEOUT) {
      break;
    } else {
      Serial.print("CAN RX error: ");
      Serial.println(esp_err_to_name(result));
      break;
    }
  }
}

void clearCANBuffer() {
  // Clear any remaining messages in buffer
  while (twai_receive(&rx_message, pdMS_TO_TICKS(1)) == ESP_OK) {
    // Just consume the messages
  }
}

// =======================
// THERMISTOR READING (removed - not used)
// =======================

// =======================
// MOCK DATA (for testing)
// =======================

void generateMockData() {
  static float baseVoltage = 48.0;
  static float baseCurrent = 15.0;
  static float baseMotorTemp = 35.0;
  static int baseRpm = 1800;
  
  voltage = baseVoltage + random(-15, 15) / 10.0;
  current = baseCurrent + random(-25, 25) / 10.0;
  motorTemp = baseMotorTemp + random(-10, 20) / 10.0;
  rpm = baseRpm + random(-300, 300);
  
  voltage = constrain(voltage, 35.0, 60.0);
  current = constrain(current, -40.0, 40.0);
  motorTemp = constrain(motorTemp, 20.0, 80.0);
  rpm = constrain(rpm, 0, 3500);
  
  power = voltage * current;
}

// =======================
// LOGGING FUNCTIONS
// =======================

void startLogging() {
  if (!sdInitialized) {
    Serial.println("❌ Cannot start logging - SD card not ready!");
    return;
  }
  
  if (isLogging) {
    stopLogging();
    delay(200);
  }
  
  String timestamp = timeSync ? getCurrentISOTime().substring(0, 10) : "notime";
  timestamp.replace("-", "");
  currentFileName = "/bamocar_" + timestamp + "_" + String(fileNumber) + ".csv";
  
  File dataFile = SD.open(currentFileName, FILE_WRITE);
  if (dataFile) {
    // Header matching your working code structure
    dataFile.println("Time,Voltage(V),Current(A),Power(W),MotorTemp(°C),Ambient(°C),RPM,ISO_Timestamp");
    
    if (timeSync) {
      dataFile.print("# Session started: ");
      dataFile.println(getCurrentISOTime());
    }
    dataFile.close();
    
    isLogging = true;
    isTransmitting = true;  // Also start CAN transmission like your working code
    bufferIndex = 0;
    sessionStartTime = millis();
    
    Serial.println("🎬 LOGGING AND TRANSMISSION STARTED!");
    Serial.println("📁 File: " + currentFileName);
    Serial.print("🔧 Data source: ");
    Serial.println(useMockData ? "MOCK DATA" : "REAL CAN BUS");
  } else {
    Serial.println("❌ Failed to create log file!");
  }
}

void stopLogging() {
  if (!isLogging) {
    Serial.println("Not currently logging");
    return;
  }
  
  if (bufferIndex > 0) {
    flushBufferToSD();
  }
  
  isLogging = false;
  isTransmitting = false;  // Also stop CAN transmission
  
  // Send stop command like your working code
  if (canInitialized) {
    stopSpeedTransmission();
  }
  
  Serial.println("⏹️  LOGGING AND TRANSMISSION STOPPED");
  Serial.println("💾 Data saved to: " + currentFileName);
  
  fileNumber++;
}

void addDataToBuffer() {
  dataBuffer[bufferIndex].timestamp = millis() - sessionStartTime;
  dataBuffer[bufferIndex].isoTimestamp = getCurrentISOTime();
  dataBuffer[bufferIndex].voltage = voltage;
  if(current <10000){
    dataBuffer[bufferIndex].current = current;
  }
  dataBuffer[bufferIndex].power = power;
  dataBuffer[bufferIndex].motorTemp = motorTemp;
  dataBuffer[bufferIndex].bamoTemp = bamoTemp;
  dataBuffer[bufferIndex].rpm = rpm;
  
  bufferIndex++;
  
  if (bufferIndex >= BUFFER_SIZE) {
    flushBufferToSD();
  }
}

void flushBufferToSD() {
  if (bufferIndex == 0) return;
  
  File dataFile = SD.open(currentFileName, FILE_APPEND);
  if (dataFile) {
    for (int i = 0; i < bufferIndex; i++) {
      float elapsedSec = dataBuffer[i].timestamp / 1000.0;
      
      // CSV format matching your working code
      dataFile.print(elapsedSec, 2);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].voltage, 2);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].current, 2);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].power, 2);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].motorTemp, 1);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].bamoTemp, 1);
      dataFile.print(",");
      dataFile.print(dataBuffer[i].rpm);
      dataFile.print(",");
      dataFile.println(dataBuffer[i].isoTimestamp);
    }
    dataFile.close();
    bufferIndex = 0;
    Serial.println("💾 Data written to SD Card.");
  } else {
    Serial.println("❌ Error writing to SD Card");
  }
}

// =======================
// DISPLAY AND UI FUNCTIONS
// =======================

void displayValues() {
  // Show timestamp for better tracking
  Serial.print("[");
  Serial.print(millis()/1000);
  Serial.print("s] ");
  
  Serial.print("V: ");
  Serial.print(voltage, 2);
  Serial.print("V  I: ");
  if (current < 10000){
    Serial.print(current, 2);
    
  }
  Serial.print("A  P: ");
  Serial.print(power, 1);
  Serial.print("W  T: ");
  Serial.print(motorTemp, 1);
  Serial.print("°C  RPM: ");
  Serial.print(rpm);
  
  // Add data quality indicators
  if (dataReceived) {
    Serial.print(" ✅");
  } else {
    Serial.print(" ⚠️");
  }
  
  // Add motor status indicators with more detail
  if (rpm > 50) {
    Serial.print(" 🔄");
    if (current > 2.0) {
      Serial.print("_LOAD");
    } else {
      Serial.print("_LIGHT"); 
    }
  } else if (current > 1.0) {
    Serial.print(" ⚡_IDLE");
  } else if (voltage < 20.0) {
    Serial.print(" 🔋_LOW");
  } else {
    Serial.print(" ⏸️_STOP");
  }
  
  Serial.println();
}

void handleCommands() {
  if (Serial.available()) {
    char command = Serial.read();
    Serial.println();
    
    switch (command) {
      case '1':
        Serial.println(">>> Starting transmission and logging...");
        startLogging();
        break;
        
      case 't':
        // Quick SD test
        {
          File testFile = SD.open("/test.txt", FILE_WRITE);
          if (testFile) {
            testFile.println("Test successful");
            testFile.close();
            Serial.println("✅ SD test passed");
          } else {
            Serial.println("❌ SD test failed");
          }
        }
        break;
        
      case '0':
        Serial.println(">>> Stopping transmission and logging...");
        stopLogging();
        break;
        
      case 's':
        Serial.println(">>> System Status:");
        showStatus();
        break;
        
      case 'm':
        useMockData = !useMockData;
        Serial.print(">>> Data source: ");
        Serial.println(useMockData ? "MOCK DATA" : "REAL CAN BUS");
        break;
        
      case 'l':
        listFiles();
        break;
        
      case 'd':
        deleteLogFiles();
        break;
        
      case 'f':
        if (isLogging && bufferIndex > 0) {
          flushBufferToSD();
        }
        break;
        
      case 'i':
        showSDInfo();
        break;
        
      case 'c':
        showCANStatus();
        break;
        
      case 'w':
        showWiFiStatus();
        break;
        
      case 'r':
        initWiFi();
        break;
        
      case 'n':
        syncTime();
        break;
        
      case 'x':
        Serial.println(">>> Extended CAN diagnostics...");
        extendedCANDiagnostics();
        break;
        
      case 'p':
        Serial.println(">>> Motor control test...");
        testMotorControl();
        break;
    }
  }
}

void showStatus() {
  Serial.println("=== SYSTEM STATUS ===");
  Serial.print("🎬 Logging: ");
  Serial.println(isLogging ? "✅ ACTIVE" : "⏹️ STOPPED");
  Serial.print("📡 Transmitting: ");
  Serial.println(isTransmitting ? "✅ ACTIVE" : "⏹️ STOPPED");
  Serial.print("🔧 Data source: ");
  Serial.println(useMockData ? "MOCK" : "REAL CAN");
  Serial.print("💾 SD Card: ");
  Serial.println(sdInitialized ? "✅ Ready" : "❌ Failed");
  Serial.print("🔌 CAN Bus: ");
  Serial.println(canInitialized ? "✅ Ready" : "❌ Failed");
  Serial.print("📶 WiFi: ");
  Serial.println(wifiConnected ? "✅ Connected" : "❌ Disconnected");
  Serial.print("🕐 Time Sync: ");
  Serial.println(timeSync ? "✅ Synced" : "❌ Not synced");
  
  Serial.println("\n📊 Current Readings:");
  displayValues();
  Serial.println("=====================");
}

void showCANStatus() {
  Serial.println("=== CAN STATUS ===");
  Serial.print("🔌 Initialized: ");
  Serial.println(canInitialized ? "✅ YES" : "❌ NO");
  Serial.print("📍 TX Pin: GPIO ");
  Serial.println(CAN_TX_PIN);
  Serial.print("📍 RX Pin: GPIO ");
  Serial.println(CAN_RX_PIN);
  Serial.println("🏃 Speed: 500 kbps");
  Serial.print("📨 Response ID: 0x");
  Serial.println(CAN_ID_RESPONSE, HEX);
  Serial.println("🔧 Using BAMOCAR protocol from working code");
  Serial.println("==================");
}

void showWiFiStatus() {
  Serial.println("=== WiFi STATUS ===");
  Serial.print("📶 Connected: ");
  Serial.println(wifiConnected ? "✅ YES" : "❌ NO");
  if (wifiConnected) {
    Serial.print("📡 SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("📶 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("📶 Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
  Serial.println("===================");
}

void listFiles() {
  if (!sdInitialized) return;
  
  Serial.println("=== LOG FILES ===");
  File root = SD.open("/");
  int count = 0;
  
  File file = root.openNextFile();
  while (file) {
    String fileName = String(file.name());
    if (fileName.startsWith("bamocar_")) {
      Serial.print("📄 ");
      Serial.print(fileName);
      Serial.print(" (");
      Serial.print(file.size());
      Serial.println(" bytes)");
      count++;
    }
    file = root.openNextFile();
  }
  
  Serial.print("Total: ");
  Serial.print(count);
  Serial.println(" files");
  Serial.println("==================");
}

void deleteLogFiles() {
  if (!sdInitialized) return;
  
  File root = SD.open("/");
  int deleted = 0;
  
  File file = root.openNextFile();
  while (file) {
    String fileName = String(file.name());
    if (fileName.startsWith("bamocar_")) {
      file.close();
      if (SD.remove(fileName)) {
        Serial.print("🗑️ Deleted: ");
        Serial.println(fileName);
        deleted++;
      }
    }
    file = root.openNextFile();
  }
  
  Serial.print("✅ Deleted ");
  Serial.print(deleted);
  Serial.println(" files");
  fileNumber = 1;
}

void showSDInfo() {
  if (!sdInitialized) return;
  
  Serial.println("=== SD CARD INFO ===");
  Serial.print("💳 Size: ");
  Serial.print(SD.cardSize() / (1024 * 1024));
  Serial.println(" MB");
  Serial.print("💾 Used: ");
  Serial.print(SD.usedBytes() / (1024 * 1024));
  Serial.println(" MB");
  Serial.println("====================");
}

// =======================
// DIAGNOSTIC FUNCTIONS
// =======================

void extendedCANDiagnostics() {
  if (!canInitialized) {
    Serial.println("❌ CAN not initialized");
    return;
  }
  
  Serial.println("🔍 EXTENDED CAN DIAGNOSTICS");
  Serial.println("Listening for ALL CAN messages for 10 seconds...");
  Serial.println("Format: ID [DATA_BYTES] - Description");
  Serial.println("=====================================");
  
  unsigned long startTime = millis();
  int messageCount = 0;
  
  while (millis() - startTime < 10000) {
    twai_message_t msg;
    esp_err_t result = twai_receive(&msg, pdMS_TO_TICKS(100));
    
    if (result == ESP_OK) {
      messageCount++;
      
      Serial.print("ID: 0x");
      if (msg.identifier < 0x100) Serial.print("0");
      if (msg.identifier < 0x10) Serial.print("0");
      Serial.print(msg.identifier, HEX);
      Serial.print(" [");
      
      for (int i = 0; i < msg.data_length_code; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.print("] - ");
      
      // Decode known BAMOCAR messages
      if (msg.identifier == CAN_ID_RESPONSE) {
        uint8_t regID = msg.data[0];
        Serial.print("BAMOCAR Response: ");
        switch (regID) {
          case REGID_VOLTAGE:
            Serial.print("Voltage");
            break;
          case REGID_CURRENT:
            Serial.print("Current");
            break;
          case REGID_MOTOR_TEMP:
            Serial.print("Motor Temp");
            break;
          case REGID_SPEED_ACTUAL:
            Serial.print("RPM");
            break;
          default:
            Serial.print("Unknown Reg 0x");
            Serial.print(regID, HEX);
        }
      } else {
        Serial.print("Unknown message");
      }
      Serial.println();
    }
    
    delay(10);
  }
  
  Serial.println("=====================================");
  Serial.print("📊 Total messages received: ");
  Serial.println(messageCount);
  
  if (messageCount == 0) {
    Serial.println("❌ No CAN messages detected!");
    Serial.println("🔧 Check:");
    Serial.println("   - CAN bus termination (120Ω)");
    Serial.println("   - BAMOCAR power and CAN enable");
    Serial.println("   - Wiring (CAN_H/CAN_L not swapped)");
  } else {
    Serial.println("✅ CAN communication working");
  }
}

void testMotorControl() {
  Serial.println("🔧 MOTOR CONTROL TEST");
  Serial.println("Motor works with pedal - testing CAN control priority");
  Serial.println("⚠️  ENSURE MOTOR IS SAFE TO RUN!");
  Serial.println("Press 'y' to continue or any other key to cancel...");
  
  // Wait for user confirmation
  while (!Serial.available()) {
    delay(100);
  }
  
  char confirm = Serial.read();
  if (confirm != 'y' && confirm != 'Y') {
    Serial.println("❌ Motor test cancelled");
    return;
  }
  
  Serial.println("🚀 Testing CAN control vs analog throttle...");
  Serial.println("💡 Since motor works with pedal, issue is likely:");
  Serial.println("   - CAN control disabled in BAMOCAR config");
  Serial.println("   - Analog throttle has priority over CAN");
  Serial.println("   - Wrong CAN register addresses");
  Serial.println("   - CAN control mode not selected");
  Serial.println();
  
  // BAMOCAR control register IDs - try multiple possibilities
  const uint8_t REGID_ENABLE = 0x51;
  const uint8_t REGID_SPEED_SETPOINT = 0x31;
  const uint8_t REGID_TORQUE_SETPOINT = 0x90;
  const uint8_t REGID_THROTTLE = 0x26;        // Alternative throttle register
  const uint8_t REGID_CONTROL_MODE = 0x58;    // Control mode selection
  const uint8_t REGID_CAN_ENABLE = 0x2C;      // CAN control enable
  
  Serial.println("📊 Initial state (with pedal at zero):");
  for (int i = 0; i < 3; i++) {
    sendCANRequest(REGID_SPEED_ACTUAL);
    sendCANRequest(REGID_CURRENT);
    sendCANRequest(REGID_VOLTAGE);
    delay(200);
    readCANResponse();
    displayValues();
    delay(300);
  }
  Serial.println();
  
  Serial.println("1. Trying to enable CAN control mode...");
  sendControlCommand(REGID_CAN_ENABLE, 1);     // Enable CAN control
  delay(500);
  sendControlCommand(REGID_CONTROL_MODE, 2);   // Try speed control mode
  delay(500);
  sendControlCommand(REGID_ENABLE, 1);         // Enable motor
  delay(1000);
  
  Serial.println("📊 After CAN enable commands:");
  sendCANRequest(REGID_SPEED_ACTUAL);
  sendCANRequest(REGID_CURRENT);
  delay(200);
  readCANResponse();
  displayValues();
  Serial.println();
  
  Serial.println("2. Testing different speed command registers...");
  
  // Try standard speed setpoint
  Serial.println("   Trying REGID_SPEED_SETPOINT (0x31)...");
  sendControlCommand(REGID_SPEED_SETPOINT, 200);
  delay(2000);
  sendCANRequest(REGID_SPEED_ACTUAL);
  sendCANRequest(REGID_CURRENT);
  delay(200);
  readCANResponse();
  displayValues();
  Serial.println("   👁️  Any movement with speed setpoint?");
  delay(1000);
  
  // Try throttle register
  Serial.println("   Trying REGID_THROTTLE (0x26)...");
  sendControlCommand(REGID_THROTTLE, 200);
  delay(2000);
  sendCANRequest(REGID_SPEED_ACTUAL);
  sendCANRequest(REGID_CURRENT);
  delay(200);
  readCANResponse();
  displayValues();
  Serial.println("   👁️  Any movement with throttle register?");
  delay(1000);
  
  // Try torque mode
  Serial.println("   Trying REGID_TORQUE_SETPOINT (0x90)...");
  sendControlCommand(REGID_TORQUE_SETPOINT, 100);
  delay(2000);
  sendCANRequest(REGID_SPEED_ACTUAL);
  sendCANRequest(REGID_CURRENT);
  delay(200);
  readCANResponse();
  displayValues();
  Serial.println("   👁️  Any movement with torque setpoint?");
  delay(1000);
  
  Serial.println("3. Testing if analog throttle overrides CAN...");
  Serial.println("   💡 Try pressing pedal slightly while speed command is active...");
  Serial.println("   📊 Monitoring for 10 seconds:");
  
  sendControlCommand(REGID_SPEED_SETPOINT, 300); // Set speed command
  
  for (int i = 0; i < 10; i++) {
    sendCANRequest(REGID_SPEED_ACTUAL);
    sendCANRequest(REGID_CURRENT);
    delay(100);
    readCANResponse();
    Serial.print("   [");
    Serial.print(i+1);
    Serial.print("/10] ");
    displayValues();
    delay(900);
  }
  
  Serial.println("4. Checking BAMOCAR status registers...");
  // Try reading status and configuration registers
  uint8_t statusRegs[] = {0x40, 0x41, 0x42, 0x50, 0x58, 0x2C};
  const char* statusNames[] = {"Status", "Fault", "State", "Enable", "Control Mode", "CAN Enable"};
  
  for (int i = 0; i < 6; i++) {
    Serial.print("   Reading ");
    Serial.print(statusNames[i]);
    Serial.print(" (0x");
    Serial.print(statusRegs[i], HEX);
    Serial.print("): ");
    sendCANRequest(statusRegs[i]);
    delay(100);
    
    // Try to read the response
    if (twai_receive(&rx_message, pdMS_TO_TICKS(200)) == ESP_OK) {
      if (rx_message.data[0] == statusRegs[i]) {
        uint16_t value = (rx_message.data[2] << 8) | rx_message.data[1];
        Serial.print("0x");
        Serial.print(value, HEX);
        Serial.print(" (");
        Serial.print(value);
        Serial.println(")");
      } else {
        Serial.println("No response");
      }
    } else {
      Serial.println("Timeout");
    }
  }
  
  Serial.println("5. Stopping all commands...");
  sendControlCommand(REGID_SPEED_SETPOINT, 0);
  sendControlCommand(REGID_TORQUE_SETPOINT, 0);
  sendControlCommand(REGID_THROTTLE, 0);
  delay(500);
  
  Serial.println("✅ Enhanced motor test complete");
  Serial.println();
  Serial.println("🔍 ENHANCED ANALYSIS:");
  Serial.println("Since motor works with pedal but not CAN:");
  Serial.println();
  Serial.println("🔧 Most likely causes:");
  Serial.println("   1. BAMOCAR in 'Analog Priority' mode");
  Serial.println("   2. CAN control not enabled in BAMOCAR config");
  Serial.println("   3. Wrong control mode (speed vs torque vs throttle)");
  Serial.println("   4. Analog throttle input overriding CAN commands");
  Serial.println();
  Serial.println("💡 Solutions to try:");
  Serial.println("   1. Use BAMOCAR configuration software");
  Serial.println("   2. Set control mode to 'CAN' or 'Digital'");
  Serial.println("   3. Disable analog throttle input");
  Serial.println("   4. Check DIP switches on BAMOCAR");
  Serial.println("   5. Ensure analog throttle is at zero/idle");
}

void sendControlCommand(uint8_t regID, int16_t value) {
  if (!canInitialized) return;
  
  // Check CAN bus status before sending
  twai_status_info_t status_info;
  esp_err_t status_result = twai_get_status_info(&status_info);
  
  if (status_result == ESP_OK && status_info.state != TWAI_STATE_RUNNING) {
    Serial.println("⚠️ CAN not in running state, attempting recovery...");
    if (status_info.state == TWAI_STATE_BUS_OFF) {
      twai_initiate_recovery();
      delay(100);
    }
  }
  
  tx_message.identifier = CAN_ID_REQUEST;
  tx_message.extd = 0;
  tx_message.rtr = 0;
  tx_message.data_length_code = 6;
  tx_message.data[0] = 0x3C;  // Write command (not 0x3D read)
  tx_message.data[1] = regID;
  tx_message.data[2] = value & 0xFF;        // LSB
  tx_message.data[3] = (value >> 8) & 0xFF; // MSB
  tx_message.data[4] = 0x00;
  tx_message.data[5] = 0x00;
  
  // Try sending with longer timeout and retry
  esp_err_t result = twai_transmit(&tx_message, pdMS_TO_TICKS(500));
  
  if (result == ESP_ERR_TIMEOUT) {
    Serial.println("⚠️ First attempt timeout, retrying...");
    delay(100);
    result = twai_transmit(&tx_message, pdMS_TO_TICKS(1000));
  }
  
  if (result == ESP_OK) {
    Serial.print("✅ Sent control command: Reg 0x");
    Serial.print(regID, HEX);
    Serial.print(" = ");
    Serial.println(value);
  } else {
    Serial.print("❌ Failed to send command after retry: ");
    Serial.println(esp_err_to_name(result));
    
    // Show CAN status for debugging
    if (twai_get_status_info(&status_info) == ESP_OK) {
      Serial.print("   CAN State: ");
      Serial.print(status_info.state);
      Serial.print(", TX Queue: ");
      Serial.print(status_info.msgs_to_tx);
      Serial.print(", TX Errors: ");
      Serial.println(status_info.tx_error_counter);
    }
  }
}

String formatElapsedTime(unsigned long seconds) {
  unsigned long hours = seconds / 3600;
  unsigned long minutes = (seconds % 3600) / 60;
  unsigned long secs = seconds % 60;
  
  String result = "";
  if (hours > 0) result += String(hours) + "h ";
  if (minutes > 0) result += String(minutes) + "m ";
  result += String(secs) + "s";
  
  return result;
}