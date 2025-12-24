/*
 * Bamocar CAN Bus Data Reader
 * 
 * This file contains program code for:
 * - CAN Bus initialization and configuration
 * - Requesting Bamocar motor controller data via CAN
 * - Processing and reading CAN responses from Bamocar
 * - Converting raw Bamocar data to meaningful values
 * 
 * Required Libraries:
 * - ESP32-TWAI-CAN by miwagner (or CAN library for ESP32)
 */

#include <Arduino.h>
#include <driver/twai.h>  // ESP32 built-in CAN (TWAI) driver

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
// GLOBAL VARIABLES - CAN Bus
// ============================================================================
bool canBusReady = false;
unsigned long lastCANRequest = 0;
uint8_t canRequestState = 0;  // 0=motor temp, 1=controller temp, 2=voltage, 3=current

// ============================================================================
// GLOBAL VARIABLES - Bamocar Data
// ============================================================================
float motorTemp = 0.0;
float controllerTemp = 0.0;
float canVoltage = 0.0;      // DC Link Voltage from CAN
float canCurrent = 0.0;      // Motor Current from CAN
bool motorTempValid = false;
bool controllerTempValid = false;
bool canVoltageValid = false;
bool canCurrentValid = false;

// ============================================================================
// CAN BUS INITIALIZATION
// ============================================================================

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

// ============================================================================
// REQUEST BAMOCAR DATA
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

// ============================================================================
// PROCESS CAN MESSAGES
// ============================================================================

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

// ============================================================================
// BAMOCAR DATA CONVERSION
// ============================================================================

float convertBamocarTemp(uint16_t rawValue) {
  // Bamocar temperature conversion formula
  // Common conversion: Temperature in 0.1°C steps
  // Example: rawValue = 250 means 25.0°C
  float tempC = (float)rawValue / 10.0;
  
  return tempC;
}
