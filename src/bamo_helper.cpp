#include <Arduino.h>
#include <driver/twai.h>
#include <bamo_helper.h>
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
    // if (regAddress == BAMOCAR_REG_MOTOR_TEMP) Serial.print("Motor Temp");
    // else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) Serial.print("Controller Temp");
    // else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) Serial.print("DC Voltage");
    // else if (regAddress == BAMOCAR_REG_DC_CURRENT) Serial.print("DC Current");
    // else Serial.print("UnkSESSION_TIMEn");
    // Serial.println(")");
}

// Non Fixed is not request
void process_ResponseBamocarMsg(twai_message_t* msg, BAMOCar* bamocar) {

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

    // === UNKSESSION_TIMEN REGISTER ===
    else {
      Serial.print("[CAN] UnkSESSION_TIMEn register 0x");
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
  else Serial.print("UnkSESSION_TIMEn");
  
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
