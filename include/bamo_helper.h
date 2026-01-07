#include <cstdint>


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
float convertNTCtoTemp(uint16_t resistance);
float convertKTYtoTemp(uint16_t resistance);
void decodeBAMO_rawmsg(twai_message_t &message);
void scanBamocarIDs();
void analyzeBamoData(uint8_t reg, uint8_t* data);