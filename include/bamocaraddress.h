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