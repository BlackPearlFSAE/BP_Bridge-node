#include <cstdint>
// class hw_timer_t;
typedef struct{
  // Analog
  float Wheel_RPM_L =0.0;
  float Wheel_RPM_R =0.0;  
  float STR_Heave_mm =0.0; // in mm scale
  float STR_Roll_mm =0.0;
} Mechanical;

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

// -- ADC config
// May add macro for esp32 exclusive later
const float aref = 3.3; 
const int pwmres = 4095; // 12 bit ADC resolution
const float max_distance1 = 75.00; // recalibrated with vernier -> Needs to recheck // 52.91
const float max_distance2 = 75.00; // Not_sure needs to check again

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

void StrokesensorInit(int Heave,int Roll);
void StrokesensorUpdate(Mechanical *MechSensors,int Heave,int Roll);


void RPMinit_withExtPullUP(int EncoderPIN_L, int EncoderPIN_R);
void RPM_setCalcPeriod(hw_timer_t* My_timer, int period);
void RPMsensorUpdate(Mechanical *MechSensors,int Period, int encoder_res);


// --- ELECTRICAL SENSOR FUNCTIONS ---
void ElectSensorsInit(int* pinArrays);
void ElectSensorsUpdate(Electrical *ElectSensors,int* pinArrays);

void mockMechanicalData(Mechanical *MechSensors);
void mockElectricalData(Electrical *ElectSensors);

// Teleplot debug functions
void teleplotMechanical(Mechanical *MechSensors);
void teleplotElectrical(Electrical *ElectSensors);