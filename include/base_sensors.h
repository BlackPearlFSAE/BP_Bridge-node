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
  float steering = 0.0;
} Electrical;

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