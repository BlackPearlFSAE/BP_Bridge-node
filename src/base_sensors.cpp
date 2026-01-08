#include <Arduino.h>
#include <base_sensors.h>

// ISR shared variable
volatile int counterL = 0;
volatile int counterR = 0;
volatile bool startCalculate = 0;

void IRAM_ATTR ISR_COUNT_L() {
  counterL++;
}
void IRAM_ATTR ISR_COUNT_R() {
  counterR++;
}
void IRAM_ATTR ISRreset() {
  startCalculate = true;
}

void RPMinit_withExtPullUP(int EncoderPIN_L, int EncoderPIN_R){
  pinMode(EncoderPIN_L, INPUT);
  pinMode(EncoderPIN_R, INPUT);
  attachInterrupt(EncoderPIN_L, ISR_COUNT_L, FALLING);
  attachInterrupt(EncoderPIN_R, ISR_COUNT_R, FALLING);
}

void RPM_setCalcPeriod(hw_timer_t* My_timer, int period) {
  // Timer interrupt 100 ms (May change to input capture or just a trigger)
  int refresh_calculation_time = period * 1000;
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, ISRreset, true);
  timerAlarmWrite(My_timer, refresh_calculation_time, true);
  timerAlarmEnable(My_timer);
}

void RPMsensorUpdate(Mechanical *MechSensors, int Period,int encoder_res){
  // ----- RPM sensors (Wheel speeds)
  if (startCalculate) {
    noInterrupts();
    MechSensors->Wheel_RPM_L = (30000) / Period * ((float)counterL / encoder_res);
    MechSensors->Wheel_RPM_R = (30000) / Period * ((float)counterR / encoder_res);
    counterL = 0;
    counterR = 0;
    startCalculate = false;
    interrupts();
  }
}

void StrokesensorInit(int Heave,int Roll){
  // The Roll is pin 5 or 6
  pinMode(Heave,INPUT_PULLDOWN);
  pinMode(Roll,INPUT_PULLDOWN);
}
void StrokesensorUpdate(Mechanical *MechSensors,int Heave,int Roll){
  // ----- Stroke distances
  // Conversion from ADC value back to its sensor reading voltage
  // Conversion of sensor reading voltage KPM18-50mm distance, total distance is 52.91 mm
  MechSensors->STR_Heave_mm = analogRead(Heave) * (aref / pwmres);
  MechSensors->STR_Roll_mm = analogRead(Roll) * (aref / pwmres);
}


void ElectSensorsInit(int* pinArrays){
  // เด๋วใช้ระบบ function call appender เหมือนกัน เน้นเอา param เดียวปะ
  pinMode(pinArrays[0],INPUT_PULLDOWN);
  pinMode(pinArrays[1],INPUT_PULLDOWN);
  pinMode(pinArrays[2],INPUT_PULLDOWN);
  pinMode(pinArrays[3],INPUT_PULLDOWN);

  pinMode(pinArrays[4],INPUT_PULLDOWN);
  pinMode(pinArrays[5],INPUT_PULLDOWN);
  pinMode(pinArrays[6],INPUT_PULLDOWN);
  pinMode(pinArrays[7],INPUT_PULLDOWN);
  return;
}

void ElectSensorsUpdate(Electrical *ElectSensors, int* pinArrays){
  // Read raw ADC values
  uint16_t raw_i_sense = analogRead(pinArrays[0]);
  uint16_t raw_tmp = analogRead(pinArrays[1]);
  uint16_t raw_apps = analogRead(pinArrays[2]);
  uint16_t raw_bpps = analogRead(pinArrays[3]);

  // Convert ADC to voltage (0-3.3V mapped to 0-4095)
  float volt_i_sense = (raw_i_sense / (float)pwmres) * aref;
  float volt_tmp = (raw_tmp / (float)pwmres) * aref;
  float volt_apps = (raw_apps / (float)pwmres) * aref;
  float volt_bpps = (raw_bpps / (float)pwmres) * aref;

  // --- Hall Effect Current Sensor ---
  // Formula: I = (V_sensor - V_offset) / Sensitivity
  ElectSensors->I_SENSE = (volt_i_sense - i_sense_offset) / i_sense_sensitivity;

  // --- Temperature Sensor (NTC Thermistor in voltage divider) ---
  // Calculate NTC resistance from voltage divider
  // Vout = Vcc * (R_NTC / (R_series + R_NTC))
  // Solving for R_NTC: R_NTC = (Vout * R_series) / (Vcc - Vout)
  if (volt_tmp < aref - 0.01) {  // Avoid division by zero
    uint16_t ntc_resistance = (volt_tmp * tmp_series_res) / (aref - volt_tmp);
    ElectSensors->TMP = (float)ntc_resistance; // then we do look up Table for later
  } else {
    ElectSensors->TMP = 0.0;  // For iInvalid reading
  }

  // --- APPS (Accelerator Position) - Linear potentiometer ---
  // Voltage maps linearly to distance (0V = 0mm, 3.3V = 75mm)
  ElectSensors->APPS = (volt_apps / aref) * apps_max_dist_mm + apps_offset_mm;

  // --- BPPS (Brake Position) - Linear potentiometer ---
  // Voltage maps linearly to distance (0V = 0mm, 3.3V = 75mm)
  ElectSensors->BPPS = (volt_bpps / aref) * bpps_max_dist_mm + bpps_offset_mm;

  // --- Digital Fault Status Signals ---
  ElectSensors->AMS_OK = digitalRead(pinArrays[4]);
  ElectSensors->IMD_OK = digitalRead(pinArrays[5]);
  ElectSensors->HV_ON = digitalRead(pinArrays[6]);
  ElectSensors->BSPD_OK = digitalRead(pinArrays[7]);
}


// --- MECHANICAL SENSORS MOCK DATA ---
void mockMechanicalData(Mechanical *MechSensors) {
  // Simulate wheel RPM (0-500 RPM range with some variation)
  MechSensors->Wheel_RPM_L = 250.0 + random(-50, 50);
  MechSensors->Wheel_RPM_R = 245.0 + random(-50, 50);

  // Simulate stroke sensors (0-75mm range)
  MechSensors->STR_Heave_mm = 35.0 + random(-10, 10);
  MechSensors->STR_Roll_mm = 40.0 + random(-10, 10);
}

// --- ELECTRICAL SENSORS MOCK DATA ---
void mockElectricalData(Electrical *ElectSensors) {
  // Simulate current sensor (-20A to +20A range)
  ElectSensors->I_SENSE = 5.0 + random(-20, 80) / 10.0;

  // Simulate temperature (20-80°C range)
  ElectSensors->TMP = 45.0 + random(-10, 20);

  // Simulate APPS and BPPS positions (0-75mm range)
  ElectSensors->APPS = 30.0 + random(-15, 30);
  ElectSensors->BPPS = 10.0 + random(-5, 15);

  // Simulate digital fault status (mostly OK, occasional faults)
  ElectSensors->AMS_OK = random(0, 10) > 1;    // 90% OK
  ElectSensors->IMD_OK = random(0, 10) > 1;    // 90% OK
  ElectSensors->HV_ON = random(0, 10) > 2;     // 80% ON
  ElectSensors->BSPD_OK = random(0, 10) > 1;   // 90% OK
}
