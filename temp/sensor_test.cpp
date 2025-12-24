/*
 * CAN Bus sending receiving boiler plate code
 * 
 * This sketch combines:
 * - There are sensor reading
 * - Stroke sensor reading (and offset calibration code)
 *    - Linear potentiometer used is KPM18-50mm , change potdist according to the stroke sensors used
 */

#include <Arduino.h>

/* --------------------Define , variable*/
// Camshaft position sensor Left, Right
#define ENCODER_PINL 41
#define ENCODER_PINR 42
#define ENCODER_N 50 // encoding resolution


// GPIO 5,6  ADC reading
#define stroke1_black 6
#define stroke2_green 5

hw_timer_t *My_timer = NULL;

/* ============== Stroke Sensors ================= */

// init variables
float distance_stroke1_black = 0.0;
float distance_stroke2_green = 0.0;
float potvolts1_black = 0.0;
float potvolts2_green = 0.0;

// constant 
const float aref = 3.3; 
const int pwmres = 4095; // 10 bit ADC resolution
// const float max_distance1 = 52.91; // recalibrated with vernier -> Needs to recheck
const float max_distance1 = 55.00; // recalibrated with vernier -> Needs to recheck
const float max_distance2 = 75.00; // Not_sure needs to check again

// Calibration offset 1 
float offset1 = 0.0;
// Calibration offset 2
float offset2 = 0.0;

/* ============== RPM Sensors ================= */

// RPM measurement Variables
float Wheel_RPM_left = 0.0;
float Wheel_RPM_right = 0.0;

unsigned long Period = 100; // 100 ms
// ISR shared variable
volatile int counterL = 0;
volatile int counterR = 0;
volatile bool startCalculate = 0;
// Add these debug variables at the top with other globals
volatile int lastCounterL = 0;
volatile int lastCounterR = 0;
volatile int resetCount = 0;
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

unsigned long lastTime = 0;

/*===================================*/
void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PINL, INPUT); // The module has internal Pull up Resistor
  pinMode(ENCODER_PINR, INPUT); // The module has internal Pull up Resistor
  
  // ------ interupt
  attachInterrupt(ENCODER_PINL,ISR_COUNT_L, FALLING);
  attachInterrupt(ENCODER_PINR,ISR_COUNT_R, FALLING);

  // Timer interrupt 100 ms
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &ISRreset, true);
  timerAlarmWrite(My_timer, Period*1000, true);  
  timerAlarmEnable(My_timer);

  
  // When Object blocks (Open Switch) The signal is HIGH , so RISING edge of pulse will fires ISR
  // pull up , so when change should be increment on falling edge
}

// unsigned long lasttime = 0;
void loop() {

  //  // Conversion from ADC value back to its sensor reading voltage
  // potvolts1_black = analogRead(stroke1_black) * (aref/pwmres);
  // potvolts2_green = analogRead(stroke2_green) * (aref/pwmres);

  // // Conversion of sensor reading voltage KPM18-50mm distance , total distance is 52.91 mm
  // distance_stroke1_black = potvolts1_black * (max_distance1/aref);
  // distance_stroke2_green = potvolts2_green * (max_distance2/aref);

  // if(millis()-lastTime >= 200){
  //   /* The equilibrium is about the center of stroke length*/
  //   Serial.print("Stroke_heave: "); Serial.println(distance_stroke1_black);
  //   Serial.print("Stroke_roll: ");  Serial.println(distance_stroke2_green);
  //   lastTime = millis();
  // }  
  if(startCalculate){
    noInterrupts();
    // Serial.println(counterL);
    // Serial.println(counterR);
    Wheel_RPM_left = (60000)/Period * ((float)counterL/ ENCODER_N);
    Wheel_RPM_right = (60000)/Period * ((float)counterR/ ENCODER_N);
    // Serial.print("RPM_L: "); Serial.println(Wheel_RPM_left);
    // Serial.print("RPM_R: "); Serial.println(Wheel_RPM_right);
    counterL = 0; counterR = 0; 
    startCalculate = false;
    interrupts();
  }
}

