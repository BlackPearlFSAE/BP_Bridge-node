#include "Arduino.h"
#include <driver/twai.h>
#include <CAN32_util.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include "shared_config.h"
#include "helper.h"

/************************* Mechanical ***************************/

void StrokesensorInit(int Heave, int Roll) {
  pinMode(Heave, INPUT_PULLDOWN);
  pinMode(Roll,  INPUT_PULLDOWN);
}

void StrokesensorUpdate(Mechanical* m, int Heave, int Roll) {
  analogRead(Heave);
  m->STR_Heave_mm = (float)(analogRead(Heave) * (max_distance1 / pwmres));
  analogRead(Roll);
  m->STR_Roll_mm  = (float)(analogRead(Roll)  * (max_distance2 / pwmres));
}

void mockMechanicalData(Mechanical* m) {
  m->Wheel_RPM_L  = 250.0 + random(-50, 50);
  m->Wheel_RPM_R  = 245.0 + random(-50, 50);
  m->STR_Heave_mm = 35.0 + random(-10, 10);
  m->STR_Roll_mm  = 40.0 + random(-10, 10);
}

void teleplotMechanical(Mechanical* m) {
  Serial.printf(">Wheel_RPM_L:%.2f\n", m->Wheel_RPM_L);
  Serial.printf(">Wheel_RPM_R:%.2f\n", m->Wheel_RPM_R);
  Serial.printf(">STR_Heave_mm:%.2f\n", m->STR_Heave_mm);
  Serial.printf(">STR_Roll_mm:%.2f\n", m->STR_Roll_mm);
}

/************************* Electrical ***************************/

void ElectSensorsInit(int* pins) {
  for (int i = 0; i < 9; i++) pinMode(pins[i], INPUT_PULLDOWN);
}

void ElectSensorsUpdate(Electrical* e, int* pins) {
  uint16_t raw_i_sense  = analogRead(pins[0]);
  uint16_t raw_tmp      = analogRead(pins[1]);
  uint16_t raw_apps     = analogRead(pins[2]);
  uint16_t raw_bpps     = analogRead(pins[3]);
  uint16_t raw_steering = analogRead(pins[8]);

  float volt_i_sense  = (raw_i_sense  / (float)pwmres) * aref;
  float volt_tmp      = (raw_tmp      / (float)pwmres) * aref;
  float volt_bpps     = (raw_bpps     / (float)pwmres) * aref;
  float volt_steering = (raw_steering / (float)pwmres) * aref;

  e->I_SENSE = (volt_i_sense - i_sense_offset) / i_sense_sensitivity;

  if (volt_tmp < aref - 0.01) {
    uint16_t ntc_resistance = (volt_tmp * tmp_series_res) / (aref - volt_tmp);
    e->TMP = (float)ntc_resistance;
  } else {
    e->TMP = 0.0;
  }

  e->APPS = raw_apps;
  e->BPPS = (volt_bpps - bpps_offset_v) / max_volt5;
  e->steering = ((volt_steering / steering_aref) * steering_max__angle) - steering_offset_angle;

  e->AMS_OK  = digitalRead(pins[4]);
  e->IMD_OK  = digitalRead(pins[5]);
  e->HV_ON   = digitalRead(pins[6]);
  e->BSPD_OK = digitalRead(pins[7]);
}

void mockElectricalData(Electrical* e) {
  e->I_SENSE = 5.0 + random(-20, 80) / 10.0;
  e->TMP     = 45.0 + random(-10, 20);
  e->APPS    = 30.0 + random(-15, 30);
  e->BPPS    = 10.0 + random(-5, 15);
  e->AMS_OK  = random(0, 10) > 1;
  e->IMD_OK  = random(0, 10) > 1;
  e->HV_ON   = random(0, 10) > 2;
  e->BSPD_OK = random(0, 10) > 1;
  e->steering = random(0, 10) > 1;
}

void teleplotElectrical(Electrical* e) {
  Serial.printf(">I_SENSE:%.2f\n", e->I_SENSE);
  Serial.printf(">TMP:%.2f\n", e->TMP);
  Serial.printf(">APPS:%.2f\n", e->APPS);
  Serial.printf(">BPPS:%.2f\n", e->BPPS);
  Serial.printf(">AMS_OK:%d\n", e->AMS_OK ? 1 : 0);
  Serial.printf(">IMD_OK:%d\n", e->IMD_OK ? 1 : 0);
  Serial.printf(">HV_ON:%d\n", e->HV_ON ? 1 : 0);
  Serial.printf(">BSPD_OK:%d\n", e->BSPD_OK ? 1 : 0);
  Serial.printf(">STEERING:%.2f\n", e->steering);
}

/************************* Odometry ***************************/

bool GPSinit(HardwareSerial& gpsSerial, int tx, int rx, uint32_t baud) {
  gpsSerial.begin(baud, SERIAL_8N1, rx, tx);
  if (!gpsSerial) return false;
  Serial.println("AN251 GPS Module with TinyGPS++");
  Serial.println("Waiting for GPS fix...");
  return true;
}

void GPSupdate(Odometry* o, HardwareSerial& gpsSerial, TinyGPSPlus& gps, bool& available) {
  if (!available) return;
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        o->gps_lat    = gps.location.lat();
        o->gps_lng    = gps.location.lng();
        o->gps_age    = gps.location.age();
        o->gps_course = gps.course.deg();
        o->gps_speed  = gps.speed.mps();
      }
    }
  }
}

bool IMUinit([[maybe_unused]] TwoWire* WireIMU, Adafruit_BNO055& bno) {
  if (!bno.begin()) {
    Serial.println(F("BNO055 not detected. Check wiring or I2C ADDR (0x28/0x29)."));
    return false;
  }
  Serial.println(F("BNO055 connected and configured."));
  delay(1000);
  return true;
}

void IMUcalibrate(Adafruit_BNO055& bno, bool& available) {
  if (!available) return;
  Serial.println("Reading BNO055 calibration status, move sensor through its ranges");
  uint8_t system_s, gyro_s, accel_s, mag_s = 0;
  bno.getCalibration(&system_s, &gyro_s, &accel_s, &mag_s);
  Serial.printf("Calibration: Sys=%u Gyro=%u Accel=%u Mag=%u\n", system_s, gyro_s, accel_s, mag_s);
}

void IMUupdate(Odometry* o, Adafruit_BNO055& bno, bool& available) {
  if (!available) return;
  sensors_event_t linA, ang, ori, mag, grav;
  bno.getEvent(&linA, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&ang,  Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&ori,  Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&mag,  Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&grav, Adafruit_BNO055::VECTOR_GRAVITY);
  o->imu_accelx = linA.acceleration.x;
  o->imu_accely = linA.acceleration.y;
  o->imu_accelz = linA.acceleration.z;
  o->imu_gyrox  = ang.gyro.x;
  o->imu_gyroy  = ang.gyro.y;
  o->imu_gyroz  = ang.gyro.z;
  // BNO055 NED/left-hand convention: negate all to match right-hand rule.
  // Register order: x=heading(yaw), y=BNO-roll(->vehicle pitch), z=BNO-pitch(->vehicle roll)
  o->imu_euler_yaw   = -ori.orientation.x;
  o->imu_euler_pitch = -ori.orientation.y;
  o->imu_euler_roll  = -ori.orientation.z;
  o->imu_magx = mag.magnetic.x;
  o->imu_magy = mag.magnetic.y;
  o->imu_magz = mag.magnetic.z;
  o->imu_gravx = grav.acceleration.x;
  o->imu_gravy = grav.acceleration.y;
  o->imu_gravz = grav.acceleration.z;
}

void mockOdometryData(Odometry* o) {
  o->gps_lat = 13.7563 + random(-100, 100) / 10000.0;
  o->gps_lng = 100.5018 + random(-100, 100) / 10000.0;
  o->gps_age = random(0, 1000);
  o->gps_course = random(0, 360);
  o->gps_speed = random(0, 50);
  o->imu_accelx = random(-100, 100) / 50.0;
  o->imu_accely = random(-100, 100) / 50.0;
  o->imu_accelz = 9.81 + random(-50, 50) / 100.0;
  o->imu_gyrox = random(-50, 50) / 10.0;
  o->imu_gyroy = random(-50, 50) / 10.0;
  o->imu_gyroz = random(-50, 50) / 10.0;
  o->imu_euler_roll  = random(-1800, 1800) / 10.0;
  o->imu_euler_pitch = random(-900, 900) / 10.0;
  o->imu_euler_yaw   = random(0, 3600) / 10.0;
  o->imu_magx = random(-500, 500) / 10.0;
  o->imu_magy = random(-500, 500) / 10.0;
  o->imu_magz = random(-500, 500) / 10.0;
  o->imu_gravx = random(-100, 100) / 100.0;
  o->imu_gravy = random(-100, 100) / 100.0;
  o->imu_gravz = 9.81 + random(-50, 50) / 100.0;
}

void teleplotMotion(Odometry* o) {
  Serial.printf(">GPS_Lat:%.4f\n", o->gps_lat);
  Serial.printf(">GPS_Lng:%.4f\n", o->gps_lng);
  Serial.printf(">GPS_Speed:%.2f\n", o->gps_speed);
  Serial.printf(">GPS_Course:%.2f\n", o->gps_course);
  Serial.printf(">IMU_AccelX:%.2f\n", o->imu_accelx);
  Serial.printf(">IMU_AccelY:%.2f\n", o->imu_accely);
  Serial.printf(">IMU_AccelZ:%.2f\n", o->imu_accelz);
  Serial.printf(">IMU_GyroX:%.2f\n", o->imu_gyrox);
  Serial.printf(">IMU_GyroY:%.2f\n", o->imu_gyroy);
  Serial.printf(">IMU_GyroZ:%.2f\n", o->imu_gyroz);
  Serial.printf(">IMU_EulerRoll:%.2f\n",  o->imu_euler_roll);
  Serial.printf(">IMU_EulerPitch:%.2f\n", o->imu_euler_pitch);
  Serial.printf(">IMU_EulerYaw:%.2f\n",   o->imu_euler_yaw);
  Serial.printf(">IMU_MagX:%.2f\n", o->imu_magx);
  Serial.printf(">IMU_MagY:%.2f\n", o->imu_magy);
  Serial.printf(">IMU_MagZ:%.2f\n", o->imu_magz);
  Serial.printf(">IMU_GravX:%.2f\n", o->imu_gravx);
  Serial.printf(">IMU_GravY:%.2f\n", o->imu_gravy);
  Serial.printf(">IMU_GravZ:%.2f\n", o->imu_gravz);
}

/************************* BAMOCar ***************************/

static float convertNTCtoTemp(uint16_t rawValue) {
  const int numPoints = 28;
  const int tempTable[numPoints] = {
    -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
    35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
  };
  const int valueTable[numPoints] = {
    16245, 16308, 16387, 16487, 16609, 16759, 16938, 17151, 17400, 17688, 18017, 18387, 18797, 19247,
    19733, 20250, 20793, 21357, 21933, 22515, 23097, 23671, 24232, 24775, 25296, 25792, 26261, 26702
  };
  if (rawValue <= valueTable[0]) return tempTable[0];
  if (rawValue >= valueTable[numPoints - 1]) return tempTable[numPoints - 1];
  for (int i = 0; i < numPoints - 1; i++) {
    if (rawValue >= valueTable[i] && rawValue <= valueTable[i + 1]) {
      float t1 = tempTable[i];
      float t2 = tempTable[i + 1];
      float v1 = valueTable[i];
      float v2 = valueTable[i + 1];
      return t1 + (rawValue - v1) * (t2 - t1) / (v2 - v1);
    }
  }
  return 25.0;
}

void pack_RequestBamocarMsg(twai_message_t* msg, uint8_t regAddress) {
  msg->identifier = BAMOCAR_REQUEST_ID;
  msg->extd = 0;
  msg->rtr = 0;
  msg->data_length_code = 3;
  msg->data[0] = 0x3D;
  msg->data[1] = regAddress;
  msg->data[2] = 0x00;
}

// One-shot request for the N-100% scaling parameter (reg 0xC8).
// Call once after CAN init to verify controller scaling matches BAMOCAR_N100_RPM.
// Response arrives on 0x181 with data[0]=0xC8 and is logged by process_ResponseBamocarMsg.
void request_BamocarN100(twai_message_t* msg) {
  pack_RequestBamocarMsg(msg, BAMOCAR_REG_N100);
}

void process_ResponseBamocarMsg(twai_message_t* msg, BAMOCar* bamocar) {
  if (msg->identifier == 0x181 && msg->data_length_code >= 3) {
    uint8_t regAddress = msg->data[0];
    uint16_t rawValue = (msg->data[2] << 8) | msg->data[1];

    if (regAddress == BAMOCAR_REG_MOTOR_TEMP) {
      bamocar->motorTemp1 = (float)msg->data[1] / 10.0;
      bamocar->motorTemp2 = bamocar->motorTemp1 + 8.0;
      bamocar->motorTempValid = true;
      Serial.print("[CAN DECODED] Motor Temp: ");
      Serial.print(bamocar->motorTemp2, 1);
      Serial.print(" °C (raw byte: 0x");
      Serial.print(msg->data[1], HEX);
      Serial.print(" = ");
      Serial.print(msg->data[1]);
      Serial.println(")");
    }
    else if (regAddress == BAMOCAR_REG_CONTROLLER_TEMP) {
      bamocar->controllerTemp = convertNTCtoTemp(rawValue);
      bamocar->controllerTempValid = true;
      Serial.print("[CAN DECODED] Controller Temp: ");
      Serial.print(bamocar->controllerTemp, 1);
      Serial.print(" °C (resistance: ");
      Serial.print(rawValue);
      Serial.println(" Ω)");
    }
    else if (regAddress == BAMOCAR_REG_DC_VOLTAGE) {
      bamocar->canVoltage = (float)rawValue / 55.1204;
      bamocar->canVoltageValid = true;
      Serial.print("[CAN DECODED] DC Voltage: ");
      Serial.print(bamocar->canVoltage, 2);
      Serial.print(" V (raw: ");
      Serial.print(rawValue);
      Serial.println(")");
    }
    else if (regAddress == BAMOCAR_REG_DC_CURRENT) {
      if (rawValue == 0xFFFF || rawValue > 60000) {
        Serial.println("[CAN DECODED] DC Current: INVALID (0xFFFF) - ignored");
      } else {
        bamocar->canCurrent = (float)rawValue * 0.373832;
        bamocar->canCurrentValid = true;
        Serial.print("[CAN DECODED] DC Current: ");
        Serial.print(bamocar->canCurrent, 2);
        Serial.print(" A (raw: ");
        Serial.print(rawValue);
        Serial.println(")");
      }
    }
    else if (regAddress == BAMOCAR_REG_SPEED_ACTUAL) {
      // Signed: negative = reverse. Scale: RPM = Num / 32767 * N-100%
      // (was: 3000.0f / 32767.0f — wrong hardcoded value, replaced by BAMOCAR_N100_RPM)
      bamocar->rpm = (float)(int16_t)rawValue * (BAMOCAR_N100_RPM / 32767.0f);
      bamocar->rpmValid = true;
      Serial.print("[CAN DECODED] RPM: ");
      Serial.print(bamocar->rpm, 0);
      Serial.print(" RPM (raw: ");
      Serial.print((int16_t)rawValue);
      Serial.println(")");
    }
    else if (regAddress == BAMOCAR_REG_N100) {
      // N-100% readback: log the controller's actual scaling value for verification
      Serial.print("[CAN DECODED] N-100% (reg 0xC8): ");
      Serial.print(rawValue);
      Serial.print(" RPM (firmware constant BAMOCAR_N100_RPM=");
      Serial.print((int)BAMOCAR_N100_RPM);
      Serial.println(rawValue == (uint16_t)BAMOCAR_N100_RPM ? ") MATCH" : ") MISMATCH - check define");
    }
    else {
      Serial.print("[CAN] Unknown register 0x");
      Serial.print(regAddress, HEX);
      Serial.print(" with value: ");
      Serial.println(rawValue);
    }
  }
}

void mockBAMOCarData(BAMOCar* b) {
  b->motorTemp1 = 55.0 + random(-10, 15);
  b->motorTemp2 = b->motorTemp1 + random(5, 10);
  b->motorTempValid = true;
  b->controllerTemp = 50.0 + random(-10, 15);
  b->controllerTempValid = true;
  b->canVoltage = 220.0 + random(-20, 20);
  b->canVoltageValid = true;
  b->canCurrent = 30.0 + random(-10, 50);
  b->canCurrentValid = true;
  b->power = b->canVoltage * b->canCurrent;
  b->rpm = 2500.0 + random(-500, 500);
  b->rpmValid = true;
}

void teleplotBAMOCar(BAMOCar* b) {
  Serial.printf(">BAMO_Voltage:%.2f\n", b->canVoltage);
  Serial.printf(">BAMO_Current:%.2f\n", b->canCurrent);
  Serial.printf(">BAMO_Power:%.2f\n", b->power);
  Serial.printf(">BAMO_MotorTemp:%.1f\n", b->motorTemp2);
  Serial.printf(">BAMO_CtrlTemp:%.1f\n", b->controllerTemp);
  Serial.printf(">BAMO_RPM:%.0f\n", b->rpm);
}
