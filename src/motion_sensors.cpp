#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
// #include <MPU6050_light.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <motion_sensors.h>

bool GPSinit(HardwareSerial &gpsSerial,int tx,int rx, uint32_t baud) {
  gpsSerial.begin(baud, SERIAL_8N1, rx, tx);
  if(!gpsSerial) return false;
  else { 
    Serial.println("AN251 GPS Module with TinyGPS++");
    Serial.println("Waiting for GPS fix...");
    return true;
  }
}

void GPSupdate(Odometry *mygps, HardwareSerial &gpsSerial, TinyGPSPlus &gps,bool &AvailableFlag) {
  if(!AvailableFlag) {
    Serial.println("GPS isn't available: Can't read");
    return;
  }
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Serial.println(F("\n===== GPS Lat lng ====="));
      if (gps.location.isValid()) {
        mygps->gps_lat= gps.location.lat();
        mygps->gps_lng= gps.location.lng();
        mygps->gps_age= gps.location.age();
        mygps->gps_course= gps.course.deg(); //deg
        mygps->gps_speed= gps.speed.mps(); // mps
      } else {
        Serial.println(F("INVALID"));
      }
    }
  }  
}

// --- Old MPU6050 implementation (kept for reference) ---
// bool IMUinit([[maybe_unused]]TwoWire* WireIMU, MPU6050 &mpu){
//
//   byte status = mpu.begin();
//   Serial.printf("MPU6050 status: %c\n", status);
//
//   switch (status) {
//     case 0:
//       Serial.println(F("Status 0: Success! MPU6050 is connected and configured."));
//       return true;
//     case 1:
//       Serial.println(F("Status 1: Data too long to fit in transmit buffer or Invalid Config."));
//       break;
//     case 2:
//       Serial.println(F("Status 2: Received NACK on transmit of address. (Check wiring/I2C address)"));
//       break;
//     case 3:
//       Serial.println(F("Status 3: Received NACK on transmit of data."));
//       break;
//     case 4:
//       Serial.println(F("Status 4: Unknown I2C error."));
//       break;
//     default:
//       Serial.print(F("Status "));
//       Serial.print(status);
//       Serial.println(F(": Unexpected return value."));
//       break;
//   }
//   return false;
// }
//
// void IMUcalibrate(MPU6050 &mpu, bool &AvailableFlag){
//   if(!AvailableFlag){
//     Serial.println("IMU isn't available: Can't Calibrate");
//     return;
//   }
//   Serial.println("Calculating offsets, do not move MPU6050");
//   // delay(1000);
//   mpu.calcOffsets(true,true); // gyro and accelero
//   Serial.println("Done!\n");
// }
// void IMUupdate(Odometry* myimu, MPU6050 &mpu, bool &AvailableFlag){
//   // --- IMU update ---
//   if(!AvailableFlag) {
//     Serial.println("IMU isn't available: Can't read");
//     return;
//   }
//   mpu.update();
//   myimu->imu_accelx = mpu.getAccX();
//   myimu->imu_accely = mpu.getAccY();
//   myimu->imu_accelz = mpu.getAccZ();
//   myimu->imu_gyrox = mpu.getGyroX();
//   myimu->imu_gyroy = mpu.getGyroY();
//   myimu->imu_gyroz = mpu.getGyroZ();
// }

// --- IMU (BNO055) ---
bool IMUinit([[maybe_unused]]TwoWire* WireIMU, Adafruit_BNO055 &bno){

  if (!bno.begin()) {
    Serial.println(F("BNO055 not detected. Check wiring or I2C ADDR (0x28/0x29)."));
    return false;
  }
  Serial.println(F("BNO055 connected and configured."));
  delay(1000);
  return true;
}

void IMUcalibrate(Adafruit_BNO055 &bno, bool &AvailableFlag){
  if(!AvailableFlag){
    Serial.println("IMU isn't available: Can't Calibrate");
    return;
  }
  Serial.println("Reading BNO055 calibration status, move sensor through its ranges");
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.printf("Calibration: Sys=%u Gyro=%u Accel=%u Mag=%u\n", system, gyro, accel, mag);
  Serial.println("Done!\n");
}
void IMUupdate(Odometry* myimu, Adafruit_BNO055 &bno, bool &AvailableFlag){
  // --- IMU update ---
  if(!AvailableFlag) {
    Serial.println("IMU isn't available: Can't read");
    return;
  }
  sensors_event_t linearAccelData, angVelocityData, orientationData, magData, gravityData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&magData,         Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&gravityData,     Adafruit_BNO055::VECTOR_GRAVITY);
  myimu->imu_accelx = linearAccelData.acceleration.x;
  myimu->imu_accely = linearAccelData.acceleration.y;
  myimu->imu_accelz = linearAccelData.acceleration.z;
  myimu->imu_gyrox = angVelocityData.gyro.x;
  myimu->imu_gyroy = angVelocityData.gyro.y;
  myimu->imu_gyroz = angVelocityData.gyro.z;
  // Euler: BNO055 reports x=heading(yaw), y=roll, z=pitch
  myimu->imu_euler_yaw   = orientationData.orientation.x;
  myimu->imu_euler_roll  = orientationData.orientation.y;
  myimu->imu_euler_pitch = orientationData.orientation.z;
  myimu->imu_magx = magData.magnetic.x;
  myimu->imu_magy = magData.magnetic.y;
  myimu->imu_magz = magData.magnetic.z;
  myimu->imu_gravx = gravityData.acceleration.x;
  myimu->imu_gravy = gravityData.acceleration.y;
  myimu->imu_gravz = gravityData.acceleration.z;
}

// --- ODOMETRY SENSORS MOCK DATA ---
void mockOdometryData(Odometry *OdomSensors) {
  // GPS Mock Data (Bangkok area coordinates)
  OdomSensors->gps_lat = 13.7563 + random(-100, 100) / 10000.0;
  OdomSensors->gps_lng = 100.5018 + random(-100, 100) / 10000.0;
  OdomSensors->gps_age = random(0, 1000);
  OdomSensors->gps_course = random(0, 360);
  OdomSensors->gps_speed = random(0, 50);  // 0-50 m/s

  // IMU Mock Data (accelerometer in m/s^2)
  OdomSensors->imu_accelx = random(-100, 100) / 50.0;
  OdomSensors->imu_accely = random(-100, 100) / 50.0;
  OdomSensors->imu_accelz = 9.81 + random(-50, 50) / 100.0;

  // IMU Mock Data (gyroscope in deg/s)
  OdomSensors->imu_gyrox = random(-50, 50) / 10.0;
  OdomSensors->imu_gyroy = random(-50, 50) / 10.0;
  OdomSensors->imu_gyroz = random(-50, 50) / 10.0;

  // IMU Mock Data (Euler in deg)
  OdomSensors->imu_euler_roll  = random(-1800, 1800) / 10.0;
  OdomSensors->imu_euler_pitch = random(-900, 900) / 10.0;
  OdomSensors->imu_euler_yaw   = random(0, 3600) / 10.0;

  // IMU Mock Data (magnetometer in uT)
  OdomSensors->imu_magx = random(-500, 500) / 10.0;
  OdomSensors->imu_magy = random(-500, 500) / 10.0;
  OdomSensors->imu_magz = random(-500, 500) / 10.0;

  // IMU Mock Data (gravity in m/s^2)
  OdomSensors->imu_gravx = random(-100, 100) / 100.0;
  OdomSensors->imu_gravy = random(-100, 100) / 100.0;
  OdomSensors->imu_gravz = 9.81 + random(-50, 50) / 100.0;
}

// --- TELEPLOT DEBUG FUNCTION ---
void teleplotOdometry(Odometry *OdomSensors) {
  Serial.printf(">GPS_Lat:%.4f\n", OdomSensors->gps_lat);
  Serial.printf(">GPS_Lng:%.4f\n", OdomSensors->gps_lng);
  Serial.printf(">GPS_Speed:%.2f\n", OdomSensors->gps_speed);
  Serial.printf(">GPS_Course:%.2f\n", OdomSensors->gps_course);
  Serial.printf(">IMU_AccelX:%.2f\n", OdomSensors->imu_accelx);
  Serial.printf(">IMU_AccelY:%.2f\n", OdomSensors->imu_accely);
  Serial.printf(">IMU_AccelZ:%.2f\n", OdomSensors->imu_accelz);
  Serial.printf(">IMU_GyroX:%.2f\n", OdomSensors->imu_gyrox);
  Serial.printf(">IMU_GyroY:%.2f\n", OdomSensors->imu_gyroy);
  Serial.printf(">IMU_GyroZ:%.2f\n", OdomSensors->imu_gyroz);
  Serial.printf(">IMU_EulerRoll:%.2f\n",  OdomSensors->imu_euler_roll);
  Serial.printf(">IMU_EulerPitch:%.2f\n", OdomSensors->imu_euler_pitch);
  Serial.printf(">IMU_EulerYaw:%.2f\n",   OdomSensors->imu_euler_yaw);
  Serial.printf(">IMU_MagX:%.2f\n", OdomSensors->imu_magx);
  Serial.printf(">IMU_MagY:%.2f\n", OdomSensors->imu_magy);
  Serial.printf(">IMU_MagZ:%.2f\n", OdomSensors->imu_magz);
  Serial.printf(">IMU_GravX:%.2f\n", OdomSensors->imu_gravx);
  Serial.printf(">IMU_GravY:%.2f\n", OdomSensors->imu_gravy);
  Serial.printf(">IMU_GravZ:%.2f\n", OdomSensors->imu_gravz);
}
