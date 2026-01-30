#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <MPU6050_light.h>
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

bool IMUinit([[maybe_unused]]TwoWire* WireIMU, MPU6050 &mpu){
  
  byte status = mpu.begin();
  Serial.printf("MPU6050 status: %c\n", status);
  
  switch (status) {
    case 0:
      Serial.println(F("Status 0: Success! MPU6050 is connected and configured."));    
      return true;
    case 1:
      Serial.println(F("Status 1: Data too long to fit in transmit buffer or Invalid Config."));
      break;
    case 2:
      Serial.println(F("Status 2: Received NACK on transmit of address. (Check wiring/I2C address)"));
      break;
    case 3:
      Serial.println(F("Status 3: Received NACK on transmit of data."));
      break;
    case 4:
      Serial.println(F("Status 4: Unknown I2C error."));
      break;
    default:
      Serial.print(F("Status "));
      Serial.print(status);
      Serial.println(F(": Unexpected return value."));
      break;
  }
  return false;
}

void IMUcalibrate(MPU6050 &mpu, bool &AvailableFlag){
  if(!AvailableFlag){
    Serial.println("IMU isn't available: Can't Calibrate");
    return;
  }
  Serial.println("Calculating offsets, do not move MPU6050");
  // delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}
void IMUupdate(Odometry* myimu, MPU6050 &mpu, bool &AvailableFlag){
  // --- IMU update ---
  if(!AvailableFlag) {
    Serial.println("IMU isn't available: Can't read");
    return;
  } 
  mpu.update();
  myimu->imu_accelx = mpu.getAccX();
  myimu->imu_accely = mpu.getAccY();
  myimu->imu_accelz = mpu.getAccZ();
  myimu->imu_gyrox = mpu.getGyroX();
  myimu->imu_gyroy = mpu.getGyroY();
  myimu->imu_gyroz = mpu.getGyroZ();
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
}

// --- TELEPLOT DEBUG FUNCTION ---
void teleplotOdometry(Odometry *OdomSensors) {
  Serial.printf(">GPS_Lat:%.6f\n", OdomSensors->gps_lat);
  Serial.printf(">GPS_Lng:%.6f\n", OdomSensors->gps_lng);
  Serial.printf(">GPS_Speed:%.2f\n", OdomSensors->gps_speed);
  Serial.printf(">GPS_Course:%.2f\n", OdomSensors->gps_course);
  Serial.printf(">IMU_AccelX:%.3f\n", OdomSensors->imu_accelx);
  Serial.printf(">IMU_AccelY:%.3f\n", OdomSensors->imu_accely);
  Serial.printf(">IMU_AccelZ:%.3f\n", OdomSensors->imu_accelz);
  Serial.printf(">IMU_GyroX:%.3f\n", OdomSensors->imu_gyrox);
  Serial.printf(">IMU_GyroY:%.3f\n", OdomSensors->imu_gyroy);
  Serial.printf(">IMU_GyroZ:%.3f\n", OdomSensors->imu_gyroz);
}
