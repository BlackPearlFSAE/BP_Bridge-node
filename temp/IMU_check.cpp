/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */
#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>

#define I2C_SDA 42
#define I2C_SCL 41
// #define I2C_SDA 1
// #define I2C_SCL 2

TwoWire WireRTC;
TwoWire WireIMU;
MPU6050 mpu(WireIMU);

unsigned long timer = 0;


void IMUinitwrapper(){
  WireIMU.begin(I2C_SDA,I2C_SCL);
  byte status = mpu.begin();
  Serial.printf("MPU6050 status: %c", status);
}

void IMUcalibrate(){
  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}

void setup() {
  Serial.begin(115200);
  IMUinitwrapper();
  IMUcalibrate();
}

void loop() {
  mpu.update();
  
  // Data stuffing

  // Serial debugging
  if(millis() - timer > 100){ // print data every second
    Serial.printf("ACCEL X: %2f",mpu.getAccX());
    Serial.printf("ACCEL Y: %2f",mpu.getAccY());
    Serial.printf("ACCEL Z: %2f",mpu.getAccZ());
  
    Serial.printf("GYRO X: %2f",mpu.getGyroX());
    Serial.printf("GYRO Y: %2f",mpu.getGyroY());
    Serial.printf("GYRO Z: %2f",mpu.getGyroY());
  
    Serial.printf("ACC ANGLE X: %2f",mpu.getAccAngleX());
    Serial.printf("ACC ANGLE Y: %2f",mpu.getAccAngleY());
    
    // Not Euler angle
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }

}

