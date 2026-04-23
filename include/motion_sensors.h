
#include <cstdint>

typedef struct{
  // GPS
  double gps_lat =0;
  double gps_lng =0;
  double gps_age =0;
  double gps_course =0;
  double gps_speed  =0;
  // IMU - linear acceleration (m/s^2, gravity removed)
  float imu_accelx =0;
  float imu_accely =0;
  float imu_accelz =0;
  // IMU - angular velocity (deg/s)
  float imu_gyrox  =0;
  float imu_gyroy  =0;
  float imu_gyroz  =0;
  // IMU - Euler orientation (deg)
  float imu_euler_roll  =0;
  float imu_euler_pitch =0;
  float imu_euler_yaw   =0;
  // IMU - magnetometer (uT)
  float imu_magx =0;
  float imu_magy =0;
  float imu_magz =0;
  // IMU - gravity vector (m/s^2)
  float imu_gravx =0;
  float imu_gravy =0;
  float imu_gravz =0;
} Odometry;

// --- GPS (TinyGPS++ ANsH51 GNSS) ---

bool GPSinit(HardwareSerial &gpsSerial,int tx, int rx,uint32_t baud);
void GPSupdate(Odometry *mygps, HardwareSerial &gpsSerial, TinyGPSPlus &gps,bool &AvailableFlag);

// --- IMU (BNO055) ---
// Old MPU6050 signatures kept for reference:
// bool IMUinit([[maybe_unused]]TwoWire* WireIMU, MPU6050 &mpu);
// void IMUcalibrate(MPU6050 &mpu, bool &AvailableFlag);
// void IMUupdate(Odometry* myimu, MPU6050 &mpu, bool &AvailableFlag);

bool IMUinit([[maybe_unused]]TwoWire* WireIMU, Adafruit_BNO055 &bno);
void IMUcalibrate(Adafruit_BNO055 &bno, bool &AvailableFlag);
void IMUupdate(Odometry* myimu, Adafruit_BNO055 &bno, bool &AvailableFlag);


void mockOdometryData(Odometry *OdomSensors);

// Teleplot debug function
void teleplotOdometry(Odometry *OdomSensors);