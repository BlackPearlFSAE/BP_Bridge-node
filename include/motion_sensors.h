
#include <cstdint>

typedef struct{
  // GPS
  double gps_lat =0;
  double gps_lng =0;
  double gps_age =0;
  double gps_course =0;
  double gps_speed  =0;
  // IMU
  float imu_accelx =0;
  float imu_accely =0;
  float imu_accelz =0;
  float imu_gyrox  =0;
  float imu_gyroy  =0;
  float imu_gyroz  =0;
} Odometry;

// --- GPS (TinyGPS++ ANsH51 GNSS) ---

bool GPSinit(HardwareSerial &gpsSerial,int tx, int rx,uint32_t baud);
void GPSupdate(Odometry *mygps, HardwareSerial &gpsSerial, TinyGPSPlus &gps,bool &AvailableFlag);

// --- IMU (MPU6050) ---

bool IMUinit([[maybe_unused]]TwoWire* WireIMU, MPU6050 &mpu);
void IMUcalibrate(MPU6050 &mpu, bool &AvailableFlag);
void IMUupdate(Odometry* myimu, MPU6050 &mpu, bool &AvailableFlag);


void mockOdometryData(Odometry *OdomSensors);

// Teleplot debug function
void teleplotOdometry(Odometry *OdomSensors);