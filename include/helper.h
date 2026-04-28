
#include <cstdint>
/************************* Mechanical ***************************/

typedef struct {
  float Wheel_RPM_L  = 0.0;
  float Wheel_RPM_R  = 0.0;
  float STR_Heave_mm = 0.0;
  float STR_Roll_mm  = 0.0;
} Mechanical;

void StrokesensorInit(int Heave, int Roll);
void StrokesensorUpdate(Mechanical* m, int Heave, int Roll);
void mockMechanicalData(Mechanical* m);
void teleplotMechanical(Mechanical* m);

/************************* Electrical ***************************/

typedef struct {
  float I_SENSE = 0.0;
  float TMP     = 0.0;
  float APPS    = 0.0;
  float BPPS    = 0.0;
  bool  AMS_OK  = 0;
  bool  IMD_OK  = 0;
  bool  HV_ON   = 0;
  bool  BSPD_OK = 0;
  float steering = 0.0;
} Electrical;

void ElectSensorsInit(int* pins);
void ElectSensorsUpdate(Electrical* e, int* pins);
void mockElectricalData(Electrical* e);
void teleplotElectrical(Electrical* e);

/************************* Odometry ***************************/

typedef struct {
  double gps_lat    = 0;
  double gps_lng    = 0;
  double gps_age    = 0;
  double gps_course = 0;
  double gps_speed  = 0;
  float imu_accelx = 0;
  float imu_accely = 0;
  float imu_accelz = 0;
  float imu_gyrox  = 0;
  float imu_gyroy  = 0;
  float imu_gyroz  = 0;
  float imu_euler_roll  = 0;
  float imu_euler_pitch = 0;
  float imu_euler_yaw   = 0;
  float imu_magx = 0;
  float imu_magy = 0;
  float imu_magz = 0;
  float imu_gravx = 0;
  float imu_gravy = 0;
  float imu_gravz = 0;
} Odometry;

bool GPSinit(HardwareSerial& gpsSerial, int tx, int rx, uint32_t baud);
void GPSupdate(Odometry* o, HardwareSerial& gpsSerial, TinyGPSPlus& gps, bool& available);
bool IMUinit(TwoWire* WireIMU, Adafruit_BNO055& bno);
void IMUcalibrate(Adafruit_BNO055& bno, bool& available);
void IMUupdate(Odometry* o, Adafruit_BNO055& bno, bool& available);
void mockOdometryData(Odometry* o);
void teleplotMotion(Odometry* o);

/************************* BAMOCar ***************************/

#define BAMOCAR_REQUEST_ID          0x201
#define BAMOCAR_RESPONSE_ID         0x181
#define BAMOCAR_REG_MOTOR_TEMP      0x49
#define BAMOCAR_REG_CONTROLLER_TEMP 0x4A
#define BAMOCAR_REG_DC_VOLTAGE      0xEB
#define BAMOCAR_REG_DC_CURRENT      0x20
// #define BAMOCAR_REG_SPEED_ACTUAL 0xA8  // WRONG: was guessed, not from datasheet
#define BAMOCAR_REG_SPEED_ACTUAL    0x30  // N actual (SPEED_IST), unfiltered, per BAMOCar CAN doc
#define BAMOCAR_REG_N100            0xC8  // N-100% scaling register: defines RPM at Num=32767

// RPM scaling: RPM = (int16_t)rawNum / 32767.0 * BAMOCAR_N100_RPM
// N-100% options (set via controller parameter reg 0xC8):
//   N-100% = 6500 RPM → 1 Num = 6500 / 32767 = 0.1984 RPM/Num
//   N-100% = 5500 RPM → 1 Num = 5500 / 32767 = 0.1679 RPM/Num  ← selected
// #define BAMOCAR_N100_RPM 6500.0f
#define BAMOCAR_N100_RPM 5500.0f

typedef struct {
  float motorTemp1 = 0.0;
  float motorTemp2 = 0.0;
  float controllerTemp = 0.0;
  float canVoltage = 0.0;
  float canCurrent = 0.0;
  float power = 0.0;
  float rpm = 0.0;
  bool motorTempValid = false;
  bool controllerTempValid = false;
  bool canVoltageValid = false;
  bool canCurrentValid = false;
  bool rpmValid = false;
} BAMOCar;

void pack_RequestBamocarMsg(twai_message_t* msg, uint8_t regAddress);
void request_BamocarN100(twai_message_t* msg);
void process_ResponseBamocarMsg(twai_message_t* msg, BAMOCar* bamocar);
void mockBAMOCarData(BAMOCar* b);
void teleplotBAMOCar(BAMOCar* b);
