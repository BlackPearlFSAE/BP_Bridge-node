// LEDs
#define WIFI_LED 3
#define WS_LED 4

// Mechanical Sensors
#define ENCODER_PINL 3
#define ENCODER_PINR 9
#define ENCODER_N 50
#define STR_Roll 5
#define STR_Heave 6

// Electrical Pins
#define I_SENSE_PIN 4
#define TMP_PIN 8
#define APPS_PIN 15
#define BPPS_PIN 7
#define AMS_OK_PIN 37
#define IMD_OK_PIN 38
#define HV_ON_PIN 35
#define BSPD_OK_PIN 36
#define STEERING 16

extern int ElectPinArray[9];

// SD SPI
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_SCK_PIN 12

// UART
#define UART0_BAUD 115200

// I2C Buses for RTC
#define I2C1_SDA 18
#define I2C1_SCL 17

// Motion Sensors
#define GPS_RX_PIN 2
#define GPS_TX_PIN 1
// #define IMU_SDA 42
// #define IMU_SCL 41
#define IMU_SDA 9
#define IMU_SCL 8
#define GPS_BAUD 9600

// can bus
#define CAN_TX_PIN 48
#define CAN_RX_PIN 47

// SD card Datalog config
#define DEFAULT_SD_LOG_INTERVAL 200 // 0.2 sec write
#define DEFAULT_SD_FLUSH_INTERVAL 5000 // 5 sec flush
#define DEFAULT_SD_CLOSE_INTERVAL 10000 // 15 sec close
#define DEFAULT_SD_ROW_LIMIT 5000 // 1000 rows 

// Network handling config
#define DEFAULT_SSID "realme C55"
#define DEFAULT_PASSWORD "realme1234"
// #define DEFAULT_SSID "dlink-D66C"
// #define DEFAULT_PASSWORD "kdapk67358"

#define DEFAULT_SERVER_HOST "blackpearl-ws-8z9a.onrender.com"
// #define DEFAULT_SERVER_HOST "blackpearl-dashboard.netlify.app"
// #define DEFAULT_SERVER_HOST "Device IP Address"
#define DEFAULT_SERVER_PORT 443
// #define DEFAULT_SERVER_PORT 3000

#define DEFAULT_CLIENT_NAME "ESP32"
#define DEFAULT_PUBLISH_RATE 5.0
#define DEFAULT_WS_PATH "/ws"

// Time sync handling
#define DEFAULT_LOCAL_SYNC_INTERVAL 1000
#define DEFAULT_REMOTE_SYNC_INTERVAL 60000

/************************* Build Flags ***************************/

#define MOCK_FLAG 0
#define DEBUG_MODE 2  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot
#define SD_ENABLED   1     // 0 = Disable SD card init + logging, 1 = Enabled
#define WIFI_ENABLED 0    // 0   = Disable WiFi init (also disables WS), 1 = Enabled
#define calibrate_RTC 0
#define TIME_SRC 0 // 0 = RTC , 1 = WiFI NTP Pool
#define WS_ENABLED 0    // 0 = Disable WebSocket + BPMobile task (WiFi still runs), 1 = Enabled

// -- ADC config
// May add macro for esp32 exclusive later
const float aref = 3.3; 
const int pwmres = 4095; // 12 bit ADC resolution
const float max_distance1 = 75.00; // recalibrated with vernier -> Needs to recheck // 52.91
const float max_distance2 = 75.00; // Not_sure needs to check again

const float max_volt5 =  5.0;
const float max_volt12 =  12.0;
const float max_volt24 =  24.0;  // Fixed: was 5.0, should be 24.0

// Pedal travel distance (APPS/BPPS) 
const float apps_offset_v = 0.5;       
const float bpps_offset_v = 0.0;       
const float steering_aref = 1.9;
// const float steering_aref = 1.95;
const float steering_max__angle = 200.00 ;
const float steering_offset_angle =  0.00;

// NTC thermistor in voltage divider config
const uint16_t tmp_series_res = 10000;  // 10k ohm series res
// Vout = Vcc * (R_NTC / (temp_series_res + R_NTC))

// Hall effect current sensor
const float i_sense_sensitivity = 0.1;  // V/A (example: ACS712-20A = 0.1 V/A)
const float i_sense_offset = 2.5;       // V