// LEDs
#define WIFI_LED 3
#define WS_LED 4

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

// can bus
#define CAN_TX_PIN 48
#define CAN_RX_PIN 47

// SD card Datalog config
#define DEFAULT_SD_LOG_INTERVAL 500 // 0.5 sec write
#define DEFAULT_SD_FLUSH_INTERVAL 5000 // 5 sec flush
#define DEFAULT_SD_CLOSE_INTERVAL 15000 // 15 sec close
#define DEFAULT_SD_ROW_LIMIT 1000

// Network handling config
#define DEFAULT_SSID "realme C55"
#define DEFAULT_PASSWORD "realme1234"
// #define DEFAULT_SSID "dlink-D66C"
// #define DEFAULT_PASSWORD "kdapk67358"

#define DEFAULT_SERVER_HOST "blackpearl-ws.onrender.com"
// #define DEFAULT_SERVER_HOST "blackpearl-dashboard.netlify.app"

#define DEFAULT_SERVER_PORT 443
// #define DEFAULT_SERVER_PORT 3000

#define DEFAULT_CLIENT_NAME "ESP32"
#define DEFAULT_PUBLISH_RATE 1.0

// Time sync handling
#define DEFAULT_LOCAL_SYNC_INTERVAL 1000
#define DEFAULT_REMOTE_SYNC_INTERVAL 60000

/************************* Build Flags ***************************/

#define MOCK_FLAG 1
#define calibrate_RTC 0
#define DEBUG_MODE 0  // 0 = Disabled, 1 = Regular Serial, 2 = Teleplot
#define TIME_SRC 1 // 0 = RTC , 1 = WiFI NTP Pool