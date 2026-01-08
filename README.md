## BP16B Sensor Node and Data Logger

Intended to be used on ESP32 Bridge PCB `here`

It acts as a local sd card logger , and a wifi gateway to websocket pub sub

Here are wiring diagram of how the board are supposed to be connected in BP16B setting

Front node are connected to
- Front Mechanical sensors
- Electrical sensors plug
- BAMOCAR-P3-400 [CANBUS]

Rear node (2 board)
*Board 1
- Rear Mechanical sensor
- Odometry sensor or motion sensor [GPS, 9 axis IMU]
*Board 2
- Battery Management system [CANBUS]

## How to use this project

