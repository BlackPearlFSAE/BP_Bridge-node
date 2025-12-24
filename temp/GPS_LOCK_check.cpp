/*
 * AN251 GPS Module with TinyGPS++ Library
 * Much simpler and more reliable than manual parsing
 */
// #include <SPI.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ===== CONFIGURATION =====
// #define GPS_RX_PIN 41  // Connect to GPS TX
// #define GPS_TX_PIN 42  // Connect to GPS RX
// HardwareSerial gpsSerial(2);

#define GPS_RX_PIN 2  // Connect to GPS TX
#define GPS_TX_PIN 1  // Connect to GPS RX
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

#define USB0_BAUD 115200
#define GPS_BAUD 115200

void GPSlocation();
void GPS_ISOdatetime();
void GPSext_info();

// ===== SETUP =====
void setup() {
  Serial.begin(USB0_BAUD);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  Serial.println(F("AN251 GPS Module with TinyGPS++"));
  Serial.println(F("Waiting for GPS fix..."));
}

// ===== MAIN LOOP =====
void loop() {
  // Feed GPS data to TinyGPS++
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      GPSlocation();
      delay(1000);
    }
  }
  
  // Check if no data received for 5 seconds
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
    while(true);
  }
}


// ===== DISPLAY GPS INFORMATION =====

void GPSlocation() {
  Serial.println(F("\n===== GPS Lat lng ====="));
  
  // Location
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {

    // Data stuffing

    // Serial debugging
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.println(gps.location.lng(), 6);
    Serial.print(F("Age: "));
    Serial.print(gps.location.age());
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("INVALID"));
  }  
  
}

void GPS_ISOdatetime(){
    // GET ISO Date & Time
  Serial.print(F("Date/Time: "));
  if (gps.date.isValid() && gps.time.isValid()) {
    // Data stuffing

    // Serial debugging
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    Serial.print(F(" "));
    
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.println(gps.time.second());
  } else {
    Serial.println(F("INVALID"));
  }
}

void GPSext_info(){
  // Altitude
  Serial.print(F("Altitude: "));
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters());
    Serial.print(F(" m ("));
    Serial.print(gps.altitude.feet());
    Serial.println(F(" ft)"));
  } else {
    Serial.println(F("INVALID"));
  }
  
  // Speed
  Serial.print(F("Speed: "));
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph());
    Serial.print(F(" km/h ("));
    Serial.print(gps.speed.mph());
    Serial.print(F(" mph, "));
    Serial.print(gps.speed.knots());
    Serial.println(F(" knots)"));
  } else {
    Serial.println(F("INVALID"));
  }
  
  // Course/Heading
  Serial.print(F("Course: "));
  if (gps.course.isValid()) {
    Serial.print(gps.course.deg());
    Serial.print(F("° ("));
    Serial.print(TinyGPSPlus::cardinal(gps.course.deg()));
    Serial.println(F(")"));
  } else {
    Serial.println(F("INVALID"));
  }
  
  // // Satellites
  // Serial.print(F("Satellites: "));
  // if (gps.satellites.isValid()) {
  //   Serial.println(gps.satellites.value());
  // } else {
  //   Serial.println(F("INVALID"));
  // }
  
  // // HDOP (Horizontal Dilution of Precision)
  // Serial.print(F("HDOP: "));
  // if (gps.hdop.isValid()) {
  //   Serial.println(gps.hdop.hdop());
  // } else {
  //   Serial.println(F("INVALID"));
  // }
  
  // // Statistics
  // Serial.print(F("Chars: "));
  // Serial.print(gps.charsProcessed());
  // Serial.print(F(" Sentences: "));
  // Serial.print(gps.sentencesWithFix());
  // Serial.print(F(" Failed: "));
  // Serial.println(gps.failedChecksum());
  
  // Serial.println(F("=====================\n"));
}


