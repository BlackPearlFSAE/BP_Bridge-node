#include <Arduino.h>
#include <WiFi.h>
#include <WIFI32_util.h>

void initWiFi(const char* ssid, const char* password, int attempt) {
  Serial.println("--- WiFi Initialization ---");
  Serial.printf("Connecting to: %s\n",ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < attempt) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection FAILED!");
    return;
  }
  Serial.printf("WiFi connected! at IP: %d\n",WiFi.localIP());
  Serial.printf("Signal strength:%d dbm\n",WiFi.RSSI());
}