/*
 * CAN Bus sending receiving boiler plate code
 * 
 * This sketch combines:
 * - CAN receive
 * 
 * Required Libraries:
 * - twai.h by esp32 driver api library
 */

#include "driver/twai.h"
#include "Arduino.h"

// Pin definitions (adjust for your board)
// #define TX_GPIO_NUM GPIO_NUM_14
// #define RX_GPIO_NUM GPIO_NUM_13

#define TX_GPIO_NUM GPIO_NUM_48
#define RX_GPIO_NUM GPIO_NUM_47

void setup() {

  Serial.begin(115200);
  
  // TWAI timing configuration for 250 kbps
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  
  // TWAI filter configuration (accept all)
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  // TWAI general configuration
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
  
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installed");
  } else {
    Serial.println("Failed to install TWAI driver");
    return;
  }
  
  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI driver started");
  } else {
    Serial.println("Failed to start TWAI driver");
  }
}

void loop() {
  // SENDING STD message
  twai_message_t tx_msg;
  tx_msg.data_length_code = 8;         // 8 bytes

  tx_msg.identifier = 0x123;           // CAN ID
  tx_msg.flags = TWAI_MSG_FLAG_NONE;   // Standard frame
  /* For extended frame
  tx_msg.flags = TWAI_MSG_FLAG_EXTD;
  tx_msg.identifier = 0x12345678;  // 29-bit ID
  */
  
  for (int i = 0; i < 8; i++) {
    tx_msg.data[i] = i; // pack data 1-8
  }
  
  if (twai_transmit(&tx_msg, pdMS_TO_TICKS(500)) == ESP_OK) {
    Serial.println("Message sent");
  } else {
    Serial.println("Failed to send message");
  }
  
  // RECEIVING
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(500)) == ESP_OK) {
    Serial.printf("Received - ID: 0x%X, DLC: %d, Data: ", rx_msg.identifier, rx_msg.data_length_code);
    
    for (int i = 0; i < rx_msg.data_length_code; i++) {
      Serial.printf("0x%02X ", rx_msg.data[i]);
    }
    Serial.println();
  }
  
  delay(500);
}


/*

// CAN Bus pins
#define CAN_TX_PIN 14  
#define CAN_RX_PIN 13
// Create message to send
  twai_message_t message;
  message.identifier = 0x123;        // CAN ID
  message.extd = 0;                  // Standard frame
  message.rtr = 0;                   // Data frame
  message.data_length_code = 8;      // 8 bytes
  
  // Fill data bytes
  message.data[0] = 0x01;
  message.data[1] = 0x02;
  message.data[2] = 0x03;
  message.data[3] = 0x04;
  message.data[4] = 0x05;
  message.data[5] = 0x06;
  message.data[6] = 0x07;
  message.data[7] = 0x08;


*/