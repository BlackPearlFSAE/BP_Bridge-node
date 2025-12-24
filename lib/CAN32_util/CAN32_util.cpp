#include <Arduino.h>
#include <driver/twai.h>
#include "CAN32_util.h"


// No BUS ID Filter (Accept all)
void initCANBus(int can_tx,int can_rx, bool readyflag,twai_timing_config_t t_config) {
  Serial.println("--- CAN Bus Initialization ---");
  Serial.print("Initializing CAN bus...");
  
  // Configure CAN timing for 250 kbps (as per your code)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)can_tx,(gpio_num_t)can_rx,TWAI_MODE_NORMAL);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println(" Driver installed");
  } else {
    Serial.println(" FAILED to install driver!");
    readyflag = false;
    return;
  }
  
  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("CAN bus started successfully!");
    readyflag = true;
    delay(500);
  } else {
    Serial.println("FAILED to start CAN bus!");
    readyflag = false;
  }
  Serial.println();
}
// With BUS ID Filter
void initCANBus(int can_tx,int can_rx, bool readyflag,
                twai_timing_config_t t_config, twai_filter_config_t f_config) {
  Serial.println("--- CAN Bus Initialization ---");
  Serial.print("Initializing CAN bus...");
  
  // Configure CAN timing for 250 kbps (as per your code)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)can_tx,(gpio_num_t)can_rx,TWAI_MODE_NORMAL);
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println(" Driver installed");
  } else {
    Serial.println(" FAILED to install driver!");
    readyflag = false;
    return;
  }
  
  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("CAN bus started successfully!");
    readyflag = true;
    delay(500);
  } else {
    Serial.println("FAILED to start CAN bus!");
    readyflag = false;
  }
  Serial.println();
}

// SENDING STD message
void sendCAN(twai_message_t* txmsg, int periodMS, bool readyflag, void packmessage(void)) {
  if (!readyflag) return;
  packmessage();
  // Transmit , and if it failed.
  if (twai_transmit(txmsg, pdMS_TO_TICKS(periodMS)) != ESP_OK)
    Serial.println("Failed to send message");
  // May need to write a simple non-block timer afteward 
}

// SENDING EXT message (Wait, we just adjust its ID flag)

// Polling CAN Reading
void receiveCAN(twai_message_t* rx_msg, int pollingPeriodMS, bool readyflag, void processmessage(void)){
  if (!readyflag) return;
  if (twai_receive(rx_msg, pdMS_TO_TICKS(pollingPeriodMS)) == ESP_OK) {
    // Serial.printf("Received - ID: 0x%X, DLC: %d, Data: ", rx_msg->identifier, rx_msg->data_length_code);
    processmessage();
    // process message accordingly (OR we will just try the same method define type of function pointer , and make a setter method )
  } else {
    Serial.println("RX Buffer = 0");
  }

}

void twai_debug(uint32_t alerts_trigger){
  //Debug and troubleshoot TWAI bus
  /*
  TWAI_ALERT_RX_DATA        0x00000004    Alert(4)    : A frame has been received and added to the RX queue
  TWAI_ALERT_ERR_PASS       0x00001000    Alert(4096) : TWAI controller has become error passive
  TWAI_ALERT_BUS_ERROR      0x00000200    Alert(512)  : A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus
  TWAI_ALERT_RX_QUEUE_FULL  0x00000800    Alert(2048) : The RX queue is full causing a frame to be lost
  */
  //Error Alert message
  twai_status_info_t status_info;
  // Configure alert anable
  twai_read_alerts(&alerts_trigger, pdMS_TO_TICKS(1));
  twai_get_status_info(&status_info);
  // Serial.println(alerts_triggered); // can be twai alert all
  
  // Error state
  if (status_info.state != TWAI_STATE_RUNNING) {
    Serial.print("[CAN ERROR] Bus state: ");
    switch(status_info.state) {
      case TWAI_STATE_STOPPED:
        Serial.println("STOPPED");
        break;
      case TWAI_STATE_BUS_OFF:
        Serial.println("BUS OFF - Recovery needed!");
        break;
      case TWAI_STATE_RECOVERING:  
        Serial.println("REOCVERING TEC REC PLESE WAIT");
        break;
      case TWAI_STATE_RUNNING: 
        Serial.println("RUNNING");
        break;
      default:
        Serial.println("UNKNOWN");
    }
    // Show error counters
    Serial.print("RX Errors REC: "); Serial.println(status_info.rx_error_counter);
    Serial.print("TX Errors TEC: "); Serial.println(status_info.tx_error_counter);
 
  }
}