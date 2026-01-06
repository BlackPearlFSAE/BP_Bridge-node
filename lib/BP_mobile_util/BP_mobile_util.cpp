#include <Arduino.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <BP_mobile_util.h>
// ============================================================================
// BP Mobile helper
// ============================================================================
/* Local instance (only use here) */

// typedef void (*RegisterTopicFn)(const char* name);
// typedef uint64_t (*TimeProviderFn)(uint64_t time);

// // // static stored callbacks + client name
// static const char* stored_client_name = nullptr;
// static RegisterTopicFn registerClientTopics_fn = nullptr;
// static TimeProviderFn serverSyncTime_fn  = nullptr;

// // // Callback function setter
// void setRegisterCallback(RegisterTopicFn cb) { registerClientTopics_fn = cb; }
// void setTimeProvider(TimeProviderFn fn) {serverSyncTime_fn = fn; }

// socketstatus webSocketstatus;    
// WebSocketsClient webSocket;

// User calls these in their Arduino Sketch
void BPMobileConfig::setRegisterCallback(RegisterTopicFn callback) { _registration_cb = callback; }
void BPMobileConfig::setTimeProvider(TimeProviderFn callback) { _timesourceProvider_fn = callback;}
void BPMobileConfig::setClientName(const char* name) { _client_name = name; }


void BPMobileConfig::registerMCUTopic() {
        if (_registration_cb!= nullptr)
            _registration_cb(_client_name); 
    }
void BPMobileConfig::syncMCUtime_with_provider(uint64_t time){
        if (_timesourceProvider_fn!= nullptr)
            _timesourceProvider_fn(time); 
    }

void BPMobileConfig::initWebSocket(const char* serverHost, const int serverPort, const char* clientName) {
  Serial.println("--- WebSocket Initialization ---");
  Serial.print("Connecting to: ws://");
  Serial.print(serverHost);
  Serial.print(":");
  Serial.println(serverPort);

  // Debug: Check pointers before using
  Serial.print("[initWebSocket] webSocket=0x");
  // Not sure if this is what causes the crash , but I can't read core dumo
  Serial.print((unsigned long)this->webSocket, HEX);
  Serial.print(", webSocketstatus=0x");
  Serial.println((unsigned long)this->webSocketstatus, HEX);

  // IMPORTANT: Set event handler BEFORE calling begin() to avoid race conditions
  this->webSocket->onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
    this->webSocketEvent(type, payload, length);
  });

  this->webSocket->setReconnectInterval(5000);

  // Start connection last, after everything is set up // THE BREAK POINT IS HERE , BUT I don't know how to use the 
  this->webSocket->begin(serverHost, serverPort, "/");

  if (this->webSocketstatus != nullptr) {
    this->webSocketstatus->connectionStartTime = millis();
  } else {
    Serial.println("[ERROR] webSocketstatus is NULL!");
  }
}
void BPMobileConfig::webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  // Debug: Check this pointer FIRST before accessing members
  Serial.print("[webSocketEvent] type=");
  Serial.print(type);
  Serial.print(", this=0x");
  Serial.println((unsigned long)this, HEX);

  if (!this) {
    Serial.println("[ERROR] this pointer is NULL in webSocketEvent!");
    return;
  }

  Serial.print("[webSocketEvent] webSocketstatus=0x");
  Serial.println((unsigned long)this->webSocketstatus, HEX);

  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WebSocket] Disconnected");
      if (this->webSocketstatus) {
        this->webSocketstatus->isConnected = false;
        this->webSocketstatus->isRegistered = false;
      }
      break;

    case WStype_CONNECTED:
      Serial.println("[WebSocket] Connected!");
      if (this->webSocketstatus) {
        this->webSocketstatus->isConnected = true;
      }
      this->registerMCUTopic();
      break;
      
    case WStype_TEXT:
      handleMessage((char*)payload);
      break;
      
    case WStype_ERROR:
      Serial.print("[WebSocket] Error: ");
      Serial.println((char*)payload);
      break;

    default:
      break;
  }
}
void BPMobileConfig::handleMessage(const char* message) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("[WebSocket] JSON error: ");
    Serial.println(error.c_str());
    return;
  }
  String type = doc["type"] | "";
  
  if (type == "registration_response") {
    String status = doc["status"] | "";
    
    if (status == "accepted") {
      Serial.println("[WebSocket] ✓ Registration ACCEPTED!");
      this->webSocketstatus->isRegistered = true;
      
      // Sync time
      if (doc["system_time"].is<unsigned long long>()) {
        JsonObject sysTime = doc["system_time"];
        if (sysTime["timestamp_ms"].is<unsigned long long>()) {
          uint64_t serverTime = sysTime["timestamp_ms"];
          this->syncMCUtime_with_provider(serverTime);
          // synchronizeTime(serverTime); // Synctime after it is done // still depends on other function
        }
      }
      
      Serial.println("[WebSocket] Ready to stream data!");
      
    } else if (status == "rejected") {
      Serial.println("[WebSocket] ✗ Registration REJECTED!");
      String msg = doc["message"] | "Unknown error";
      Serial.print("  Reason: ");
      Serial.println(msg);
      this->webSocketstatus->isRegistered = false;
      
      if (msg.indexOf("already exists") >= 0) {
        Serial.println("\n*** Change 'clientName' to a unique value! ***");
      }
    }
    
  } else if (type == "ping") {
    String pingId = doc["ping_id"] | "";
    this->webSocketstatus->lastPingReceived = millis();
    
    // StaticJsonDocument<128> pongDoc;
    JsonDocument pongDoc;
    pongDoc["type"] = "pong";
    pongDoc["ping_id"] = pingId;
    pongDoc["timestamp"] = millis();
    
    String pong;
    serializeJson(pongDoc, pong);
    webSocket->sendTXT(pong);
    
    Serial.print("[WebSocket] Ping/Pong (ID: ");
    Serial.print(pingId);
    Serial.println(")");
  }
}

// constructor
BPMobileConfig::BPMobileConfig(WebSocketsClient* ws,socketstatus* status)
{
  // if websocket object were provided
  if (ws) {
    this->webSocket = ws;
    this->_owns_webSocket = false;
  // if not provided , instantiate here
  } else {
    this->webSocket = new WebSocketsClient();
    this->_owns_webSocket = true;
  }

  // Handle socketstatus object
  if (status) {
    this->webSocketstatus = status;
    this->_owns_socketstatus = false;
  } else {
    this->webSocketstatus = new socketstatus();  // Create if not provided
    this->_owns_socketstatus = true;
  }

  // Debug: Verify pointers are valid
  Serial.print("[BPMobileConfig] Constructor: webSocket=0x");
  Serial.print((unsigned long)this->webSocket, HEX);
  Serial.print(", webSocketstatus=0x");
  Serial.println((unsigned long)this->webSocketstatus, HEX);

  // ensure callbacks default to null (header already set, but be explicit)
  _registration_cb = nullptr;
  _timesourceProvider_fn = nullptr;
  _client_name = "Unknown";
}

// destructor
BPMobileConfig::~BPMobileConfig()
{
  if (_owns_webSocket && this->webSocket) {
    delete this->webSocket;
    this->webSocket = nullptr;
  }
  if (_owns_socketstatus && this->webSocketstatus) {
    delete this->webSocketstatus;
    this->webSocketstatus = nullptr;
  }
}
