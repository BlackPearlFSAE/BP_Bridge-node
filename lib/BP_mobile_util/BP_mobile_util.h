#include <stddef.h> // for size_t
#include <cstdint>

typedef struct {
  bool isRegistered = false;
  bool isConnected = false;
  unsigned long lastPingReceived = 0;
  unsigned long connectionStartTime = 0;
} socketstatus;


// Forward declaration 
class WebSocketsClient;

// CALLBACK TYPE def
typedef void (*RegisterTopicFn)(const char* clientName);
typedef uint64_t (*TimeProviderFn)(uint64_t); 

// CALLBACK SETTER Class
class BPMobileConfig {
private:
    // Internal storage
    RegisterTopicFn _registration_cb = nullptr;
    TimeProviderFn  _timesourceProvider_fn = nullptr;
    const char* _client_name = ".";
    void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
    void handleMessage(const char* message);

    // ownership flags for pointer members (set by constructor)
    bool _owns_webSocket = false;
public:
  // Accessible websocket related class , for developer to call in main.cpp
    socketstatus *webSocketstatus = nullptr;    
    WebSocketsClient *webSocket = nullptr;    
    
    BPMobileConfig(WebSocketsClient* ws = nullptr, socketstatus* status = nullptr);
    ~BPMobileConfig();

    // set the callback function for internal class, for developer to call in main.cpp
    void setRegisterCallback(RegisterTopicFn callback);
    void setTimeProvider(TimeProviderFn callback);
    void setClientName(const char* name);

    void registerMCUTopic();
    void syncMCUtime_with_provider(uint64_t time);

    void initWebSocket(const char* serverHost, int serverPort, const char* clientName);
    
};




/*

// Header (keep in BP_mobile_util.h)
class BPMobileConfig {
public:
  BPMobileConfig();
  ~BPMobileConfig();

  // public API
  void initWebSocket(const char* serverHost, int serverPort, const char* clientName);
  void handleMessage(const char* message);

  // public (or provide accessors)
  socketstatus* webSocketstatus = nullptr;
  WebSocketsClient* webSocket = nullptr;

  // disable copy
  BPMobileConfig(const BPMobileConfig&) = delete;
  BPMobileConfig& operator=(const BPMobileConfig&) = delete;

private:
  RegisterTopicFn _registration_cb = nullptr;
  TimeProviderFn  _timesourceProvider_fn = nullptr;
  const char* _client_name = "Unknown";
};

// CPP implementation (put in BP_mobile_util.cpp)
// BPMobileConfig::BPMobileConfig()
// : webSocketstatus(nullptr), webSocket(nullptr),
//   _registration_cb(nullptr), _timesourceProvider_fn(nullptr), _client_name("Unknown") {}
//
// BPMobileConfig::~BPMobileConfig() {
//   if (webSocket) { delete webSocket; webSocket = nullptr; }
//   if (webSocketstatus) { delete webSocketstatus; webSocketstatus = nullptr; }
// }

*/
