#include <cstdint>

void initCANBus(int can_tx,int can_rx, bool readyflag,twai_timing_config_t t_config);
void initCANBus(int can_tx,int can_rx, bool readyflag,
                twai_timing_config_t t_config, twai_filter_config_t f_config);
void sendCAN(twai_message_t* txmsg, int periodMS, bool readyflag, void packmessage(void));
void receiveCAN(twai_message_t* rx_msg, int pollingPeriodMS, bool readyflag, void processmessage(void));
void twai_debug(uint32_t alerts_trigger);

// Example to write for filter -> Make it an output of twai_filter_config_t type , to get the data strcuture need