
#include <cstdint>

// Time API
unsigned long long getSynchronizedTime();
void syncDevice_to_serverTime(unsigned long long serverTimeMs);
// bool timeIsSynchronized();

// Formatting helpers that do not require Arduino String in header.
// Caller supplies a buffer to be filled (safe for header-only).
void formatDateTime(char* outBuf, uint64_t timestampMs); // UTC
void formatDateTimeBangkok(char* outBuf, uint64_t timestampMs); // UTC+7


extern bool timeIsSynchronized;
extern uint64_t baseTimestamp;
extern uint64_t syncMillis;