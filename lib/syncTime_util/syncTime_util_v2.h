#ifndef SYNCTIME_UTIL_V2_H
#define SYNCTIME_UTIL_V2_H

#include <cstdint>
#include <RTClib.h>  // For DateTime support

// ============================================================================
// EXISTING API - Backward compatible
// ============================================================================

// Time API (milliseconds-based)
unsigned long long getSynchronizedTime();
void syncDevice_to_serverTime(unsigned long long serverTimeMs);

// Formatting helpers (buffer must be at least 32 chars)
void formatDateTime(char* outBuf, uint64_t timestampMs);        // UTC
void formatDateTimeBangkok(char* outBuf, uint64_t timestampMs); // UTC+7

// Global state
extern bool timeIsSynchronized;
extern uint64_t baseTimestamp;
extern uint64_t syncMillis;

// ============================================================================
// NEW: DateTime Object API
// ============================================================================

/**
 * Get synchronized time as DateTime object
 * Returns invalid DateTime(0) if not synced
 */
DateTime getSynchronizedDateTime();

/**
 * Get RTC hardware time as DateTime object
 * Pass your RTC instance (e.g., RTC_DS3231)
 */
DateTime RTC_getDateTime(RTC_DS3231& rtc);

/**
 * Get RTC Unix time converted to DateTime
 * This is "RTC_getUnix but returns DateTime"
 */
DateTime RTC_getDateTimeFromUnix(RTC_DS3231& rtc);

/**
 * Sync device time using a DateTime object
 * Works with any DateTime source (RTC, NTP, manual)
 */
void syncDevice_to_DateTime(DateTime dt);

/**
 * Format DateTime object to string (overloads)
 */
void formatDateTime(char* outBuf, DateTime dt);
void formatDateTimeBangkok(char* outBuf, DateTime dt);

#endif // SYNCTIME_UTIL_V2_H
