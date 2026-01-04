#include <Arduino.h>
#include <syncTime_util.h>
#include <RTClib.h>  // For DateTime object support

// ============================================================================
// TIME FORMATTING and SYNC TIME HELPER FUNCTIONS (Version 2 - DateTime Support)
// ============================================================================
bool timeIsSynchronized = false;
uint64_t baseTimestamp = 0;
uint64_t syncMillis = 0; // the time that has been sync

// ============================================================================
// NEW: DateTime Object Support
// ============================================================================

/**
 * Get synchronized time as RTClib DateTime object
 * Returns: DateTime object with current time, or invalid DateTime if not synced
 */
DateTime getSynchronizedDateTime() {
  uint64_t currentMs = getSynchronizedTime();
  if (currentMs == 0) {
    // Return invalid DateTime if not synced
    return DateTime(0);
  }
  // Convert milliseconds to seconds for DateTime constructor
  return DateTime((uint32_t)(currentMs / 1000ULL));
}

/**
 * Get RTC time as DateTime object (from hardware RTC)
 * Requires: External RTC instance passed in
 * Returns: DateTime object from RTC
 */
DateTime RTC_getDateTime(RTC_DS3231& rtc) {
  return rtc.now();
}

/**
 * Alternative: Get Unix timestamp from RTC and return as DateTime
 * This is what you mentioned - "RTC_getUnix but return the DateTime"
 */
DateTime RTC_getDateTimeFromUnix(RTC_DS3231& rtc) {
  uint32_t unixTime = rtc.now().unixtime();
  return DateTime(unixTime);
}

// ============================================================================
// EXISTING FUNCTIONS - Compatible with DateTime approach
// ============================================================================

// getMCUtime that has been synchronized with time source
unsigned long long getSynchronizedTime() {
  if (timeIsSynchronized) {
    // We have synced at least once - calculate time based on millis() offset
    unsigned long currentMillis = millis();
    unsigned long elapsedMillis = currentMillis - syncMillis;

    // Return base timestamp + elapsed time
    return baseTimestamp + (unsigned long long)elapsedMillis;

  } else {
    // Not synced yet - return 0 or a flag value
    return 0ULL;
  }
}

/**
 * Sync device time using DateTime object
 * Compatible with RTC, NTP, or any DateTime source
 */
void syncDevice_to_DateTime(DateTime dt) {
  uint64_t serverTimeMs = (uint64_t)dt.unixtime() * 1000ULL;
  syncDevice_to_serverTime(serverTimeMs);
}

/**
 * Original sync function - still works as before
 * Only resyncs if drift exceeds 1 second
 */
void syncDevice_to_serverTime(uint64_t serverTimeMs) {
  const unsigned long long MIN_DRIFT_MS = 1000ULL;
  // Drifted about 1 s should be enough to resync
  if (timeIsSynchronized) {
    uint64_t nowMs = getSynchronizedTime();
    int64_t diff = (int64_t)serverTimeMs - (int64_t)nowMs;
    if (llabs(diff) < (long long)MIN_DRIFT_MS) return; // no-op if drift small
  }
  baseTimestamp = serverTimeMs;
  syncMillis = millis();
  timeIsSynchronized = true;
}

// ============================================================================
// FORMATTING FUNCTIONS - Work with both uint64_t and DateTime
// ============================================================================

void formatDateTime(char* outBuf, uint64_t timestampMs) {
  // Convert milliseconds to seconds
  uint64_t timestampSec = timestampMs / 1000ULL;

  // This gives UTC time
  time_t rawtime = (time_t)timestampSec;
  struct tm * timeinfo;
  timeinfo = gmtime(&rawtime);

  // Format: YYYY-MM-DD HH:MM:SS
  sprintf(outBuf, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
}

void formatDateTimeBangkok(char* outBuf, uint64_t timestampMs) {
  // Convert milliseconds to seconds
  uint64_t timestampSec = timestampMs / 1000ULL;
  // Add 7 hours for Bangkok timezone (UTC+7)
  timestampSec += (7 * 3600);

  time_t rawtime = (time_t)timestampSec;
  struct tm * timeinfo;
  timeinfo = gmtime(&rawtime);

  // Format: YYYY-MM-DD HH:MM:SS
  sprintf(outBuf, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
}

/**
 * NEW: Format DateTime object to string
 * Overload for direct DateTime formatting
 */
void formatDateTime(char* outBuf, DateTime dt) {
  uint64_t timestampMs = (uint64_t)dt.unixtime() * 1000ULL;
  formatDateTime(outBuf, timestampMs);
}

void formatDateTimeBangkok(char* outBuf, DateTime dt) {
  uint64_t timestampMs = (uint64_t)dt.unixtime() * 1000ULL;
  formatDateTimeBangkok(outBuf, timestampMs);
}
