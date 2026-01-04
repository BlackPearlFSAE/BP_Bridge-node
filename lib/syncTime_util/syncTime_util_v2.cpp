#include <Arduino.h>
#include <syncTime_util_v2.h>

// ============================================================================
// CLEAN TIME SYSTEM - Source-Agnostic Unix Timestamp Management
// ============================================================================

// ABSOLUTE TIME (from external source - RTC/NTP/Server)
uint64_t deviceAbsoluteTime_ms = 0;  // Unix timestamp in milliseconds

// RELATIVE TIME (device uptime tracking)
unsigned long deviceRelativeTime_syncPoint = 0;  // millis() when last synced

// Sync status flag
bool deviceTimeIsSynced = false;

// ============================================================================
// CORE TIME FUNCTIONS
// ============================================================================

/**
 * Set absolute time from ANY source (RTC, NTP, Server, Manual)
 * Accepts: Unix timestamp in milliseconds
 * This is your single point of time synchronization
 */
void setDeviceAbsoluteTime(uint64_t unixTimeMs) {
  deviceAbsoluteTime_ms = unixTimeMs;
  deviceRelativeTime_syncPoint = millis();  // Mark sync point
  deviceTimeIsSynced = true;
}

/**
 * Get current device time (absolute if synced, relative if not)
 * Returns: Unix timestamp in milliseconds
 * This auto-calculates elapsed time since last sync
 */
uint64_t getDeviceRelativeTime() {
  if (!deviceTimeIsSynced) {
    // Not synced - return device uptime
    return (uint64_t)millis();
  }

  // Calculate elapsed time since sync
  unsigned long elapsed = millis() - deviceRelativeTime_syncPoint;
  return deviceAbsoluteTime_ms + (uint64_t)elapsed;
}

/**
 * Check if device time has been synchronized
 * Returns: true if synced with external source, false if using millis()
 */
bool isDeviceTimeSynced() {
  return deviceTimeIsSynced;
}

/**
 * Resync with drift checking (optional - prevents unnecessary resyncs)
 * Only updates if drift exceeds threshold (default: 1 second)
 * Returns: true if resynced, false if skipped
 */
bool resyncDeviceTime(uint64_t newUnixTimeMs, uint64_t driftThreshold_ms) {
  if (!deviceTimeIsSynced) {
    // First sync - always accept
    setDeviceAbsoluteTime(newUnixTimeMs);
    return true;
  }

  // Check drift
  uint64_t currentTime = getDeviceRelativeTime();
  int64_t drift = (int64_t)newUnixTimeMs - (int64_t)currentTime;

  if (llabs(drift) >= (long long)driftThreshold_ms) {
    // Drift exceeded - resync
    setDeviceAbsoluteTime(newUnixTimeMs);
    return true;
  }

  // Drift within tolerance - skip resync
  return false;
}

// ============================================================================
// FORMATTING FUNCTIONS - Timezone Support
// ============================================================================

/**
 * Format Unix timestamp to human-readable string with timezone offset
 * Parameters:
 *   - outBuf: Output buffer (minimum 32 chars)
 *   - unixMs: Unix timestamp in milliseconds
 *   - timezoneOffsetHours: Hours offset from UTC (e.g., 7 for Bangkok, -5 for EST)
 */
void formatUnixTime(char* outBuf, uint64_t unixMs, int timezoneOffsetHours) {
  // Convert to seconds and apply timezone offset
  uint64_t unixSec = (unixMs / 1000ULL) + (timezoneOffsetHours * 3600);

  time_t rawtime = (time_t)unixSec;
  struct tm* timeinfo = gmtime(&rawtime);

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
 * Convenience wrapper for UTC formatting
 */
void formatUnixTime_UTC(char* outBuf, uint64_t unixMs) {
  formatUnixTime(outBuf, unixMs, 0);
}

/**
 * Convenience wrapper for Bangkok time (UTC+7)
 */
void formatUnixTime_Bangkok(char* outBuf, uint64_t unixMs) {
  formatUnixTime(outBuf, unixMs, 7);
}

// ============================================================================
// BACKWARD COMPATIBILITY LAYER (for existing code)
// ============================================================================

/**
 * Legacy function name mapping for backward compatibility
 * Maps old getSynchronizedTime() to new getDeviceRelativeTime()
 */
unsigned long long getSynchronizedTime() {
  return getDeviceRelativeTime();
}

/**
 * Legacy function name mapping for backward compatibility
 * Maps old syncDevice_to_serverTime() to new setDeviceAbsoluteTime()
 */
void syncDevice_to_serverTime(uint64_t serverTimeMs) {
  resyncDeviceTime(serverTimeMs, 1000ULL);  // 1 second drift tolerance
}

/**
 * Legacy function name mapping for backward compatibility
 */
void formatDateTimeBangkok(char* outBuf, uint64_t timestampMs) {
  formatUnixTime_Bangkok(outBuf, timestampMs);
}

/**
 * Legacy function name mapping for backward compatibility
 */
void formatDateTime(char* outBuf, uint64_t timestampMs) {
  formatUnixTime_UTC(outBuf, timestampMs);
}

// Legacy global variable exports
bool timeIsSynchronized = false;  // Deprecated - use isDeviceTimeSynced()
uint64_t baseTimestamp = 0;       // Deprecated - use deviceAbsoluteTime_ms
uint64_t syncMillis = 0;          // Deprecated - use deviceRelativeTime_syncPoint

// Sync legacy globals on every call (for backward compatibility)
void _syncLegacyGlobals() {
  timeIsSynchronized = deviceTimeIsSynced;
  baseTimestamp = deviceAbsoluteTime_ms;
  syncMillis = deviceRelativeTime_syncPoint;
}
