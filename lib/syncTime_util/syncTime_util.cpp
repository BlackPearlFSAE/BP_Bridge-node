#include <Arduino.h>
#include <syncTime_util.h>

// ============================================================================
// CLEAN TIME SYSTEM - Source-Agnostic Unix Timestamp Management
// ============================================================================

// ABSOLUTE TIME (from external source - RTC/NTP/Server)
// uint64_t deviceAbsoluteTime = 0;  // Unix timestamp
// RELATIVE TIME (device uptime tracking)
unsigned long deviceRelativeTime_syncPoint = 0;  // millis() when last synced
// Sync status flag
bool deviceTimeIsSynced = false;

// ============================================================================
// CORE TIME FUNCTIONS
// ============================================================================

/**
 * Set absolute time from ANY source (RTC, NTP, Server, Manual)
 * Accepts: Unix timestamp in milliseconds or nanosec
**/
void syncTime_setAbsolute(uint64_t &deviceAbsoluteTime , uint64_t unixTimeMs) {
  deviceAbsoluteTime = unixTimeMs;
  deviceRelativeTime_syncPoint = millis();  // Mark sync point
  deviceTimeIsSynced = true;
}

/**
 * Get current device time (absolute if synced, relative if not)
 * Returns: Unix timestamp in milliseconds
 * This auto-calculates elapsed time since last sync
 */
uint64_t syncTime_getRelative(uint64_t deviceAbsoluteTime) {
  if (!deviceTimeIsSynced) {
    // Not synced - return device uptime
    return (uint64_t)millis();
  }

  // Calculate elapsed time since sync
  unsigned long elapsed = millis() - deviceRelativeTime_syncPoint;
  return deviceAbsoluteTime + (uint64_t)elapsed;
}

/**
 * Check if device time has been synchronized
 * Returns: true if synced with external source, false if using millis()
 */
bool syncTime_isSynced() {
  return deviceTimeIsSynced;
}

/**
 * Resync with drift checking (optional - prevents unnecessary resyncs)
 * Only updates if drift exceeds threshold (default: 1 second)
 * Returns: true if resynced, false if skipped
 */
bool syncTime_resync(uint64_t &deviceAbsoluteTime,uint64_t newUnixTimeMs, uint64_t driftThreshold_ms) {
  if (!deviceTimeIsSynced) {
    // First sync - always accept
    syncTime_setAbsolute(deviceAbsoluteTime,newUnixTimeMs);
    return true;
  }

  // Check drift
  uint64_t currentTime = syncTime_getRelative(newUnixTimeMs);
  int64_t drift = (int64_t)newUnixTimeMs - (int64_t)currentTime;

  if (llabs(drift) >= (long long)driftThreshold_ms) {
    // Drift exceeded - resync
    syncTime_setAbsolute(deviceAbsoluteTime,newUnixTimeMs);
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
void syncTime_formatUnix(char* outBuf, uint64_t unixMs, int timezoneOffsetHours) {
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
void syncTime_formatUnix_UTC(char* outBuf, uint64_t unixMs) {
  syncTime_formatUnix(outBuf, unixMs, 0);
}
