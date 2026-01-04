#ifndef SYNCTIME_UTIL_V2_H
#define SYNCTIME_UTIL_V2_H

#include <cstdint>

// ============================================================================
// CLEAN TIME SYSTEM API - Source-Agnostic Unix Timestamp Management
// ============================================================================

/**
 * CORE CONCEPT:
 * - deviceAbsoluteTime_ms: Unix timestamp from ANY source (RTC/NTP/Server)
 * - deviceRelativeTime: Calculated on-the-fly using millis() offset
 *
 * USAGE:
 * 1. Sync from any source: setDeviceAbsoluteTime(unixMs)
 * 2. Get current time: getDeviceRelativeTime()
 * 3. Format for display: formatUnixTime(buf, unixMs, timezoneOffset)
 */

// ============================================================================
// GLOBAL STATE (Direct Access)
// ============================================================================

extern uint64_t deviceAbsoluteTime_ms;           // Unix timestamp in ms (from external source)
extern unsigned long deviceRelativeTime_syncPoint; // millis() when last synced
extern bool deviceTimeIsSynced;                  // Sync status flag

// ============================================================================
// CORE TIME FUNCTIONS (New Clean API)
// ============================================================================

/**
 * Set absolute time from ANY source (RTC, NTP, Server, Manual)
 * This is your SINGLE point of time synchronization
 *
 * Example usage:
 *   setDeviceAbsoluteTime(rtc.now().unixtime() * 1000ULL);  // From RTC
 *   setDeviceAbsoluteTime(ntpTime);                          // From NTP
 *   setDeviceAbsoluteTime(serverTime);                       // From WebSocket
 */
void setDeviceAbsoluteTime(uint64_t unixTimeMs);

/**
 * Get current device time (auto-calculates from sync point)
 * Returns: Unix timestamp in milliseconds
 *
 * - If synced: deviceAbsoluteTime_ms + elapsed millis()
 * - If not synced: millis()
 */
uint64_t getDeviceRelativeTime();

/**
 * Check if device time has been synchronized with external source
 * Returns: true if synced, false if using millis()
 */
bool isDeviceTimeSynced();

/**
 * Resync with drift checking (prevents unnecessary resyncs)
 * Only updates if drift exceeds threshold
 *
 * Parameters:
 *   - newUnixTimeMs: New Unix timestamp in ms
 *   - driftThreshold_ms: Minimum drift to trigger resync (default: 1000ms = 1s)
 *
 * Returns: true if resynced, false if skipped (drift too small)
 */
bool resyncDeviceTime(uint64_t newUnixTimeMs, uint64_t driftThreshold_ms = 1000ULL);

// ============================================================================
// FORMATTING FUNCTIONS (Timezone-Aware)
// ============================================================================

/**
 * Format Unix timestamp to human-readable string with timezone support
 *
 * Parameters:
 *   - outBuf: Output buffer (minimum 32 chars)
 *   - unixMs: Unix timestamp in milliseconds
 *   - timezoneOffsetHours: Hours offset from UTC
 *     Examples: 0=UTC, 7=Bangkok, -5=EST, 8=Singapore
 *
 * Output format: "YYYY-MM-DD HH:MM:SS"
 */
void formatUnixTime(char* outBuf, uint64_t unixMs, int timezoneOffsetHours);

/**
 * Convenience wrappers for common timezones
 */
void formatUnixTime_UTC(char* outBuf, uint64_t unixMs);     // UTC (GMT+0)
void formatUnixTime_Bangkok(char* outBuf, uint64_t unixMs); // Bangkok (GMT+7)

// ============================================================================
// BACKWARD COMPATIBILITY LAYER (Legacy API)
// ============================================================================

/**
 * Legacy function names - maps to new API
 * Keep using these if you don't want to change existing code
 */
unsigned long long getSynchronizedTime();              // → getDeviceRelativeTime()
void syncDevice_to_serverTime(uint64_t serverTimeMs); // → resyncDeviceTime(...)
void formatDateTimeBangkok(char* outBuf, uint64_t timestampMs); // → formatUnixTime_Bangkok(...)
void formatDateTime(char* outBuf, uint64_t timestampMs);        // → formatUnixTime_UTC(...)

/**
 * Legacy global variables (deprecated - use new API)
 */
extern bool timeIsSynchronized;  // Deprecated - use isDeviceTimeSynced()
extern uint64_t baseTimestamp;   // Deprecated - use deviceAbsoluteTime_ms
extern uint64_t syncMillis;      // Deprecated - use deviceRelativeTime_syncPoint

#endif // SYNCTIME_UTIL_V2_H
