#ifndef SYNCTIME_UTIL_V2_H
#define SYNCTIME_UTIL_V2_H

#include <cstdint>
/**
 * CORE CONCEPT:
 * - deviceAbsoluteTime_ms: Unix timestamp from ANY source (RTC/NTP/Server)
 * - deviceRelativeTime: Calculated on-the-fly using millis() offset
 *
 * USAGE:
 * 1. Sync from any source: syncTime_setAbsolute(unixMs)
 * 2. Get current time: syncTime_getRelative()
 * 3. Format for display: syncTime_formatUnix(buf, unixMs, timezoneOffset)
 */

// extern uint64_t deviceAbsoluteTime;           // Unix timestamp in ms (from external source)
// extern unsigned long deviceRelativeTime_syncPoint; // millis() when last synced
// extern bool deviceTimeIsSynced;                  // Sync status flag

// ============================================================================
// CORE TIME FUNCTIONS (New Clean API)
// ============================================================================
/**
 * Set absolute time from ANY source (RTC, NTP, Server, Manual)
 * This is your SINGLE point of time synchronization
 *
 * Example usage:
 *   syncTime_setAbsolute(rtc.now().unixtime() * 1000ULL);  // From RTC
 *   syncTime_setAbsolute(ntpTime);                          // From NTP
 *   syncTime_setAbsolute(serverTime);                       // From WebSocket
 */
void syncTime_setAbsolute(uint64_t &deviceAbsoluteTime , uint64_t unixTimeMs);

/**
 * Get current device time (auto-calculates from sync point)
 * Returns: Unix timestamp in milliseconds
 *
 * - If synced: deviceAbsoluteTime_ms + elapsed millis()
 * - If not synced: millis()
 */
uint64_t syncTime_getRelative(uint64_t deviceAbsoluteTime);

/**
 * Check if device time has been synchronized with external source
 * Returns: true if synced, false if using millis()
 */
bool syncTime_isSynced();

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
bool syncTime_resync(uint64_t &deviceAbsoluteTime, uint64_t newUnixTimeMs, uint64_t driftThreshold_ms = 1000ULL);

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
void syncTime_formatUnix(char* outBuf, uint64_t unixMs, int timezoneOffsetHours);

/**
 * Convenience wrappers for common timezones
 */
void syncTime_formatUnix_UTC(char* outBuf, uint64_t unixMs);     // UTC (GMT+0)

#endif // SYNCTIME_UTIL_V2_H