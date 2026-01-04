// #include <Arduino.h>
// #include <syncTime_util.h>

// // ============================================================================
// // TIME FORMATTING and SYNC TIME HELPER FUNCTIONS
// // ============================================================================
// bool timeIsSynchronized = false;
// uint64_t baseTimestamp = 0;
// uint64_t syncMillis = 0; // the time that has been sync

// // getMCUtime that has been synchronized with time source
// unsigned long long getSynchronizedTime() {
//   if (timeIsSynchronized) {
//     // We have synced at least once - calculate time based on millis() offset
//     unsigned long currentMillis = millis();
//     unsigned long elapsedMillis = currentMillis - syncMillis;
    
//     // Return base timestamp + elapsed time
//     return baseTimestamp + (unsigned long long)elapsedMillis;
    
//   } else {
//     // Not synced yet - return 0 or a flag value
//     return 0ULL;
//   }
// }

// // void syncDevice_to_serverTime(uint64_t serverTimeMs) {
// //   baseTimestamp = serverTimeMs;
// //   syncMillis = millis();
// //   timeIsSynchronized = true;
  
// //   // Show synchronized time in both formats
// //   char syncedTime[32] = "";
// //   formatDateTimeBangkok(syncedTime, serverTimeMs);
  
// //   Serial.println("\n╔════════════════════════════════════════╗");
// //   Serial.println("║      TIME SYNCHRONIZED SUCCESS!        ║");
// //   Serial.println("╠════════════════════════════════════════╣");
// //   Serial.print("║ Unix Time:   ");
// //   Serial.print(serverTimeMs / 1000ULL);
// //   Serial.println(" sec       ║");
// //   Serial.print("║ Bangkok Time: ");
// //   Serial.print(syncedTime);
// //   Serial.println("   ║");
// //   Serial.print("║ Sync Millis:  ");
// //   Serial.print(syncMillis);
// //   Serial.println(" ms            ║");
// //   Serial.println("║ Time will now run independently!       ║");
// //   Serial.println("╚════════════════════════════════════════╝\n");
// // }

// void syncDevice_to_serverTime(uint64_t serverTimeMs) {
//   const unsigned long long MIN_DRIFT_MS = 1000ULL;
//   // Drifted about 1 s should be enough to resync
//   if (timeIsSynchronized) {
//     uint64_t nowMs = getSynchronizedTime();
//     int64_t diff = (int64_t)serverTimeMs - (int64_t)nowMs;
//     if (llabs(diff) < (long long)MIN_DRIFT_MS) return; // no-op if drift small
//   }
//   baseTimestamp = serverTimeMs;
//   syncMillis = millis();
//   timeIsSynchronized = true;
// }


// void formatDateTime(char* outBuf, uint64_t timestampMs) {
//   // Convert milliseconds to seconds
//   uint64_t timestampSec = timestampMs / 1000ULL;
  
//   // This gives UTC time
//   time_t rawtime = (time_t)timestampSec;
//   struct tm * timeinfo;
//   timeinfo = gmtime(&rawtime);
  
//   // char buffer[32];
//   // Format: YYYY-MM-DD HH:MM:SS
//   sprintf(outBuf, "%04d-%02d-%02d %02d:%02d:%02d",
//           timeinfo->tm_year + 1900,
//           timeinfo->tm_mon + 1,
//           timeinfo->tm_mday,
//           timeinfo->tm_hour,
//           timeinfo->tm_min,
//           timeinfo->tm_sec);

// }

// void formatDateTimeBangkok(char* outBuf, uint64_t timestampMs) {
//   // Convert milliseconds to seconds
//   uint64_t timestampSec = timestampMs / 1000ULL;
//   // Add 7 hours for Bangkok timezone (UTC+7)
//   timestampSec += (7 * 3600);
  
//   time_t rawtime = (time_t)timestampSec;
//   struct tm * timeinfo;
//   timeinfo = gmtime(&rawtime);
  
//   // Format: YYYY-MM-DD HH:MM:SS
//   sprintf(outBuf, "%04d-%02d-%02d %02d:%02d:%02d",
//           timeinfo->tm_year + 1900,
//           timeinfo->tm_mon + 1,
//           timeinfo->tm_mday,
//           timeinfo->tm_hour,
//           timeinfo->tm_min,
//           timeinfo->tm_sec);
// }
