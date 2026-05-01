#ifndef SD32_UTIL_H
#define SD32_UTIL_H

#include <FS.h>

// ============================================================================
// SD CARD INITIALIZATION
// ============================================================================
void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs, bool &sdCardReady);
bool SD32_checkSDconnect();
void SD32_getSDsize();

// ============================================================================
// SESSION & FILENAME MANAGEMENT
// ============================================================================
void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename, const char* prefix);
void SD32_createSessionDir(int &sessionNumber, char* sessionDirPath, const char* prefix);
void SD32_generateFilenameInDir(char* filepath, const char* dirPath, const char* prefix, int index);

// ============================================================================
// CSV FILE CREATION
// ============================================================================
void SD32_createCSVFile(char* csvFilename, const char* csvHeader);

#endif
