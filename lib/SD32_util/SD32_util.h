#ifndef SD32_UTIL_H
#define SD32_UTIL_H

#include "SdFat.h"

// SdFat global instance (extern so AMS can use it for multi-file ops)
extern SdFat32 sd;

// Type alias for SdFat file handle
typedef File32 SD32File;

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
void SD32_generateFilenameInDir(char* filepath, const char* dirPath, const char* prefix, int index, const char* ext = "csv");

// ============================================================================
// CSV LOGGING - APPENDER ARCHITECTURE
// ============================================================================
typedef void (*AppenderFunc)(SD32File&, void*);

// Create CSV file with header
void SD32_createCSVFile(const char* csvFilename, const char* csvHeader);
// Non-persistent: opens file, appends, closes (simple but slower)
void SD32_appendBulkDataToCSV(const char* filepath, AppenderFunc* appenders, void** dataArray, size_t count);

// ============================================================================
// PERSISTENT FILE HANDLE (recommended for continuous logging)
// ============================================================================
// Open file once, keep open for continuous logging
bool SD32_openPersistentFile(const char* filepath);
void SD32_closePersistentFile();
bool SD32_isPersistentFileOpen();

// Append data to persistent file
// flushIntervalMs: time between flushes (0 = flush every write)
void SD32_appendBulkDataPersistent(AppenderFunc* appenders, void** dataArray, size_t count, unsigned long flushIntervalMs, unsigned long closeIntervalMs);

// Force flush (call before power off or SD removal)
void SD32_flushPersistentFile();

// ============================================================================
// BINARY LOGGING
// ============================================================================
#pragma pack(push, 1)
struct BinFileHeader {
    uint32_t magic;        // 0x42504C47 ("BPLG" = BlackPearl LoG)
    uint8_t  version;      // Header format version (start at 1)
    uint16_t entrySize;    // sizeof(SDLogEntry) for this node
    uint16_t headerSize;   // sizeof(BinFileHeader)
    char     nodeName[16]; // "Front", "Rear", "Bamo", "AMS_bmuN"
};
#pragma pack(pop)

// Create binary file with header
void SD32_createBinFile(const char* filepath, const char* nodeName, uint16_t entrySize);

// Append raw binary data to persistent file
void SD32_appendBinaryPersistent(const void* data, size_t dataSize, unsigned long flushIntervalMs, unsigned long closeIntervalMs);

#endif
