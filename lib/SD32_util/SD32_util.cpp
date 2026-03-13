#include "Arduino.h"
#include "SPI.h"
#include "SD32_util.h"

// ============================================================================
// GLOBAL SdFat INSTANCE
// ============================================================================
SdFat32 sd;

// ============================================================================
// PERSISTENT FILE HANDLE
// ============================================================================
static SD32File _persistentFile;
static bool _persistentFileOpen = false;
static unsigned long _lastFlushTime = 0;
static unsigned long _lastCloseTime = 0;
static char _persistentFilePath[48] = {0};

// ============================================================================
// SD CARD INITIALIZATION
// ============================================================================

void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs, bool &sdCardReady) {
  Serial.println("--- SD Card Initialization ---");
  Serial.print("Initializing SD card...");
  SPI.begin(sd_sck, sd_miso, sd_mosi, sd_cs);

  if (!sd.begin(SdSpiConfig(sd_cs, DEDICATED_SPI, SD_SCK_MHZ(25)))) {
    Serial.println(" FAILED!");
    Serial.println("SD card logging disabled.");
    Serial.println("Check:");
    Serial.println("  - SD card is inserted");
    Serial.println("  - Connections are correct");
    Serial.println("  - SD card is formatted (FAT32)");
    sdCardReady = false;
    return;
  }
  Serial.println(" SUCCESS!");
  sdCardReady = true;

  uint8_t cardType = sd.card()->type();
  Serial.print("Card Type: ");
  if (cardType == SD_CARD_TYPE_SD1) Serial.println("SD1");
  else if (cardType == SD_CARD_TYPE_SD2) Serial.println("SD2");
  else if (cardType == SD_CARD_TYPE_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  uint64_t cardSize = (uint64_t)sd.card()->sectorCount() * 512ULL / (1024 * 1024);
  Serial.printf("Card Size: %lluMB\n", cardSize);
}

bool SD32_checkSDconnect() {
  cid_t cid;
  return sd.card()->readCID(&cid);
}

void SD32_getSDsize() {
  uint64_t cardSize = (uint64_t)sd.card()->sectorCount() * 512ULL / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// ============================================================================
// SESSION DIRECTORY MANAGEMENT
// ============================================================================

void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename, const char* prefix) {
  sessionNumber = 0;
  char searchPrefix[32];
  snprintf(searchPrefix, sizeof(searchPrefix), "%s_", prefix);
  int prefixLen = strlen(searchPrefix);

  SD32File root;
  if (root.open("/")) {
    SD32File entry;
    char nameBuf[64];
    while (entry.openNext(&root, O_RDONLY)) {
      entry.getName(nameBuf, sizeof(nameBuf));
      entry.close();

      // Check if filename matches pattern: prefix_NNN.csv
      if (strncmp(nameBuf, searchPrefix, prefixLen) == 0) {
        char* dotPos = strstr(nameBuf, ".csv");
        if (dotPos && dotPos > nameBuf + prefixLen) {
          // Extract number between prefix and .csv
          char numStr[8] = {0};
          int numLen = dotPos - (nameBuf + prefixLen);
          if (numLen > 0 && numLen < (int)sizeof(numStr)) {
            strncpy(numStr, nameBuf + prefixLen, numLen);
            int fileNum = atoi(numStr);
            if (fileNum >= sessionNumber) {
              sessionNumber = fileNum + 1;
            }
          }
        }
      }
    }
    root.close();
  }

  snprintf(csvFilename, 48, "/%s_%03d.csv", prefix, sessionNumber);
  Serial.print("Generated unique filename: ");
  Serial.println(csvFilename);
}

void SD32_createSessionDir(int &sessionNumber, char* sessionDirPath, const char* prefix) {
  sessionNumber = 0;

  char searchPattern[32];
  snprintf(searchPattern, sizeof(searchPattern), "%s_session_", prefix);
  int patternLen = strlen(searchPattern);

  SD32File root;
  if (root.open("/")) {
    SD32File entry;
    char nameBuf[64];
    while (entry.openNext(&root, O_RDONLY)) {
      if (entry.isDir()) {
        entry.getName(nameBuf, sizeof(nameBuf));
        if (strncmp(nameBuf, searchPattern, patternLen) == 0) {
          int num = atoi(nameBuf + patternLen);
          if (num >= sessionNumber) {
            sessionNumber = num + 1;
          }
        }
      }
      entry.close();
    }
    root.close();
  }

  snprintf(sessionDirPath, 48, "/%s_session_%03d", prefix, sessionNumber);
  if (!sd.exists(sessionDirPath)) {
    sd.mkdir(sessionDirPath);
    Serial.printf("[SD] Created session directory: %s\n", sessionDirPath);
  }
}

void SD32_generateFilenameInDir(char* filepath, const char* dirPath, const char* prefix, int index, const char* ext) {
  if (index >= 0) {
    snprintf(filepath, 48, "%s/%s_%d.%s", dirPath, prefix, index, ext);
  } else {
    snprintf(filepath, 48, "%s/%s.%s", dirPath, prefix, ext);
  }
}

// ============================================================================
// CSV LOGGING
// ============================================================================

void SD32_createCSVFile(const char* csvFilename, const char* csvHeader) {
  SD32File dataFile;
  if (dataFile.open(csvFilename, O_WRONLY | O_CREAT | O_TRUNC)) {
    dataFile.println(csvHeader);
    dataFile.flush();
    dataFile.close();
    Serial.printf("[SD] CSV file created: %s\n", csvFilename);
  } else {
    Serial.println("[SD] ERROR: Could not create CSV file!");
  }
}

void SD32_appendBulkDataToCSV(const char* filepath, AppenderFunc* appenders, void** dataArray, size_t count) {
  SD32File file;
  if (!file.open(filepath, O_WRONLY | O_CREAT | O_APPEND)) {
    Serial.println("[SD] ERROR: Could not open file!");
    return;
  }

  for (size_t i = 0; i < count; i++) {
    appenders[i](file, dataArray[i]);
  }

  file.println();
  file.flush();
  file.close();
}

// ============================================================================
// PERSISTENT FILE FUNCTIONS
// ============================================================================

bool SD32_openPersistentFile(const char* filepath) {
  if (_persistentFileOpen) {
    Serial.println("[SD] Persistent file already open");
    return true;
  }

  if (!_persistentFile.open(filepath, O_WRONLY | O_CREAT | O_APPEND)) {
    Serial.println("[SD] ERROR: Could not open persistent file!");
    _persistentFileOpen = false;
    return false;
  }

  _persistentFileOpen = true;
  _lastFlushTime = millis();
  strncpy(_persistentFilePath, filepath, sizeof(_persistentFilePath) - 1);
  _persistentFilePath[sizeof(_persistentFilePath) - 1] = '\0';
  Serial.printf("[SD] Persistent file opened: %s\n", filepath);
  return true;
}

void SD32_closePersistentFile() {
  if (_persistentFileOpen && _persistentFile.isOpen()) {
    _persistentFile.flush();
    _persistentFile.close();
    _persistentFileOpen = false;
    _persistentFilePath[0] = '\0';
    Serial.println("[SD] Persistent file closed");
  }
}

bool SD32_isPersistentFileOpen() {
  return _persistentFileOpen;
}

void SD32_flushPersistentFile() {
  if (_persistentFileOpen && _persistentFile.isOpen()) {
    _persistentFile.flush();
    _lastFlushTime = millis();
  }
}

void SD32_appendBulkDataPersistent(AppenderFunc* appenders, void** dataArray, size_t count, unsigned long flushIntervalMs, unsigned long closeIntervalMs) {
  if (!_persistentFileOpen || !_persistentFile.isOpen()) {
    Serial.println("[SD] ERROR: Persistent file not open!");
    return;
  }

  for (size_t i = 0; i < count; i++) {
    appenders[i](_persistentFile, dataArray[i]);
  }

  _persistentFile.println();

  unsigned long now = millis();
  if (flushIntervalMs == 0 || (now - _lastFlushTime >= flushIntervalMs)) {
    _persistentFile.flush();
    _lastFlushTime = now;
  }
  if (closeIntervalMs > 0 && (now - _lastCloseTime >= closeIntervalMs)) {
    _persistentFile.flush();
    _persistentFile.close();
    // Reopen the file
    if (!_persistentFile.open(_persistentFilePath, O_WRONLY | O_CREAT | O_APPEND)) {
      _persistentFileOpen = false;
      Serial.println("[SD] ERROR: Could not reopen persistent file after cycle!");
    }
    _lastCloseTime = now;
  }
}

// ============================================================================
// BINARY LOGGING
// ============================================================================

void SD32_createBinFile(const char* filepath, const char* nodeName, uint16_t entrySize) {
  SD32File f;
  if (f.open(filepath, O_WRONLY | O_CREAT | O_TRUNC)) {
    BinFileHeader hdr = {};
    hdr.magic = 0x42504C47;  // "BPLG"
    hdr.version = 1;
    hdr.entrySize = entrySize;
    hdr.headerSize = sizeof(BinFileHeader);
    strncpy(hdr.nodeName, nodeName, sizeof(hdr.nodeName) - 1);

    f.write((uint8_t*)&hdr, sizeof(hdr));
    f.flush();
    f.close();
    Serial.printf("[SD] BIN file created: %s (entrySize=%u)\n", filepath, entrySize);
  } else {
    Serial.println("[SD] ERROR: Could not create BIN file!");
  }
}

void SD32_appendBinaryPersistent(const void* data, size_t dataSize, unsigned long flushIntervalMs, unsigned long closeIntervalMs) {
  if (!_persistentFileOpen || !_persistentFile.isOpen()) {
    Serial.println("[SD] ERROR: Persistent file not open!");
    return;
  }

  _persistentFile.write((const uint8_t*)data, dataSize);

  unsigned long now = millis();
  if (flushIntervalMs == 0 || (now - _lastFlushTime >= flushIntervalMs)) {
    _persistentFile.flush();
    _lastFlushTime = now;
  }
  if (closeIntervalMs > 0 && (now - _lastCloseTime >= closeIntervalMs)) {
    _persistentFile.flush();
    _persistentFile.close();
    if (!_persistentFile.open(_persistentFilePath, O_WRONLY | O_CREAT | O_APPEND)) {
      _persistentFileOpen = false;
      Serial.println("[SD] ERROR: Could not reopen persistent file after cycle!");
    }
    _lastCloseTime = now;
  }
}
