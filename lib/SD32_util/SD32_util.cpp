
#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SD32_util.h>
// int dataPoint = 0;

void SD32_getSDsize(){
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs,bool &sdCardReady) {
  Serial.println("--- SD Card Initialization ---");
  Serial.print("Initializing SD card...");
  SPI.begin(sd_sck, sd_miso, sd_mosi, sd_cs);
  
  if (!SD.begin(sd_cs)) {
    Serial.println(" FAILED!");
    Serial.println("SD card logging disabled.");
    Serial.println("Check:");
    Serial.println("  - SD card is inserted");
    Serial.println("  - Connections are correct");
    Serial.println("  - SD card is formatted (FAT32)");
    sdCardReady = false; // Passed referece
    return;
  }
  Serial.println(" SUCCESS!");
  sdCardReady = true;
  
  // // Display card info
  uint8_t cardType = SD.cardType();
  Serial.print("Card Type: ");
  if (cardType == CARD_MMC) 
    Serial.println("MMC");
  else if (cardType == CARD_SD) 
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC) 
    Serial.println("SDHC");
  else 
    Serial.println("UNKNOWN");
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card Size: %lluMB\n", cardSize);
  

  // Let's leave this to the devs to decided
  // // Generate unique filename for this session
  // SD32_generateUniqueFilename(csvFil);
  
  // // Create new CSV file
  // Serial.println("Creating new CSV file...");
  // SD_32createCSVFile();
}

void SD32_createCSVFile(char* csvFilename, const char* csvHeader){
  File dataFile = SD.open((const char*)csvFilename, FILE_WRITE);
  
  if (dataFile) {
    // Updated CSV header with DateTime column
    dataFile.println(csvHeader);
    dataFile.flush();
    dataFile.close();
    Serial.println("[SD Card] CSV header created with DateTime column");
  } else {
    Serial.println("[SD Card] ERROR: Could not create CSV file!");
  }

}
// Pass unique file name in
void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename) {
  // Find the next available session number
  sessionNumber = 0;
  
  // Check existing files and find the highest session number
  File root = SD.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      String filename = entry.name();
      // Look for files matching pattern: datalog_NNN.csv
      if (filename.startsWith("datalog_") && filename.endsWith(".csv")) {
        // Extract the number
        int startIdx = 8;  // Length of "datalog_"
        int endIdx = filename.indexOf(".csv");
        if (endIdx > startIdx) {
          String numStr = filename.substring(startIdx, endIdx);
          int fileNum = numStr.toInt();
          if (fileNum >= sessionNumber) {
            sessionNumber = fileNum + 1;
          }
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }
  
  // Generate filename with 3-digit session number
  char buffer[30];
  sprintf(buffer, "/datalog_%03d.csv", sessionNumber);
  sprintf(csvFilename,buffer);

  
  Serial.print("Generated unique filename: ");
  Serial.println(csvFilename);
}


// Generic CSV Logging Function
void logDataToSD(const char* filename, AppenderFunc* appenders, void** dataArray, size_t count) {
  File dataFile = SD.open(filename, FILE_APPEND);

  if (!dataFile) {
    Serial.println("[SD Card] ERROR: Could not open file!");
    return;
  }

  // Call all appenders
  for (size_t i = 0; i < count; i++) {
    appenders[i](dataFile, dataArray[i]);
  }

  // End CSV line
  dataFile.println();

  dataFile.flush();
  dataFile.close();
}

// List Directory
void SD32_listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        SD32_listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// Make Directory folders
void SD32_createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

// remove Directory folders
void SD32_removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

// read
void SD32_readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

// Write
void SD32_writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Update
void SD32_appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

// rename
void SD32_renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

// delete
void SD32_deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

// Tester
void SD32_testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
  file.close();
}

