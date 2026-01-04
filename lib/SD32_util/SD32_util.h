#include <cstdint>
// declare class and namespace of file system libary (include and call in .cpp src file)
namespace fs{
class FS;
};

// ------- SD CARD LOGGING - APPENDER ARCHITECTURE
typedef void (*AppenderFunc)(File&, void*);
void logDataToSD(const char* filename, AppenderFunc* appenders, void** dataArray, size_t count);

void getSDsize();
void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs,bool &sdCardReady);

void SD32_writeFile(fs::FS &fs, const char *path, const char *message);
void SD32_appendFile(fs::FS &fs, const char *path, const char *message);

void SD32_listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void SD32_createDir(fs::FS &fs, const char *path);
void SD32_removeDir(fs::FS &fs, const char *path);
void SD32_readFile(fs::FS &fs, const char *path);
void SD32_renameFile(fs::FS &fs, const char *path1, const char *path2);
void SD32_deleteFile(fs::FS &fs, const char *path);
void SD32_testFileIO(fs::FS &fs, const char *path);

void SD32_createCSVFile(char* csvFilename, const char* csvHeader);
void SD32_generateUniqueFilename(int &sessionNumber,char* csvFilename);