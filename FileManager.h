#ifndef __FILE_MANAGER_H__
#define __FILE_MANAGER_H__
#include "SPIFFS.h"
#include "FS.h"

// define library for LittleFS manage
// #include <LittleFS.h>
// #define SPIFFS LittleFS

#define FORMAT_LITTLEFS_IF_FAILED true
class FileManager
{
private:
    // File paths to save input values permanently
    // const char wifiConfigPath[24] = "/config/wifiConfig.json";
    // const char deviceInfoPath[24] = "/config/deviceInfo.json";
    // const char paymentInfoPath[25] = "/config/paymentInfo.json";
    // const char boxIdPath[18] = "/config/boxId.txt";
    // const char bankAccountPath[24] = "/config/bankAccount.txt";

public:
    FileManager(){};
    ~FileManager(){};
    void init();
    String readFile(fs::FS &fs, const char *path);
    void writeFile(fs::FS &fs, const char *path, const char *message);
    void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
    void createDir(fs::FS &fs, const char *path);
    void removeDir(fs::FS &fs, const char *path);
    void appendFile(fs::FS &fs, const char *path, const char *message);

};

// Create pointer object for class EspSPIFFS
// extern FileManager fileManager;
#endif