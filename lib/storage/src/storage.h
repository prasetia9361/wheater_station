#ifndef STORAGE_H
#define STORAGE_H
#include <Arduino.h>

#include <SPIFFS.h>

#include <ArduinoJson.h>

#include <freertos/FreeRTOS.h>
#include "FS.h"

class storage {
private:
    typedef struct wifi
    {
        String ssid;
        String pass;
        String serverIP;
        String clientIP;
        int serverPort;
        String ID_Device;
    }wifi;
    wifi setWifi;

    SemaphoreHandle_t semaphore;
    
public:
    storage();
    ~storage();
    bool readWifi();
    String getSsid() { return setWifi.ssid; }
    String getPass() {return setWifi.pass;}
    const char* getServerIP() { return setWifi.serverIP.c_str(); }
    const char* getClientIP() { return setWifi.clientIP.c_str(); }
    int getServerPort() { return setWifi.serverPort; }
    String getID_Device() { return setWifi.ID_Device; }

    
    bool init();
    bool writeWifi(const String& ssid, const String& pass, const String& serverIP,const String& clientIP, int serverPort, const String& ID_Device = "");
};
#endif