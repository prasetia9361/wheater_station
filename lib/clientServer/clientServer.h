#ifndef CLIENTSERVER_H
#define CLIENTSERVER_H

#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "../storage/src/storage.h"

#define WEB_SERVER_PORT 80

class Storage;

class clientServer {
private:
    storage *_memory;
    AsyncWebServer server;

    // Data Sensor
    float currentSpeed = 0; 
    long currentSteps = 0;
    bool isCharging = false;
    bool isLowBat = false;
    
    // Config Status
    bool shouldReboot = false;

    void indexHtml();
    void wifiCredentials();
    void firstInput();
    void notFound();

public:
    clientServer(storage *memory) : _memory(memory), server(WEB_SERVER_PORT) {}
    
    // Data Setter
    void setCurrnentSpeed(float speed){ currentSpeed = speed; }
    void setCurrentSteps(long steps){ currentSteps = steps; }
    void setBatteryStatus(bool charging, bool low) {
        isCharging = charging;
        isLowBat = low;
    }
    
    bool isRebootNeeded() { return shouldReboot; }
    void setRebootNeeded(bool reboot) { shouldReboot = reboot; }
    void begin();
};
#endif