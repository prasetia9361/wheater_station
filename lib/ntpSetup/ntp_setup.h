// ntp_setup.h
#ifndef NTP_SETUP_H
#define NTP_SETUP_H

#include <Arduino.h>
#include <time.h>

class NTPSetup {
public:
    NTPSetup(const char* timezone, const char* server1, const char* server2 = nullptr, const char* server3 = nullptr);
    bool initialize();
    void update();
    bool isInitialized();
    bool getLocalTime(struct tm* timeinfo);
    String getTimeString();
    String getDateString();
    String getDateTimeString();
    void setUpdateInterval(unsigned long interval);
    
private:
    const char* timezone;
    const char* server1;
    const char* server2;
    const char* server3;
    bool initialized;
    unsigned long lastUpdate;
    unsigned long updateInterval;
    
    bool syncTime();
};

#endif