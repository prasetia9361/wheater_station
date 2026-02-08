// ntp_setup.cpp
#include "ntp_setup.h"
#include <WiFi.h>

NTPSetup::NTPSetup(const char* timezone, const char* server1, const char* server2, const char* server3) {
    this->timezone = timezone;
    this->server1 = server1;
    this->server2 = server2;
    this->server3 = server3;
    this->initialized = false;
    this->lastUpdate = 0;
    this->updateInterval = 3600000; // Default 1 hour
}

bool NTPSetup::initialize() {
    if (WiFi.status() != WL_CONNECTED) {
        // Serial.println("WiFi not connected - cannot initialize NTP");
        return false;
    }
    
    // Serial.println("Initializing NTP...");
    
    // Configure NTP with timezone and servers
    if (server3 != nullptr) {
        configTime(0, 0, server1, server2, server3);
    } else if (server2 != nullptr) {
        configTime(0, 0, server1, server2);
    } else {
        configTime(0, 0, server1);
    }
    
    // Set timezone
    setenv("TZ", timezone, 1);
    tzset();
    
    // Wait for time to be set
    if (syncTime()) {
        initialized = true;
        lastUpdate = millis();
        // Serial.println("NTP initialized successfully!");
        
        // Display current time
        struct tm timeinfo;
        if (::getLocalTime(&timeinfo)) {
            // Serial.printf("Current time: %s", getDateTimeString().c_str());
        }
        
        return true;
    } else {
        // Serial.println("Failed to initialize NTP");
        return false;
    }
}

void NTPSetup::update() {
    if (!initialized) {
        return;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        // Serial.println("WiFi disconnected - skipping NTP update");
        return;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastUpdate >= updateInterval) {
        // Serial.println("Updating NTP time...");
        
        // Re-configure NTP servers
        if (server3 != nullptr) {
            configTime(0, 0, server1, server2, server3);
        } else if (server2 != nullptr) {
            configTime(0, 0, server1, server2);
        } else {
            configTime(0, 0, server1);
        }
        
        lastUpdate = currentTime;
        // Serial.println("NTP update requested");
    }
}

bool NTPSetup::isInitialized() {
    return initialized;
}

bool NTPSetup::getLocalTime(struct tm* timeinfo) {
    if (!initialized) {
        return false;
    }
    return ::getLocalTime(timeinfo);
}

String NTPSetup::getTimeString() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Time not available";
    }
    
    char timeStr[10];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
    return String(timeStr);
}

String NTPSetup::getDateString() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Date not available";
    }
    
    char dateStr[12];
    strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &timeinfo);
    return String(dateStr);
}

String NTPSetup::getDateTimeString() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "DateTime not available\n";
    }
    
    char dateTimeStr[22];
    strftime(dateTimeStr, sizeof(dateTimeStr), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(dateTimeStr);
}

void NTPSetup::setUpdateInterval(unsigned long interval) {
    this->updateInterval = interval;
}

bool NTPSetup::syncTime() {
    struct tm timeinfo;
    int attempts = 0;
    const int maxAttempts = 10;
    
    // Serial.print("Waiting for NTP sync");
    while (!::getLocalTime(&timeinfo) && attempts < maxAttempts) {
        // Serial.print(".");
        delay(1000);
        attempts++;
    }
    // Serial.println();
    
    if (attempts < maxAttempts) {
        // Serial.println("NTP time synchronized!");
        return true;
    } else {
        // Serial.println("Failed to obtain time from NTP server");
        return false;
    }
}