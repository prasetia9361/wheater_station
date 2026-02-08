#ifndef MQTT_SETUP_H
#define MQTT_SETUP_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

typedef void (*CommandCallback)(const JsonDocument&);

enum commandType {
    NONE,
    RESTART,
    SET_INTERVAL,
    OTA_UPDATE
};

class MQTTSetup {
private:
    PubSubClient client;
    WiFiClient espClient;
    
    const char* mqtt_server;
    int mqtt_port;
    const char* mqtt_user;
    const char* mqtt_password;
    String station_id;
    // commandType _modeCommand;
    
    bool connected;
    unsigned long lastPublish;
    unsigned long publishInterval;
    CommandCallback commandCallback;
    String timestamp;
    
    static MQTTSetup* instance;
    static void staticCallback(char* topic, byte* payload, unsigned int length);
    void handleMessage(char* topic, byte* payload, unsigned int length);
    void setInterval(unsigned long interval);

    String _otaUrl;
    bool _otaRequested = false;
    
public:
    MQTTSetup();
    
    bool begin(const char* server, int port, const char* user, 
               const char* password, const String& deviceID);
    bool connect();
    bool isConnected();
    void loop();

    commandType _modeCommand;
    
    // Publishing functions
    bool publishSensorData(float temperatureIn, float temperatureOut, float humidity, float pressure,
                           float windSpeed,
                           float rainfall, int uvIndex, float lux,
                           float batteryVoltage, int signalStrength = 0);
    
    bool publishStatus(const String& status, const String& fwVersion,
                      unsigned long uptime, const String& ipAddress,
                      int freeHeap = 0);
    
    // Command subscription
    void setCommandCallback(CommandCallback callback);
    void subscribe();
    
    void setPublishInterval(unsigned long interval);
    unsigned long getPublishInterval();
    void setTimeStamp(String _timestamp = ""){ timestamp = _timestamp; };

    bool isOTARequested() { return _otaRequested; }
    String getOTAUrl() { return _otaUrl; }
    void clearOTARequest() { _otaRequested = false; }
    
};

#endif
