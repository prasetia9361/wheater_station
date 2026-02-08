#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <Wire.h>
#include <time.h>
#include "wifi_setup.h"
#include "clientServer.h"
#include "storage.h"
#include "ntp_setup.h"
#include "mqttSetup.h"

#define SDA_PIN         21
#define SCL_PIN         22

#define BAT_VOLT_PIN    32  // ADC1_CH1 (47k & 10k Divider)

// Safety Thresholds
#define CHARGE_THRESHOLD_VOLT   12.7 // LiFePO4 Charging
#define LOW_BAT_THRESHOLD       11.1 // LiFePO4 Critical Low

// NTP Server Configuration
#define NTP_TIMEZONE "WIB-7"
#define NTP_SERVER1 "id.pool.ntp.org"
#define NTP_SERVER2 "id.pool.ntp.org"
#define NTP_SERVER3 "time.google.com"

// --- Calibration Constants ---
// Voltage Divider: (R1+R2)/R2 => (10k+1k+1k)/1k = 11
const float VOLTAGE_MULTIPLIER = 11; 

// Global Variables
bool chargingStatus = false;
bool lowBatteryStatus = false; 
bool isLowBatReturn = false; 

enum wifiCindition {
    WIFI_DISCONNECTED,
    WIFI_STACONNECTED,
    WIFI_APCONNECTED
};

wifiCindition wifiStatus = WIFI_DISCONNECTED;

float batteryVoltage = 0.0;

WifiSetup *wifiSetup;
clientServer *webServer;
storage *memory;
NTPSetup *ntpSetup;
MQTTSetup *mqttClient;

// Sensor data structure
struct SensorData {
    float temperature = 0.0;
    float humidity = 0.0;
    float pressure = 0.0;
    float windSpeed = 0.0;
    int windDirection = 0;
    float rainfall = 0.0;
    int uvIndex = 0;
} currentSensorData;

float previousError = 0;
float integral = 0;

// Function declarations
void handleMqttCommand(const JsonDocument& command);
void initSensors();
void readSensors();
void appWifi(void *param);
void appSensors(void *param);

SemaphoreHandle_t _semaphore;
SemaphoreHandle_t _sensorDataMutex;

void loop() {
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    wifiSetup = new WifiSetup();
    memory = new storage();
    webServer = new clientServer(memory);
    ntpSetup = new NTPSetup(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
    mqttClient = new MQTTSetup();

    Serial.println("=== Weather Station Initialization ===");

    _semaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(_semaphore);
    
    _sensorDataMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(_sensorDataMutex);

    Serial.println("initialising pin sensor");
    analogReadResolution(12); // 0 - 4095
    pinMode(BAT_VOLT_PIN, INPUT);
    
    Wire.begin(SDA_PIN, SCL_PIN);
    initSensors();

    // WiFi Task (Core 0)
    TaskHandle_t taskWifi;
    xTaskCreatePinnedToCore(
        appWifi,
        "appWifi",
        10000,
        NULL,
        2,
        &taskWifi,
        0
    );

    // Sensor Reading Task (Core 1)
    TaskHandle_t taskSensors;
    xTaskCreatePinnedToCore(
        appSensors,
        "appSensors",
        10000,
        NULL,
        1,
        &taskSensors,
        1
    );

    Serial.println("=== Setup Complete ===\n");
}

void initSensors() {
    Serial.println("[SENSOR] Initializing sensors...");
    // TODO: Initialize your specific sensors here
    // Examples: BME280 (temperature, humidity, pressure), 
    // anemometer, rain gauge, UV sensor, etc.
    Serial.println("[SENSOR] Sensors initialized");
}

void readSensors() {
    // Read sensor values
    // TODO: Implement actual sensor reading based on your hardware
    // For now, using simulated values for demonstration
    
    if (xSemaphoreTake(_sensorDataMutex, portMAX_DELAY)) {
        // Simulated sensor reading - replace with actual sensor code
        currentSensorData.temperature = 25.5 + (random(-10, 10) / 10.0);
        currentSensorData.humidity = 60.0 + random(-5, 5);
        currentSensorData.pressure = 1013.25;
        currentSensorData.windSpeed = 5.2;
        currentSensorData.windDirection = random(0, 360);
        currentSensorData.rainfall = 0.0;
        currentSensorData.uvIndex = 5;
        
        xSemaphoreGive(_sensorDataMutex);
    }
}

void handleMqttCommand(const JsonDocument& command) {
    Serial.println("[MQTT] Processing command...");
    
    if (command["command"].is<String>()) {
        String cmd = command["command"].as<String>();
        
        if (cmd == "restart") {
            Serial.println("[MQTT] Received restart command");
            delay(1000);
            ESP.restart();
        }
        else if (cmd == "set_interval") {
            if (command["interval"].is<unsigned long>()) {
                unsigned long interval = command["interval"].as<unsigned long>();
                mqttClient->setPublishInterval(interval);
                Serial.print("[MQTT] Publish interval set to ");
                Serial.print(interval);
                Serial.println(" ms");
            }
        }
        else if (cmd == "ota_update") {
            Serial.println("[MQTT] OTA update command received");
            // TODO: Implement OTA update logic
        }
        else {
            Serial.print("[MQTT] Unknown command: ");
            Serial.println(cmd.c_str());
        }
    }
}

void appWifi(void *param) {
    
    // Initialize SPIFFS before accessing files
    if (!memory->init()) {
        Serial.println("[ERROR] SPIFFS Mount Failed!");
        vTaskDelete(NULL);
        return;
    }
    memory->readWifi();
    
    // Set MQTT command callback
    mqttClient->setCommandCallback(handleMqttCommand);
    
    if (memory->getSsid() != "" && memory->getID_Device() != "") {
        wifiStatus = WIFI_DISCONNECTED;
    } else {
        wifiSetup->disconnectSTA();
        wifiSetup->connectAP();
        wifiSetup->setupWiFiAP("wheater_Station_AP", "BTGsmart");
        webServer->begin();
        wifiStatus = WIFI_APCONNECTED;
        Serial.println("[WiFi] AP Mode activated - waiting for configuration");
    }

    String lastCommand = "";
    unsigned long lastBatteryCheck = 0;
    unsigned long lastMqttPublish = 0;
    unsigned long lastStatusPublish = 0;
    bool ntpInitialized = false;
    
    // Local variables for data snapshot
    bool _charging = false;
    bool _lowbat = false;

    while (true) {
        // Update WiFi status
        if (WiFi.status() != WL_CONNECTED && WiFi.getMode() == WIFI_STA && wifiSetup->isConnectedAP() == false) {
            wifiStatus = WIFI_DISCONNECTED;
        } else if (WiFi.status() == WL_CONNECTED && WiFi.getMode() == WIFI_STA) {
            wifiStatus = WIFI_STACONNECTED;
        }

        // Battery monitoring every 500ms
        if (millis() - lastBatteryCheck > 500) {
            float adcMv = analogReadMilliVolts(BAT_VOLT_PIN);
            float voltRaw = (adcMv / 1000.0) * VOLTAGE_MULTIPLIER;
            
            if (batteryVoltage == 0) {
                batteryVoltage = voltRaw;
            } else {
                batteryVoltage = (0.1 * voltRaw) + (0.9 * batteryVoltage);
            }
            
            _lowbat = (batteryVoltage < LOW_BAT_THRESHOLD);

            Serial.print("[BATTERY] Voltage: ");
            Serial.print(batteryVoltage);
            Serial.print(" V | LowBat: ");
            Serial.println(_lowbat ? "YES" : "NO");
            
            xSemaphoreTake(_semaphore, portMAX_DELAY);
            lowBatteryStatus = _lowbat;
            xSemaphoreGive(_semaphore);
            lastBatteryCheck = millis();
        }

        switch (wifiStatus) {
            case WIFI_DISCONNECTED: {
                Serial.println("[WiFi] WiFi Lost. Reconnecting...");
                WiFi.disconnect();
                wifiSetup->connectSTA();
                wifiSetup->setupWiFiSTA(memory->getSsid().c_str(), memory->getPass().c_str());
                
                if (wifiSetup->isConnectedAP()) {
                    Serial.println("[WiFi] Failed to connect to STA, switching to AP Mode");
                    wifiStatus = WIFI_APCONNECTED;
                    webServer->begin();
                } else if (wifiSetup->isConnectedSTA()) {
                    Serial.println("[WiFi] Connected to STA");
                    wifiStatus = WIFI_STACONNECTED;
                    wifiSetup->setconnectedSTA(false);
                    
                    // Initialize NTP when WiFi is connected
                    if (!ntpInitialized && WiFi.status() == WL_CONNECTED) {
                        ntpSetup->initialize();
                        ntpInitialized = true;
                        Serial.println("[NTP] NTP initialized");
                    }
                }
                break;
            }
            
            case WIFI_STACONNECTED: {
                // Initialize NTP on first STA connection
                if (!ntpInitialized && WiFi.status() == WL_CONNECTED) {
                    ntpSetup->setUpdateInterval(3600000);
                    ntpSetup->initialize();
                    ntpInitialized = true;
                    Serial.println("[NTP] NTP initialized");
                }
                
                // Update NTP periodically
                if (ntpInitialized) {
                    ntpSetup->update();
                    ntpSetup->getDateTimeString();
                }
                
                // Connect to MQTT broker
                if (!mqttClient->isConnected()) {
                    mqttClient->begin(memory->getServerIP(), memory->getServerPort(),
                                     "weather_station", "your_mqtt_password",
                                     memory->getID_Device());
                    mqttClient->connect();
                }
                
                // Keep MQTT connection alive
                mqttClient->loop();
                
                // Publish sensor data periodically
                if (mqttClient->isConnected() && 
                    millis() - lastMqttPublish > mqttClient->getPublishInterval()) {
                    
                    if (xSemaphoreTake(_sensorDataMutex, portMAX_DELAY)) {
                        mqttClient->publishSensorData(
                            ntpSetup->getDateTimeString(),
                            currentSensorData.temperature,
                            currentSensorData.humidity,
                            currentSensorData.pressure,
                            currentSensorData.windSpeed,
                            currentSensorData.windDirection,
                            currentSensorData.rainfall,
                            currentSensorData.uvIndex,
                            batteryVoltage,
                            WiFi.RSSI()
                        );
                        xSemaphoreGive(_sensorDataMutex);
                    }
                    lastMqttPublish = millis();
                }
                
                // Publish status periodically (every 5 minutes)
                if (mqttClient->isConnected() && 
                    millis() - lastStatusPublish > 300000) {
                    
                    mqttClient->publishStatus(
                        "online",
                        "1.0.0",
                        millis() / 1000,
                        WiFi.localIP().toString(),
                        ESP.getFreeHeap(),
                        ntpSetup->getDateTimeString()
                    );
                    lastStatusPublish = millis();
                }
                break;
            }
            
            case WIFI_APCONNECTED: {
                if (webServer->isRebootNeeded()) {
                    wifiStatus = WIFI_DISCONNECTED;
                    webServer->setRebootNeeded(false);
                } else {
                    if (WiFi.getMode() == WIFI_STA) {
                        wifiSetup->setconnectedAP(false);
                        wifiSetup->disconnectSTA();
                        wifiSetup->connectAP();
                        wifiSetup->setupWiFiAP("wheater_Station_AP", "BTGsmart");
                        webServer->begin();
                    }
                }
                break;
            }
        }

        // DNS processing in AP mode
        if (WiFi.getMode() == WIFI_AP) {
            wifiSetup->loopDns();
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void appSensors(void *param) {
    Serial.println("[TASK] Sensor task started on core 1");
    
    unsigned long lastRead = 0;
    const unsigned long readInterval = 5000; // Read sensors every 5 seconds
    
    while (true) {
        if (millis() - lastRead >= readInterval) {
            readSensors();
            lastRead = millis();
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}