#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <Wire.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MAX31865.h>
#include <LTR390.h>
#include "wifi_setup.h"
#include "clientServer.h"
#include "storage.h"
#include "ntp_setup.h"
#include "mqttSetup.h"

#define FRAMEWORK_VERSION "1.2.0"

// --- PIN DEFINITIONS ---
#define SDA_PIN         SDA
#define SCL_PIN         SCL

// MAX31865 (SPI)
#define MAX_CS_PIN      SS //CS
#define MAX_DI_PIN      MOSI // MOSI
#define MAX_DO_PIN      MISO // MISO
#define MAX_CLK_PIN     SCK // SCK

// RS485 Anemometer (Hardware Serial 2)
#define RS485_RX        18
#define RS485_TX        17
#define RS485_EN        16  // DE/RE Pin

// Rain Gauge (Hall Effect A3144)
#define RAIN_PIN        7

#define RREF      4300.0    // PT100 (430 for 3wire, 4300 for PT1000)
#define RNOMINAL  1000.0    // 100 for PT100, 1000 for PT1000

#define BAT_VOLT_PIN    2  // ADC1_CH1 (47k & 10k Divider)

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
float integrationTime = 0.00;

// ... (Variabel Global Lain) ...
bool chargingStatus = false;
bool lowBatteryStatus = false; 
bool isLowBatReturn = false; 

enum wifiCindition {
    WIFI_DISCONNECTED,
    WIFI_STACONNECTED,
    WIFI_APCONNECTED
};

wifiCindition wifiStatus = WIFI_DISCONNECTED;

// --- SENSOR OBJECTS ---
Adafruit_BME280 bme;
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(MAX_CS_PIN, MAX_DI_PIN, MAX_DO_PIN, MAX_CLK_PIN);
LTR390 ltr;

WifiSetup *wifiSetup;
clientServer *webServer;
storage *memory;
NTPSetup *ntpSetup;
MQTTSetup *mqttClient;



// --- RAIN GAUGE VARIABLES ---
volatile long rainTips = 0;
const float RAIN_FACTOR = 0.2794; // mm per tip (sesuaikan dengan spesifikasi rain gauge Anda)

// --- ANEMOMETER RS485 ---
// Command Modbus standard untuk baca Wind Speed (Address 0x01)
byte windRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};

// --- UPDATED SENSOR DATA STRUCT ---
struct SensorData {
    float temperature_in = 0.0; // temp of bme280
    float temperature_out = 0.0; // temp of max31865
    float humidity = 0.0;
    float pressure = 0.0;
    float windSpeed = 0.0;
    float rainfall = 0.0;
    int uvIndex = 0;
    float lux = 0.0;
    float batteryVoltage = 0.0;
} currentSensorData;

// Interrupt Service Routine untuk Hujan
void IRAM_ATTR rainInterrupt() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTime > 200) { // Debounce 200ms
        rainTips++;
        lastTime = currentTime;
    }
}

// Function declarations
void initSensors();
void readSensors();
void performOTA(String url);
void handleMqttCommand();
void appWifi(void *param);
void appSensors(void *param);

SemaphoreHandle_t _semaphore;
SemaphoreHandle_t _sensorDataMutex;

void setup() {
    Serial.begin(115200);
    _semaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(_semaphore);
    
    _sensorDataMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(_sensorDataMutex);

    analogReadResolution(12); // 0 - 4095
    pinMode(BAT_VOLT_PIN, INPUT);
    
    // Wire.begin(SDA_PIN, SCL_PIN);
    initSensors();
    Serial.println("Init ToF Sensors...");

    TaskHandle_t taskWifi;
    xTaskCreatePinnedToCore(
        appWifi,
        "appWifi",
        10000,
        NULL,
        1,
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


}

void loop() {
delay(1000);
}

// --- INITIALIZATION ---
void initSensors() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
    pinMode(RS485_EN, OUTPUT);
    digitalWrite(RS485_EN, LOW); // Mode Receive

    // 1. Init BME280
    if (!bme.begin(0x76, &Wire)) {
        Serial.println("[SENSOR] BME280 not found!");
    }

    // 2. Init MAX31865 (2-wire PT1000)
    max31865.begin(MAX31865_2WIRE); 

    // 3. Init LTR390
    if (!ltr.begin()) {
        Serial.println("[SENSOR] LTR390 not found!");
    }

    ltr.setResolution(2);
    integrationTime = ltr.getIntegrationTime();

    
    
    ltr.enable();

    // 4. Init Rain Gauge
    pinMode(RAIN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainInterrupt, FALLING);

    Serial.println("[SENSOR] All sensors initialized");
}

// --- READING LOGIC ---
float readWindSpeed() {
    digitalWrite(RS485_EN, HIGH); // Mode Transmit
    Serial2.write(windRequest, sizeof(windRequest));
    Serial2.flush();
    digitalWrite(RS485_EN, LOW); // Mode Receive

    delay(100);

    byte response[7];
    int i = 0;
    while (Serial2.available() && i < 7) {
        response[i++] = Serial2.read();
    }

    if (i == 7 && response[0] == 0x01) {
        // Data speed ada di byte 3 & 4 (Hex to Decimal)
        int rawSpeed = (response[3] << 8) | response[4];
        return rawSpeed / 10.0; // Biasanya anemometer mengembalikan nilai speed * 10
    }
    return 0.0;
}

enum LTR390State {
    LTR390_IDLE,
    LTR390_READING_LUX,
    LTR390_WAITING_LUX,
    LTR390_READING_UV,
    LTR390_WAITING_UV,
    LTR390_COMPLETE
};

LTR390State ltr390State = LTR390_IDLE;
unsigned long lastModeChange = 0;
unsigned long lastReadTimeLux = 0;
unsigned long lastReadTimeUV = 0;
float _lux = 0.0;
int _uvIndex = 0;
bool switchToLux = false;

void readSensors() {
    unsigned long currentTime = millis();

    if (!switchToLux)
    {
        ltr.setUVSMode();
        if ( currentTime - lastReadTimeUV >= integrationTime + 50)
        {
            ltr.getUVSData();
            _uvIndex = ltr.getUVIndex();
            lastReadTimeUV = currentTime;
            switchToLux = true;
        }
    }else{
        ltr.setALSMode();
        if ( currentTime - lastReadTimeLux >= (integrationTime + 50) * 2)
        {
            ltr.getALSData();
            _lux = ltr.getLUX();
            lastReadTimeLux = currentTime;
            switchToLux = false;
        }
    }
    
    if (xSemaphoreTake(_sensorDataMutex, portMAX_DELAY)) {
        currentSensorData.temperature_in = bme.readTemperature();
        currentSensorData.humidity = bme.readHumidity();
        currentSensorData.pressure = bme.readPressure() / 100.0F;

        uint16_t rtd = max31865.readRTD();
        currentSensorData.temperature_out = max31865.temperature(RNOMINAL, RREF);

        currentSensorData.windSpeed = readWindSpeed();

        currentSensorData.rainfall = rainTips * RAIN_FACTOR;

        currentSensorData.uvIndex = _uvIndex;
        currentSensorData.lux = _lux;

        Serial.printf("Temp: %.2f | Wind: %.2f | Rain: %.2f mm | UV: %d | Lux: %.2f\n", 
                      currentSensorData.temperature_out, 
                      currentSensorData.windSpeed, 
                      currentSensorData.rainfall,
                      currentSensorData.uvIndex,
                      currentSensorData.lux);
        
        xSemaphoreGive(_sensorDataMutex);
    }
}

void performOTA(String url) {
    Serial.println("[OTA] Starting Update...");
    
    // Matikan MQTT loop sementara agar tidak mengganggu download
    // mqttClient->loop() tidak akan dipanggil selama proses ini
    
    WiFiClientSecure client;
    client.setInsecure(); // Gunakan ini jika server OTA menggunakan HTTPS tapi tidak ingin cek sertifikat

    // Callback progress (opsional)
    httpUpdate.onProgress([](size_t progress, size_t total) {
        static int lastPercent = -1;
        int percent = (progress * 100) / total;
        if (percent % 10 == 0 && percent != lastPercent) {
            Serial.printf("[OTA] Progress: %d%%\n", percent);
            lastPercent = percent;
        }
    });

    t_httpUpdate_return ret = httpUpdate.update(client, url);

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            Serial.printf("[OTA] Update Failed! Error (%d): %s\n", 
                          httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            mqttClient->_modeCommand = NONE;
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("[OTA] No Updates Available");
            mqttClient->_modeCommand = NONE;
            break;
        case HTTP_UPDATE_OK:
            Serial.println("[OTA] Update Success! Rebooting...");
            ESP.restart();
            break;
    }
}

void handleMqttCommand(){
    switch (mqttClient->_modeCommand ){
        case RESTART:{
            Serial.println("[MQTT] Received restart command");
            delay(1000);
            ESP.restart();
            break;
        }
            
        case SET_INTERVAL:{
            mqttClient->_modeCommand = NONE;
            break;
        }
        case OTA_UPDATE:{
            if (mqttClient->isOTARequested()) {
                String url = mqttClient->getOTAUrl();
                mqttClient->clearOTARequest(); // Reset flag agar tidak loop
                performOTA(url);
            }
            break;
        }
        default:
            break;
    }
    
}

void appWifi(void *param) {
    wifiSetup = new WifiSetup();
    memory = new storage();
    webServer = new clientServer(memory);
    ntpSetup = new NTPSetup(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
    mqttClient = new MQTTSetup();
    Serial.println("WiFi Task Started on Core 0");
    // Inisialisasi SPIFFS duluan sebelum akses file
    if (!memory->init()) {
        Serial.println("SPIFFS Mount Failed!");
        vTaskDelete(NULL);
        return;
    }
    
    memory->readWifi();
    
    if (memory->getSsid() != "" && memory->getID_Device() != "")
    {
        wifiStatus = WIFI_DISCONNECTED;
    }else{
        wifiSetup->disconnectSTA();
        wifiSetup->connectAP();
        wifiSetup->setupWiFiAP("Wheater_Station_AP", "BTGsmart");
        

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Beri nafas untuk lwIP
        
        webServer->begin();
        wifiStatus = WIFI_APCONNECTED;
        Serial.println("Web server started in AP mode");
    }

    String lastCommand = "";
    unsigned long lastBatteryCheck = 0;
    unsigned long lastTelemetry = 0;
    unsigned long lastMqttPublish = 0;
    unsigned long lastStatusPublish = 0;
    bool ntpInitialized = false;
    
    // Variabel lokal untuk menampung data snapshot dari global
    bool _charging = false;
    bool _lowbat = false;

    while (true) {
        if (WiFi.status() != WL_CONNECTED  && WiFi.getMode() == WIFI_STA && wifiSetup->isConnectedAP() == false){
            wifiStatus = WIFI_DISCONNECTED;
        }else if(WiFi.status() == WL_CONNECTED && WiFi.getMode() == WIFI_STA){
            wifiStatus = WIFI_STACONNECTED;
        }
        

        if (millis() - lastBatteryCheck > 500) {
                        
            float adcMv = analogReadMilliVolts(BAT_VOLT_PIN); 
            float voltRaw = (adcMv / 1000.0) * VOLTAGE_MULTIPLIER;
            // static float filteredVolt = 0;
            if (currentSensorData.batteryVoltage == 0) currentSensorData.batteryVoltage = voltRaw;
            else currentSensorData.batteryVoltage = (0.1 * voltRaw) + (0.9 * currentSensorData.batteryVoltage);
            // _charging = (batteryVoltage > CHARGE_THRESHOLD_VOLT);
            _lowbat = (currentSensorData.batteryVoltage < LOW_BAT_THRESHOLD);

            Serial.print("[BATTERY] Voltage: ");
            Serial.print(currentSensorData.batteryVoltage);
            Serial.print(" V | LowBat: ");
            Serial.println(_lowbat ? "YES" : "NO");
            // chargingStatus = _charging;
            xSemaphoreTake(_semaphore, portMAX_DELAY);
            lowBatteryStatus = _lowbat;
            xSemaphoreGive(_semaphore);
            lastBatteryCheck = millis();
        }

        switch (wifiStatus)
        {
            case WIFI_DISCONNECTED:
            {
                Serial.println("WiFi Lost. Reconnecting...");
                WiFi.disconnect(); 
                wifiSetup->connectSTA();
                wifiSetup->setupWiFiSTA(memory->getSsid().c_str(), memory->getPass().c_str());
                if (wifiSetup->isConnectedAP() == true)
                {
                    Serial.println("\nGagal konek ke : " + String(wifiSetup->counterConnect()));
                    wifiStatus = WIFI_APCONNECTED;
                    Serial.println("Switching to AP Mode :"+ String(wifiStatus));
                }else if (wifiSetup->isConnectedSTA() == true)
                {
                    wifiStatus = WIFI_STACONNECTED;
                    wifiSetup->setconnectedSTA(false);
                }
                break;
            }
            case WIFI_STACONNECTED:{
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
                    mqttClient->setTimeStamp(ntpSetup->getDateTimeString());
                }
                
                // Connect to MQTT broker
                if (!mqttClient->isConnected()) {
                    mqttClient->begin("broker.emqx.io", 1883,
                                        "weather_station", "your_mqtt_password",
                                        memory->getID_Device());
                    mqttClient->connect();
                    mqttClient->publishStatus(
                        "online",
                        FRAMEWORK_VERSION,
                        0,
                        WiFi.localIP().toString(),
                        ESP.getFreeHeap()
                    );
                }

                if (mqttClient->isConnected() && mqttClient->_modeCommand != OTA_UPDATE){
                    if ( millis() - lastMqttPublish > mqttClient->getPublishInterval()){
                        if (xSemaphoreTake(_sensorDataMutex, portMAX_DELAY)) {
                            mqttClient->publishSensorData(
                                currentSensorData.temperature_in,
                                currentSensorData.temperature_out,
                                currentSensorData.humidity,
                                currentSensorData.pressure,
                                currentSensorData.windSpeed,
                                currentSensorData.rainfall,
                                currentSensorData.uvIndex,
                                currentSensorData.lux,
                                currentSensorData.batteryVoltage,
                                WiFi.RSSI()
                            );
                            xSemaphoreGive(_sensorDataMutex);
                        }
                        lastMqttPublish = millis();
                    }

                    if ( millis() - lastMqttPublish > mqttClient->getPublishInterval() * 5){
                        mqttClient->publishStatus(
                            "online",
                            FRAMEWORK_VERSION,
                            millis() / 1000,
                            WiFi.localIP().toString(),
                            ESP.getFreeHeap()
                        );
                        lastStatusPublish = millis();
                    }

                    mqttClient->loop();
                }

                handleMqttCommand();

              break;
            }
            case WIFI_APCONNECTED:
            {
                if (webServer->isRebootNeeded())
                {
                    wifiStatus = WIFI_DISCONNECTED;
                    webServer->setRebootNeeded(false);
                }else{
                    if (WiFi.getMode() == WIFI_STA)
                    {
                    wifiSetup->setconnectedAP(false);
                    wifiSetup->disconnectSTA();
                    wifiSetup->connectAP();
                    wifiSetup->setupWiFiAP("Wheater_Station_AP", "BTGsmart");
                    
                    // // Wait for WiFi to stabilize
                    // delay(500);
                    // vTaskDelay(500 / portTICK_PERIOD_MS);
                    
                    webServer->begin();
                    }
                }
                break;
            }
        }

        if (WiFi.getMode() == WIFI_AP)
        {
            wifiSetup->loopDns();
        }
        vTaskDelay(10); 
    }
}

void appSensors(void *param) {
    unsigned long lastRead = 0;
    const unsigned long readInterval = 5000; 

    while (true) {
        if (millis() - lastRead >= readInterval) {
            readSensors();
            lastRead = millis();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}