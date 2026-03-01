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
#define MQTT_SERVER       "broker.emqx.io"
#define MQTT_PORT         1883
#define MQTT_USER         "weather_station"
#define MQTT_PASSWORD     "your_mqtt_password"

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
#define RS485_EN        4  // DE/RE Pin

// Rain Gauge (Hall Effect A3144)
#define RAIN_PIN        7

// Battery Voltage Measurement
#define BATTERY_PIN    2  // GPIO 2 ADC1_CH1 (47k & 10k Divider)

// Push Button
#define BUTTON_RESET   42  // GPIO 42

#define BUTTON_ENABLE_AP 41 // GPIO 41

#define BUTTON_MAINTENANCE 40 // GPIO 40

#define LED_AP 21 // GPIO 21

// --- PT100 Constants ---
#define RREF      430.0    // PT100 (430 for 3wire, 4300 for PT1000)
#define RNOMINAL  100.0    // 100 for PT100, 1000 for PT1000

// Safety Thresholds
#define CHARGE_THRESHOLD_VOLT   12.7 // LiFePO4 Charging
#define LOW_BAT_THRESHOLD       11.1 // LiFePO4 Critical Low

const float R1 = 47000.0;   // 47k
const float R2 = 10000.0;   // 10k

// Faktor Kalibrasi
// Ubah nilai ini jika pembacaan serial monitor berbeda dengan Multimeter
// Jika di serial 12.5V tapi di multimeter 12.8V, naikkan sedikit.
const float calibrationFactor = 1.0625; 
  long totalMilliVolts = 0;
  int sampleCount = 20;
  int counterSample = 0;

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
    WIFI_APCONNECTED,
    MAINTENANCE_MODE
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

SemaphoreHandle_t _sensorDataMutex;

void setup() {
    Serial.begin(115200);
    
    _sensorDataMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(_sensorDataMutex);

    wifiSetup = new WifiSetup();
    memory = new storage();
    webServer = new clientServer(memory);
    ntpSetup = new NTPSetup(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
    mqttClient = new MQTTSetup();

    analogReadResolution(12); // 0 - 4095
    analogSetAttenuation(ADC_11db);
    pinMode(BATTERY_PIN, INPUT);
    pinMode(BUTTON_RESET, INPUT_PULLUP);
    pinMode(BUTTON_ENABLE_AP, INPUT_PULLUP);
    pinMode(BUTTON_MAINTENANCE, INPUT_PULLUP);
    pinMode(LED_AP, OUTPUT);
    digitalWrite(LED_AP, LOW);
    
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
    Serial2.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);
    pinMode(RS485_EN, OUTPUT);
    digitalWrite(RS485_EN, LOW); // Mode Receive

    // 1. Init BME280
    if (!bme.begin(0x76, &Wire)) {
        Serial.println("[SENSOR] BME280 not found!");
    }

    // 2. Init MAX31865 (2-wire PT1000)
    max31865.begin(MAX31865_3WIRE); 

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
    digitalWrite(RS485_EN, HIGH); 
    delayMicroseconds(50); // Stabilisasi
    Serial2.write(windRequest, sizeof(windRequest));
    Serial2.flush();              
    delayMicroseconds(500); // JEDA KRUSIAL: Menunggu bit terakhir keluar kabel
    digitalWrite(RS485_EN, LOW);  

    // RECEIVE
    uint8_t response[20];
    int bytesRead = 0;
    unsigned long startTime = millis();

    // Tunggu data selama 1 detik
    while (millis() - startTime < 1000) {
        if (Serial2.available()) {
        response[bytesRead++] = Serial2.read();
        startTime = millis(); // Reset timeout tiap ada byte masuk
        }
    }

    if (bytesRead > 0) {
        // Serial.print("HASIL: ");
        // for (int i = 0; i < bytesRead; i++) {
        // if (response[i] < 0x10) Serial.print("0");
        // Serial.print(response[i], HEX);
        // Serial.print(" ");
        // }
        // Serial.println();
        
        // Jika respon diawali 01 03, berarti SUKSES
        if (response[0] == 0x01 && response[1] == 0x03) {
        int value = (response[3] << 8) | response[4];
        return value / 10.0; // Konversi ke m/s
        // Serial.print("DATA TERDETEKSI! Raw Value: ");
        // Serial.println(value);
        }
    } else {
        Serial.println("TETAP TIDAK ADA RESPON.");
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
float _lux = 0.00;
int _uvIndex = 0;
float _windSpeed = 0.00;
bool switchToLux = false;
unsigned long lastRead = 0;

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

    _windSpeed = readWindSpeed();

    if (millis() - lastRead > mqttClient->getPublishInterval() / sampleCount)
    {
        if (xSemaphoreTake(_sensorDataMutex, portMAX_DELAY)) {
            currentSensorData.temperature_in = bme.readTemperature();
            currentSensorData.humidity = bme.readHumidity();
            currentSensorData.pressure = bme.readPressure() / 100.0F;

            uint16_t rtd = max31865.readRTD();
            currentSensorData.temperature_out = max31865.temperature(RNOMINAL, RREF);
              // Check and print any faults
            uint8_t fault = max31865.readFault();
            if (fault) {
                Serial.print("Fault 0x"); Serial.println(fault, HEX);
                if (fault & MAX31865_FAULT_HIGHTHRESH) {
                Serial.println("RTD High Threshold"); 
                }
                if (fault & MAX31865_FAULT_LOWTHRESH) {
                Serial.println("RTD Low Threshold"); 
                }
                if (fault & MAX31865_FAULT_REFINLOW) {
                Serial.println("REFIN- > 0.85 x Bias"); 
                }
                if (fault & MAX31865_FAULT_REFINHIGH) {
                Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
                }
                if (fault & MAX31865_FAULT_RTDINLOW) {
                Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
                }
                if (fault & MAX31865_FAULT_OVUV) {
                Serial.println("Under/Over voltage"); 
                }
                max31865.clearFault();
            }

            currentSensorData.windSpeed = _windSpeed;

            currentSensorData.rainfall = rainTips * RAIN_FACTOR;

            currentSensorData.uvIndex = _uvIndex;
            currentSensorData.lux = _lux;

            // Serial.printf("Temp: %.2f | Wind: %.2f | Rain: %.2f mm | UV: %d | Lux: %.2f\n", 
            //             currentSensorData.temperature_out, 
            //             currentSensorData.windSpeed, 
            //             currentSensorData.rainfall,
            //             currentSensorData.uvIndex,
            //             currentSensorData.lux);
            
            xSemaphoreGive(_sensorDataMutex);
        }
        lastRead = millis();
    }
}

void performOTA(String url) {
    Serial.println("[OTA] Starting Update...");
    
    WiFiClientSecure client;
    client.setInsecure();

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
    bool ismaintenance = false;

    while (true) {
        if (WiFi.status() != WL_CONNECTED  && WiFi.getMode() == WIFI_STA && wifiSetup->isConnectedAP() == false){
            wifiStatus = WIFI_DISCONNECTED;
        }else if(WiFi.status() == WL_CONNECTED && WiFi.getMode() == WIFI_STA){
            wifiStatus = WIFI_STACONNECTED;
        }
        
        // membaca tegangan baterai sebanyak 20 kali setiap satuan waktu mqtt interval untuk ambil nilai rata rata
        if (millis() - lastBatteryCheck > mqttClient->getPublishInterval() / sampleCount) {
            totalMilliVolts += analogReadMilliVolts(BATTERY_PIN);
            counterSample++;
            if (counterSample == sampleCount)
            {
                float avgMilliVolts = totalMilliVolts / sampleCount;
                float voltageGPIO = avgMilliVolts / 1000.0; 
                // Vin = Vout * (R1 + R2) / R2
                float voltageBattery = voltageGPIO * ((R1 + R2) / R2);
                currentSensorData.batteryVoltage = voltageBattery * calibrationFactor;
            }
            
            lastBatteryCheck = millis();
        }

        switch (wifiStatus)
        {
            case WIFI_DISCONNECTED:
            {
                Serial.println("WiFi Lost. Reconnecting...");
                digitalWrite(LED_AP, HIGH);
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
                    mqttClient->begin(MQTT_SERVER, MQTT_PORT,
                                        MQTT_USER, MQTT_PASSWORD,
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
                            Serial.printf("Temp: %.2f | Wind: %.2f | Rain: %.2f mm | UV: %d | Lux: %.2f | battery: %.2f V\n", 
                                currentSensorData.temperature_out, 
                                currentSensorData.windSpeed, 
                                currentSensorData.rainfall,
                                currentSensorData.uvIndex,
                                currentSensorData.lux,
                                currentSensorData.batteryVoltage);
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
                    // Periodic Status Publish setiap 5x publish data
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
            case MAINTENANCE_MODE:
            {
                if (!ismaintenance)
                {
                    if (!mqttClient->isConnected()) {
                        mqttClient->begin(MQTT_SERVER, MQTT_PORT,
                                            MQTT_USER, MQTT_PASSWORD,
                                            memory->getID_Device());
                        mqttClient->connect();
                        ismaintenance = mqttClient->publishStatus(
                            "maintenance",
                            FRAMEWORK_VERSION,
                            millis() / 1000,
                            WiFi.localIP().toString(),
                            ESP.getFreeHeap()
                        );
                    }else
                    {
                        ismaintenance = mqttClient->publishStatus(
                            "maintenance",
                            FRAMEWORK_VERSION,
                            millis() / 1000,
                            WiFi.localIP().toString(),
                            ESP.getFreeHeap()
                        );
                    }
                }
                
                break;
            }
        }

        if (WiFi.getMode() == WIFI_AP)
        {
            wifiSetup->loopDns();
        }

        digitalRead(BUTTON_RESET) == LOW ? mqttClient->_modeCommand = RESTART:false;
        digitalRead(BUTTON_ENABLE_AP) == LOW ? wifiStatus = WIFI_APCONNECTED:false;
        digitalRead(BUTTON_MAINTENANCE) == LOW ? wifiStatus = MAINTENANCE_MODE:false;
        vTaskDelay(10); 
    }
}

void appSensors(void *param) {
    while (true) {
        readSensors();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}