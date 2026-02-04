#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include "wifi_setup.h"
#include "clientServer.h"
#include "storage.h"

// Static IP Configuration untuk ESP32 (TCP Client)
IPAddress staticIP(192, 168, 18, 100);      // IP statis ESP32
IPAddress gateway(192, 168, 18, 1);          // Gateway
IPAddress subnet(255, 255, 255, 0);          // Subnet mask
IPAddress dns(8, 8, 8, 8);                   // DNS Google
// ==================================

#define STEP_PIN        4
#define DIR_PIN         5
#define ENABLE_PIN      26
#define SDA_PIN         21
#define SCL_PIN         22
#define XSHUT_LO_PIN    16
#define XSHUT_HI_PIN    17

#define BAT_VOLT_PIN    32  // ADC1_CH1 (47k & 10k Divider)
#define BAT_CURR_PIN    33  // ADC1_CH0 (ACS712 Output)

#define MAX_SPEED       400.0  // Kecepatan PID Maks (turun dari 1500 - kurangi getaran)
#define CALIB_SPEED     800.0  // Kecepatan saat kalibrasi (turun dari 600)
#define HOMING_SPEED    -800.0 // Kecepatan Homing (turun dari -400)
#define PID_INTERVAL    10     // ms
#define LIMIT_THRESHOLD_MM  50 
#define PAUSE_TIME      1000   // Waktu diam di ujung (1 detik)
// Safety Thresholds
#define CHARGE_THRESHOLD_VOLT   12.7 // LiFePO4 Charging
#define LOW_BAT_THRESHOLD       11.1 // LiFePO4 Critical Low


#define LO_ADDR 0x30
#define HI_ADDR 0x31

// --- Calibration Constants ---
// Voltage Divider: (R1+R2)/R2 => (10k+1k+1k)/1k = 11
const float VOLTAGE_MULTIPLIER = 11; 
//1/()
// ACS712 5A = 185mV/A, 20A = 100mV/A, 30A = 66mV/A. 
// Ganti 185.0 sesuai modul anda.
const float ACS712_SENSITIVITY = 185.0; 
// Tegangan tengah ACS712 (biasanya 2500mV jika VCC 5V, atau 1650mV jika VCC 3.3V)
// Perlu dikalibrasi manual saat arus 0.
double ACS712_OFFSET_MV = 0.0; 

float Kp = 1.5;  // Turun dari 5.0 (kurangi overshoot)
float Ki = 0.01;
float Kd = 0.3; // Turun dari 30.0 (kurangi osilasi/getaran)

// ... (Variabel Global Lain) ...
bool chargingStatus = false;
bool lowBatteryStatus = false; 
bool isLowBatReturn = false; 

const char* serverIP = "192.168.18.10";
// strcmp(serverIP_cstr, serverIP.c_str());
int serverPort = 9999;


// State Machine
enum SystemState {
    STATE_IDLE,         
    STATE_HOMING,       
    STATE_CALIBRATING,  
    STATE_RUNNING_UP,   
    STATE_RUNNING_DOWN, 
    STATE_STOPPED,
    STATE_WAITING       // [TAMBAH] State khusus untuk diam di ujung
};

SystemState currentState = STATE_IDLE;
SystemState nextStateAfterWait = STATE_IDLE; // [TAMBAH] Untuk logic waiting

enum StatusMotor {
    STATUS_STOPPED,
    STATUS_MOVING,
    STATUS_CHARGING
};

StatusMotor motorStatus = STATUS_STOPPED;

enum wifiCindition {
    WIFI_DISCONNECTED,
    WIFI_STACONNECTED,
    WIFI_APCONNECTED
};

wifiCindition wifiStatus = WIFI_DISCONNECTED;

long maxPositionSteps = 0;
long targetPosition = 0;
unsigned long lastPIDTime = 0;
unsigned long waitStartTime = 0;
bool isWaiting = false;
bool waitCalibrateDone = false;
// bool emergencyStop = false;
float currentSpeed = 0;
long currentSteps = 0;
float batteryVoltage = 0.0; 
float batteryCurrent = 0.0; 
float distanceHi = 0.0;
float distanceLo = 0.0;
float distLo = 0.0;
float distHi = 0.0;

WifiSetup *wifiSetup;
WiFiClient client;
clientServer *webServer;
storage *memory;

float previousError = 0;
float integral = 0;

void initSensors();
void motorEnable(bool enable);
void runSystemLogic(bool lowBatteryStatus, bool chargingStatus);
void calculatePID();
void parseSerialCommand();
void checkSafetyLimits();


void appMotor(void *param);
void appWifi(void *param);

SemaphoreHandle_t _semaphore;

void setup() {
    Serial.begin(115200);
    delay(2000);

    _semaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(_semaphore);

    analogReadResolution(12); // 0 - 4095
    pinMode(BAT_VOLT_PIN, INPUT);
    pinMode(BAT_CURR_PIN, INPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH); // Enable motor aktif (LOW = aktif, HIGH = disable)
    Serial.println("--- ESP32-C3 Slider Auto-Loop Integrated ---");
    Serial.println("Ketik 'START' untuk mulai kalibrasi & loop.");
    Serial.println("Ketik 'STOP' untuk berhenti.");
    
    Wire.begin(SDA_PIN, SCL_PIN);
    initSensors();
    // asli
    currentState = STATE_IDLE;
    // currentState = STATE_HOMING;

    

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


    TaskHandle_t taskMotor;
    xTaskCreatePinnedToCore(
        appMotor,
        "appMotor",
        10000,
        NULL,
        3,
        &taskMotor,
        1
    );


}

void loop() {
delay(1000);
}

void initSensors() {
    Serial.println("Init ToF Sensors...");

}

void appWifi(void *param) {
    wifiSetup = new WifiSetup();
    memory = new storage();
    webServer = new clientServer(memory);
    
    // Inisialisasi SPIFFS duluan sebelum akses file
    if (!memory->init()) {
        Serial.println("SPIFFS Mount Failed!");
        vTaskDelete(NULL);
        return;
    }
    memory->readWifi();
    IPAddress clientIP;
    // ===== KONFIGURASI IP STATIS =====
    // Atur IP statis sebelum koneksi WiFi

    // ===================================
    
    if (memory->getSsid() != "" && memory->getServerIP() != "" && memory->getID_Device() != "")
    {
        
        clientIP.fromString(memory->getClientIP());
        WiFi.config(clientIP, gateway, subnet, dns);
        Serial.println("IP Statis dikonfigurasi: " + String(memory->getClientIP()));
        // wifiSetup->connectSTA();
        // wifiSetup->setupWiFiSTA(memory->getSsid().c_str(), memory->getPass().c_str());
        wifiStatus = WIFI_DISCONNECTED;
    }else{
        wifiSetup->disconnectSTA();
        wifiSetup->connectAP();
        wifiSetup->setupWiFiAP(memory->getID_Device().c_str(), "slider1234");
        webServer->begin();
        wifiStatus = WIFI_APCONNECTED;
    }

    String lastCommand = "";
    unsigned long lastBatteryCheck = 0;
    unsigned long lastTelemetry = 0;
    
    // Variabel lokal untuk menampung data snapshot dari global
    float _currentSpeed = 0;
    long _currentSteps = 0;
    bool _charging = false;
    bool _lowbat = false;
    float _distanceHi = 0.0;
    float _distanceLo = 0.0;
    bool apMode = false;
    int countTCPFail = 0;

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
            if (batteryVoltage == 0) batteryVoltage = voltRaw;
            else batteryVoltage = (0.1 * voltRaw) + (0.9 * batteryVoltage);
            // _charging = (batteryVoltage > CHARGE_THRESHOLD_VOLT);
            _lowbat = (batteryVoltage < LOW_BAT_THRESHOLD);

            Serial.print("Volt: "); Serial.print(batteryVoltage);
            // Serial.print(" | Charging: "); Serial.println(_charging);
            Serial.print(" | LowBat: "); Serial.println(_lowbat);
            xSemaphoreTake(_semaphore, portMAX_DELAY);
            // chargingStatus = _charging;
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
                    // wifiSetup->disconnectSTA();
                    // wifiSetup->connectAP();
                    Serial.println("Switching to AP Mode :"+ String(wifiStatus));
                }else if (wifiSetup->isConnectedSTA() == true)
                {
                    wifiStatus = WIFI_STACONNECTED;
                    wifiSetup->setconnectedSTA(false);
                }
                break;
            }
            case WIFI_STACONNECTED:
            {
                if (!client.connected()) {
                    Serial.println("TCP Connecting to " + String(memory->getServerIP()) + ":" + String(memory->getServerPort()) + "...");
                    if (client.connect(memory->getServerIP(), memory->getServerPort())) {
                        Serial.println("TCP Connected!");
                        countTCPFail = 0;                    
                        // Auto Start saat connect (Opsional)
                        // xSemaphoreTake(_semaphore, portMAX_DELAY);
                        // currentState = STATE_HOMING; 
                        // xSemaphoreGive(_semaphore);
                        client.setNoDelay(true); // Agar pengiriman paket lebih cepat
                    } else {
                        Serial.println("TCP Connect Failed.");
                        // countTCPFail = (countTCPFail + 1) % 3;
                        // if (countTCPFail == 2)
                        // {
                        //     xSemaphoreTake(_semaphore, portMAX_DELAY);
                        //     currentState = STATE_STOPPED; 
                        //     xSemaphoreGive(_semaphore);
                        // }
                        vTaskDelay(2000 / portTICK_PERIOD_MS);
                    }
                }

                // 3. Logika Utama saat Terhubung
                if (client.connected()) {
                    
                    // --- A. TERIMA DATA (RECEIVE) ---
                    if (client.available()) {
                        // Baca command raw (contoh: "start", "stop")
                        String line = client.readStringUntil('\n');
                        line.trim(); 
                        Serial.println("TCP RX: " + line);
                        
                        // Gunakan Semaphore saat mengubah variabel kontrol motor
                        xSemaphoreTake(_semaphore, portMAX_DELAY);
                        if (line.equalsIgnoreCase("@start#")) { 
                            targetPosition = 0; 
                            currentState = STATE_HOMING; 
                        }
                        else if (line.equalsIgnoreCase("@stop#")) { 
                            currentState = STATE_STOPPED; 
                        }
                        xSemaphoreGive(_semaphore);
                    }

                    // --- B. KIRIM DATA (SEND TELEMETRY) ---
                    // misal gagal sampe 3 kali motor berhenti looping dan reconnect dan lanjut lagi
                    if (millis() - lastTelemetry > 1000) { // Kirim setiap 1s
                        lastTelemetry = millis();
                        
                        // String payload = "@RB_"+ String(ESP.getEfuseMac()) + ","
                        // String payload = "@RB_"+ String((uint32_t)(ESP.getEfuseMac() >> 32), HEX) + ","
                        String payload = "@RB_" + memory->getID_Device() + ","
                            // + String((int)_currentSteps) + "," 
                            + String((int)_currentSpeed) + "," 
                            + String(batteryVoltage) + ","
                            + String(batteryCurrent) + ","
                            + String(motorStatus) + ","
                            + String(_distanceHi) + "," 
                            + String(_distanceLo) + "#";

                        client.print(payload);
                        // Serial.println("TCP TX: " + payload);
                    }
                }
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
                    wifiSetup->setupWiFiAP(memory->getID_Device().c_str(), "BTGsmart");
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
        vTaskDelay(50 / portTICK_PERIOD_MS); 
    }
}