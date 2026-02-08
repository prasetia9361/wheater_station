#include <Arduino.h>
// ... (include lainnya)
#include <Adafruit_BME280.h>
#include <Adafruit_MAX31865.h>
#include <LTR390.h>

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

// --- SENSOR OBJECTS ---
Adafruit_BME280 bme;
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(MAX_CS_PIN, MAX_DI_PIN, MAX_DO_PIN, MAX_CLK_PIN);
LTR390 ltr;

// --- RAIN GAUGE VARIABLES ---
volatile long rainTips = 0;
const float RAIN_FACTOR = 0.2794; // mm per tip (sesuaikan dengan spesifikasi rain gauge Anda)

// Interrupt Service Routine untuk Hujan
void IRAM_ATTR rainInterrupt() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTime > 200) { // Debounce 200ms
        rainTips++;
        lastTime = currentTime;
    }
}

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

void readSensors() {
    // if (xSemaphoreTake(_sensorDataMutex, portMAX_DELAY)) {
        // 1. Read BME280
        currentSensorData.temperature_in = bme.readTemperature();
        currentSensorData.humidity = bme.readHumidity();
        currentSensorData.pressure = bme.readPressure() / 100.0F;

        // 2. Read MAX31865 (Precision Temperature)
        uint16_t rtd = max31865.readRTD();
        currentSensorData.temperature_out = max31865.temperature(RNOMINAL, RREF);

        // 3. Read LTR390
        currentSensorData.uvIndex = ltr.getUVIndex();
        currentSensorData.lux = ltr.getLUX();

        // 4. Read Anemometer
        currentSensorData.windSpeed = readWindSpeed();

        // 5. Read Rain Gauge (Total accumulation)
        currentSensorData.rainfall = rainTips * RAIN_FACTOR;

        // xSemaphoreGive(_sensorDataMutex);
        
        // Debugging
        Serial.printf("Temp: %.2f | Wind: %.2f | Rain: %.2f mm | UV: %d\n", 
                      currentSensorData.temperature_out, 
                      currentSensorData.windSpeed, 
                      currentSensorData.rainfall,
                      currentSensorData.uvIndex);
    // }
}

// --- TASK SENSORS (Core 1) ---
void appSensors(void *param) {
    initSensors();
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