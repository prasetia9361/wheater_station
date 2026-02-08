# MQTT Setup Library Documentation

## Overview

`MQTTSetup` adalah library C++ yang menyederhanakan koneksi MQTT dan pengiriman data sensor ke broker MQTT sesuai dengan dokumentasi protokol Weather Station.

## Features

- ✅ Koneksi MQTT dengan username/password authentication
- ✅ Auto-reconnect ke MQTT broker
- ✅ Publishing sensor data dengan format JSON sesuai dokumentasi
- ✅ Publishing status device
- ✅ Subscribe ke command topic untuk menerima perintah dari backend
- ✅ Callback function untuk handle command
- ✅ Support QoS 1 (At least once delivery)
- ✅ Keep-alive mechanism
- ✅ Thread-safe dengan semaphore

## Installation

Library ini sudah terintegrasi di folder `lib/mqttSetup/`

Pastikan dependensi di `platformio.ini` sudah ada:
```ini
lib_deps = 
    knolleary/PubSubClient @ ^2.8
    bblanchon/ArduinoJson@^7.0.4
```

## Usage

### 1. Inisialisasi MQTT Client

```cpp
#include "mqttSetup.h"

MQTTSetup *mqttClient = new MQTTSetup();

// Configure MQTT broker
mqttClient->begin(
    "your-backend.com",      // MQTT broker host
    1883,                      // Port (1883 for TCP, 8883 for TLS)
    "weather_station",         // Username
    "your_mqtt_password",      // Password
    "device-unique-id"         // Device ID (station_id)
);

// Set command callback
mqttClient->setCommandCallback(handleMqttCommand);
```

### 2. Connect ke MQTT Broker

```cpp
// Di loop WiFi
if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient->isConnected()) {
        mqttClient->connect();
    }
    mqttClient->loop(); // Keep connection alive
}
```

### 3. Publish Sensor Data

```cpp
// Publish sensor readings setiap waktu tertentu
if (mqttClient->isConnected()) {
    mqttClient->publishSensorData(
        25.5,      // temperature (°C)
        60.0,      // humidity (%)
        1013.25,   // pressure (hPa)
        5.2,       // wind_speed (m/s) - optional
        180,       // wind_direction (0-360°) - optional
        0.0,       // rainfall (mm) - optional
        5,         // uv_index (0-11+) - optional
        12.5,      // battery_voltage (V) - optional
        -65        // signal_strength (dBm) - optional
    );
}
```

### 4. Publish Status

```cpp
// Publish device status (recommended setiap 5 menit)
mqttClient->publishStatus(
    "online",                          // status: online/offline/sleeping/maintenance
    "1.0.0",                          // firmware_version
    millis() / 1000,                  // uptime (seconds)
    WiFi.localIP().toString(),        // ip_address
    ESP.getFreeHeap()                 // free_heap (optional)
);
```

### 5. Handle Commands

Implementasikan callback function untuk handle command dari backend:

```cpp
void handleMqttCommand(const JsonDocument& command) {
    if (command.containsKey("command")) {
        String cmd = command["command"].as<String>();
        
        if (cmd == "restart") {
            Serial.println("Restarting...");
            ESP.restart();
        }
        else if (cmd == "set_interval") {
            unsigned long interval = command["interval"].as<unsigned long>();
            mqttClient->setPublishInterval(interval);
        }
        else if (cmd == "ota_update") {
            // Implement OTA update
        }
    }
}
```

### 6. Set Publish Interval

```cpp
// Set interval antara publikasi sensor data (default 60000 ms)
mqttClient->setPublishInterval(30000); // 30 seconds
unsigned long interval = mqttClient->getPublishInterval();
```

## MQTT Topics

### Publishing Topics

| Topic | QoS | Retain | Deskripsi |
|-------|-----|--------|-----------|
| `weather/{station_id}/data` | 1 | false | Sensor readings |
| `weather/{station_id}/status` | 1 | true | Device status |

### Subscribing Topics

| Topic | QoS | Deskripsi |
|-------|-----|-----------|
| `weather/{station_id}/command` | 1 | Remote commands |

## Supported Commands

Command yang dapat diterima dari backend:

### 1. Restart
```json
{
  "command": "restart",
  "timestamp": "2026-02-02T10:30:00Z"
}
```

### 2. Set Publish Interval
```json
{
  "command": "set_interval",
  "interval": 30000,
  "timestamp": "2026-02-02T10:30:00Z"
}
```

### 3. OTA Update
```json
{
  "command": "ota_update",
  "url": "http://your-backend.com/firmware/latest.bin",
  "timestamp": "2026-02-02T10:30:00Z"
}
```

## JSON Message Format

### Sensor Data

```json
{
  "station_id": "device-550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-02-05T10:30:45Z",
  "temperature": "25.50",
  "humidity": "60.00",
  "pressure": "1013.25",
  "wind_speed": "5.20",
  "wind_direction": 180,
  "rainfall": "0.00",
  "uv_index": 5,
  "battery_voltage": "12.50",
  "signal_strength": -65
}
```

### Status Message

```json
{
  "station_id": "device-550e8400-e29b-41d4-a716-446655440000",
  "status": "online",
  "firmware_version": "1.0.0",
  "uptime": 3600,
  "ip_address": "192.168.1.100",
  "free_heap": 524288,
  "timestamp": "2026-02-05T10:30:45Z"
}
```

## API Reference

### Constructor
```cpp
MQTTSetup();
```

### Methods

#### begin()
```cpp
bool begin(const char* server, int port, const char* user, 
           const char* password, const String& deviceID);
```
Initialize MQTT client dengan broker settings.

#### connect()
```cpp
bool connect();
```
Connect ke MQTT broker. Return true jika berhasil.

#### isConnected()
```cpp
bool isConnected();
```
Cek status koneksi MQTT. Return true jika connected.

#### loop()
```cpp
void loop();
```
Maintain MQTT connection dan process incoming messages. Call this regularly dari main loop.

#### publishSensorData()
```cpp
bool publishSensorData(float temperature, float humidity, float pressure,
                      float windSpeed = 0.0, int windDirection = 0,
                      float rainfall = 0.0, int uvIndex = 0,
                      float batteryVoltage = 0.0, int signalStrength = 0);
```
Publish sensor readings ke broker.

#### publishStatus()
```cpp
bool publishStatus(const String& status, const String& fwVersion,
                  unsigned long uptime, const String& ipAddress,
                  int freeHeap = 0);
```
Publish device status ke broker.

#### setCommandCallback()
```cpp
void setCommandCallback(CommandCallback callback);
```
Set callback function untuk handle incoming commands.

#### setPublishInterval()
```cpp
void setPublishInterval(unsigned long interval);
```
Set interval antara publikasi sensor data (milliseconds).

#### getPublishInterval()
```cpp
unsigned long getPublishInterval();
```
Get current publish interval.

#### subscribe()
```cpp
void subscribe();
```
Subscribe ke command topic. Automatically called di connect().

## Error Handling

Library menampilkan debug messages ke Serial:

```
[MQTT] Setup initialized
[MQTT] Server: your-backend.com:1883
[MQTT] Connecting to broker as device-xxx
[MQTT] Connected to broker
[MQTT] Published to weather/device-xxx/data
[MQTT] Subscribed to weather/device-xxx/command
[MQTT] Message received on topic: weather/device-xxx/command
[MQTT] JSON parsed successfully
[MQTT] Processing command...
```

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Connection failed | WiFi tidak connected | Pastikan WiFi connected dulu |
| JSON parse error | Invalid JSON payload | Check format JSON dari backend |
| Publish failed | Not connected to broker | Call `connect()` dulu |

## Thread Safety

Library menggunakan static callback untuk thread-safe message handling. Semua operasi internal sudah protected.

## Memory Usage

- Heap: ~2-3 KB untuk library
- Buffer: 256 bytes untuk JSON parsing
- Rekomendasi: Stack min 4096 bytes untuk task

## Performance

- Keep-alive interval: 60 seconds (default)
- Reconnect interval: 5 seconds
- JSON serialization: < 50ms
- Typical message size: 200-400 bytes

## Compatibility

- **Framework**: Arduino ESP32
- **Compiler**: GCC
- **C++ Standard**: C++11 or higher
- **PubSubClient**: v2.8 or newer
- **ArduinoJson**: v7.0.4 or newer

## License

Bagian dari Weather Station project.
