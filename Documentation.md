# MQTT Protocol Documentation - Weather Station

Dokumentasi lengkap protokol MQTT untuk mengirim data sensor ke backend Weather Station.

---

## 📋 Daftar Isi

1. [Overview](#overview)
2. [MQTT Broker Configuration](#mqtt-broker-configuration)
3. [Topic Structure](#topic-structure)
4. [Message Format](#message-format)
5. [Authentication](#authentication)
6. [Publishing Data](#publishing-data)
7. [Subscribing to Commands](#subscribing-to-commands)
8. [Testing with MQTT Tools](#testing-with-mqtt-tools)
9. [Error Codes & Troubleshooting](#error-codes--troubleshooting)

---

## Overview

Backend menggunakan **Mosquitto MQTT Broker** untuk menerima data real-time dari weather stations.

### Key Features:

- **Protocol**: MQTT v3.1.1
- **QoS**: 0, 1, 2 (recommended: QoS 1)
- **Transport**: TCP (port 1883) atau TLS (port 8883)
- **Authentication**: Username/Password atau API Key
- **Message Format**: JSON
- **Max Payload**: 256 KB
- **Keep Alive**: 60 seconds

---

## MQTT Broker Configuration

### Connection Parameters

| Parameter         | Value                       | Description                    |
| ----------------- | --------------------------- | ------------------------------ |
| **Host**          | `your-backend.com`          | MQTT broker hostname atau IP   |
| **Port**          | `1883` (TCP) / `8883` (TLS) | Standard MQTT ports            |
| **Username**      | `weather_station`           | MQTT username                  |
| **Password**      | `your_mqtt_password`        | MQTT password atau API key     |
| **Client ID**     | `device-{unique_id}`        | Unique per device              |
| **Keep Alive**    | `60` seconds                | Connection keep-alive interval |
| **Clean Session** | `true`                      | Recommended untuk IoT devices  |

### Connection String Examples

**Non-TLS (Development):**

```
mqtt://weather_station:password@your-backend.com:1883
```

**TLS/SSL (Production):**

```
mqtts://weather_station:password@your-backend.com:8883
```

---

## Topic Structure

### Topic Pattern

```
weather/{station_id}/{message_type}
```

### Available Topics

| Topic                          | Direction | QoS | Retain | Description         |
| ------------------------------ | --------- | --- | ------ | ------------------- |
| `weather/{station_id}/data`    | Publish   | 1   | false  | Sensor readings     |
| `weather/{station_id}/status`  | Publish   | 1   | true   | Station status      |
| `weather/{station_id}/command` | Subscribe | 1   | false  | Remote commands     |
| `weather/alerts/{alert_id}`    | Subscribe | 1   | false  | Alert notifications |

**Example:**

```
weather/550e8400-e29b-41d4-a716-446655440000/data
weather/550e8400-e29b-41d4-a716-446655440000/status
weather/550e8400-e29b-41d4-a716-446655440000/command
```

---

## Message Format

### 1. Sensor Data Message

**Topic:** `weather/{station_id}/data`  
**QoS:** 1 (At least once delivery)  
**Retain:** false

```json
{
  "station_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2026-02-02T10:30:00Z",
  "temperature": 28.5,
  "humidity": 65.2,
  "pressure": 1013.25,
  "wind_speed": 12.3,
  "wind_direction": 180,
  "rainfall": 0.5,
  "uv_index": 6,
  "battery_voltage": 3.7,
  "signal_strength": -65
}
```

**Required Fields:**

- `station_id` (string): UUID stasiun
- `timestamp` (string): ISO 8601 format (UTC)
- `temperature` (float): Suhu dalam Celsius
- `humidity` (float): Kelembaban dalam persen (0-100)
- `pressure` (float): Tekanan udara dalam hPa

**Optional Fields:**

- `wind_speed` (float): Kecepatan angin (m/s)
- `wind_direction` (int): Arah angin (0-360°)
- `rainfall` (float): Curah hujan (mm)
- `uv_index` (int): Indeks UV (0-11+)
- `battery_voltage` (float): Voltase baterai (V)
- `signal_strength` (int): WiFi/Network RSSI (dBm)

### 2. Status Message

**Topic:** `weather/{station_id}/status`  
**QoS:** 1  
**Retain:** true

```json
{
  "station_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "online",
  "firmware_version": "1.2.0",
  "uptime": 3600,
  "ip_address": "192.168.1.100",
  "free_heap": 45000,
  "timestamp": "2026-02-02T10:30:00Z"
}
```

**Fields:**

- `station_id` (string): UUID stasiun
- `status` (string): `online`, `offline`, `sleeping`, `maintenance`
- `firmware_version` (string): Versi firmware
- `uptime` (int): Uptime dalam detik
- `ip_address` (string): IP address
- `free_heap` (int): Free memory dalam bytes (optional)
- `timestamp` (string): ISO 8601 timestamp

### 3. Command Messages (Backend → Device)

**Topic:** `weather/{station_id}/command`

#### a) Restart Command

```json
{
  "command": "restart",
  "timestamp": "2026-02-02T10:30:00Z"
}
```

#### b) Update Interval Command

```json
{
  "command": "set_interval",
  "interval": 60,
  "timestamp": "2026-02-02T10:30:00Z"
}
```

#### c) OTA Update Command

```json
{
  "command": "ota_update",
  "firmware_url": "https://backend.com/firmware/v1.3.0.bin",
  "timestamp": "2026-02-02T10:30:00Z"
}
```

---

## Authentication

### Username/Password

Default authentication method:

```
Username: weather_station
Password: your_mqtt_password
```

Dapatkan credentials dari administrator backend.

### API Key (Alternative)

Gunakan API key sebagai username:

```
Username: sk_live_xxxxxxxxxxxxx
Password: (kosong atau anything)
```

### TLS/SSL (Production)

Untuk production, gunakan port 8883 dengan TLS:

```
Host: your-backend.com
Port: 8883
Protocol: TLS 1.2+
```

Certificate validation diperlukan untuk keamanan.

---

## Publishing Data

### ESP32/Arduino Example

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker
const char* mqtt_server = "your-backend.com";
const int mqtt_port = 1883;
const char* mqtt_user = "weather_station";
const char* mqtt_password = "your_mqtt_password";
const char* station_id = "550e8400-e29b-41d4-a716-446655440000";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  
  // Connect WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  
  // Connect MQTT
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
  
  // Publish data
  publishSensorData();
}

void publishSensorData() {
  // Build JSON
  StaticJsonDocument<512> doc;
  doc["station_id"] = station_id;
  doc["timestamp"] = "2026-02-02T10:30:00Z"; // Get from NTP
  doc["temperature"] = 28.5;
  doc["humidity"] = 65.2;
  doc["pressure"] = 1013.25;
  doc["wind_speed"] = 12.3;
  doc["wind_direction"] = 180;
  doc["rainfall"] = 0.0;
  doc["uv_index"] = 6;
  doc["battery_voltage"] = 3.7;
  doc["signal_strength"] = WiFi.RSSI();
  
  // Serialize JSON
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  
  // Build topic
  char topic[100];
  snprintf(topic, sizeof(topic), "weather/%s/data", station_id);
  
  // Publish
  if (client.publish(topic, jsonBuffer, false)) {
    Serial.println("Data published successfully!");
  } else {
    Serial.println("Failed to publish data!");
  }
}

void loop() {
  client.loop();
}
```

---

## Subscribing to Commands

### ESP32/Arduino Example

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* station_id = "550e8400-e29b-41d4-a716-446655440000";
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  
  // Parse JSON
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.println("Failed to parse JSON!");
    return;
  }
  
  const char* command = doc["command"];
  
  // Handle commands
  if (strcmp(command, "restart") == 0) {
    Serial.println("Restart command received!");
    delay(1000);
    ESP.restart();
  }
  else if (strcmp(command, "set_interval") == 0) {
    int interval = doc["interval"];
    Serial.printf("Update interval to %d seconds\n", interval);
    // Update your interval variable here
  }
  else if (strcmp(command, "ota_update") == 0) {
    const char* firmware_url = doc["firmware_url"];
    Serial.printf("OTA update: %s\n", firmware_url);
    // Implement OTA update logic
  }
}

void setup() {
  Serial.begin(115200);
  
  // Connect WiFi (see previous example)
  // ...
  
  // Setup MQTT
  client.setServer("your-backend.com", 1883);
  client.setCallback(mqttCallback);
  
  // Connect and subscribe
  if (client.connect("ESP32Client", "weather_station", "your_password")) {
    char command_topic[100];
    snprintf(command_topic, sizeof(command_topic), "weather/%s/command", station_id);
    client.subscribe(command_topic, 1); // QoS 1
    Serial.println("Subscribed to command topic!");
  }
}

void loop() {
  if (!client.connected()) {
    // Reconnect logic here
  }
  client.loop();
}
```

---

## Testing with MQTT Tools

### 1. MQTT Explorer (GUI Tool)

Download dari: http://mqtt-explorer.com/

**Cara menggunakan:**

1. Buka MQTT Explorer
2. Klik "+" untuk tambah connection
3. Masukkan:
   - Host: `your-backend.com`
   - Port: `1883`
   - Username: `weather_station`
   - Password: `your_mqtt_password`


   
4. Klik "Connect"
5. Subscribe ke topic: `weather/#`
6. Publish test message ke topic yang diinginkan

### 2. Mosquitto CLI

**Install:**

```bash
# macOS
brew install mosquitto

# Ubuntu/Debian
sudo apt install mosquitto-clients

# Windows
# Download from: https://mosquitto.org/download/
```

**Usage:**

```bash
# Subscribe to all weather topics
mosquitto_sub -h your-backend.com \
  -p 1883 \
  -u weather_station \
  -P your_password \
  -t "weather/#" \
  -v

# Subscribe to specific station
mosquitto_sub -h your-backend.com \
  -u weather_station \
  -P your_password \
  -t "weather/550e8400-e29b-41d4-a716-446655440000/data"

# Publish test data
mosquitto_pub -h your-backend.com \
  -u weather_station \
  -P your_password \
  -t "weather/550e8400-e29b-41d4-a716-446655440000/data" \
  -m '{
    "station_id":"550e8400-e29b-41d4-a716-446655440000",
    "timestamp":"2026-02-02T10:30:00Z",
    "temperature":28.5,
    "humidity":65.2,
    "pressure":1013.25,
    "wind_speed":12.3,
    "wind_direction":180,
    "rainfall":0.0,
    "uv_index":6,
    "battery_voltage":3.7,
    "signal_strength":-65
  }'

# Send restart command to device
mosquitto_pub -h your-backend.com \
  -u weather_station \
  -P your_password \
  -t "weather/550e8400-e29b-41d4-a716-446655440000/command" \
  -m '{"command":"restart","timestamp":"2026-02-02T10:30:00Z"}'

# Test TLS connection (port 8883)
mosquitto_pub -h your-backend.com \
  -p 8883 \
  --cafile ca.crt \
  -u weather_station \
  -P your_password \
  -t "weather/test/data" \
  -m '{"test": true}'
```

### 3. MQTTX (Cross-Platform GUI)

Download: https://mqttx.app/

Features:

- Cross-platform (Windows, macOS, Linux)
- Multiple connections
- Message history
- JSON formatting
- Dark mode

---

## Error Codes & Troubleshooting

### MQTT Connection Error Codes

| Code | Meaning                | Solution                                       |
| ---- | ---------------------- | ---------------------------------------------- |
| `-4` | Connection timeout     | Check network, firewall, broker accessibility  |
| `-3` | Connection lost        | Network unstable, implement reconnection logic |
| `-2` | Connect failed         | Broker down or wrong host/port                 |
| `-1` | Disconnected           | Normal state, call connect()                   |
| `0`  | Connected              | Success                                        |
| `1`  | Wrong protocol version | Use MQTT v3.1.1                                |
| `2`  | Client ID rejected     | Client ID must be unique                       |
| `3`  | Server unavailable     | Broker down or overloaded                      |
| `4`  | Bad credentials        | Wrong username/password                        |
| `5`  | Unauthorized           | User doesn't have permissions                  |

### Common Issues

#### 1. Cannot connect to broker

**Symptoms:**

- Connection timeout
- Error code -4 or -2

**Solutions:**

```bash
# Test network connectivity
ping your-backend.com

# Test MQTT port
telnet your-backend.com 1883

# Test with mosquitto_pub
mosquitto_pub -h your-backend.com -t test -m "hello" -d
```

#### 2. Authentication failed

**Symptoms:**

- Error code 4 (bad credentials)
- Error code 5 (unauthorized)

**Solutions:**

- Verify username/password
- Check if user exists in MQTT ACL
- Try with different credentials
- Contact backend administrator

#### 3. Messages not received by backend

**Check:**

1. **Topic format**: Harus `weather/{station_id}/data`
2. **JSON valid**: Test di https://jsonlint.com/
3. **QoS level**: Gunakan QoS 1
4. **Payload size**: Max 256 KB
5. **Backend subscriber**: Check logs backend

**Debug command:**

```bash
# Monitor backend logs
docker-compose logs -f backend | grep -i mqtt

# Check if backend receives messages
mosquitto_sub -h localhost -t "weather/#" -v
```

#### 4. High message loss

**Solutions:**

- Increase QoS to 1 or 2
- Implement message buffering
- Check network stability
- Reduce publish frequency

#### 5. JSON parsing errors

**Symptoms:**

- Backend logs show "Invalid JSON"
- Data not stored in database

**Solutions:**

```cpp
// Validate JSON with ArduinoJson
StaticJsonDocument<512> doc;
doc["temperature"] = 28.5;
doc["humidity"] = 65.2;

char jsonBuffer[512];
size_t n = serializeJson(doc, jsonBuffer);

if (n == 0) {
  Serial.println("Failed to serialize JSON!");
} else {
  Serial.println("Valid JSON:");
  Serial.println(jsonBuffer);
}
```

### Best Practices

1. **Always use QoS 1** untuk guaranteed delivery
2. **Implement reconnection logic** dengan exponential backoff
3. **Validate JSON** sebelum publish
4. **Buffer messages** saat offline
5. **Monitor connection status** dan send heartbeat
6. **Use retained messages** untuk status updates
7. **Set proper keepalive** (60 seconds recommended)
8. **Unique client IDs** untuk setiap device
9. **Close connections properly** saat shutdown
10. **Log all errors** untuk debugging

---

## Quick Reference

### Minimal ESP32 Publisher

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* mqtt_server = "your-backend.com";
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  
  client.setServer(mqtt_server, 1883);
  client.connect("ESP32", "weather_station", "password");
  
  // Build and publish
  StaticJsonDocument<256> doc;
  doc["station_id"] = "550e8400-e29b-41d4-a716-446655440000";
  doc["timestamp"] = "2026-02-02T10:30:00Z";
  doc["temperature"] = 28.5;
  doc["humidity"] = 65.2;
  doc["pressure"] = 1013.25;
  
  char json[256];
  serializeJson(doc, json);
  client.publish("weather/550e8400-e29b-41d4-a716-446655440000/data", json, 1);
}

void loop() { client.loop(); }
```

### Complete ESP32 Example with Sensors

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>

#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;
WiFiClient espClient;
PubSubClient client(espClient);

const char* station_id = "550e8400-e29b-41d4-a716-446655440000";

void setup() {
  Serial.begin(115200);
  dht.begin();
  bmp.begin(0x76);
  
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  
  client.setServer("your-backend.com", 1883);
  client.connect("ESP32", "weather_station", "password");
}

void publishData() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float pres = bmp.readPressure() / 100.0;
  
  StaticJsonDocument<512> doc;
  doc["station_id"] = station_id;
  doc["timestamp"] = "2026-02-02T10:30:00Z"; // Use NTP time
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["pressure"] = pres;
  doc["signal_strength"] = WiFi.RSSI();
  
  char json[512];
  serializeJson(doc, json);
  
  char topic[100];
  snprintf(topic, 100, "weather/%s/data", station_id);
  client.publish(topic, json, 1);
}

void loop() {
  client.loop();
  static unsigned long lastPublish = 0;
  if (millis() - lastPublish > 30000) {
    publishData();
    lastPublish = millis();
  }
}
```

### Mosquitto Pub One-Liner

```bash
mosquitto_pub -h your-backend.com -u weather_station -P password \
  -t "weather/550e8400-e29b-41d4-a716-446655440000/data" \
  -m '{"station_id":"550e8400-e29b-41d4-a716-446655440000","timestamp":"2026-02-02T10:30:00Z","temperature":28.5,"humidity":65.2,"pressure":1013.25}'
```

---

## Resources

### Official Documentation

- **MQTT Protocol**: https://mqtt.org/
- **Mosquitto Broker**: https://mosquitto.org/
- **Paho MQTT Clients**: https://www.eclipse.org/paho/

### Tools

- **MQTT Explorer**: http://mqtt-explorer.com/
- **MQTTX**: https://mqttx.app/
- **HiveMQ WebSocket Client**: https://www.hivemq.com/demos/websocket-client/

### Backend Documentation

- [API Documentation](./API_DOCUMENTATION.md)
- [Architecture Overview](./ARCHITECTURE.md)
- [Deployment Guide](./DEPLOYMENT_GUIDE.md)

---

**Last Updated**: February 2026  
**Version**: 1.0  
**Contact**: support@weatherstation.com
