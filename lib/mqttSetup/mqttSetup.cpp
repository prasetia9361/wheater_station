#include "mqttSetup.h"

// Static member initialization
MQTTSetup* MQTTSetup::instance = nullptr;

MQTTSetup::MQTTSetup() : client(espClient) {
    _modeCommand = NONE;
    connected = false;
    lastPublish = 0;
    publishInterval = 60000; // Default 60 seconds
    commandCallback = nullptr;
    instance = this;
    timestamp = "";
}

bool MQTTSetup::begin(const char* server, int port, const char* user,
                      const char* password, const String& deviceID) {
    mqtt_server = server;
    mqtt_port = port;
    mqtt_user = user;
    mqtt_password = password;
    station_id = deviceID;
    
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(staticCallback);
    
    Serial.println("[MQTT] Setup initialized");
    Serial.print("[MQTT] Server: ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.println(mqtt_port);
    
    return true;
}

bool MQTTSetup::connect() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[MQTT] WiFi not connected");
        return false;
    }
    
    if (client.connected()) {
        return true;
    }
    
    String clientID = "device-" + station_id;
    
    Serial.print("[MQTT] Connecting to broker as ");
    Serial.println(clientID.c_str());
    
    if (client.connect(clientID.c_str()/*, mqtt_user, mqtt_password*/)) {
        Serial.println("[MQTT] Connected to broker");
        connected = true;
        subscribe();
        
        // Publish initial status
        // publishStatus("online", "1.0.0", millis() / 1000, WiFi.localIP().toString(), ESP.getFreeHeap());
        
        return true;
    } else {
        Serial.print("[MQTT] Connection failed, rc=");
        Serial.println(client.state());
        connected = false;
        return false;
    }
}

bool MQTTSetup::isConnected() {
    return client.connected();
}

void MQTTSetup::loop() {
    if (!client.connected()) {
        if (WiFi.status() == WL_CONNECTED) {
            if (millis() - lastPublish > 5000) {
                connect();
                lastPublish = millis();
            }
        }
    }
    
    client.loop();
}

bool MQTTSetup::publishSensorData(float temperatureIn, float temperatureOut, float humidity, float pressure,
                                  float windSpeed,
                                  float rainfall, int uvIndex, float lux,
                                  float batteryVoltage, int signalStrength) {
    if (!isConnected()) {
        Serial.println("[MQTT] Not connected, cannot publish sensor data");
        return false;
    }
    
    JsonDocument doc;
    
    doc["station_id"] = station_id;
    doc["timestamp"] = timestamp;
    doc["temp_in"] = serialized(String(temperatureIn, 2));
    doc["temp_out"] = serialized(String(temperatureOut, 2));
    doc["humidity"] = serialized(String(humidity, 2));
    doc["pressure"] = serialized(String(pressure, 2));
    
    if (windSpeed > 0) doc["wind_speed"] = serialized(String(windSpeed, 2));
    // if (windDirection > 0) doc["wind_direction"] = windDirection;
    if (rainfall > 0) doc["rainfall"] = serialized(String(rainfall, 2));
    // if (uvIndex > 0) doc["uv_index"] = uvIndex;
    doc["uv_index"] = uvIndex;
    if (lux > 0) doc["lux"] = serialized(String(lux, 2));
    if (batteryVoltage > 0) doc["v"] = serialized(String(batteryVoltage, 2));
    if (signalStrength != 0) doc["signal"] = signalStrength;
    
    String topic = "weather/" + station_id + "/data";
    String payload;
    serializeJson(doc, payload);
    
    bool success = client.publish(topic.c_str(), (const uint8_t*)payload.c_str(), payload.length(), false);
    
    if (success) {
        Serial.print("[MQTT] Published to ");
        Serial.println(topic.c_str());
    } else {
        Serial.print("[MQTT] Failed to publish to ");
        Serial.println(topic.c_str());
    }
    
    return success;
}

bool MQTTSetup::publishStatus(const String& status, const String& fwVersion,
                              unsigned long uptime, const String& ipAddress,
                              int freeHeap) {
    if (!isConnected()) {
        Serial.println("[MQTT] Not connected, cannot publish status");
        return false;
    }
    
    JsonDocument doc;
    
    doc["station_id"] = station_id;
    doc["status"] = status;
    doc["firmware_version"] = fwVersion;
    doc["uptime"] = (int)uptime;
    doc["ip_address"] = ipAddress;
    if (freeHeap > 0) doc["free_heap"] = freeHeap;
    doc["timestamp"] = timestamp;
    
    String topic = "weather/" + station_id + "/status";
    String payload;
    serializeJson(doc, payload);
    
    bool success = client.publish(topic.c_str(), (const uint8_t*)payload.c_str(), payload.length(), true);
    
    if (success) {
        Serial.print("[MQTT] Published status to ");
        Serial.println(topic.c_str());
    } else {
        Serial.print("[MQTT] Failed to publish status");
    }
    
    return success;
}

void MQTTSetup::setCommandCallback(CommandCallback callback) {
    commandCallback = callback;
}

void MQTTSetup::subscribe() {
    String topic = "weather/" + station_id + "/command";
    if (client.subscribe(topic.c_str(), 1)) {
        Serial.print("[MQTT] Subscribed to ");
        Serial.println(topic.c_str());
    } else {
        Serial.print("[MQTT] Failed to subscribe to ");
        Serial.println(topic.c_str());
    }
}

void MQTTSetup::setPublishInterval(unsigned long interval) {
    publishInterval = interval;
}

void MQTTSetup::setInterval(unsigned long interval) {
    publishInterval = interval;
}

unsigned long MQTTSetup::getPublishInterval() {
    return publishInterval;
}

void MQTTSetup::staticCallback(char* topic, byte* payload, unsigned int length) {
    if (instance != nullptr) {
        instance->handleMessage(topic, payload, length);
    }
}

void MQTTSetup::handleMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print("[MQTT] Message received on topic: ");
    Serial.println(topic);
    
    // Parse JSON
    char buffer[256];
    memcpy(buffer, payload, min(length, (unsigned int)(sizeof(buffer) - 1)));
    buffer[min(length, (unsigned int)(sizeof(buffer) - 1))] = '\0';
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, buffer);
    
    if (error) {
        Serial.print("[MQTT] JSON parse error: ");
        Serial.println(error.c_str());
        return;
    }
    
    Serial.println("[MQTT] JSON parsed successfully");
    
    const char* command = doc["command"];

    if (strcmp(command, "restart") == 0) {
        publishStatus("maintenance", "1.2.0", millis()/1000, WiFi.localIP().toString());
        _modeCommand = RESTART;
    } else if (strcmp(command, "set_interval") == 0) {
        if (doc["interval"].is<unsigned long>()) {
            unsigned long interval = doc["interval"].as<unsigned long>();
            setInterval(interval);
            Serial.print("[MQTT] Publish interval set to ");
            Serial.print(interval);
            Serial.println(" ms");
        }
    } else if (strcmp(command, "ota_update") == 0) {
        if (doc["firmware_url"].is<const char*>()) {
            _otaUrl = doc["firmware_url"].as<String>();
            _otaRequested = true;
            Serial.println("[MQTT] OTA update command received!");
            Serial.println("[MQTT] URL: " + _otaUrl);
            _modeCommand = publishStatus("maintenance", "1.2.0", millis()/1000, WiFi.localIP().toString()) ? OTA_UPDATE : NONE;
            
        } else {
            Serial.println("[MQTT] OTA Error: firmware_url missing");
        }
    }
    
}
