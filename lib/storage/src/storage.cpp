#include "storage.h"

storage::storage() {
    setWifi.ssid = "";
    setWifi.pass = "";
    setWifi.serverIP = "";
    setWifi.clientIP = "";
    setWifi.serverPort = 0;
    setWifi.ID_Device = "";
}

storage::~storage() {
}

bool storage::init() {
    Serial.println("[DEBUG] Memulai inisialisasi storage");
    
    // Tidak perlu ambil semaphore

    if (!SPIFFS.begin(true)) {
        Serial.println("[ERROR] SPIFFS init failed");
        return false;
    }
    Serial.println("[DEBUG] SPIFFS berhasil diinisialisasi");

    // Tidak perlu xSemaphoreGive
    Serial.println("[DEBUG] Inisialisasi storage selesai");
    return true;
}

bool storage::readWifi(){
    if (SPIFFS.exists("/wifi.json")) {
        JsonDocument wifiDoc;
        File wifiFile = SPIFFS.open("/wifi.json", FILE_READ);
        
        if (!wifiFile) {
            Serial.println("Gagal membuka file wifi.json untuk dibaca");
            setWifi.ssid = "";
            setWifi.pass = "";
            setWifi.serverIP = "";
            setWifi.clientIP = "";
            setWifi.serverPort = 0;
            setWifi.ID_Device = "";
            return false;
        }
        
        char wifiStr[256];
        wifiFile.readBytes(wifiStr, wifiFile.size());
        wifiFile.close();
        
        DeserializationError wifiError = deserializeJson(wifiDoc, wifiStr);
        if (wifiError) {
            Serial.print("deserializeJson() untuk wifi.json returned ");
            Serial.println(wifiError.c_str());
            setWifi.ssid = "";
            setWifi.pass = "";
            setWifi.serverIP = "";
            setWifi.clientIP = "";
            setWifi.serverPort = 0;
            setWifi.ID_Device = "";
            return false;
        }
        
        setWifi.ssid = wifiDoc["ssid"].as<String>();
        setWifi.pass = wifiDoc["pass"].as<String>();
        setWifi.serverIP = wifiDoc["serverIP"].as<String>();
        setWifi.clientIP = wifiDoc["client_ip"].as<String>();
        setWifi.serverPort = wifiDoc["serverPort"].as<int>();
        setWifi.ID_Device = wifiDoc["ID_Device"].as<String>();
        
        Serial.println("Data WiFi berhasil dibaca");
        Serial.print("SSID: ");
        Serial.println(setWifi.ssid);
        Serial.print("Password: ");
        Serial.println(setWifi.pass);
        Serial.print("Server IP: ");
        Serial.println(setWifi.serverIP);
        Serial.print("Client IP: ");
        Serial.println(setWifi.clientIP);
        Serial.print("Server Port: ");
        Serial.println(setWifi.serverPort);
        Serial.print("ID Device: ");
        Serial.println(setWifi.ID_Device);
    } else {
        Serial.println("File wifi.json tidak ditemukan");
        setWifi.ssid = "";
        setWifi.pass = "";
        setWifi.serverIP = "";
        setWifi.clientIP = "";
        setWifi.serverPort = 0;
        setWifi.ID_Device = "";
        return false;
    }
    return true;

}
bool storage::writeWifi(const String& ssid, const String& pass, const String& serverIP, const String& clientIP, int serverPort, const String& ID_Device) {
    File file = SPIFFS.open("/wifi.json", FILE_WRITE);
    if (!file) {
        Serial.println("- gagal membuka file untuk penulisan");
        return false;
    }
    
    JsonDocument doc;
    doc["ssid"] = ssid;
    doc["pass"] = pass;
    doc["serverIP"] = serverIP;
    doc["client_ip"] = clientIP; // Placeholder untuk client_ip
    doc["serverPort"] = serverPort;
    doc["ID_Device"] = ID_Device;
    
    serializeJson(doc, file);
    file.close();
    
    setWifi.ssid = ssid;
    setWifi.pass = pass;
    setWifi.serverIP = serverIP;
    setWifi.clientIP = clientIP;
    setWifi.serverPort = serverPort;
    setWifi.ID_Device = ID_Device;
    
    Serial.println("Data WiFi berhasil disimpan");
    return true;
}



