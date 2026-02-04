#include "clientServer.h"

void clientServer::begin() {
    indexHtml();
    firstInput();      
    wifiCredentials(); 
    notFound();     
    server.begin();
}

void clientServer::indexHtml() {
    const String localUrl = "http://192.168.7.2";
    auto redirectRoot = [this, localUrl](AsyncWebServerRequest *request) {
        request->redirect(localUrl);
    };
    
    server.on("/generate_204", HTTP_GET, redirectRoot);
    server.on("/fwlink", HTTP_GET, redirectRoot);
    server.on("/hotspot-detect.html", HTTP_GET, redirectRoot);
    server.on("/ncsi.txt", HTTP_GET, redirectRoot);
    server.on("/success.txt", HTTP_GET, redirectRoot);

    server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
}

void clientServer::firstInput() {
    server.on("/firstInput", HTTP_GET, [this](AsyncWebServerRequest *request) {
        String ssid, pass, ip;
        int port;
        // Baca data dari memory
        if(_memory->readWifi()) {
            String json = "{\"ssid\":\"" + _memory->getSsid() +
                          "\", \"password\":\"" + _memory->getPass() + 
                          "\", \"ip\":\"" + _memory->getServerIP() +
                          "\", \"client_ip\":\"" + _memory->getClientIP() +
                          "\", \"port\":\"" + String(_memory->getServerPort()) +
                          "\",\"ID_Device\":\"" + _memory->getID_Device() + "\"}";
            request->send(200, "application/json", json);
        } else {
            request->send(200, "application/json", "{}");
        }
    });
}

void clientServer::wifiCredentials() {
    server.on("/inputAddress", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("ssid") && request->hasParam("ip")) {
            String ssid = request->getParam("ssid")->value();
            String pass = request->getParam("password")->value();
            String ip = request->getParam("ip")->value();
            String client_ip = request->getParam("client_ip")->value();
            String portStr = request->getParam("port")->value();
            String ID_Device = request->getParam("ID_Device")->value();
            
            if(_memory->writeWifi(ssid, pass, ip, client_ip, portStr.toInt(), ID_Device)) {
                shouldReboot = true; // Trigger reboot di main loop
                request->send(200, "application/json", "{\"status\":\"success\"}");
            } else {
                request->send(500, "application/json", "{\"status\":\"failed\"}");
            }
        } else {
            request->send(400, "text/plain", "Bad Request");
        }
    });
}

void clientServer::notFound() {
    const String localUrl = "http://192.168.7.2";
    server.onNotFound([this, localUrl](AsyncWebServerRequest *request) {
        request->redirect(localUrl);
    });
}