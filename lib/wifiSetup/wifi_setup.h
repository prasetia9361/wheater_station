#pragma once
#include <DNSServer.h>
#include <WiFi.h>
class WifiSetup {
   private:
    DNSServer dnsServer;
    bool modeAP = false;
    bool modeSTA = false;
    String macAddress = WiFi.macAddress();
    int counter = 0;

   public:
    // void begin();
    WifiSetup();
    // void begin();
    void setupWiFiAP(const char *ssid, const char *password);
    void loopDns();
    void setupWiFiSTA(const char *ssid, const char *pass);
    void connectAP();
    void connectSTA();
    // String getMac() { return macAddress; }
    void disconnectAP();
    void disconnectSTA();
    bool isConnectedAP() { return modeAP; }
    void setconnectedAP(bool status) { modeAP = status; }
    bool isConnectedSTA() { return modeSTA; }
    void setconnectedSTA(bool status) { modeSTA = status; }
    int counterConnect() { return counter; }
    bool reconnect() { return WiFi.reconnect(); }
    void disconnect();
};
