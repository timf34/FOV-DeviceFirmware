#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>

class WiFiConnection
{
public:
    WiFiConnection(const char* awsCertCA, const char* awsCertCRT, const char* awsCertPrivate);
    void connectWiFi(const char *ssid, const char *password);
    void autoConnectWiFi(const char *apName, const char *apPassword);
    bool isConnected();
    int getStatus();
    WiFiClientSecure &getSecureClient();
    String getUserInputTopic();
    String getConnectedSSID();
    String getConnectedPSK();

private:
    WiFiClientSecure _secureClient;
    String _userInputTopic;
    String _shared_wifi_ssid;
    String _shared_wifi_psk;
};

#endif // WIFI_CONNECTION_H
