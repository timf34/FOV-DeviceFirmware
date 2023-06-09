#include "WiFiConnection.h"
#include "secrets.h"

WiFiConnection::WiFiConnection(const char* awsCertCA, const char* awsCertCRT, const char* awsCertPrivate) {
    _secureClient.setCACert(awsCertCA);
    _secureClient.setCertificate(awsCertCRT);
    _secureClient.setPrivateKey(awsCertPrivate);
}


void WiFiConnection::connectWiFi(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.println("Connecting to Wi-Fi...");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to Wi-Fi.");
}

void WiFiConnection::autoConnectWiFi(const char* apName, const char* apPassword) {
    WiFi.mode(WIFI_STA);
    WiFiManager wm;

    // Erases wifi creds 
    // wm.resetSettings();
    bool res;
    WiFiManagerParameter custom_text_box("my_text", "Enter your string here", "default string", 50);
    wm.addParameter(&custom_text_box);

    res = wm.autoConnect(apName, apPassword); // anonymous ap

    if (!res)
    {
        Serial.println("Failed to connect");
    }
    else
    {
        Serial.println("connected");
    }
    _userInputTopic = custom_text_box.getValue();
    _shared_wifi_psk = WiFi.psk();
    _shared_wifi_ssid = WiFi.SSID();
}

bool WiFiConnection::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

int WiFiConnection::getStatus() {
    return WiFi.status();
}

WiFiClientSecure& WiFiConnection::getSecureClient() {
    return _secureClient;
}

String WiFiConnection::getUserInputTopic() {
    return _userInputTopic;
}

String WiFiConnection::getConnectedSSID() {
    return _shared_wifi_ssid;
}

String WiFiConnection::getConnectedPSK() {
    return _shared_wifi_psk;
}
