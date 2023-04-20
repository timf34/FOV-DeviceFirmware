#include <Arduino.h>
#include <SPI.h>
#include <PubSubClient.h>
#include "secrets.h"
#include "WiFiConnection.h"

#define AWS_IOT_PUBLISH_TOPIC "esp32/globalTest"
#define AWS_IOT_SUBSCRIBE_TOPIC "dalymount_IRL/pub"

WiFiConnection wifiConnection(AWS_CERT_CA, AWS_CERT_CRT, AWS_CERT_PRIVATE);
PubSubClient client(wifiConnection.getSecureClient());

const char *ssid = "tim";
const char *password = "shopkeeper2";
const char *apName = "ESP32";
const char *apPassword = "password";

void callback(char *topic, byte *payload, unsigned int length)
{
    // handle message arrived
    Serial.print("incoming: ");
    Serial.println(topic);
}

void setup()
{
    Serial.begin(9600);

    // Connect to Wi-Fi
    wifiConnection.connectWiFi(ssid, password);
    // or use autoConnectWiFi to automatically connect to Wi-Fi
    // wifiConnection.autoConnectWiFi(apName, apPassword);

    // Print connection details
    Serial.println("Connected to: " + wifiConnection.getConnectedSSID());
    Serial.println("PSK: " + wifiConnection.getConnectedPSK());
}

void loop()
{
    Serial.print("WiFi status: ");
    Serial.println(wifiConnection.isConnected());

    if (!wifiConnection.isConnected())
    {
        Serial.println("Reconnecting to Wi-Fi...");
        wifiConnection.connectWiFi(ssid, password);
    }
}
