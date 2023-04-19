/*
 Reconnecting MQTT example - non-blocking

 This sketch demonstrates how to keep the client connected
 using a non-blocking reconnect function. If the client loses
 its connection, it attempts to reconnect every 5 seconds
 without blocking the main loop.

*/
#include <Arduino.h>
#include <SPI.h>
#include <PubSubClient.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "WiFi.h"

#define AWS_IOT_PUBLISH_TOPIC "esp32/globalTest"
#define AWS_IOT_SUBSCRIBE_TOPIC "dalymount_IRL/pub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void callback(char *topic, byte *payload, unsigned int length)
{
    // handle message arrived
    Serial.print("incoming: ");
    Serial.println(topic);
}

void wifiManagerSetup()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin("tim", "shopkeeper2");
    Serial.println("Connecting to Wi-Fi (ensure that it's 2.4Ghz!!!");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
}

long lastReconnectAttempt = 0;

boolean reconnect()
{
    if (client.connect(THINGNAME))
    {
        client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    }
    return client.connected();
}

void wifi_and_permissions_setup()
{
    wifiManagerSetup();

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(callback);
}

void setup()
{
    Serial.begin(9600);
    wifi_and_permissions_setup();

    delay(1500);
    lastReconnectAttempt = 0;

    while (!client.connect(THINGNAME)){
        Serial.print(".");
        delay(100);
    }
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
}

void loop()
{
    Serial.print("Client state: ");
    Serial.println(client.state());
    if (!client.connected())
    {
        long now = millis();
        if (now - lastReconnectAttempt > 5000)
        {
            lastReconnectAttempt = now;
            // Attempt to reconnect
            if (reconnect())
            {
                lastReconnectAttempt = 0;
            }
        }
    }
    else
    {
        // Client connected

        client.loop();
    }

    
    delay(500);   
}