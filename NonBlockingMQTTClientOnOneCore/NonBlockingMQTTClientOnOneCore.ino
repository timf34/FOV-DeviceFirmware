#include "secrets.h"
#include <WiFiClientSecure.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mutex>

#define AWS_IOT_PUBLISH_TOPIC "esp32/globalTest"
#define AWS_IOT_SUBSCRIBE_TOPIC "dalymount_IRL/pub"

#define run_second_core true

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

bool loopTrue = true; // This will make it move!
bool spdChange = false;

int xReceived = 0;
int yReceived = 0;

float xSpd = 0;
float ySpd = 0;

int prevX = 0;
int prevY = 0;

class FPSCounter
{
private:
    unsigned long start_time;
    unsigned long end_time;
    float fps;

public:
    FPSCounter() {}

    void start()
    {
        start_time = millis();
    }

    void stop()
    {
        end_time = millis();
        unsigned long execution_time = end_time - start_time;
        fps = 1000.0 / execution_time;
    }

    float getFPS()
    {
        return fps;
    }
};

FPSCounter fpsCounter;

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

void connectAWS()
{
    wifiManagerSetup();

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker on the AWS endpoint we defined earlier
    client.setServer(AWS_IOT_ENDPOINT, 8883);

    // Create a message handler
    client.setCallback(messageHandler);

    Serial.print("Connecting to AWS IOT");

    while (!client.connect(THINGNAME))
    {
        Serial.print(".");
        delay(100);
    }

    if (!client.connected())
    {
        Serial.println("AWS IoT Timeout!");
        return;
    }

    // Subscribe to a topic
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.println("AWS IoT Connected!");
}

void messageHandler(char *topic, byte *payload, unsigned int length)
{

    Serial.print("incoming: ");
    Serial.println(topic);

    // Read in the variable.
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    float timestamp = doc["T"];
    xReceived = doc["X"];
    yReceived = doc["Y"];

    Serial.print("X Coord: ");
    Serial.println(xReceived);
    Serial.print("Y Coord: ");
    Serial.println(yReceived);

    speedCalc(prevX, prevY, xReceived, yReceived);

    Serial.print("SpdC X Coord: ");
    Serial.println(xReceived);
    Serial.print("SpdCY Coord: ");
    Serial.println(yReceived);

    prevX = xReceived;
    prevY = yReceived;
}

void Core0Code(void *pvParameters)
{
    // For our AWS code:)
    for (;;)
    {
        fpsCounter.start();
        client.loop();
        vTaskDelay(100);

        fpsCounter.stop();
        Serial.print("FPS: ");
        float fps = fpsCounter.getFPS();
        Serial.println(fps);
    }
}

void Core1Code(void *pvParameters)
{
    for (;;)
    {
        // Random task to ensure that the core is running and core0 is non-blocking
        Serial.println("Core 1 is running");

        // Print the resources available
        Serial.print("Free memory: ");
        Serial.println(ESP.getFreeHeap());

        Serial.print("CPU Frequency: ");
        Serial.println(ESP.getCpuFreqMHz());

        vTaskDelay(1000);
    }
}

void setup()
{
    Serial.begin(9600);
    connectAWS();

    // Print how much memory is available
    Serial.print("Free memory: ");
    Serial.println(ESP.getFreeHeap());

    xTaskCreatePinnedToCore(
        Core0Code,   /* Task function. */
        "Core0Code", /* name of task. */
        5000,        /* Stack size of task */
        NULL,        /* parameter of the task */
        2,           /* priority of the task */
        NULL,        /* Task handle to keep track of created task */
        0            /* pin task to core 0 */
    );

    xTaskCreatePinnedToCore(
        Core1Code,   /* Task function. */
        "Core1Code", /* name of task. */
        5000,        /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task */
        NULL,        /* Task handle to keep track of created task */
        1            /* pin task to core 1 */
    );
}

void loop()
{
    Serial.println("Loop");
    delay(5000);
}

void speedCalc(float x1, float y1, float x2, float y2)
{                // calculate required stepper speed based on distance the ball has the travel within an alotted time
    float t = 2; // timeframe to complete movement

    float dx = abs(x2 - x1); // distance to next coordinate
    float dy = abs(y2 - y1);
    float sx = dx / t; // in mm/s //speed needed to get to next point within allowed timeframe
    float sy = dy / t;
    xSpd = (sx * (26000 / 103)); // convert speed  mm/s to stepper speed
    ySpd = (sy * (18500 / 65));

    // Top speed is 20000 before motors start jamming
    if (xSpd > 18000)
    {
        xSpd = 18000;
    }
    if (ySpd > 18000)
    {
        ySpd = 18000;
    }
}