#include "secrets.h"
#include <WiFiClientSecure.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#define AWS_IOT_PUBLISH_TOPIC "esp32/globalTest"
#define AWS_IOT_SUBSCRIBE_TOPIC "dalymount_IRL/pub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

int xReceived = 0;
int yReceived = 0;

int prevX = 0;
int prevY = 0;

int pass = 0;
int vibeMode = 10;

// Vibration Motor in pitch
#define VIB_GPIO1 9
#define PWM1_Ch 0
#define PWM1_Res 8
#define PWM1_Freq 1000

// Vibration motor on body
#define VIB_GPIO2 21
#define PWM2_Ch 1
#define PWM2_Res 8
#define PWM2_Freq 1000

int PWM1_DutyCycle = 0;
int PWM2_DutyCycle = 0;

void pwmPinsSetup()
{

    // Setup Motor 1
    ledcAttachPin(VIB_GPIO1, PWM1_Ch);
    ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
    // Set up Motor 2
    ledcAttachPin(VIB_GPIO2, PWM2_Ch);
    ledcSetup(PWM2_Ch, PWM2_Freq, PWM2_Res);

    ledcWrite(PWM1_Ch, 0);
    ledcWrite(PWM2_Ch, 0);
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

    pass = doc["Pa"];

    Serial.print("X Coord: ");
    Serial.println(xReceived);
    Serial.print("Y Coord: ");
    Serial.println(yReceived);
    Serial.print("Pass: ");
    Serial.println(pass);

    Serial.print("SpdC X Coord: ");
    Serial.println(xReceived);
    Serial.print("SpdCY Coord: ");
    Serial.println(yReceived);

    prevX = xReceived;
    prevY = yReceived;

    pwmMotor(pass); // Where pass is equivalent to vibeMode
}

void Core0Code(void *pvParameters)
{
    for (;;)
    {
        vTaskDelay(100);
    }
}

void Core1Code(void *pvParameters)
{
    // For our AWS code:)
    for (;;)
    {
        client.loop();
    }
}

void setup()
{
    Serial.begin(9600);

    connectAWS();
    pwmPinsSetup();

    // TODO: note this code doesn't execute if we set the priotity to 0... and if we set it to 1, the above code doesn't execute... Lets try switching them around once I'm back.
    xTaskCreatePinnedToCore(
        Core1Code,   /* Task function. */
        "Core1Code", /* name of task. */
        20000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        2,           /* priority of the task */
        NULL,        /* Task handle to keep track of created task */
        1            /* pin task to core 1 */
    );
}

void loop()
{

    Serial.println("Looping");
    Serial.println("Client state: " + String(client.state()));
    delay(5000);
}

// vibration response depending on events
void pwmMotor(int vibeMode)
{
    // VibeMode = 1 Pass
    if (vibeMode == 1)
    {
        ledcWrite(PWM2_Ch, 200);
        ledcWrite(PWM1_Ch, 200);
        delay(40);
        ledcWrite(PWM2_Ch, 105);
        ledcWrite(PWM1_Ch, 65);
        delay(350);
        ledcWrite(PWM2_Ch, 0);
        ledcWrite(PWM1_Ch, 0);
    }
}