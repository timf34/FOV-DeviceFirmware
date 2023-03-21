#include <AccelStepper.h>
#include <MultiStepper.h>

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

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define AWS_IOT_PUBLISH_TOPIC "esp32/globalTest"
#define AWS_IOT_SUBSCRIBE_TOPIC "dalymount_IRL/pub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

#define ENABLE_Y 19
#define DIR_Y 13
#define STEP_Y 14
#define motorInterfaceType 1
AccelStepper stepper_Y = AccelStepper(motorInterfaceType, STEP_Y, DIR_Y);

#define ENABLE_X 23
#define DIR_X 32
#define STEP_X 33
AccelStepper stepper_X = AccelStepper(motorInterfaceType, STEP_X, DIR_X);

MultiStepper steppers;
long positionMove[2];

#define xConvert (26000 / 102)
#define yConvert (18500 / 64)

float xSpd = 8000;
float ySpd = 8000;

// End Stop Hall Sensor
#define hall_X 36
#define hall_Y 38

#define use_cores true

// Homing Sequence Variables
long initial_homing_X = -1; // X Axis
long initial_homing_Y = 1;  // Y Axis

bool loopTrue = true; // This will make it move!
bool spdChange = false;

int xReceived = 0;
int yReceived = 0;

int prevX = 0;
int prevY = 0;

long stepper_x_pos = 0;
long stepper_y_pos = 0;

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

    print_stepper_positions();

    prevX = xReceived;
    prevY = yReceived;
}

void stepperSetup()
{
    pinMode(ENABLE_X, OUTPUT);
    pinMode(ENABLE_Y, OUTPUT);
    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
    // Configure each stepper
    stepper_X.setMaxSpeed(25000);
    stepper_Y.setMaxSpeed(25000);
    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper_X);
    steppers.addStepper(stepper_Y);
}

void hallSensorsSetup()
{
    pinMode(hall_X, INPUT);
    pinMode(hall_Y, INPUT);
}

void Core0Code(void *pvParameters)
{
    stepperSetup();
    hallSensorsSetup();
    homeSteppers();

    for (;;)
    {
        loopMovement();
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

    stepper_X.setCurrentPosition(0);
    stepper_Y.setCurrentPosition(0);

    connectAWS();

    if (use_cores == true)
    {

        xTaskCreatePinnedToCore(
            Core0Code,   /* Task function. */
            "Core0Code", /* name of task. */
            20000,       /* Stack size of task */
            NULL,        /* parameter of the task */
            1,           /* priority of the task */
            NULL,        /* Task handle to keep track of created task */
            0            /* pin task to core 0 */
        );

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
}

void loop()
{

    Serial.println("Looping");
    Serial.println("Client state: " + String(client.state()));
    delay(5000);
}

long r = 20;            // Constant radius
float theta;            // Angle
float delta = PI / 100; // Increment

float a[9] = {0, PI / 4, PI / 2, (3 * PI) / 4, PI, (5 * PI) / 4, (3 * PI) / 2, (7 * PI) / 4, 2 * PI};
void loopMovement()
{
    moveStepsToPos(0, 0);
    print_stepper_positions();
    moveStepsToPos(48, 50);
    print_stepper_positions();
}

void moveStepsToPos(long x, long y)
{
    // stepper_X.setMaxSpeed(xSpd);
    // stepper_Y.setMaxSpeed(ySpd);
    // stepper_X.setAcceleration(xSpd * 50);
    // stepper_Y.setAcceleration(ySpd * 50);
    stepper_X.setMaxSpeed(15000);
    stepper_Y.setMaxSpeed(15000);
    stepper_X.setAcceleration(15000 * 50);
    stepper_Y.setAcceleration(15000 * 50);

    Serial.print("X Speed: ");
    Serial.println(xSpd);
    Serial.print("Y Speed: ");
    Serial.println(ySpd);

    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, HIGH);
    positionMove[0] = x * xConvert;
    positionMove[1] = y * yConvert;
    steppers.moveTo(positionMove);

    while (stepper_X.distanceToGo() != 0 || stepper_Y.distanceToGo() != 0)
    {
        steppers.run();
        delayMicroseconds(1); // Using Microseconds works in allowing it to run fast!
    }

    // steppers.runSpeedToPosition();

    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, HIGH);
}

void print_stepper_positions()
{
    stepper_x_pos = stepper_X.currentPosition();
    stepper_y_pos = stepper_Y.currentPosition();

    // Convert the received values to our unites by dividing by the conversion factor
    stepper_x_pos = stepper_x_pos / xConvert;
    stepper_y_pos = stepper_y_pos / yConvert;

    Serial.print("X position: ");
    Serial.println(stepper_x_pos);
    Serial.print("Y position: ");
    Serial.println(stepper_y_pos);
}

void homeSteppers()
{
    /////////////////////////////////////////////////////////////////////////////////////////
    // X AXIS HOMING
    ////////////////////////////////////////////////////////////////////////////////////////
    float homingSpd = 15000.0;
    Serial.println("Homing X Axis");
    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, HIGH);
    delay(5); // Wait for EasyDriver wake up

    //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepper_X.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_X.setAcceleration(homingSpd); // Set Acceleration of Stepper

    // Start Homing procedure of Stepper Motor at startup

    Serial.print("Stepper X is Homing . . . . . . . . . . . ");

    while (digitalRead(hall_X))
    { // Make the Stepper move CCW until the switch is activated
        // Serial.println(digitalRead(home_switch));
        stepper_X.moveTo(initial_homing_X); // Set the position to move to
        initial_homing_X--;                 // Decrease by 1 for next move if needed
        stepper_X.run();                    // Start moving the stepper
        delay(1);
    }

    stepper_X.setCurrentPosition(0);      // Set the current position as zero for now
    stepper_X.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_X.setAcceleration(homingSpd); // Set Acceleration of Stepper
    initial_homing_X = -1;

    while (!digitalRead(hall_X))
    { // Make the Stepper move CW until the switch is deactivated
        stepper_X.moveTo(initial_homing_X);
        stepper_X.run();
        initial_homing_X++;
        delay(1);
    }

    stepper_X.setCurrentPosition(0);
    Serial.println("Homing X Axis Completed");

    /////////////////////////////////////////////////////////////////////////////////////////
    // Y AXIS HOMING
    ////////////////////////////////////////////////////////////////////////////////////////
    Serial.println("Homing Y Axis");
    digitalWrite(ENABLE_Y, LOW);
    digitalWrite(ENABLE_X, HIGH);
    delay(3); // Wait for EasyDriver wake up

    //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepper_Y.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_Y.setAcceleration(homingSpd); // Set Acceleration of Stepper

    // Start Homing procedure of Stepper Motor at startup

    Serial.print("Stepper Y is Homing . . . . . . . . . . . ");

    while (digitalRead(hall_Y))
    { // Make the Stepper move CCW until the switch is activated
        // Serial.println(digitalRead(home_switch));
        stepper_Y.moveTo(initial_homing_Y); // Set the position to move to
        initial_homing_Y--;                 // Decrease by 1 for next move if needed
        stepper_Y.run();                    // Start moving the stepper
        delay(1);
    }

    stepper_Y.setCurrentPosition(0);      // Set the current position as zero for now
    stepper_Y.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_Y.setAcceleration(homingSpd); // Set Acceleration of Stepper
    initial_homing_Y = 1;

    while (!digitalRead(hall_Y))
    { // Make the Stepper move CW until the switch is deactivated
        stepper_Y.moveTo(initial_homing_Y);
        stepper_Y.run();
        initial_homing_Y++;
        delay(1);
    }

    stepper_Y.setCurrentPosition(0);
    Serial.println("Homing Y Axis Completed");

    // moveStepsToPos(1, 1);
    stepper_X.setCurrentPosition(1);
    stepper_Y.setCurrentPosition(1);

    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
}
