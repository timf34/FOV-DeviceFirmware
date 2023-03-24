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

std::mutex myMutex; // create a mutex object

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

long stepper_x_pos = 0;
long stepper_y_pos = 0;

int vibeMode = 10;
int possession = 66;
int pass = 0;
int receive = 0; 
int goal = 0;  


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

struct TaskParams
{
    int param1; // x coord
    int param2; // y coord
    int param3;
    int param4;
};

TaskParams taskParams = {xReceived, yReceived, xSpd, ySpd};

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

    myMutex.lock();

    xReceived = doc["X"];
    yReceived = doc["Y"];


    possession = doc["P"];
    pass = doc["Pa"];
    receive = doc["R"];
    goal = doc["G"];

    Serial.print("X Coord: ");
    Serial.println(xReceived);
    Serial.print("Y Coord: ");
    Serial.println(yReceived);

    // Home 
    if (possession == 0)
    {
        if (pass == 1)
        {
            vibeMode = 2;
        }
        if (receive == 1)
        {
            vibeMode = 3;
        }
    }
    // Away
    else if (possession == 1)
    {
        if (pass == 1)
        {
            vibeMode = 4;
        }
        if (receive == 1)
        {
            vibeMode = 5;
        }
    }

    Serial.println("xSpd before speedCalc: " + String(xSpd));

    stepper_x_pos = stepper_X.currentPosition();
    stepper_y_pos = stepper_Y.currentPosition();

    // Convert the received values to our unites by dividing by the conversion factor
    stepper_x_pos = stepper_x_pos / xConvert;
    stepper_y_pos = stepper_y_pos / yConvert;

    speedCalc(stepper_x_pos, stepper_y_pos, xReceived, yReceived);

    Serial.println("xSpd after speedCalc: " + String(xSpd));

    // Assign the new values to the shared resource
    taskParams.param1 = xReceived;
    taskParams.param2 = yReceived;
    taskParams.param3 = xSpd;
    taskParams.param4 = ySpd;

    myMutex.unlock();

    prevX = xReceived;
    prevY = yReceived;

    pwmMotor(vibeMode); // Where pass is equivalent to vibeMode
}

void stepperSetup()
{
    pinMode(ENABLE_X, OUTPUT);
    pinMode(ENABLE_Y, OUTPUT);
    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
    // Configure each stepper
    stepper_X.setMaxSpeed(15000);
    stepper_Y.setMaxSpeed(15000);
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

    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, LOW);

    for (;;)
    {
        myMutex.lock();

        TaskParams *passedParams = (TaskParams *)pvParameters;

        int *xReceived = &passedParams->param1;
        int *yReceived = &passedParams->param2;
        int *xSpd = &passedParams->param3;
        int *ySpd = &passedParams->param4;

        if (*xSpd > 12000)
        {
            *xSpd = 12000;
        }

        if (*xSpd < 5000)
        {
            *xSpd = 5000;
        }

        if (*ySpd > 12000)
        {
            *ySpd = 12000;
        }

        if (*ySpd < 5000)
        {
            *ySpd = 5000;
        }

        // Print the shared value
        Serial.println("TaskCore1: x = " + String(*xReceived));
        Serial.println("TaskCore1: y = " + String(*yReceived));
        Serial.println("TaskCore1: xSpd = " + String(*xSpd) + " ySpd = " + String(*ySpd));

        positionMove[0] = *xReceived * xConvert;
        positionMove[1] = *yReceived * yConvert;

        // // Deepcopy xReceived and yReceived to local variables (different memory addresses)
        int xReceivedLocal = *xReceived;
        int yReceivedLocal = *yReceived;
        int xSpdLocal = *xSpd;
        int ySpdLocal = *ySpd;

        myMutex.unlock();

        moveStepsToPos(xReceivedLocal, yReceivedLocal, xSpdLocal, ySpdLocal);

        vTaskDelay(50);
    }
}

void Core1Code(void *pvParameters)
{
    for (;;)
    {
        client.loop();
        vTaskDelay(50);
    }
}

void setup()
{
    Serial.begin(9600);

    stepper_X.setCurrentPosition(0);
    stepper_Y.setCurrentPosition(0);

    // Print the available memory
    Serial.print("Free memory: ");
    Serial.println(ESP.getFreeHeap());

    connectAWS();
    pwmPinsSetup();

    if (use_cores == true)
    {

        xTaskCreatePinnedToCore(
            Core0Code,           /* Task function. */
            "Core0Code",         /* name of task. */
            20000,               /* Stack size of task */
            (void *)&taskParams, /* parameter of the task */
            1,                   /* priority of the task */
            NULL,                /* Task handle to keep track of created task */
            0                    /* pin task to core 0 */
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

void speedCalc(float x1, float y1, float x2, float y2)
{                  // calculate required stepper speed based on distance the ball has the travel within an alotted time
    float t = 0.2; // timeframe to complete movement

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
    if (xSpd < 3000)
    {
        xSpd = 3000;
    }

    if (ySpd > 18000)
    {
        ySpd = 18000;
    }
    if (ySpd < 3000)
    {
        ySpd = 3000;
    }
}

void moveStepsToPos(long x, long y, int _xSpd, int _ySpd)
{
    Serial.print("X Speed: ");
    Serial.println(_xSpd);
    Serial.print("Y Speed: ");
    Serial.println(_ySpd);

    int x_acc = _xSpd * 5;
    int y_acc = _ySpd * 5;

    // TODO: here is the current issue. I keep getting a memory error when I try to set the max speed
    stepper_X.setMaxSpeed(_xSpd);
    stepper_Y.setMaxSpeed(_ySpd);
    stepper_X.setAcceleration(x_acc);
    stepper_Y.setAcceleration(y_acc);

    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, LOW);

    positionMove[0] = x * xConvert;
    positionMove[1] = y * yConvert;

    // Working multi stepper method. Commenting out to try runToPosition
    steppers.moveTo(positionMove);

    // Commenting out to try .run()
    // steppers.runSpeedToPosition();

    while (stepper_X.distanceToGo() != 0 || stepper_Y.distanceToGo() != 0)
    {
        // Note: Acceleration doesn't work when using `steppers` as opposed to individual steppers.  
        // steppers.run();

        stepper_X.run();
        stepper_Y.run();
        delayMicroseconds(1); // Using Microseconds works in allowing it to run fast!
    }

    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
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
    {                                       // Make the Stepper move CCW until the switch is activated
        stepper_X.moveTo(initial_homing_X); // Set the position to move to
        initial_homing_X--;                 // Decrease by 1 for next move if needed
        stepper_X.run();                    // Start moving the stepper
        delay(1);
    }

    stepper_X.setCurrentPosition(0);      // Set the current position as zero for now
    stepper_X.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_X.setAcceleration(homingSpd); // Set Acceleration of Stepper
    initial_homing_X = -1;

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

    stepper_Y.setCurrentPosition(0);
    Serial.println("Homing Y Axis Completed");

    // moveStepsToPos(1, 1);
    stepper_X.setCurrentPosition(1);
    stepper_Y.setCurrentPosition(1);

    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
}

// vibration response depending on events
// vibration response depending on events
void pwmMotor(int vibeMode)
{
    // VibeMode = 1  (OLD)
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

    // VibeMode = 2 Home Pass 
    if (vibeMode == 2)
    {
        ledcWrite(PWM2_Ch, 210);
        delay(30);
        ledcWrite(PWM2_Ch, 70);
        delay(120);
        ledcWrite(PWM2_Ch, 0);
        delay(65);
        ledcWrite(PWM2_Ch, 210);
        delay(30);
        ledcWrite(PWM2_Ch, 70);
        delay(120);
        ledcWrite(PWM2_Ch, 0);
    }
    // VibeMode = 3 Home Receive
    if (vibeMode == 3)
    {
        ledcWrite(PWM2_Ch, 210);
        delay(30);
        ledcWrite(PWM2_Ch, 70);
        delay(150);
        ledcWrite(PWM2_Ch, 0);
    }

    // VibeMode = 4 Away Pass
    if (vibeMode == 4)
    {
        ledcWrite(PWM1_Ch, 210);
        delay(20);
        ledcWrite(PWM1_Ch, 50);
        delay(120);
        ledcWrite(PWM1_Ch, 0);
        delay(65);
        ledcWrite(PWM1_Ch, 210);
        delay(20);
        ledcWrite(PWM1_Ch, 50);
        delay(120);
        ledcWrite(PWM1_Ch, 0);
    }

    // VibeMode = 5 Away Receive
    if (vibeMode == 5)
    {
        ledcWrite(PWM1_Ch, 210);
        delay(20);
        ledcWrite(PWM1_Ch, 50);
        delay(120);
        ledcWrite(PWM1_Ch, 0);
    }

    // TODO: need a "goal" vibeMode buzz 

    vibeMode = 99; // reset vibeMode
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