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

#include <Audio.h>
#include "FS.h"
#include "SPIFFS.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define AWS_IOT_PUBLISH_TOPIC "esp32/globalTest"
#define AWS_IOT_SUBSCRIBE_TOPIC "dalymount_IRL/pub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Audio intialization
// Digital I/O used
#define I2S_DOUT 22 // DIN connection
#define I2S_BCLK 25 // Bit clock
#define I2S_LRC 26  // Left Right Clock
const int mute = 0;
Audio audio;

const int numberElements = 18;
String mp3_files[numberElements] =
    {
        "FovTut1a.mp3",  
        "FovTut1b.mp3",
        "FovTut2new.mp3",
        "NewTut3.mp3",
        "NewTut4.mp3",
        "NewTut5.mp3",
        "NewTut6.mp3",
        "NewTut7.mp3",
        "NewTut8.mp3",
        "NewTut9.mp3",
        "NewTut10.mp3",
        "NewTut11.mp3",
        "NewTut12.mp3",
        "ThisIsItForAwayTeam.mp3",  // index 13
        "ThisIsItForHomeTeam.mp3"}; // index 14

int i = 0;
const char *c;
int t = 0;
int period = 30;
unsigned long time_now = 0;
bool AudioOn = true;
bool eof = false;
bool exit_loop = false;
bool begin_audio = true;

// End of audio initialization

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
int prevPossession = 66;
int tutorial_num = 0;
int pass = 0;
int receive = 0;
int goal = 0;

String userInputTopic;
String shared_wifi_psk, shared_wifi_ssid;

void AudioSetup()
{
    SPIFFS.begin(true);
    pinMode(mute, OUTPUT);
    digitalWrite(mute, HIGH);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21); // 0...21
}

void listFilesInSPIFFS()
{
    // Note: ensure that `SPIFFS.begin()` has been called before this function
    Serial.println("Ensure SPIFFS.begin() has been called before calling listFilesInSPIFFS()");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    Serial.println("Files in SPIFFS:");

    while (file)
    {
        Serial.print("File: ");
        Serial.print(file.name());
        Serial.print(" - Size: ");
        Serial.println(file.size());
        file = root.openNextFile();
    }
}

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

// Hardcoded wifi
// void wifiManagerSetup()
// {
//     WiFi.mode(WIFI_STA);
//     WiFi.begin("tim", "shopkeeper2");
//     Serial.println("Connecting to Wi-Fi (ensure that it's 2.4Ghz!!!");

//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(500);
//         Serial.print(".");
//     }
// }

// // "Auto" wifi
void wifiManagerSetup()
{
    WiFi.mode(WIFI_STA);
    WiFiManager wm;

    // Uncomment for wifi page setup
    wm.resetSettings();
    bool res;
    WiFiManagerParameter custom_text_box("my_text", "Enter your string here", "default string", 50);
    wm.addParameter(&custom_text_box);

    // Change what credentials of local network generated by the board
    res = wm.autoConnect("TOV_thing420"); // anonymous ap

    if (!res)
    {
        Serial.println("Failed to connect");
    }
    else
    {
        // if you get here you have connected to the WiFi
        Serial.println("connected");
    }
    userInputTopic = custom_text_box.getValue();
    shared_wifi_psk = WiFi.psk();
    shared_wifi_ssid = WiFi.SSID();
}

void tutorial()
{
    Serial.println("before tut1");
    audio.connecttoFS(SPIFFS, mp3_files[0].c_str()); // TODO: temporarily using a shorter audio file for testing.
    Serial.println("after tut1");
    Serial.print("Here is the mp3_files array: ");
    for (int i = 0; i < 15; i++)
    {
        Serial.print("i: ");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(mp3_files[i]);
        Serial.println(" ");
    }
    i++;
}

void moveMotorsToGoalTask()
{
    xTaskCreatePinnedToCore(moveMotorsToGoal, "moveMotorsToGoal", 10000, NULL, 2, NULL, 0);
}

// void audio_eof_mp3(const char *info)
void audio_eof_mp3(const char *info)
{
    Serial.print("eof_mp3     ");
    Serial.println(info);
    static int i = 1;
    if (i == 1)
    {
        Serial.println("EOF1");
        audio.connecttoFS(SPIFFS, mp3_files[1].c_str()); // "Using whichever hand feels..."
    }
    if (i == 2)
    {
        Serial.println("EOF2");
        moveMotorsToGoalTask();
        audio.connecttoFS(SPIFFS, mp3_files[2].c_str()); // "Now the ball will start moving slowly..."
    }
    if (i == 3)
    {
        Serial.println("EOF3");
        moveMotorsToCentreTask();
        audio.connecttoFS(SPIFFS, mp3_files[3].c_str()); // "Now, the ball will move back to the centre"
    }
    if (i == 4)
    {
        Serial.println("EOF4");
        pwmMotor(4); 
        audio.connecttoFS(SPIFFS, mp3_files[4].c_str()); // "You should feel it now. It has a deeper, lower pitch rumble"
    }
    if (i == 5)
    {
        Serial.println("EOF5");
        pwmMotor(2);                                      // Away team kick
        audio.connecttoFS(SPIFFS, mp3_files[5].c_str()); // "You should feel it now, it has a sharp, high pitch buzz"
    }
    if (i == 6)
    {
        Serial.println("EOF6");
        audio.connecttoFS(SPIFFS, mp3_files[14].c_str()); // "This is it for home team"
    }
    if (i == 7)
    {
        Serial.println("EOF7");
        pwmMotor(4);                                      // Home team kick
        audio.connecttoFS(SPIFFS, mp3_files[13].c_str()); // This is it for away team"
    }
    if (i == 8)
    {
        Serial.println("EOF8");
        pwmMotor(2);                                     // Away player ball leaves player
        audio.connecttoFS(SPIFFS, mp3_files[6].c_str()); // "When possession of the ball changes..."
    }
    if (i == 9)
    {
        Serial.println("EOF9");
        pwmMotor(5); // Home team poschange
        audio.connecttoFS(SPIFFS, mp3_files[7].c_str()); // "This is possession changing to the away team"
    }
    if (i == 10)
    {
        Serial.println("EOF10");
        pwmMotor(3);  // Away team poschange
        audio.connecttoFS(SPIFFS, mp3_files[8].c_str()); // "When a goal is scored, a longer more intense vibration will occur"
    }
    if (i == 11)
    {
        Serial.println("EOF11");
        pwmMotor(4);  // Home team kick
        audio.connecttoFS(SPIFFS, mp3_files[9].c_str()); // "This is an away player kicking or heading the ball"
    }
    if (i == 12)
    {
        Serial.println("EOF12");
        pwmMotor(2);  // Away team kick
        audio.connecttoFS(SPIFFS, mp3_files[10].c_str()); // "This is a possession changing to the home team"
    }
        if (i == 13)
    {
        Serial.println("EOF12");
        pwmMotor(5); // Home team poschange
        audio.connecttoFS(SPIFFS, mp3_files[11].c_str()); // "This is possession changing to the away team"
    }
    if (i == 14)
    {
        Serial.println("EOF12");
        pwmMotor(3);  // Away team poschange
        audio.connecttoFS(SPIFFS, mp3_files[12].c_str()); // "With this information..."
    }
    if (i == 15)
    {
        Serial.println("EOF15");
        exit_loop = true;
        Serial.println("exiting loop");
    }
    i++;
}

void moveMotorsToGoal(void *pvParameters)
{
    Serial.println("before moveStepsToPos");
    moveStepsToPos(0, 32, 5000, 5000);
    vTaskDelete(NULL);
    Serial.println("Task deleted: moveMotorsToGoal");
}

void moveMotorsToCentre(void *pvParameters)
{
    moveStepsToPos(52, 32, 8000, 8000);
    vTaskDelete(NULL);
    Serial.println("Task deleted: moveMotorsToCentre");
}

void moveMotorsToCentreTask()
{
    xTaskCreatePinnedToCore(moveMotorsToCentre, "moveMotorsToCentre", 10000, NULL, 1, NULL, 0);
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
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC, 0);
    Serial.println("AWS IoT Connected!");
}

void audio_tutorial()
{

    // Disconnect from the MQTT broker
    client.unsubscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    client.disconnect();

    // Delete core1 task
    // vTaskDelete(Core1Code)

    audio.connecttoFS(SPIFFS, mp3_files[0].c_str()); // "Welcome to the tutorial"
    i++;

    while (!exit_loop)
    {
        audio.loop();

        if (begin_audio == true)
        {
            tutorial();
            begin_audio = false;
        }
    }

    reconnect_to_aws();
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
    tutorial_num = doc["T"];

    possession = doc["P"];
    pass = doc["Pa"];
    goal = doc["G"];

    if (tutorial_num == DEVICE_NUM)
    {
        Serial.println("Tutorial 1");
        audio_tutorial();
    }

    // home
    if (possession == 0)
    {
        if (pass == 1)
        {
            vibeMode = 2;
        }
        else if (possession != prevPossession)
        {
            vibeMode = 3;
        }
        else if (goal == 1)
        {
            vibeMode = 1;
        }
        else
        {
            vibeMode = 0;
        }
        prevPossession = possession; // TODO: If I make this a typo, why doesn't VSCode pick it up!!??? (i.e. it should catch if its 'posesion' but it doesn't!)
    }
    // home
    else if (possession == 1)
    {
        if (pass == 1)
        {
            vibeMode = 4;
        }
        else if (possession != prevPossession)
        {
            vibeMode = 5;
        }
        else if (goal == 1)
        {
            vibeMode = 1;
        }
        else
        {
            vibeMode = 0;
        }
        prevPossession = possession;
    }

    else
    {
        vibeMode = 0;
    }

    stepper_x_pos = stepper_X.currentPosition();
    stepper_y_pos = stepper_Y.currentPosition();

    // Convert the received values to our unites by dividing by the conversion factor
    stepper_x_pos = stepper_x_pos / xConvert;
    stepper_y_pos = stepper_y_pos / yConvert;

    speedCalc(stepper_x_pos, stepper_y_pos, xReceived, yReceived);

    // Serial.println("xSpd after speedCalc: " + String(xSpd));

    // Assign the new values to the shared resource
    taskParams.param1 = xReceived;
    taskParams.param2 = yReceived;
    taskParams.param3 = xSpd;
    taskParams.param4 = ySpd;

    myMutex.unlock();

    prevX = xReceived;
    prevY = yReceived;

    Serial.print("vibeMode: ");
    Serial.println(vibeMode);

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
    // Note: these seem to have to be initialized here for things to work.
    // stepperSetup();
    // hallSensorsSetup();
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

        if (*xSpd > 8500)
        {
            *xSpd = 8500;
        }

        if (*xSpd < 5000)
        {
            *xSpd = 5000;
        }

        if (*ySpd > 8500)
        {
            *ySpd = 8500;
        }

        if (*ySpd < 5000)
        {
            *ySpd = 5000;
        }

        // Print the shared value
        // Serial.println("TaskCore1: x = " + String(*xReceived));
        // Serial.println("TaskCore1: y = " + String(*yReceived));
        // Serial.println("TaskCore1: xSpd = " + String(*xSpd) + " ySpd = " + String(*ySpd));
        Serial.print("Client status: ");
        Serial.println(client.state());

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

long lastReconnectAttempt = 0;

boolean reconnect_to_aws()
{
    while (!client.connect(THINGNAME))
    {
        Serial.print(".");
        delay(100);
    }
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    return client.connected();
}

void reconnectToWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(shared_wifi_ssid.c_str(), shared_wifi_psk.c_str());
    Serial.println("Connecting to Wi-Fi (ensure that it's 2.4Ghz!!!");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
}

void Core1Code(void *pvParameters)
{
    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {   
            // Hardcoded wifi reconnect
            // Serial.println("WiFi Disconnected! Reconnecting...");
            // wifiManagerSetup();

            // Auto wifi reconnect 
            reconnectToWiFi();
        }
        else if (!client.connected())
        {
            long now = millis();
            if (now - lastReconnectAttempt > 5000)
            {
                lastReconnectAttempt = now;
                // Attempt to reconnect
                if (reconnect_to_aws())
                {
                    lastReconnectAttempt = 0;
                }
            }
        }  
        else
        {
            client.loop();
        }
        Serial.println("Client 1 code");
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status());
        Serial.print("Client status: ");
        Serial.println(client.state());
        vTaskDelay(60);
    }
}


void setup()
{
    Serial.begin(9600);

    client.setKeepAlive(300);  // Set the keep alive interval to 300 seconds

    stepper_X.setCurrentPosition(0);
    stepper_Y.setCurrentPosition(0);

    // Print the available memory
    Serial.print("Free memory: ");
    Serial.println(ESP.getFreeHeap());

    pwmPinsSetup();
    stepperSetup();
    hallSensorsSetup();
    homeSteppers();
    moveStepsToPos(52, 32, 5000, 5000); // Centre up the fingerpiece

    AudioSetup();
    listFilesInSPIFFS();

    connectAWS(); 

    client.unsubscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    client.disconnect();

    // while (!exit_loop)
    // {
    //     audio.loop();

    //     if (begin_audio == true)
    //     {
    //         tutorial();
    //         begin_audio = false;
    //     }
    // }

    reconnect_to_aws();  // Note: I might not be able to call this as a void function, but I'll try it for now.

    Serial.println("Entering game day mode");

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
            10000,       /* Stack size of task */
            NULL,        /* parameter of the task */
            2,           /* priority of the task */
            NULL,        /* Task handle to keep track of created task */
            1            /* pin task to core 1 */
        );
    }
}

void loop()
{
    // Serial.println("Looping");
    // Serial.println("Client state: " + String(client.state()));
    // Note: literally printing any statement here while we are doing the audio tutorial, will cause no audio to play! Although the timing of the fingerpiece movement and vibrations will still be correct!
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
    if (xSpd > 8500)
    {
        xSpd = 8500;
    }
    if (xSpd < 3000)
    {
        xSpd = 3000;
    }

    if (ySpd > 8500)
    {
        ySpd = 8500;
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
    Serial.print("Client state: ");
    Serial.println(client.state());

    int x_acc = 65000;
    int y_acc = 65000;

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

    // Start a timer to check if the homing is taking too long
    unsigned long homingTimer = millis();

    while (digitalRead(hall_X))
    {                                       // Make the Stepper move CCW until the switch is activated
        stepper_X.moveTo(initial_homing_X); // Set the position to move to
        initial_homing_X--;                 // Decrease by 1 for next move if needed
        stepper_X.run();                    // Start moving the stepper
        delay(1);
        if (millis() - homingTimer > 30000)
        {
            Serial.println("Homing X Axis Timed Out");
            break;
        }
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

    // Start a timer to check if the homing is taking too long
    homingTimer = millis();

    while (digitalRead(hall_Y))
    {                                       // Make the Stepper move CCW until the switch is activated
        stepper_Y.moveTo(initial_homing_Y); // Set the position to move to
        initial_homing_Y--;                 // Decrease by 1 for next move if needed
        stepper_Y.run();                    // Start moving the stepper
        delay(1);
        if (millis() - homingTimer > 30000)
        {
            Serial.println("Homing Y Axis Timed Out");
            break;
        }
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

void pwmMotor(int vibeMode)
{

    // VibeMode = 1  goal
    if (vibeMode == 1)
    {
        ledcWrite(PWM1_Ch, 120);
        ledcWrite(PWM2_Ch, 70);
        delay(2000);
        ledcWrite(PWM1_Ch, 0);
        ledcWrite(PWM2_Ch, 0);
    }

    if (vibeMode == 2) // away pass
    {
        ledcWrite(PWM1_Ch, 210);
        delay(25);
        ledcWrite(PWM1_Ch, 145);
        delay(60);
        ledcWrite(PWM1_Ch, 0);
    }
    // VibeMode = 3 away POSCHANGE
    if (vibeMode == 3)
    {
        ledcWrite(PWM1_Ch, 210);
        delay(25);
        ledcWrite(PWM1_Ch, 145);
        delay(240);
        ledcWrite(PWM1_Ch, 0);
    }

    // VibeMode = 4 home Pass
    if (vibeMode == 4)
    {
        ledcWrite(PWM2_Ch, 210);
        delay(40);
        ledcWrite(PWM2_Ch, 75);
        delay(100);
        ledcWrite(PWM2_Ch, 0);
    }

    // VibeMode = 5 home POSCHANGE
    if (vibeMode == 5)
    {
        ledcWrite(PWM2_Ch, 210);
        delay(50);
        ledcWrite(PWM2_Ch, 70);
        delay(350);
        ledcWrite(PWM2_Ch, 0);
    }

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