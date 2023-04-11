#include <AccelStepper.h>
#include <MultiStepper.h>

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mutex>

#include <Audio.h>
#include "FS.h"
#include "SPIFFS.h"

// Digital I/O used
#define I2S_DOUT 22 // DIN connection
#define I2S_BCLK 25 // Bit clock
#define I2S_LRC 26  // Left Right Clock
const int mute = 0;
Audio audio;

const int numberElements = 18   ;
String mp3_files[numberElements] =
    {
        // "FovTut6thisIsItForAwayTeam.mp3",
        // "FovTut8ThisIsItForHomeTeam.mp3",
        // "FovTut6thisIsItForAwayTeam.mp3",
        // "FovTut8ThisIsItForHomeTeam.mp3",
        // "FovTut6thisIsItForAwayTeam.mp3",
        // "FovTut8ThisIsItForHomeTeam.mp3",
        // "FovTut6thisIsItForAwayTeam.mp3",
        // "FovTut8ThisIsItForHomeTeam.mp3"
        "ThisIsItForAwayTeam.mp3",
        "ThisIsItForAwayTeam.mp3",
        "ThisIsItForAwayTeam.mp3",
        // "FovTut2new.mp3",
        "FovTut3.mp3",
        "FovTut4New.mp3",
        "FovTut5New.mp3",
        "FovTut6.mp3",
        "FovTut7.mp3",
        "FovTut8.mp3",
        "FovTut9.mp3",
        "FovTut10.mp3",
        "FovTut11.mp3",
        "ThisIsItForAwayTeam.mp3",
        "FovTut8ThisIsItForHomeTeam.mp3"};

int i = 0;
const char *c;
int t = 0;
int period = 30;
unsigned long time_now = 0;
bool AudioOn = true;
bool eof = false;

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

int vibeMode = 0;

int pass = 0;

void AudioSetup()
{
    SPIFFS.begin(true);
    pinMode(mute, OUTPUT);
    digitalWrite(mute, HIGH);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(15); // 0...21
}

void listFilesInSPIFFS()
{
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

void tutorial()
{
    Serial.println("before tut1");
    audio.connecttoFS(SPIFFS, mp3_files[0].c_str());
    Serial.println("after tut1");
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
        audio.connecttoFS(SPIFFS, mp3_files[1].c_str());
        moveMotorsToGoalTask();
    }
    if (i == 2)
    {
        Serial.println("EOF2");
        audio.connecttoFS(SPIFFS, mp3_files[2].c_str());
        moveMotorsToCentreTask();
    }
    if (i == 3)
    {
        Serial.println("EOF3");
        audio.connecttoFS(SPIFFS, mp3_files[3].c_str());
        // Top vib
        // pwmMotor(4);
    }
    if (i == 4)
    {
        Serial.println("EOF4");
        audio.connecttoFS(SPIFFS, mp3_files[4].c_str());
        // Bottom vib
        // pwmMotor(3);
    }
    if (i == 5)
    {
        Serial.println("EOF5");
        audio.connecttoFS(SPIFFS, mp3_files[12].c_str());
        pwmMotor(5);
    }
    if (i == 6)
    {
        Serial.println("EOF6");
        audio.connecttoFS(SPIFFS, mp3_files[11].c_str());
        pwmMotor(2);
    }
    if (i == 7)
    {
        Serial.println("EOF7");
        audio.connecttoFS(SPIFFS, mp3_files[5].c_str());
        pwmMotor(2);
    }
    if (i == 8)
    {
        Serial.println("EOF8");
        audio.connecttoFS(SPIFFS, mp3_files[12].c_str());
        pwmMotor(2);
    }
    if (i == 9)
    {
        Serial.println("EOF9");
        audio.connecttoFS(SPIFFS, mp3_files[11].c_str());
        pwmMotor(2);
    }
    if (i == 10)
    {
        Serial.println("EOF10");
        audio.connecttoFS(SPIFFS, mp3_files[6].c_str());
        pwmMotor(2);
    }
    if (i == 11)
    {
        Serial.println("EOF11");
        audio.connecttoFS(SPIFFS, mp3_files[7].c_str());
        pwmMotor(2);
    }
    if (i == 12)
    {
        Serial.println("EOF12");
        audio.connecttoFS(SPIFFS, mp3_files[8].c_str());
        pwmMotor(2);
    }
    if (i == 13)
    {
        Serial.println("EOF5");
        audio.connecttoFS(SPIFFS, mp3_files[9].c_str());
        pwmMotor(2);
    }
    if (i == 14)
    {
        Serial.println("EOF5");
        audio.connecttoFS(SPIFFS, mp3_files[10].c_str());
        pwmMotor(2);
    }
    if (i == 15)
    {
        Serial.println("EOF5");
        audio.connecttoFS(SPIFFS, mp3_files[11].c_str());
        pwmMotor(2);
    }

    i++;
}

void moveMotorsToGoal(void *pvParameters)
{
    Serial.println("before moveStepsToPos");
    moveStepsToPos(0, 32, 5000, 5000);
    vTaskDelete(NULL);
}

void moveMotorsToCentre(void *pvParameters)
{
    moveStepsToPos(52, 32, 8000, 8000);
    vTaskDelete(NULL);
}

void moveMotorsToCentreTask()
{
    xTaskCreatePinnedToCore(moveMotorsToCentre, "moveMotorsToCentre", 10000, NULL, 1, NULL, 0);
}

void setup()
{
    Serial.begin(9600);

    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS initialization failed");
        return;
    }
    Serial.println("SPIFFS initialization successful");

    stepperSetup();
    hallSensorsSetup();
    homeSteppers(); // Commented just to speed up testing.
    pwmPinsSetup();
    listFilesInSPIFFS();

    AudioSetup();
}

bool begin_audio = true;

void loop()
{
    // audio.loop();
    Serial.println("hello");
    delay(5000);

    while (true)
    {
        audio.loop();

        if (begin_audio == true)
        {
            tutorial();
            begin_audio = false;
        }
    }
}

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

    int x_acc = 1000;
    int y_acc = 1000;

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
        steppers.run();
        delayMicroseconds(1); // Using Microseconds works in allowing it to run fast!
    }

    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
}

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

    vibeMode = 99; // reset vibeMode
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