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

const int numberElements = 4;
String mp3_files[numberElements] = {"FovTut6thisIsItForAwayTeam.mp3", "FovTut8ThisIsItForHomeTeam.mp3", "FovTut2.mp3", "FovTut12.mp3"};

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

int pass = 0;

void AudioSetup()
{
    SPIFFS.begin();
    pinMode(mute, OUTPUT);
    digitalWrite(mute, HIGH);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21); // 0...21
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

void audio_eof_mp3(const char *info)
{
    Serial.print("eof_mp3     ");
    Serial.println(info);
    static int i = 1;
    if (i == 1)
    {
        Serial.println("EOF1");
        audio.connecttoFS(SPIFFS, mp3_files[1].c_str());
    }
    if (i == 2)
    {
        Serial.println("EOF2");
        audio.connecttoFS(SPIFFS, mp3_files[2].c_str());
        // Top vib
        vTaskDelete(NULL);  // As we are running it on the core; needs to be deleted. 

    }
    i++;    

}

// void Core0Code(void *pvParameters)
// {
//     // Setup 
//     for(;;)
//     {
//         Serial.println("Core 0");
//         vTaskDelay(50);
//     }
// }

void setup()
{
    Serial.begin(9600);

    stepperSetup();
    hallSensorsSetup();
    homeSteppers();
    pwmPinsSetup();
    AudioSetup();

    // xTaskCreatePinnedToCore(
    //     audio_eof_mp3,           /* Task function. */
    //     "audio_eof_mp3",         /* name of task. */
    //     20000,               /* Stack size of task */
    //     NULL /* parameter of the task */
    //     1,                   /* priority of the task */
    //     NULL,                /* Task handle to keep track of created task */
    //     0                    /* pin task to core 0 */
    // );

    tutorial();
}

void loop()
{
    Serial.println("Looping");
    delay(5000);
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

    int x_acc = _xSpd * 15;
    int y_acc = _ySpd * 15;

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