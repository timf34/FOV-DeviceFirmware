// NOTE!!!: This file works but doesn't do much! We didn't finish writing out the tutorial. 


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

void AudioSetup()
{
    SPIFFS.begin(true);
    pinMode(mute, OUTPUT);
    digitalWrite(mute, HIGH);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(15); // 0...21
}

void tutorial()
{
    Serial.println("before tut1");
    audio.connecttoFS(SPIFFS, mp3_files[0].c_str());
    Serial.println("after tut1");
    i++;
}

// void audio_eof_mp3(const char *info)
void audio_eof_mp3()
{
    Serial.print("eof_mp3     ");
    // Serial.println(info);
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
        vTaskDelete(NULL); // As we are running it on the core; needs to be deleted.
    }
    i++;
}

void coreSetup(void *pvParameters)
{
    tutorial();
    vTaskDelete(NULL);
}

void setup()
{
    Serial.begin(9600);S
    AudioSetup();
    xTaskCreatePinnedToCore(
        coreSetup,   /* Task function. */
        "coreSetup", /* name of task. */
        20000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        2,           /* priority of the task */
        NULL,        /* Task handle to keep track of created task */
        1            /* pin task to core 1 */
    );
}

void loop()
{
    audio.loop();
    
}
