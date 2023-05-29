/*
Plays Yolo Bitches on Setup 

Press button 1 or 3 to play OneZero
Press button 2 to play SeventyTwo
*/
#include "Arduino.h"
// #include "WiFi.h"
#include <Audio.h>
#include "FS.h"
#include "SPIFFS.h"
#include <ezButton.h>

const int swt1 = 12;
const int swt2 = 4;
const int swt3 = 2;

const int BUTTON_NUM = 3;
const int BUTTON_1_PIN = 12;
const int BUTTON_2_PIN = 4;
const int BUTTON_3_PIN = 2;

ezButton buttonArray[] = {
    ezButton(BUTTON_1_PIN),
    ezButton(BUTTON_2_PIN),
    ezButton(BUTTON_3_PIN),
};

unsigned long timePress[] = {0, 0, 0};

void btnSetup()
{
    pinMode(swt1, INPUT_PULLUP);
    pinMode(swt2, INPUT_PULLUP);
    pinMode(swt3, INPUT_PULLUP);

    for (byte i = 0; i < BUTTON_NUM; i++)
    {
        buttonArray[i].setDebounceTime(50); // set debounce time to 50 milliseconds
        buttonArray[i].setCountMode(COUNT_RISING);
    }
}

#define I2S_DOUT 22 // DIN connection
#define I2S_BCLK 25 // Bit clock
#define I2S_LRC 26  // Left Right Clock
const int mute = 0;
Audio audio;

const int numberFiles = 3;
String audio_files[numberFiles] = {"YoloBitches.mp3", "OneZero.mp3", "SeventyTwo.mp3"};

unsigned long time_now = 0;
bool AudioOn = false;
bool eof = false;

void AudioSetup()
{
    SPIFFS.begin();
    pinMode(mute, OUTPUT);
    digitalWrite(mute, HIGH);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21); // 0...21
    audio.connecttoFS(SPIFFS, audio_files[0].c_str());
}

void setup()
{
    Serial.begin(115200);
    AudioSetup();
    btnSetup();
}

void loop()
{
    audio.loop();
    debounce();

    if (AudioOn == true)
    {
        if (eof == true)
        {
            eof = false;
        }
    }
}

const long holdTime = 1500;
unsigned long heldTime;
bool btnHeld = false;

void debounce()
{

    unsigned long count1 = buttonArray[0].getCount();
    unsigned long count2 = buttonArray[1].getCount();
    unsigned long count3 = buttonArray[2].getCount();

    for (byte i = 0; i < BUTTON_NUM; i++)
    {
        buttonArray[i].loop(); // MUST call the loop() function first
    }

    for (byte i = 0; i < BUTTON_NUM; i++)
    {
        if (buttonArray[i].isPressed())
        {
            timePress[i] = millis();

            Serial.print("Press Count: ");
            Serial.println(buttonArray[i].getCount());
        }

        if (buttonArray[i].isReleased())
        {
            if (i == 0 ) // button one pressed
            {
                Serial.print("Pressing button: ");
                Serial.println(i + 1);
                audio.connecttoFS(SPIFFS, audio_files[1].c_str());
                AudioOn = true;
            }
            else if (i == 1) // button two pressed
            {
                Serial.print("Pressing button: ");
                Serial.println(i + 1);
                audio.connecttoFS(SPIFFS, audio_files[2].c_str());
                AudioOn = true;
            }
            else if (i == 2) // button three pressed
            {
                Serial.print("Pressing button: ");
                Serial.println(i + 1);
                audio.connecttoFS(SPIFFS, audio_files[1].c_str());
                AudioOn = true;
            }
        }
    }
}
