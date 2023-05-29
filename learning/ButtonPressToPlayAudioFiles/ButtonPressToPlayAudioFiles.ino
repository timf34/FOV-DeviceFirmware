
#include "Arduino.h"
// #include "WiFi.h"
#include <Audio.h>
// #include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"

/*******************************************************************************************************************************************************************************************************************************************************************************/
// Vibration Variables and Setup
/*******************************************************************************************************************************************************************************************************************************************************************************/
bool vibeOn = true;
/*******************************************************************************************************************************************************************************************************************************************************************************/
// Buttons Variables and Setup
/*******************************************************************************************************************************************************************************************************************************************************************************/
// Program to test button functionality
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

/*******************************************************************************************************************************************************************************************************************************************************************************/
// Audio Variables and Setup
/*******************************************************************************************************************************************************************************************************************************************************************************/
// Digital I/O used
#define I2S_DOUT 22 // DIN connection
#define I2S_BCLK 25 // Bit clock
#define I2S_LRC 26  // Left Right Clock
const int mute = 0;
Audio audio;

int trackNum = 0;

int tdata[2];

const int numberFiles = 1;
String audio_files[numberFiles] = {"YoloBitches.mp3"};

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
/*******************************************************************************************************************************************************************************************************************************************************************************/

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

/*******************************************************************************************************************************************************************************************************************************************************************************/
// User Input Functions
/*******************************************************************************************************************************************************************************************************************************************************************************/

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
            //      Serial.print("The button ");
            //      Serial.print(i + 1);
            //      Serial.println(" is pressed");

            timePress[i] = millis();

            Serial.print("Press Count: ");
            Serial.println(buttonArray[i].getCount());
        }

        if (buttonArray[i].isReleased())
        {

            heldTime = millis() - timePress[i];
            if (heldTime > holdTime)
                btnHeld = true;
            else
                btnHeld = false;

            Serial.print("Held for: ");
            Serial.println(heldTime);

            if (i == 0 && btnHeld == false) // button one pressed
            {
                Serial.print("The button ");
                Serial.print(i + 1);
                Serial.println(" was pressed");
                AudioOn = true;
            }
            else if (i == 1 && btnHeld == false) // button two pressed
            {
                Serial.print("The button ");
                Serial.print(i + 1);
                Serial.println(" was pressed");
                AudioOn = true;
            }

            else if (i == 2 && btnHeld == false) // button three pressed
            {
                Serial.print("The button ");
                Serial.print(i + 1);
                Serial.println(" was pressed");
                AudioOn = true;
            }
            else if (i == 0 && btnHeld == true) // button one held
            {
                Serial.print("The button ");
                Serial.print(i + 1);
                Serial.println(" was held");
                AudioOn = true;
            }
            else if (i == 1 && btnHeld == true) // button two held
            {
                Serial.print("The button ");
                Serial.print(i + 1);
                Serial.println(" was held");
                AudioOn = true;
            }

            else if (i == 2 && btnHeld == true) // button three held
            {
                Serial.print("The button ");
                Serial.print(i + 1);
                Serial.println(" was held");
                vibeOn = !vibeOn;
                Serial.print("Vibration: ");
                if (vibeOn)
                    Serial.println("ON");
                else
                    Serial.println("OFF");
            }
        }
    }
}
/*******************************************************************************************************************************************************************************************************************************************************************************/
// Audio Functions
/*******************************************************************************************************************************************************************************************************************************************************************************/

void audio_info(const char *info)
{
    // Serial.print("info        "); Serial.println(info);
}

void audio_eof_mp3(const char *info)
{ // end of file
    Serial.print("Track Number: ");
    Serial.println(trackNum);

    eof = true;
}
