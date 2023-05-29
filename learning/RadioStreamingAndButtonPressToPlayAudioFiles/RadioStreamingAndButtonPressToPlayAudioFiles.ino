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
bool AudioOn = true;
bool eof = false;

void wifiSetup()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin("tim", "shopkeeper2");
    WiFi.setSleep(false);
    while (WiFi.status() != WL_CONNECTED) delay(1500);
}

void AudioSetup()
{
    SPIFFS.begin();
    pinMode(mute, OUTPUT);
    digitalWrite(mute, HIGH);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10); // 0...21
    // audio.connecttoFS(SPIFFS, audio_files[0].c_str());
    audio.connecttohost("http://mp3.ffh.de/ffhchannels/80er.aac");  // aac -> This works.
}

void setup()
{
    Serial.begin(115200);
    wifiSetup();
    AudioSetup();
    btnSetup();
}

void loop()
{
    audio.loop();
    debounce();
}

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

            }
            else if (i == 1) // button two pressed
            {
                Serial.print("Pressing button: ");
                Serial.println(i + 1);
                audio.connecttoFS(SPIFFS, audio_files[2].c_str());
            }
            else if (i == 2) // button three pressed
            {
                Serial.print("Pressing button: ");
                Serial.println(i + 1);
                audio.connecttoFS(SPIFFS, audio_files[1].c_str());
            }
        }
    }
}

// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);
    audio.connecttohost("http://mp3.ffh.de/ffhchannels/80er.aac");  // aac -> This works.

}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}