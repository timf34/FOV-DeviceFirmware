// Note: Remeber to click the _ESP32 Sketch Data Upload_ button when using new audio files!!!

#include "Arduino.h"
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
        "FovTut1.mp3",
        "FovTut2new.mp3",
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

void listFilesInSPIFFS()
{
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    while (file)
    {
        Serial.print("File: ");
        Serial.print(file.name());
        Serial.print(" - Size: ");
        Serial.println(file.size());
        file = root.openNextFile();
    }
}

void AudioSetup()
{
    SPIFFS.begin(true);
    pinMode(mute, OUTPUT);
    digitalWrite(mute, HIGH);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(17); // 0...21
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
    }
    if (i == 3)
    {
        Serial.println("EOF3");
        audio.connecttoFS(SPIFFS, mp3_files[3].c_str());
        // Top vib
    }
    if (i == 4)
    {
        Serial.println("EOF4");
        audio.connecttoFS(SPIFFS, mp3_files[4].c_str());
        // Top vib
    }
    if (i == 5)
    {
        Serial.println("EOF5");
        audio.connecttoFS(SPIFFS, mp3_files[5].c_str());
        // Top vib
    }
    if (i == 6)
    {
        Serial.println("EOF6");
        audio.connecttoFS(SPIFFS, mp3_files[6].c_str());
        // Top vib
    }
    if (i == 7)
    {
        Serial.println("EOF7");
        audio.connecttoFS(SPIFFS, mp3_files[7].c_str());
        // Top vib
    }    
    if (i == 8)
    {
        Serial.println("EOF8");
        audio.connecttoFS(SPIFFS, mp3_files[8].c_str());
        // Top vib
    }
    if (i == 9)
    {
        Serial.println("EOF9");
        audio.connecttoFS(SPIFFS, mp3_files[9].c_str());
        // Top vib
    }
    if (i == 10)
    {
        Serial.println("EOF10");
        audio.connecttoFS(SPIFFS, mp3_files[10].c_str());
        // Top vib
    }
    if (i == 11)
    {
        Serial.println("EOF11");
        audio.connecttoFS(SPIFFS, mp3_files[11].c_str());
        // Top vib
    }   
    if (i == 12)
    {
        Serial.println("EOF12");
        audio.connecttoFS(SPIFFS, mp3_files[12].c_str());
        // Top vib
    }
    i++;
}

void setup()
{
    Serial.begin(9600);
    Serial.println("before audio setup");

    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS initialization failed");
      return;
    }
    Serial.println("SPIFFS initialization successful");

    listFilesInSPIFFS();

    AudioSetup();
    tutorial();
}

void loop()
{
    audio.loop();

    // Note: ensure there are no `delay()` calls in this loop with audio.loop()!

    // if (AudioOn == true)
    // {
    //     if (eof == true)
    //     {
    //         eof = false;
    //     }
    // }
}
