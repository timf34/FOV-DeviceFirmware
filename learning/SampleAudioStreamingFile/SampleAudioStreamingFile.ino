#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include "SD.h"
#include "FS.h"

// Digital I/O used
#define SD_CS          5
// #define SPI_MOSI      23
// #define SPI_MISO      19
// #define SPI_SCK       18
#define I2S_DOUT      22
#define I2S_BCLK      25
#define I2S_LRC       26

Audio audio;

String ssid =     "tim";
String password = "shopkeeper2";

void setup() {
    pinMode(SD_CS, OUTPUT);      digitalWrite(SD_CS, HIGH);
    // SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);  
    Serial.begin(9600);
    SD.begin(SD_CS);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    WiFi.setSleep(false);
    while (WiFi.status() != WL_CONNECTED) delay(1500);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21); // default 0...21
    audio.connecttohost("http://mp3.ffh.de/ffhchannels/80er.aac");  // aac -> appears to work 
    // audio.connecttohost("http://iskatel.hostingradio.ru:8015/iskatel-320.aac");  // aac -> see log, get errors, but was able to reduce it. 
    // audio.connecttohost("http://open.live.bbc.co.uk/mediaselector/5/select/version/2.0/mediaset/http-icy-mp3-a/vpid/bbc_world_service/format/pls.pls");  // bbc
    // audio.connecttohost("http://stream.antennethueringen.de/live/aac-64/stream.antennethueringen.de/");  // aac - should work. 
    // audio.connecttohost("http://a.files.bbci.co.uk/media/live/manifesto/audio/simulcast/hls/nonuk/sbr_vlow/ak/bbc_world_service.m3u8");  // bbc - 403 forbidden 
    // audio.connecttohost("http://a.files.bbci.co.uk/media/live/manifesto/audio/simulcast/hls/nonuk/sbr_low/ak/bbc_world_service.m3u8");  // bbc - 403 forbidden 
    // audio.connecttohost("http://as-hls-ww-live.akamaized.net/pool_904/live/ww/bbc_asian_network/bbc_asian_network.isml/bbc_asian_network-audio%3d96000.norewind.m3u8");  // bbc  - doesn't work on firmware I don't think - get "new request: _new url_"
}

void loop()
{
    audio.loop();
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