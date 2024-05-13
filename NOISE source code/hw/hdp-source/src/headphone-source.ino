/* 
Title; ArduinoCore-avr
Author; Arduino
Date; <2022>
Code version; <1.8.6>
Availability; https://github.com/arduino/ArduinoCore-avr 
*/

/* 
Title; Arduino  - ESP8266 core for Arduino
Author; esp8266
Date; <2023>
Code version; <3.1.2>
Availability; https://github.com/esp8266/Arduino 
*/

/* 
Title; ATTinyCore
Author; SpenceKonde
Date; <2021>
Code version; <1.5.2>
Availability; https://github.com/esp8266/Arduino 
*/


#include <Arduino.h>
#include <Wire.h> // adafruit libs requirement *sigh*
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

#ifdef ESP_HW
 #include <ESP8266WiFi.h>
 #include <espnow.h>
 //#include "SoftwareSerial.h"
#endif

/* ---- Settings ---- */

uint8_t SPEAKER_SOURCE_MAC[] = {0x2C, 0x3A, 0xE8, 0x22, 0x72, 0x97}; // speaker source MAC
#define AUDIO_VOLUME 0
#define LOOP_NAME "/loop-1.mp3"

/* ---- Pin definitions ---- */

#ifdef ESP_HW // ESP8266 Wemos D1
 #define SHIELD_RESET      -1 // VS1053 reset pin (unused!)
 #define SHIELD_CS          5 // VS1053 chip select pin (output)
 #define SHIELD_DCS         2 // VS1053 Data/command select pin (output)
 #define SD_CS             16 // Card chip select pin
 #define DREQ               0 // VS1053 Data request, ideally an Interrupt pin
 #define HOOK_PWM_OUT      15 // Calibration/sensing signal to hook
 #define HEADPHONE_COMMS_1  4 // Serial communication to headphone
#else // Arduino UNO
 #define SHIELD_RESET      -1 // VS1053 reset pin (unused!)
 #define SHIELD_CS          7 // VS1053 chip select pin (output)
 #define SHIELD_DCS         6 // VS1053 Data/command select pin (output)
 #define SD_CS              4 // Card chip select pin
 #define DREQ               3 // VS1053 Data request, ideally an Interrupt pin
 #define HOOK_PWM_OUT       9 // Calibration/sensing signal to hook
 #define HEADPHONE_COMMS_1 10 // Serial communication to headphone
#endif


//Globals 

Adafruit_VS1053_FilePlayer audioPlayer =
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, SD_CS);


//message indicating the intended event action of the audience
enum event_message : uint8_t {
  EVENT_OFF = 0,
  EVENT_HEADPHONES_WORN,
  EVENT_HEADPHONES_REMOVED,
  EVENT_HEADPHONES_PLACED_ON_HOOK,
  NUM_EVENTS
};


void setup()
{
  Serial.begin(115200);
  //swSerial.begin(9600);

  delay(1000);

  Serial.println();
  Serial.print("---- ");
#ifdef ESP_HW
  Serial.print(basename(__FILE__));
#else
  Serial.print("headphone audio source");
#endif
  Serial.println(" ----");
#ifdef ESP_HW
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
#endif

  // signal output on HOOK_PWM_OUT
#ifdef ESP_HW
  analogWriteFreq(4000);
  analogWrite(HOOK_PWM_OUT, 127);
#else
  Serial.print("TCCR1B=0b");
  Serial.println(TCCR1B, BIN);
  pinMode(HOOK_PWM_OUT, OUTPUT);
  analogWrite(HOOK_PWM_OUT, 127);
#endif

/* 
Title; dafruit_VS1053_Library
Author; adafruit
Date; <2024>
Code version; <1.4.1>
Availability; https://github.com/adafruit/Adafruit_VS1053_Library
*/

 
  // initialise the mp3 player
  if (!audioPlayer.begin()) {
     Serial.println(F("error: couldn't find VS1053"));
     error();
  }
  Serial.println(F("VS1053 audio player found"));

  // configure the mp3 player
  audioPlayer.sineTest(0x44, 500);
  audioPlayer.setVolume(AUDIO_VOLUME, AUDIO_VOLUME);
  if (!audioPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)) {
    Serial.println(F("DREQ pin is not an interrupt pin"));
  }
  audioPlayer.playbackLoop(true);
  Serial.println(F("VS1053 audio player configured"));

  // initialise input SD card
  if (!SD.begin(SD_CS)) {
    Serial.println(F("error: SD failed, or not present"));
    error();
  }
  Serial.println(F("SD card found"));

 /* 
Title; Is this improper use of #ifdef?
Author; revolt_randy
Date; <2022>
Code version; <1>
Availability; https://forum.arduino.cc/t/is-this-improper-use-of-ifdef/949361
*/


  // set up ESP-NOW for communication with speaker audio source
#ifdef ESP_HW
  Serial.println(">> Setting up ESP-NOW");
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error: couldn't initialise ESP-NOW");
    error();
  }
  Serial.println("ESP-NOW started");

  // configure ESP-NOW
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(esp_now_message_sent);
  esp_now_add_peer(SPEAKER_SOURCE_MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
#endif

  // Start playing audio
  Serial.println(F("Starting audio"));
  if (!audioPlayer.startPlayingFile(LOOP_NAME)) {
    Serial.print(F("error: couldn't play"));
    Serial.println(LOOP_NAME);
    error();
  }
  Serial.println(F("Audio is playing"));
}

//Main loop 

/* 
Title; Arduino  - ESP8266 core for Arduino
Author; esp8266
Date; <2023>
Code version; <3.1.2>
Availability; https://github.com/esp8266/Arduino 
*/

void loop()
{
#ifdef TEST_SEND_ESP_NOW
  static uint32_t send_time = 0;
  static uint8_t test_msg = EVENT_OFF;
  if (millis() - send_time > 2000) {
    Serial.print("Message sent: ");
    Serial.println(test_msg);
    esp_now_send(SPEAKER_SOURCE_MAC, (uint8_t *) &test_msg, 1);
    send_time = millis();
    if (++test_msg == NUM_EVENTS) {
      test_msg = EVENT_OFF;
    }
  }
#endif
}

//Utility functions for checking esp working condition 

void esp_now_message_sent(uint8_t *mac_addr, uint8_t status)
{
  if (status == 0) {
    Serial.println("ESP-NOW delivery succeeded");
  } else {
    Serial.println("ESP-NOW delivery failed");
  }
}

//checking file information of audio files
void printDirectory(File dir, int numTabs)
{
   while(true) {

     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print(" (");
       Serial.print(entry.size(), DEC);
       Serial.println(" bytes)");
     }
     entry.close();
   }
}

// halt and flash LED quickly to show there's a problem
void error()
{
  pinMode(2, OUTPUT);
  for (;;) {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
}
