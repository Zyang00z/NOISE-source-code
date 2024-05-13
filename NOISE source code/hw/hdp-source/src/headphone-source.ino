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

// DREQ should be an interrupt capable pin

/* ---- Globals ---- */

Adafruit_VS1053_FilePlayer audioPlayer =
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, SD_CS);

//EspSoftwareSerial::UART swSerial;

enum event_message : uint8_t {
  EVENT_OFF = 0,
  EVENT_HEADPHONES_WORN,
  EVENT_HEADPHONES_REMOVED,
  EVENT_HEADPHONES_PLACED_ON_HOOK,
  NUM_EVENTS
};

/* ---- Set up ---- */

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

  // set up PWM output on HOOK_PWM_OUT
#ifdef ESP_HW
  analogWriteFreq(4000);
  analogWrite(HOOK_PWM_OUT, 127);
#else
  Serial.print("TCCR1B=0b");
  Serial.println(TCCR1B, BIN);
  pinMode(HOOK_PWM_OUT, OUTPUT);
  analogWrite(HOOK_PWM_OUT, 127);
  //TCCR1B = 0x05; // 40.64 Hz measured
  //TCCR1B = 0x03; // 490 Hz measured
  TCCR1B = 0x02; // 3.92 kHz measured - nice
  //TCCR1B = 0x01; // 31.17 kHz measured - seems to cause autocalibration?
#endif

  // initialise the audio player
  if (!audioPlayer.begin()) {
     Serial.println(F("error: couldn't find VS1053"));
     error();
  }
  Serial.println(F("VS1053 audio player found"));

  // configure the audio player
  audioPlayer.sineTest(0x44, 500);
  audioPlayer.setVolume(AUDIO_VOLUME, AUDIO_VOLUME);
  if (!audioPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)) {
    Serial.println(F("DREQ pin is not an interrupt pin"));
  }
  audioPlayer.playbackLoop(true);
  Serial.println(F("VS1053 audio player configured"));

  // initialise the SD card
  if (!SD.begin(SD_CS)) {
    Serial.println(F("error: SD failed, or not present"));
    error();
  }
  Serial.println(F("SD card found"));

  // print SD card files
  Serial.println(F("---- SD card contents ----"));
  printDirectory(SD.open("/"), 0);
  Serial.println(F("--------------------------"));

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

/* ---- Main loop ---- */

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

/* ---- Utility functions ---- */

void esp_now_message_sent(uint8_t *mac_addr, uint8_t status)
{
  if (status == 0) {
    Serial.println("ESP-NOW delivery succeeded");
  } else {
    Serial.println("ESP-NOW delivery failed");
  }
}

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
