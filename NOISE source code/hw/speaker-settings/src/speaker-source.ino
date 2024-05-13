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

/* 
Title; dafruit_VS1053_Library
Author; adafruit
Date; <2024>
Code version; <1.4.1>
Availability; https://github.com/adafruit/Adafruit_VS1053_Library
*/


#include <Arduino.h>
#include <Wire.h> // adafruit libs requirement *sigh*
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

#ifdef ESP_HW
 #include <ESP8266WiFi.h>
 #include <espnow.h>
 #include <ESP8266WiFi.h>
 #include <ESP8266mDNS.h>
 #include <WiFiUdp.h>
 #include <ArduinoOTA.h>
 #include <TelnetSpy.h>
#endif

#ifdef ESP_HW
TelnetSpy SerialT;
#else
 #define SerialT Serial
#endif

/* ---- Settings ---- */

uint8_t SPEAKER_SOURCE_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // speaker source MAC

const char* ssid = "Speaker";
const char* password = "euchooC7";

#define AUDIO_VOLUME 0
#define AUDIO

/* ---- Pin definitions ---- */

#ifdef ESP_HW // ESP8266 Wemos D1
 #define SHIELD_RESET    -1 // VS1053 reset pin (unused!)
 #define SHIELD_CS        5 // VS1053 chip select pin (output)
 #define SHIELD_DCS       2 // VS1053 Data/command select pin (output)
 #define SD_CS           16 // Card chip select pin
 #define DREQ             0 // VS1053 Data request, ideally an Interrupt pin
 #define HOOK_PWM_OUT    15 // Calibration/sensing signal to hook
 #define HEADPHONE_COMMS  4 // Serial communication to headphone
#else // Arduino UNO
 #define SHIELD_RESET    -1 // VS1053 reset pin (unused!)
 #define SHIELD_CS        7 // VS1053 chip select pin (output)
 #define SHIELD_DCS       6 // VS1053 Data/command select pin (output)
 #define SD_CS            4 // Card chip select pin
 #define DREQ             3 // VS1053 Data request, ideally an Interrupt pin
 #define HOOK_PWM_OUT     9 // Calibration/sensing signal to hook
 #define HEADPHONE_COMMS 10 // Serial communication to headphone
#endif


//Audio files

#define LOOP1 "/loop-1.mp3"
#define LOOP2 "/loop-2.mp3"
#define AUDIO_SFX_1 "/sfx-1.mp3"

//Events 

// events are defined here
enum event_message : uint8_t {
  EVENT_OFF = 0,
  EVENT_WIFI_UPDATE_MODE,
  EVENT_HEADPHONES_WORN,
  EVENT_HEADPHONES_REMOVED,
  EVENT_HEADPHONES_PLACED_ON_HOOK
};

volatile event_message HEADPHONE_STATES[2] = {EVENT_OFF};

// wireless communication message structure
struct esp_now_package {
  uint8_t id;
  uint8_t message;
};

// Actions to perform on a given event
// play audio, loops, or sound effects
void process_event(event_message event)
{
  switch (event) {
    case EVENT_OFF:
      SerialT.println("Doing nothing.");
      break;
    case EVENT_WIFI_UPDATE_MODE:
#ifdef ESP_HW
      SerialT.println("Entering WiFi configuration mode.");
      start_wifi_portal();
#else
      SerialT.println("WiFi configuration is not possible.");
#endif
      break;
    case EVENT_HEADPHONES_WORN:
      SerialT.println("Switching loops.");
      startAudioLoop(1);
      break;
    case EVENT_HEADPHONES_REMOVED:
      SerialT.println("Switching loops back.");
      startAudioLoop(0);
      break;
    case EVENT_HEADPHONES_PLACED_ON_HOOK:
      SerialT.println("Playing SFX.");
      play_sfx();
      break;
    default:
      return;
  }
}

// get the human readable description of an event
const char *get_event_name(event_message event)
{
  switch (event) {
    case EVENT_OFF:
      return "EVENT_OFF";
    case EVENT_HEADPHONES_WORN:
      return "EVENT_HEADPHONES_WORN";
    case EVENT_HEADPHONES_REMOVED:
      return "EVENT_HEADPHONES_REMOVED";
    case EVENT_HEADPHONES_PLACED_ON_HOOK:
      return "EVENT_HEADPHONES_PLACED_ON_HOOK";
    default:
      return "";
  }
}

//Globals
Adafruit_VS1053_FilePlayer audioPlayer =
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, SD_CS);


void setup()
{
  SerialT.begin(115200);

  delay(1000);

  // start up messages
  SerialT.println();
  SerialT.print("---- ");
#ifdef ESP_HW
  SerialT.print(basename(__FILE__));
#else
  SerialT.print("speaker audio source");
#endif
  SerialT.println(" ----");

  // set up ESP-NOW for communication with speaker audio source
  SerialT.println(F(">> Starting ESP-NOW"));
#ifdef ESP_HW
  SerialT.print("MAC address: ");
  SerialT.println(WiFi.macAddress());
#endif
#ifdef ESP_HW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    SerialT.println("error: couldn't initialise ESP-NOW");
    error();
  }
  SerialT.println("ESP-NOW started");

  // configure ESP-NOW
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(esp_now_message_received);
#endif

#ifdef AUDIO
  // initialise the audio player
  SerialT.println(F(">> Initialising audio player"));
  if (!audioPlayer.begin()) {
     SerialT.println(F("Error: couldn't find VS1053"));
     error();
  }
  SerialT.println(F("VS1053 audio player found"));

  // configure audio player
  if (!audioPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT)) {
    SerialT.println(F("DREQ pin is not an interrupt pin"));
  }
  SerialT.println(F("VS1053 audio player configured"));

  // print some info about the audio player
  SerialT.print("Audio playback speed register -> ");
  SerialT.println(audioPlayer.getPlaySpeed());
#endif

  // initialise the SD card
  SerialT.println(F(">> Accessing SD card"));
  if (!SD.begin(SD_CS)) {
    SerialT.println(F("Error: SD failed, or not present"));
    error();
  }
  SerialT.println("SD card found");

  // print SD card files
  SerialT.println("---- SD card contents ----");
  printDirectory(SD.open("/"), 0);
  SerialT.println("--------------------------");

#ifdef AUDIO
 #ifdef START_UP_EXPLOSION
  audioPlayer.setVolume(AUDIO_VOLUME, AUDIO_VOLUME);
  play_sfx();
 #endif

  // Start playing audio
  SerialT.println(F(">> Starting audio"));
  startAudioLoop(0);
  SerialT.println(F("Audio is playing"));
#endif
}

//Main loop 

#ifdef AUDIO
//#define DEBUG_LOOP_CYCLE
#endif

void loop()
{
#ifdef DEBUG_LOOP_CYCLE
  static bool current_loop = 0;
  static uint64_t last_loop_change = 0;
  if (millis() - last_loop_change > 5000) {
    SerialT.println(">> Swapping loop");
    current_loop = !current_loop;
    stopAudioLoop();
    startAudioLoop(current_loop);
    last_loop_change = millis();
  }
#endif
}

//Utility function

void hw_wdt_disable() {
  *((volatile uint32_t*) 0x60000900) &= ~(1); // Hardware WDT OFF
}

void hw_wdt_enable() {
  *((volatile uint32_t*) 0x60000900) |= 1; // Hardware WDT ON
}

#define NUM_LOOPS 2
int8_t CURRENT_LOOP = -1;
bool LOOP_PLAYING = false;

void play_sfx()
{
  Serial.println(">> Playing SFX ");
  stopAudioLoop();

  audioPlayer.playbackLoop(false);
  if (!audioPlayer.startPlayingFile(AUDIO_SFX_1, 0)) {
    SerialT.print(F("error: couldn't play "));
    SerialT.println(AUDIO_SFX_1);
  }
  // wait for audio to finish
  ESP.wdtDisable();
  hw_wdt_disable();
  while (!audioPlayer.stopped()) { delay(20); }
  hw_wdt_enable();
  ESP.wdtEnable(10000);
  startAudioLoop(CURRENT_LOOP);
}

uint32_t PLAYBACK_POSITION_BYTES[NUM_LOOPS] = {0};

// start (resume) a numbered audio loop - seeks to last played position
bool startAudioLoop(uint8_t n)
{
  bool ret;

  Serial.print(">> Starting loop ");
  Serial.println(n);

  // do nothing if we're already playing the requested loop)
  if (n == CURRENT_LOOP && LOOP_PLAYING) {
    Serial.print("Loop ");
    Serial.print(n);
    Serial.println(" is already playing.");
    return 0;
  }

  CURRENT_LOOP = n;

  if (LOOP_PLAYING) {
    stopAudioLoop();
  }

  switch (n) {
    case 0:
      ret = audioPlayer.playLoopSeek("/loop-1.mp3", PLAYBACK_POSITION_BYTES[n]);
      break;
    case 1:
      ret = audioPlayer.playLoopSeek("/loop-2.mp3", PLAYBACK_POSITION_BYTES[n]);
      break;
    default:
      SerialT.print("error: unknown loop! (");
      SerialT.print(n);
      SerialT.println(")");
      return false;
  }

  SerialT.print(">> Started loop ");
  SerialT.println(n);

  //delayMicroseconds(10000);
  //fade_in();

  LOOP_PLAYING = true;

  return ret;
}

// Stop the audio loop, and store the current offset into the file
// TODO: fade the file to avoid discontinuity - volume control
void stopAudioLoop()
{
  SerialT.println(">> Stopping loop");
  PLAYBACK_POSITION_BYTES[CURRENT_LOOP] = audioPlayer.stopPlaying();
  LOOP_PLAYING = false;
  //audioPlayer.softReset();
}

void fade_out()
{
  uint8_t v = 0;

  SerialT.println(">> Fade out");
  SerialT.print("|");

  do {
    //SerialT.print("-");
    audioPlayer.setVolume(v, v);
    delayMicroseconds(100);
    SerialT.print(v);
    SerialT.print(",");
  } while (v++ < 255);

  SerialT.print("|");
  SerialT.println(v);
}

void fade_in()
{
  uint8_t v = 255;

  SerialT.println(">> Fade in");

  SerialT.print("|");
  do {
    //SerialT.print("+");
    audioPlayer.setVolume(v, v);
    SerialT.print(v);
    SerialT.print(",");
    delayMicroseconds(100);
  } while (v-- > 0);
  SerialT.print("|");
  SerialT.println(v);
}

void esp_now_message_received(uint8_t *mac_addr, uint8_t *data, uint8_t len)
{
  static esp_now_package *pkg;

  // re-enable interrupts (don't want to print in interrupt context)
  interrupts();

  pkg = (esp_now_package *) data;

  SerialT.println(">> ESP-NOW message received");
  SerialT.print("Bytes -> ");
  SerialT.println(len);
  SerialT.print("Id -> 0x");
  SerialT.println(pkg->id, HEX);
  SerialT.print("Value -> ");
  SerialT.println(pkg->message);
  SerialT.print("Event -> ");
  SerialT.println(get_event_name((event_message) pkg->message));

  // update record of headphones' statuses
  switch (pkg->id) {
    case 0xAA:
      HEADPHONE_STATES[0] = (event_message) pkg->message;
      break;
    case 0xBB:
      HEADPHONE_STATES[1] = (event_message) pkg->message;
      break;
    default:
      break;
  }

  // process events to be dealt with straight away
  switch (pkg->message) {
    case EVENT_WIFI_UPDATE_MODE:
    case EVENT_HEADPHONES_WORN:
    case EVENT_HEADPHONES_PLACED_ON_HOOK:
      process_event((event_message) pkg->message);
      break;
    default:
      break;
  }

  process_combined_events();
}

// process events which depend on multiple headphones
void process_combined_events() {
  if (HEADPHONE_STATES[0] != EVENT_HEADPHONES_WORN && HEADPHONE_STATES[1] != EVENT_HEADPHONES_WORN) {
    process_event(EVENT_HEADPHONES_REMOVED);
  }
}

void printDirectory(File dir, int numTabs)
{
   while(true) {

     File entry = dir.openNextFile();
     if (! entry) {
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       SerialT.print('\t');
     }
     SerialT.print(entry.name());
     if (entry.isDirectory()) {
       SerialT.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       SerialT.print(" (");
       SerialT.print(entry.size(), DEC);
       SerialT.println(" bytes)");
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

#ifdef ESP_HW
void start_wifi_portal()
{
  SerialT.println(">> Starting WiFi access point");

  // play a sound
  audioPlayer.setVolume(AUDIO_VOLUME, AUDIO_VOLUME);
  audioPlayer.sineTest(0x44, 500);

  WiFi.softAP(ssid, password);

  SerialT.println(">> Starting over-the-air updates");

  ArduinoOTA.begin();

  SerialT.println("Portal is active.");

  for (;;) {
    wifi_loop();
  }
}

void wifi_loop()
{
  SerialT.handle();
  ArduinoOTA.handle();
}
#endif
