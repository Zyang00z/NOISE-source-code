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

#ifdef ESP8266
 #include <ESP8266WiFi.h>
 #include <espnow.h>
 #include <ESP8266WiFi.h>
 #include <ESP8266mDNS.h>
 #include <WiFiUdp.h>
 #include <ArduinoOTA.h>
 #include <TelnetSpy.h>
#endif

#include <SoftwareSerial.h>

EspSoftwareSerial::UART sensorSerial;
TelnetSpy SerialT;

//#define LOOP_SLEEP
// ~80 mA transmitting + idle
// ~70 mA transmitting + idle when enabled
//#define SEND_TEST_MESSAGES
//#define SEND_FIXED_TEST_MESSAGE

#define ESP_NOW_RETRIES 5

uint8_t SPEAKER_SOURCE_MAC[] = {0x50, 0x02, 0x91, 0xE0, 0x48, 0xE3}; // speaker source MAC address

#ifdef HEADPHONE_A
const char* ssid = "HeadphoneA";
#elif defined HEADPHONE_B
const char* ssid = "HeadphoneB";
#else
 #error Unrecognised headphone!
#endif
const char* password = "euchooC7";

//Globals

bool TRANSMIT = true;

// events are defined here
enum event_message : uint8_t {
  EVENT_OFF = 0,
  EVENT_WIFI_UPDATE_MODE,
  EVENT_HEADPHONES_WORN,
  EVENT_HEADPHONES_REMOVED,
  EVENT_HEADPHONES_PLACED_ON_HOOK
};

/* 
Title; Sharing code base between ESP32 and arduino
Author; zliudr
Date; <2020>
Code version; <1>
Availability; https://www.esp32.com/viewtopic.php?f=13&t=16358&sid=30e4019d533749dc90994024d0aed757
*/

// wireless communication message structure
struct esp_now_package {
  uint8_t id;
  uint8_t message;
};

void setup()
{
#ifdef UART_OFF
  // telnet only console
  SerialT.setSerial(NULL);
  SerialT.begin(0);
#else
  SerialT.print(">> Setting up host serial (");
 #ifdef USE_HW_UART_ONBOARD
  #error USE_HW_UART_ONBOARD seems to block proper operation, possibly due to TX pin
  SerialT.print(SERIAL_BAUD);
  SerialT.begin(SERIAL_BAUD);
 #else
  SerialT.print(115200);
  SerialT.begin(115200);
 #endif
  SerialT.println(" baud)");
#endif

#ifdef HEADPHONE_A
  SerialT.setWelcomeMsg("Headphone transmitter A\r\n");
#elif defined HEADPHONE_B
  SerialT.setWelcomeMsg("Headphone transmitter B\r\n");
#else
 #error Unrecognised headphone!
#endif

  delay(500);

  SerialT.println();
  SerialT.print("---- ");
#ifdef ESP8266
  SerialT.print(basename(__FILE__));
#else
  SerialT.print(__FILE__);
#endif
  SerialT.println(" ----");

  SerialT.print("Build date: ");
  SerialT.println(__DATE__);
  SerialT.print("Build time: ");
  SerialT.println(__TIME__);

#ifdef LED
  pinMode(LED, OUTPUT);
#endif

  pinMode(STATUS_LED, OUTPUT);

#ifdef LED2
  pinMode(LED2, OUTPUT);
#endif

#if 0
  static uint32_t start_time = 0;
  while (millis() - start_time < 4000) {
    start_flash();
  }
#endif

  // start up flash
#ifdef WIFI_EN_INVERT
 #define START_FLASH_COUNT 16
#else
 #define START_FLASH_COUNT 5
#endif
  for (int i=0; i<8; i++) {
    digitalWrite(STATUS_LED, HIGH);
#ifdef LED
    digitalWrite(LED, LOW);
#endif
#ifdef LED2
    digitalWrite(LED2, LOW);
#endif
    delay(75);
#ifdef LED
    digitalWrite(LED, HIGH);
#endif
    digitalWrite(STATUS_LED, LOW);
#ifdef LED2
    digitalWrite(LED2, HIGH);
#endif
    delay(75);
  }

  // turn off LEDs
#ifdef LED
  digitalWrite(LED, LOW);
#endif
  digitalWrite(STATUS_LED, HIGH);

  // set up ESP-NOW for communication with speaker audio source
#ifdef ESP8266
  SerialT.println(">> Setting up ESP-NOW");
  SerialT.print("MAC address: ");
  SerialT.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    SerialT.println("error: couldn't initialise ESP-NOW");
    error();
  }
  SerialT.println("ESP-NOW started");
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(esp_now_message_sent);
  esp_now_add_peer(SPEAKER_SOURCE_MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
#endif

#ifdef WIFI_EN
  pinMode(WIFI_EN, INPUT_PULLUP);

  // start access point for debugging, if jumper is set
#ifdef WIFI_EN_INVERT
  if (!digitalRead(WIFI_EN)) {
#else
  if (digitalRead(WIFI_EN)) {
#endif
    // signal to speaker to go into wifi administration mode
    SerialT.println(">> Signalling speaker to go into WiFi administration mode");
    tx_byte((uint8_t) EVENT_WIFI_UPDATE_MODE);

    // send message with retries
    uint32_t start = millis();
    while (millis() - start < 2000) {
      tx(false);
    }

    // start local access point
    SerialT.println(">> Starting WiFi access point");
    WiFi.softAP(ssid, password);
    SerialT.println(">> Starting over-the-air updates");
    ArduinoOTA.begin();
    TRANSMIT = false;
  }
#endif

  // set up software serial
#ifndef USE_HW_UART_ONBOARD
  SerialT.print(">> Setting up board serial (");
  SerialT.print(SERIAL_BAUD);
  SerialT.println(" baud)");
  sensorSerial.begin(SERIAL_BAUD, SWSERIAL_8N1, SERIAL_RX, -1, false);
  if (!sensorSerial) {
    SerialT.println("Invalid EspSoftwareSerial pin configuration");
    error();
  }
#endif

#ifndef NEON_FLASH
 #ifdef LED2
  //analogWrite(LED2, 50);
  digitalWrite(LED2, LOW);
 #endif
#endif

  SerialT.println(">> Starting application");
}

//Loop 

void loop()
{
  tx(false); // transmit esp-now message retries
  receive_serial_data(); // receive data from sensor AVR

  SerialT.handle(); // handle telnet console
  ArduinoOTA.handle(); // handle over-the-air updates

  if (TRANSMIT) {
#ifdef NEON_FLASH
    neon_flash();
#endif
  } else {
    // flash LED while in wifi mode
    digitalWrite(LED2, millis() % 4096 < 200);
  }

#ifdef SEND_TEST_MESSAGES
  send_test_message();
#endif

  // save power??
#ifdef LOOP_SLEEP
  delay(10);
#endif
}

#define ESP_NOW_RETRY
#define DEBUG_ESP_NOW_RETRY

#define ESP_NOW_RETRIES 5
#define ESP_NOW_RESEND_INTERVAL_MS 50

enum esp_now_status {
  ESPNowSent,
  ESPNowSuccess
};

volatile esp_now_status ESP_NOW_STATUS = ESPNowSuccess;
uint8_t ESP_NOW_MSG = 0;

// transmit a byte over ESP-NOW
void tx_byte(uint8_t msg)
{
#ifdef ESP_NOW_RETRY
  ESP_NOW_STATUS = ESPNowSent;
  ESP_NOW_MSG = msg;
  tx(true);
#else
  esp_now_send_msg(msg);
#endif
}

// transmit ESP-NOW message with retries, flash LED if we have STATUS_LED defined
void tx(bool force)
{
  static uint8_t retries = 0;
  static uint32_t last_send = 0;

  if (force) { retries = 0; };

  if (!force && millis() - last_send < ESP_NOW_RESEND_INTERVAL_MS) {
    return;
  }

  if (retries < ESP_NOW_RETRIES && ESP_NOW_STATUS == ESPNowSent) {
#ifdef STATUS_LED
    digitalWrite(STATUS_LED, LOW);
#endif

#ifdef DEBUG_ESP_NOW_RETRY
    SerialT.print("Transmitting message, try -> ");
    SerialT.println(retries);
#endif

    esp_now_send_msg(ESP_NOW_MSG);
    last_send = millis();
    retries++;

#ifdef STATUS_LED
    digitalWrite(STATUS_LED, HIGH);
#endif
  }
}

void esp_now_send_msg(uint8_t msg) {
  static esp_now_package package = { .id = ESP_NOW_ID, .message = 0 };
  package.message = msg;
  esp_now_send(SPEAKER_SOURCE_MAC, (uint8_t *) &package, sizeof(esp_now_package));
}

void send_test_message()
{
  static uint32_t send_time = 0;
  static uint8_t test_msg = 0;
  if (millis() - send_time > 2000) {
    SerialT.print("Message sent: ");
    SerialT.println(test_msg);
#ifdef ESP8266
    if (TRANSMIT) {
      tx_byte(test_msg);
    }
#endif
#ifndef SEND_FIXED_TEST_MESSAGE
    if (++test_msg == 4) {
      test_msg = 0;
    }
#endif
    send_time = millis();
  }
}

#ifdef ESP8266
void esp_now_message_sent(uint8_t *mac_addr, uint8_t status)
{
  if (status == 0) {
    ESP_NOW_STATUS = ESPNowSuccess;
    SerialT.println("ESP-NOW delivery succeeded");
  } else {
    SerialT.println("ESP-NOW delivery failed");
  }
}
#endif

// halt and flash LED quickly to show there's a problem
void error()
{
  pinMode(STATUS_LED, OUTPUT);
  for (;;) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
}

void run_serial_command(char cmd) {
  bool command_parsed = true;
  uint8_t message = 0;

  switch (cmd) {
    case 'W':
      SerialT.println("WORN");
      message = (uint8_t) EVENT_HEADPHONES_WORN;
      break;
    case 'H':
      SerialT.println("HOOK");
      message = (uint8_t) EVENT_HEADPHONES_PLACED_ON_HOOK;
      break;
    case 'R':
      SerialT.println("REMOVED");
      message = (uint8_t) EVENT_HEADPHONES_REMOVED;
      break;

    default:
      command_parsed = false;
      return;
  }




//#define SERIAL_FORWARD
#define SERIAL_RELAY

void _receive_serial_data() {
  static char c = 0;
  static char sBuf[6] = {0};
  static uint8_t sCount = 0;

#ifdef SERIAL_RELAY
  static char lineBuf[250] = {0};
  static uint8_t lineBufCount = 0;
#endif

  c = sensorSerial.read();

#ifdef SERIAL_FORWARD
  Serial.write(c);
#endif

  switch (c) {
    case '\r':
      break;

    case '\n':
#ifdef LED
      digitalWrite(LED, HIGH);
#endif

      run_serial_command(sBuf[0]);
      memset(sBuf, 0, 6);
      sCount = 0;

#ifdef SERIAL_RELAY
 #ifdef WIFI_EN
  #ifdef WIFI_EN_INVERT
      if (!digitalRead(WIFI_EN)) {
  #else
      if (digitalRead(WIFI_EN)) {
  #endif
 #endif
        SerialT.print("--> ");
        SerialT.println(lineBuf);
        memset(lineBuf, 0, lineBufCount);
        lineBufCount = 0;
 #ifdef WIFI_EN
      }
 #endif
#endif

#ifdef LED
      digitalWrite(LED, LOW);
#endif
      break;

    default:
      // add character to buffer
      if (sCount < 5) {
        sBuf[sCount++] = c;
      }

#ifdef SERIAL_RELAY
 #ifdef WIFI_EN
  #ifdef WIFI_EN_INVERT
      if (!digitalRead(WIFI_EN)) {
  #else
      if (digitalRead(WIFI_EN)) {
  #endif
 #endif
        // add character to line buffer
        if (lineBufCount < 250) {
          lineBuf[lineBufCount++] = c;
        }
 #ifdef WIFI_EN
      }
 #endif
#endif
  }

  //sensorSerial.flush();
}

void receive_serial_data() {
#ifdef USE_HW_UART_ONBOARD
  if (Serial.available() > 0) {
#else
  if (sensorSerial.available() > 0) {
#endif
    _receive_serial_data();
  }
}

