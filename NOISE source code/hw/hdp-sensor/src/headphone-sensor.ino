#include <Arduino.h>
#include <CapacitiveSensor.h>

#ifdef SOFTWARE_SERIAL
 #include <SoftwareSerial.h>
 SoftwareSerial sser(-1, TX);
 #define Ser sser
#else
 #define Ser Serial
#endif

/* ---- Settings ---- */

//#define TEST_COMMAND_SEND
//#define TEST_TOUCH_SENSE

typedef uint16_t buff_t;

/* ---- Globals ---- */

CapacitiveSensor cs = CapacitiveSensor(CS_RES, CS_SENSE);

/* ---- Headphone events ---- */

enum headphone_state : uint8_t {
  EVENT_OFF = 0,
  EVENT_WIFI_UPDATE_MODE,
  EVENT_HEADPHONES_WORN,
  EVENT_HEADPHONES_REMOVED,
  EVENT_HEADPHONES_PLACED_ON_HOOK
};

headphone_state STATE = EVENT_OFF;

buff_t CS_WORN = 0;
buff_t CS_HOOK = 0;
buff_t CS_MIN = 0;

/* ---- Ring buffers to generate stats on raw capacitance data ---- */

#define SMOOTH_STD_DEV
#define FLEX_BASELINE_AVERAGE

#define STATS_RING_BUFFER_LEN 6

#define AVERAGING_RING_BUFFER_LEN 6
#define FAST_AVG_BUFFER_LEN 3

#define FLEX_AVG_BUFFER_LEN 8

#ifdef SMOOTH_STD_DEV
 #define STDEV_RING_BUFFER_LEN 6
#endif

#define RING_BUFFER_MAX_LEN 64

struct buffer {
  buffer(buff_t *d, buff_t l) : len(l), data(d) {
    index = 0;
    count = 0;
    memset(data, 0, l);
  }
  uint8_t len;
  uint8_t count;
  uint8_t index;
  buff_t *data;
};

buff_t avg_buffer_data[AVERAGING_RING_BUFFER_LEN];
buffer avg_buffer(avg_buffer_data, AVERAGING_RING_BUFFER_LEN);

buff_t stats_buffer_data[STATS_RING_BUFFER_LEN];
buffer stats_buffer(stats_buffer_data, STATS_RING_BUFFER_LEN);

buff_t fast_avg_buffer_data[FAST_AVG_BUFFER_LEN];
buffer fast_avg_buffer(fast_avg_buffer_data, FAST_AVG_BUFFER_LEN);

#ifdef SMOOTH_STD_DEV
buff_t stdev_avg_buffer_data[STDEV_RING_BUFFER_LEN];
buffer stdev_avg_buffer(stdev_avg_buffer_data, STDEV_RING_BUFFER_LEN);
#endif

#ifdef FLEX_BASELINE_AVERAGE
buff_t flex_avg_buffer_data[FLEX_AVG_BUFFER_LEN];
buffer flex_avg_buffer(flex_avg_buffer_data, FLEX_AVG_BUFFER_LEN);
#endif

void push_to_buffer(buffer *b, uint32_t value)
{
  b->data[b->index] = value;

#if 0
  Ser.print("Written to ");
  Ser.println(b.index);
#endif

  if (b->count < b->len) {
    b->count++;
  }

  b->index = (b->index + 1) % b->len;
}

uint32_t get_ring_buffer_sum(buffer *b)
{
  static uint32_t i = 0;
  static uint32_t ret = 0;
  ret = 0;
  for (i=0; i<b->count; i++) {
    ret += b->data[i];
  }
  return ret;
}

uint32_t get_ring_buffer_mean(buffer *b)
{
  return get_ring_buffer_sum(b) / b->count;
}

int comp_fn(const void *a, const void *b) {
   return (*(uint32_t*)a - *(uint32_t*)b);
}

uint32_t get_ring_buffer_median(buffer *b)
{
  static uint32_t rbuf_sorted[RING_BUFFER_MAX_LEN] = {0};

  memcpy(rbuf_sorted, b->data, b->len * sizeof(uint32_t));
  qsort(rbuf_sorted, b->len, sizeof(uint32_t), comp_fn);

#if 0
  dump_ring_buffer();
  for (int i=0; i<b->len; i++) {
    Ser.print(rbuf_sorted[i]);
    Ser.print(",");
  }
  Ser.println();
#endif

  if (b->len % 2 == 0) {
    return (b->data[b->len / 2] + b->data[(b->len / 2) + 1]) / 2;
  } else {
    return b->data[(b->len / 2) + 1];
  }
}

float get_ring_buffer_variance(buffer *b)
{
  static float sum = 0.0;
  static float mean = 0.0;
  static uint32_t i = 0;

  sum = 0;
  mean = (float) get_ring_buffer_mean(b);

  for (i=0; i<b->len; i++) {
    sum += pow(b->data[i] - mean, 2);
  }

  return sum / (float) (b->len - 1);
}

void dump_ring_buffer(buffer *b)
{
  static uint32_t i = 0;
  for (i=0; i<b->len; i++) {
    Ser.print(b->data[i]);
    Ser.print(",");
  }
  Ser.println();
}

float get_ring_buffer_standard_deviation(buffer *b)
{
  return sqrt(get_ring_buffer_variance(b));
}

/* ---- Set-up ---- */

void setup()
{
  Ser.begin(SERIAL_BAUD);

  delay(1000);

  Ser.println();
  Ser.print(">> Set up board serial and host serial (");
  Ser.print(SERIAL_BAUD);
  Ser.println(" baud)");

  Ser.print(F("---- "));
  Ser.print(__FILE__);
  Ser.println(F(" ----"));

#ifdef PRINT_OSCCAL
  Ser.print("OSCCAL -> 0x");
  Ser.println(OSCCAL, HEX);
#endif

#ifdef OSCCAL_VALUE
  Ser.print(F(">> Setting OSCCAL"));
  OSCCAL = OSCCAL_VALUE;
  Ser.println();
  Ser.print(F("New OSCCAL value -> 0x"));
  Ser.println(OSCCAL, HEX);
#endif

#if defined HEADPHONE_B || defined HEADPHONE_A
  Serial.print(F(">> This is "));
#endif

#ifdef HEADPHONE_A
  Serial.println(F("Headphone A"));
#endif

#ifdef HEADPHONE_B
  Serial.println(F("Headphone B"));
#endif

#ifdef TEST_TOUCH_SENSE
  #warning using test mode TEST_TOUCH_SENSE, normal detection is turned off
  Serial.println(F(">> In test mode! Normal detection is turned off!"));
#endif

  // set up GPIO pins
#ifdef FLEX
  pinMode(FLEX, INPUT);
#endif
  pinMode(LED, OUTPUT);
#ifdef STATUS_LED
  pinMode(STATUS_LED, OUTPUT);
#endif
#ifdef DEBUG_OUTPUT_PIN
  pinMode(DEBUG_OUTPUT_PIN, INPUT_PULLUP);
#endif

  // flash LED at start
  for (int i=0; i<5; i++) {
#if defined STATUS_LED && LED != STATUS_LED
    digitalWrite(STATUS_LED, HIGH);
#endif
    digitalWrite(LED, HIGH);
    delay(75);
    digitalWrite(LED, LOW);
#if defined STATUS_LED && LED != STATUS_LED
    digitalWrite(STATUS_LED, LOW);
#endif
    delay(75);
  }

  digitalWrite(LED, LOW);

#ifdef SERIAL_TEST
 #warning serial test is active!
 error();
#endif

  // turn off CapacitiveSensor library auto-calibration
#ifndef TEST_TOUCH_SENSE
  cs.set_CS_AutocaL_Millis(0xFFFFFFFF);
#endif
}

/* ---- Loop ---- */

void tab()
{
  Ser.print(F("\t"));
}

void loop()
{
  static uint32_t start, ms;

  // get capacitative sensor reading
  start = millis();
#ifdef TEST_TOUCH_SENSE
  uint32_t cs_count_raw = cs.capacitiveSensor(100);
#else
 #ifdef HEADPHONE_B
  uint32_t cs_count_raw = cs.capacitiveSensorRaw(100);
 #else
  uint32_t cs_count_raw = cs.capacitiveSensorRaw(200);
 #endif
#endif
  ms = millis() - start;

  delay(10);

  start = millis();

  // get flex reading
  uint16_t flex = analogRead(FLEX);
#ifdef FLEX_BASELINE_AVERAGE
  push_to_buffer(&flex_avg_buffer, flex);
#endif

  // push data to statistics buffers
  if (cs_count_raw >= 0) {
    // push raw values to smoothing buffers
    push_to_buffer(&fast_avg_buffer, cs_count_raw);
    push_to_buffer(&avg_buffer, cs_count_raw);

#define AVG_PUSH
#ifdef AVG_PUSH
    // push smoothed value to stats buffer
    push_to_buffer(&stats_buffer, get_ring_buffer_mean(&avg_buffer));
#else
    push_to_buffer(&stats_buffer, cs_count_raw);
#endif
  }

  // get standard deviation
  float stdev = get_ring_buffer_standard_deviation(&stats_buffer);

  // push standard deviation to its own averaging buffer
#ifdef SMOOTH_STD_DEV
  push_to_buffer(&stdev_avg_buffer, round(stdev));
#endif

  // get headphone stats data
  buff_t fast_mean = get_ring_buffer_mean(&fast_avg_buffer);
  buff_t mean = get_ring_buffer_mean(&avg_buffer);
  buff_t stats_mean = get_ring_buffer_mean(&stats_buffer);
  float fast_stdev = get_ring_buffer_standard_deviation(&fast_avg_buffer);

#ifdef SMOOTH_STD_DEV
  buff_t smooth_stdev = get_ring_buffer_mean(&stdev_avg_buffer);
#else
  buff_t smooth_stdev = 0;
#endif

#ifdef FLEX_BASELINE_AVERAGE
  uint16_t flex_mean = get_ring_buffer_mean(&flex_avg_buffer);
  bool flexed = is_flexed(flex_mean);
#else
  uint16_t flex_mean = 0;
  bool flexed = is_flexed(flex);
#endif

  print_state();

  // debug printing, only if DEBUG_OUTPUT_PIN is pulled low
#ifdef DEBUG_OUTPUT_PIN
  if (digitalRead(DEBUG_OUTPUT_PIN)) {
#endif
    Ser.print(ms);
  #if 0
    Ser.print("\t");
    Ser.print(cs_count);
  #endif
    tab();
    Ser.print(cs_count_raw);
    tab();
    Ser.print(fast_mean);
    tab();
    Ser.print(mean);
  #if 0
    tab();
    Ser.print(get_ring_buffer_median());
    tab();
    Ser.print(get_ring_buffer_variance());
  #endif
    tab();
    Ser.print(stats_mean);
    tab();
    Ser.print(fast_stdev);
    tab();
    Ser.print(stdev);
    tab();
    Ser.print(smooth_stdev);
#ifdef FLEX
#ifdef FLEX_BASELINE_AVERAGE
    tab();
    Ser.print(flex_mean);
#else
    tab();
    Ser.print(flex);
#endif
    tab();
    Ser.print(flexed);
#endif

    tab();
    Ser.print(CS_HOOK);
    tab();
    Ser.print(CS_WORN);
    tab();
    Ser.print(CS_MIN);

    //delay(10);
#ifdef DEBUG_OUTPUT_PIN
  }
#endif

#ifdef TEST_COMMAND_SEND
  static uint32_t send_time_2 = 0;
  if (millis() - send_time_2 > 2000) {
    Ser.println("W");
    send_time_2 = millis();
  }
#endif

#ifdef TEST_TOUCH_SENSE
  headphone_detection(fast_mean, mean, stats_mean, stdev, fast_stdev, smooth_stdev, flexed);
#else

  record_minimum(mean);

  detect_stable(fast_mean, smooth_stdev, flexed, &avg_buffer);

  detect_fast(mean, flexed);

#endif

  Serial.println();

#ifdef HEARTBEAT
 #if STATUS_LED == LED
  if (STATE != EVENT_HEADPHONES_WORN) {
 #endif
    digitalWrite(STATUS_LED, millis() % 4096 < 200);
 #if STATUS_LED == LED
  }
 #endif
#endif
}

/* ---- Utility functions ---- */

void record_minimum(buff_t mean)
{
  if (STATE == EVENT_HEADPHONES_PLACED_ON_HOOK) {
    if (mean < CS_MIN) { CS_MIN = mean; }
  }
}

bool is_flexed(uint16_t flex)
{
  static bool flexed = 0;
  static uint16_t max_flex = 0;
  static uint16_t min_flex = UINT16_MAX;
  static uint32_t last_change = 0;

#if 0
  Serial.print("F ");
  Serial.print(flex);
  Serial.print(" F^ ");
  Serial.print(max_flex);
  Serial.print(" |_ ");
  Serial.print(min_flex);
  tab();
#endif

  switch (flexed) {
    case true:
      // update maximum value
      if (flex > max_flex) { max_flex = flex; }

      // test to go unflexed - decrease of less than half the range
      if (flex < max_flex - ((max_flex - min_flex) / 2)) {
        min_flex = UINT16_MAX;
        flexed = false;
        last_change = millis();
      }
      break;

    case false:
      // periodically reset minimum reading
      if (millis() - last_change > 5000) {
        min_flex = UINT16_MAX;
        last_change = millis();
      }

      // update minimum value
      if (flex < min_flex) { min_flex = flex; }

      // test to go flexed
      if (flex > min_flex + 15) {
        max_flex = 0;
        flexed = true;
        last_change = millis();
      }

      break;
  }

  return flexed;
}

/* ---- Headphone detection logic ---- */

void detect_stable(buff_t mean, buff_t stdev, bool flexed, buffer *buf)
{
  static bool last_flexed = false;
  static uint8_t count_worn = 0;
  static uint8_t count_hook = 0;

  // reset counters if flexed state changed
  if (flexed != last_flexed) {
    count_worn = 0;
    count_hook = 0;
  }

  if (flexed) {
  // the headphones have different physical characteristics
#ifdef HEADPHONE_A
    if (stdev < 25 && mean > CS_HOOK) {
#else
    //if (stdev < 20) {
    if (stdev < 55 && mean > CS_MIN + 900) {
#endif
      count_worn++;
    } else {
      count_worn = 0;
    }
  } else {
    if (stdev < 10) {
      count_hook++;
    } else {
      count_worn = 0;
    }
  }

  if (count_worn > 5) {
    count_worn = 0;
    CS_WORN = get_ring_buffer_mean(buf);
    if (STATE != EVENT_HEADPHONES_WORN) {
      signal(EVENT_HEADPHONES_WORN);
    }
  }

  if (count_hook > 5) {
    count_hook = 0;
    CS_HOOK = get_ring_buffer_mean(buf);
    if (STATE != EVENT_HEADPHONES_PLACED_ON_HOOK) {
      CS_MIN = UINT16_MAX; // reset minimum
      signal(EVENT_HEADPHONES_PLACED_ON_HOOK);
    }
  }

  last_flexed = flexed;
}

void signal(headphone_state s) {
  STATE = s;

  Serial.println(); // ensure we start a new line

  switch (STATE) {
    case EVENT_HEADPHONES_WORN:
      Serial.println("W"); // signal headphone worn status
      break;
    case EVENT_HEADPHONES_REMOVED:
      Serial.println("R"); // signal headphone removal
      break;
    case EVENT_HEADPHONES_PLACED_ON_HOOK:
      Serial.println("H"); // signal headphone hook status
    default:
      break;
  }
}

void print_state()
{
  switch (STATE) {
    case EVENT_HEADPHONES_WORN:
      Serial.print("#W ");
      break;
    case EVENT_HEADPHONES_REMOVED:
      Serial.print("#R ");
      break;
    case EVENT_HEADPHONES_PLACED_ON_HOOK:
    default:
      Serial.print("#H ");
      break;
  }
}

bool in_range(buff_t a, buff_t b)
{
  if (b > a) {
    return (b - a < 200);
  } else {
    return (a - b < 200);
  }
}

void detect_fast(buff_t cs, bool flexed)
{
  static uint8_t worn_count = 0;
  static uint8_t hook_count = 0;
  static uint8_t remove_count = 0;

  if (STATE == EVENT_HEADPHONES_WORN && cs < CS_WORN - 800) {
    remove_count++;
  } else {
    remove_count = 0;
  }

  if (STATE != EVENT_HEADPHONES_WORN && in_range(cs, CS_WORN)) {
    worn_count++;
  } else {
    worn_count = 0;
  }

  if (STATE != EVENT_HEADPHONES_PLACED_ON_HOOK && in_range(cs, CS_HOOK)) {
    hook_count++;
  } else {
    hook_count = 0;
  }

  if (remove_count > 2 || (!flexed && STATE == EVENT_HEADPHONES_WORN)) {
    remove_count = 0;
    signal(EVENT_HEADPHONES_REMOVED);
  }

#if 0
  if (worn_count > 5) {
    worn_count = 0;
    signal(EVENT_HEADPHONES_WORN);
  }

  if (hook_count > 5) {
    hook_count = 0;
    signal(EVENT_HEADPHONES_PLACED_ON_HOOK);
  }
#endif
}

// simple touch detection for testing
#ifdef TEST_TOUCH_SENSE

 #ifdef TEST_SEND_HOOK_EVENT

  void headphone_detection(
  buff_t fast_mean,
  buff_t mean,
  buff_t stats_mean,
  float stdev,
  float fast_stdev,
  buff_t smooth_stdev,
  bool flexed
)
{
  static bool state = false;
  static uint8_t i;

  if (!state && mean > 1200) {
    Ser.println("H"); // signal transmitter of hook state
    flash_twice();
    state = true;
  } else if (state && mean < 1200) {
    state = false;
  }
}

 #else

  void headphone_detection(
  buff_t fast_mean,
  buff_t mean,
  buff_t stats_mean,
  float stdev,
  float fast_stdev,
  buff_t smooth_stdev,
  bool flexed
)
{
  if (STATE != EVENT_HEADPHONES_WORN && mean > 1200) {
    digitalWrite(LED, HIGH);
    signal(EVENT_HEADPHONES_WORN);
  } else if (STATE != EVENT_HEADPHONES_REMOVED && mean < 1200) {
    digitalWrite(LED, LOW);
    signal(EVENT_HEADPHONES_REMOVED);
  }
}
 #endif

#else

/*
 * ms   raw     fm      m       sm      fstdev  stdev   smoothstdev
 * 7	56	60	73	79	34.10	3.18	0
 */

inline void headphone_detection(
  buff_t fast_mean,
  buff_t mean,
  buff_t stats_mean,
  float stdev,
  float fast_stdev,
  buff_t smooth_stdev,
  bool flexed
)
{

}

#endif

void flash_twice()
{
  static uint8_t i;
  i = 2;
  while (i --> 0) {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
  }
}

// halt and flash LED quickly to show there's a problem
void error()
{
  static uint8_t count = 0;

#ifdef STATUS_LED
  pinMode(STATUS_LED, OUTPUT);
#endif
#ifdef STATUS_LED
  pinMode(LED, OUTPUT);
#endif

  for (;;) {
#ifdef STATUS_LED
    digitalWrite(STATUS_LED, HIGH);
#endif
#ifdef LED
    digitalWrite(LED, HIGH);
#endif

    delay(100);

#ifdef STATUS_LED
    digitalWrite(STATUS_LED, LOW);
#endif
#ifdef LED
    digitalWrite(LED, LOW);
#endif

    delay(100);

    // spam serial
    if (count++ % 10 == 0) {
      Ser.println(".");
    }
  }
}
