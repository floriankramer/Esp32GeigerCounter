// USER CONFIG
// =============================================================================

// Uncomment the line matching your board
#define TTGOV21
// #define TTGOV21_OLD
// #define HELTEC32_1
// #define HELTEC32_2

// Change these values to the values provided by the ttn console.
// Choose lsb as the byte order in the console.
const char DEVEUI[8] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const char APPEUI[8] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// The appkey should be in msb.
const char APPKEY[16] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00};

// The pin to which the sensor is wired
#define GEIGER_PIN 26

// Uncomment this line to disable all serial printing
// #define NO_SERIAL

// Comment this line to prevent the printing of the lora keys
#define PRINT_KEYS

// Controls wether packets should be printed onto the hardware serial
// in addition to being seind over lora.
// #define PRINT_PACKETS

// The following settings control the packet sending behaviour. Due to the
// overhead a package might have (at least 13 bytes) combining as many
// measurements as possible into a single packet combined with a high delay in
// between packets increases the effective throughput.

// The number of seconds between two lora packets.
// DEFAULT: 900
#define SEND_PERIOD 900

// The number of measurements transferred per packet. If this number gets to
// high and error will occur during compilation, as a lmic lora packet may only
// contain 51 bytes DEFAULT: 10
#define NUM_MEASUREMENTS 10

// This can be used to prevent sending of data if nothing was counted by the
// sensor yet. As this brakes the constant time assumption between measurements
// this should only be used (set to more than 0) if NUM_MEASUREMENTS is 1
// DEFAULT: 0
#define MIN_COUNT 0

// This defines the number of insiginificant bits that should not be
// transmitted. Increasing this value exponentially reduces the accuracy of the
// transmitted numbers, but also reduces the number of bytes required to
// transfer a given value. The default of 2 leads to all transmitted numbers
// being multiples of four. This introduces an error of at most 2, but allows
// for transmitting 512 in a sinle byte using variable byte encoding.
// DEFAULT: 2
#define DISCARDED_BITS 2

// Use at most 3 bytes to encode each measurement when sending the data via
// lora.
// DEFAULT: 3
#define MAX_BYTES_PER_COUNT 3

// If the last packet has not yet been send properly new packets will land in a
// queue to be send as soon as possible. This determines the maximum number of
// packets that can be in that queue
// DEFAULT: 3
#define PACKET_QUEUE_LENGTH 3

// The lora port to which packages containing data should be send. Do not use port 0.
// DEFAULT: 1
#define LORA_DATA_PORT 1

// CONFIG CHECKS
// =============================================================================

// The worst-case packet size is MAX_BYTES_PER_COUNT bytes per measurement plus
// a 4 byte timestamp plus at least 13 bytes overhead (16 are used here to allow
// for some leeway).
#if NUM_MEASUREMENTS * MAX_BYTES_PER_COUNT + 2 + 16 > 51
#error Too many measurements for the maximum packet size
#endif

// LMIC STATIC CONFIG
// =============================================================================

#define CFG_eu868 1

// Select the correct radio chip config
#if defined(TTGOV21)
#define CFG_sx1276_radio 1
#elif defined(TTGOV21_OLD)
#define CFG_sx1276_radio 1
#elif defined(HELTEC32_1)
#define CFG_sx1276_radio 1
#elif defined(HELTEC32_2)
#define CFG_sx1276_radio 1
#else
#error No Board Selected
#endif

// The headers can only be included once the radio config was selected
// Contains all the lmic api calls
#include "src/lmic/lmic.h"
// Contains the lmic_pinmap type
#include "src/lmic/hal/hal.h"

#include "SPI.h"
#include "src/queue/queue.h"

// setup the pin map
#if defined(TTGOV21)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#elif defined(TTGOV21_OLD)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#elif defined(HELTEC32_1)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#elif defined(HELTEC32_2)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#endif

#if defined(TTGOV21)
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};
#elif defined(TTGOV21_OLD)
const lmic_pinmap lmic_pins = {.nss = 18,
                               .rxtx = LMIC_UNUSED_PIN,
                               .rst = LMIC_UNUSED_PIN,
                               .dio = {26, 33, 32}};
#elif defined(HELTEC32_1)
const lmic_pinmap lmic_pins = {.nss = 18,
                               .rxtx = LMIC_UNUSED_PIN,
                               .rst = 14,
                               .dio = {26, 33, 32}};
#elif defined(HELTEC32_2)
const lmic_pinmap lmic_pins = {.nss = 18,
                               .rxtx = LMIC_UNUSED_PIN,
                               .rst = 14,
                               .dio = {26, 33, 32}};
#endif

// GLOBALS
// =============================================================================

// This stores the number of impulses measured since the last read
volatile uint_fast16_t geiger_counts_index = 0;
volatile uint32_t geiger_counts[NUM_MEASUREMENTS];
// The point in time when the counter was last reset
ostime_t geiger_count_begin;

// The periodc update job for the counter
osjob_t geiger_counter_job;

// The number of seconds pulses from the sensor should be added to a single bin
const uint_fast16_t MEASUREMENT_PERIOD_SECONDS = SEND_PERIOD / NUM_MEASUREMENTS;

const uint_fast8_t PACKET_BUFFER_SIZE = 50;
FixedSizeQueue packet_queue;

// This number is added to a measurement before any bits are discaded to reduce
// the maximum error.
const uint_fast32_t DISCARDED_BITS_ROUNDING =
    (DISCARDED_BITS > 0 ? 1 << (DISCARDED_BITS - 1) : 0);
// LMIC CALLBACKS
// =============================================================================

/**
   @brief Writes the device eui to buf
*/
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

/**
   @brief Writes the application eui to buf
*/
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

/**
   @brief Writes the application key to buf
*/
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

/**
   @brief Handler function for all lmic callbacks.
*/
void onEvent(ev_t ev) {
  switch (ev) {
    case EV_JOINING:
#ifndef NO_SERIAL
      Serial.println("Trying to join...");
      printLoraKeys();
#endif
      break;
    case EV_JOINED:
#ifndef NO_SERIAL
      Serial.println("Join succesful");
#endif
      // Disable periodic connection checks. Must only be called once a session
      // has established. If this is enabled lmic requests and requires a
      // response be sent every couple of uplink packages. Otherwise the
      // connection is considered dead and the datarate lowered.
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
#ifndef NO_SERIAL
      Serial.println("Join failed");
#endif
      break;
    case EV_REJOIN_FAILED:
#ifndef NO_SERIAL
      Serial.println("Rejoin failed");
#endif
      break;
    case EV_RXCOMPLETE:
      // handle downstream data
      break;
    case EV_TXCOMPLETE:
      // Send the next queued message.
      sendNextPacket();
      break;
    case EV_LINK_DEAD:
#ifndef NO_SERIAL
      Serial.println("lmic link is dead");
#endif
      break;
    case EV_LINK_ALIVE:
#ifndef NO_SERIAL
      Serial.println("lmic link is alive");
#endif
      break;
    default:
#ifndef NO_SERIAL
      Serial.print("lmic event ");
      Serial.println((uint8_t)ev);
#endif
  }
}

// COUNTING AND NETWORK
// =============================================================================

void printHex(u1_t *buf, int count) {
#ifndef NO_SERIAL
  for (int i = 0; i < count; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
#endif
}

void printLoraKeys() {
#ifdef PRINT_KEYS
  uint8_t buf[16];
#ifndef NO_SERIAL
  Serial.print("DEVEUI ");
  memcpy_P(buf, DEVEUI, 8);
  printHex(buf, 8);
  Serial.println();

  Serial.print("APPEUI ");
  memcpy_P(buf, APPEUI, 8);
  printHex(buf, 8);
  Serial.println();

  Serial.print("APPKEY ");
  memcpy_P(buf, APPKEY, 16);
  printHex(buf, 16);
  Serial.println();
#endif
#endif
}

/**
   Package Format
   The package format is as follows
    - 2 bytes time stamp in seconds
    - NUM_MEASUREMENTS many variable byte encoded counts
    The counts have been divided by 2^DISCARDED_BITS
*/

// Increases the current geiger counter bin by 1
void IRAM_ATTR onGeigerCounterSignal() { geiger_counts[geiger_counts_index]++; }

// Remove the last package from the queue. Look at the next entry and check if a
// packet is queued.
void sendNextPacket() {
  if (fsq_size(&packet_queue) > 0) {
    uint8_t *data;
    uint8_t packet_length;
    fsq_front(&packet_queue, &data, &packet_length);
    // Set the next packet to be send
    LMIC_setTxData2(LORA_DATA_PORT, data, packet_length, 0);
#ifndef NO_SERIAL
    Serial.print("Data of length ");
    Serial.println(packet_length);
    printHex(data, packet_length);
    Serial.println();
    Serial.println("Data Done");
#endif
    fsq_pop(&packet_queue);
  }
}

// Resets the geiger_counters and queues a new lora package for sending
void finishCountingCycle() {
  uint8_t *length;
  uint8_t *buffer;
  fsq_push_ref(&packet_queue, &buffer, &length);

  uint_fast8_t offset = 0;
  ostime_t now = os_getTime();
  // A precision of seconds should be more than enough here
  // Internally the clock counts microseconds devided by four and thus,
  // using 32 bits, can only count to less than 20000 seconds, which easily fit
  // into a 16 bit uint.
  uint_fast16_t sec = osticks2ms(now) / 1000;

  // write the current timer in seconds to the buffer in network byte order
  buffer[offset + 1] = sec & 0xFF;
  buffer[offset] = (sec >> 8) & 0xFF;
  offset += 2;

  // Iterate over all measurements of the last cycle
  for (unsigned int i = 0; i < NUM_MEASUREMENTS; i++) {
    // discard the least significant DISCARDED_BITS bits
    uint_fast32_t m =
        (geiger_counts[i] + DISCARDED_BITS_ROUNDING) >> DISCARDED_BITS;
#ifndef NO_SERIAL
    Serial.print("Mesasurement: ");
    Serial.println(m);
#endif
    // Encode the integer using a variable byte encoding.
    // The highest bit of every byte but the last of the integer will be
    // set to 0. Use at most MAX_BYTES_PER_COUNT bytes for the encoding.
    uint_fast8_t num_bytes = 0;
    do {
      // only use the lowest 7 bits, set the most significant bit to 0
      buffer[offset] = m & 127;
      offset++;
      m = m >> 7;
      num_bytes++;
    } while (m > 0 && num_bytes < MAX_BYTES_PER_COUNT);
    // Set the most significant bit of the last byte of this integer to 1
    // to indicate that it is the last byte of this measurement.
    buffer[offset - 1] |= 128;
  }
  *length = offset;

  if (fsq_size(&packet_queue) == 1 && !(LMIC.opmode & OP_TXRXPEND)) {
    // No other packets were queued, the package sending needs to be
    // initialized. If packet_queue_length was greater than one another packet
    // is waiting to be transfered by lmic and sendNextPacket will be called as
    // soon as that packet was send.
    sendNextPacket();
  }
}

// Moves to the next entry in the geiger_counts array
void finishCountingBlock(osjob_t *j) {
  disableGeigerInterrupt();
  geiger_counts_index++;
  // check if we collected enough measurements for the next lora package
  if (geiger_counts_index >= NUM_MEASUREMENTS) {
    finishCountingCycle();
    geiger_counts_index = 0;
  }
  geiger_counts[geiger_counts_index] = 0;
  enableGeigerInterrupt();

  // reschedule this
  os_setTimedCallback(&geiger_counter_job,
                      os_getTime() + sec2osticks(MEASUREMENT_PERIOD_SECONDS),
                      finishCountingBlock);
}

void enableGeigerInterrupt() {
  attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), onGeigerCounterSignal,
                  FALLING);
}

void disableGeigerInterrupt() {
  detachInterrupt(digitalPinToInterrupt(GEIGER_PIN));
}

// Initialization
// =============================================================================

/**
   @brief Initializes the lorawan join.
*/
void initLMIC(osjob_t *job) {
#ifndef NO_SERIAL
  Serial.println("Initializing lora.");
#endif
  LMIC_reset();
  LMIC_startJoining();
  LMIC_setClockError(0.05 * MAX_CLOCK_ERROR);
  // use sf7 at 14db
  LMIC_setDrTxpow(DR_SF7, 14);
}

void setup() {
#ifndef NO_SERIAL
  Serial.begin(115200);
  Serial.println("Beginning setup");
#endif

  // Initialize the packet queue
  fsq_init(&packet_queue, PACKET_BUFFER_SIZE, PACKET_QUEUE_LENGTH);

  // The PiGI board does its own external pullup of these pins
  pinMode(GEIGER_PIN, INPUT);

#ifndef NO_SERIAL
  Serial.println("Initializing lmic.");
#endif

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // initialize the lmic environment
  os_init();
  osjob_t initjob;
  os_setCallback(&initjob, initLMIC);

  // Initialize the geiger counter
  geiger_counts_index = 0;
  for (uint_fast8_t i = 0; i < NUM_MEASUREMENTS; i++) {
    geiger_counts[i] = 0;
  }

  os_setTimedCallback(&geiger_counter_job,
                      os_getTime() + sec2osticks(MEASUREMENT_PERIOD_SECONDS),
                      finishCountingBlock);

  // Use lmic getTime function for timing.
  geiger_count_begin = os_getTime();
  enableGeigerInterrupt();
}

void loop() {
  // Run one iteration of the lmic os loop
  os_runloop_once();
}
