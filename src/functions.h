#pragma once

#include <SPI.h>
#include <EEPROM.h>

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <DataTransmission.h>
#include <RCSwitch.h>
#include "signal_data.h"
#include "hardware/adc.h"
#include "RF24.h"

#define APP_NAME "eHack Portable"
#define APP_VERSION "v0.0.1"

// Modes
enum Mode
{
    IDLE,

    HF_SPECTRUM,
    HF_ACTIVITY,
    HF_BARRIER_SCAN,
    HF_BARRIER_REPLAY,
    HF_BARRIER_BRUTE_CAME,
    HF_BARRIER_BRUTE_NICE,
    HF_SCAN,
    HF_REPLAY,
    HF_JAMMER,
    HF_TESLA,

    UHF_SPECTRUM,
    UHF_ALL_JAMMER,
    UHF_WIFI_JAMMER,
    UHF_BT_JAMMER,
    UHF_BLE_JAMMER,
};

Mode currentMode = IDLE;
/* ================== Common ================== */
bool initialized = false;
bool initializedIdle = false;

/* ================= Battery ================== */
#define BATTERY_COEFFICIENT 0.9611905
#define R1 200000 // 200k
#define R2 100000 // 100k
#define BATTERY_RESISTANCE_COEFFICIENT (1 + R1 / R2)
#define V_REF 3.3

#define BATTERY_CHECK_INTERVAL 10000
#define BATTERY_READ_ITERATIONS 10

float batVoltage;
uint32_t batteryTimer;
/* ==================== Tesla ================== */
#define pulseWidth 400     // µs
#define messageDistance 23 // ms
#define transmissions 5

/* =================== SubGHz MHz ================== */
#define GD0_PIN_CC 19
#define CSN_PIN_CC 17
#define RSSI_WINDOW_MS 100
#define RSSI_STEP_MS 50
#define RSSI_BUFFER_SIZE 90

int currentRssi = -100;
uint8_t currentFreqIndex = 1;
uint8_t currentScanFreq = 0;
const float raFrequencies[] = {315.0, 433.92, 868.0, 915.0};
const uint8_t raFreqCount = sizeof(raFrequencies) / sizeof(raFrequencies[0]);
float rssiMaxPeak[raFreqCount] = {-100, -100, -100, -100};
float rssiAbsoluteMax[raFreqCount] = {-100, -100, -100, -100};

uint8_t rssiIndex = 0;
int rssiBuffer[RSSI_BUFFER_SIZE];

RCSwitch mySwitch = RCSwitch();

struct SimpleRAData
{
    uint32_t code;
    uint16_t length;
    uint16_t protocol;
    uint16_t delay;
};

uint8_t lastUsedSlotRA = 0;
uint8_t selectedSlotRA = 0;

uint32_t capturedCode;
uint16_t capturedLength;
uint16_t capturedProtocol;
uint16_t capturedDelay;

bool mySwitchIsAvailable = false;
uint16_t mySwitchSetTime = 0;

bool attackIsActive = false;
bool signalCaptured_433MHZ = false;

/* ================ Barrier =================== */
#define MAX_DELTA_T_BARRIER 200
#define AN_MOTORS_PULSE 412

struct SimpleBarrierData
{
    uint32_t codeMain;
    uint32_t codeAdd;
    uint8_t protocol;
};

int16_t barrierBruteIndex = 4095;
volatile uint32_t barrierCodeMain, barrierCodeAdd;
volatile uint8_t barrierProtocol;
volatile uint8_t barrierBit;

uint8_t selectedSlotBarrier = 0;
uint8_t lastUsedSlotBarrier = 0;

volatile uint16_t lastEdgeMicros;
volatile uint16_t lowDurationMicros, highDurationMicros, barrierCurrentLevel;

// AN Motors
volatile byte anMotorsCounter = 0;      // количество принятых битов
volatile long code1 = 0;                // зашифрованная часть
volatile long code2 = 0;                // фиксированная часть
volatile bool anMotorsCaptured = false; // флаг, что код принят
// CAME
volatile byte cameCounter = 0;  // сохраненое количество бит
volatile uint32_t cameCode = 0; // код Came
volatile bool cameCaptured = false;
// NICE
volatile byte niceCounter = 0;  // сохраненое количество бит
volatile uint32_t niceCode = 0; // код Nice
volatile bool niceCaptured = false;

/*=================== EEPROM ==========================*/
#define MAX_RA_SIGNALS 10
// ================= 2.4 GHZ ===========================/
#define CE_PIN_NRF 21
#define CSN_PIN_NRF 20
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
DataTransmission communication(&radio_RF24, &ELECHOUSE_cc1101);

bool succsessfulConnection = false;

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

    if (mode == COMMAND_IDLE || (len < 4 || data[0] != PROTOCOL_HEADER))
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

    return IDLE;
}
/*********************** CC1101 ***************************/
void cc1101Init()
{
    ELECHOUSE_cc1101.setSpiPin(6, 4, 7, CSN_PIN_CC);

    ELECHOUSE_cc1101.setClb(1, 11, 13);
    ELECHOUSE_cc1101.setClb(2, 14, 17);
    ELECHOUSE_cc1101.setClb(3, 29, 33);
    ELECHOUSE_cc1101.setClb(4, 33, 34);

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setModulation(2); // ASK
    ELECHOUSE_cc1101.setRxBW(135);     // 58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325, 406, 464, 541, 650 and 812 kHz
    ELECHOUSE_cc1101.setGDO0(GD0_PIN_CC);
    ELECHOUSE_cc1101.setPA(12);                // TxPower: (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
    ELECHOUSE_cc1101.setMHZ(raFrequencies[1]); // 300-348 MHZ, 387-464MHZ and 779-928MHZ
    ELECHOUSE_cc1101.setDcFilterOff(0);        // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity). (leave at 0 → better sensitivity)
    ELECHOUSE_cc1101.setPQT(0);                // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted. (PQT=1 is safe for ASK)
    ELECHOUSE_cc1101.setPRE(0);                // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24 (6 or higher helps detect ASK bursts)
    ELECHOUSE_cc1101.setSyncMode(0);           // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. (2 is optimal for ASK + RCSwitch)
    ELECHOUSE_cc1101.setFEC(0);                // Enable Forward Error Correction (FEC). 0 = Disable, 1 = Enable. (leave at 0 for RCSwitch ASK)
    ELECHOUSE_cc1101.setCCMode(0);             // set config for internal transmission mode.
    ELECHOUSE_cc1101.setPktFormat(0);          // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. (needed for RCSwitch, leave at 0)
    ELECHOUSE_cc1101.setAdrChk(0);             // Controls address check configuration of received packages. 0 = No address check. (leave at 0 for RCSwitch ASK)
    ELECHOUSE_cc1101.goSleep();
}

void cc1101ReadyMode()
{
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setModulation(2); 
  ELECHOUSE_cc1101.setRxBW(135);
  ELECHOUSE_cc1101.setPA(12); 
  ELECHOUSE_cc1101.setMHZ(raFrequencies[1]); 
  ELECHOUSE_cc1101.setPQT(0);                // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted. (PQT=1 is safe for ASK)
  ELECHOUSE_cc1101.setPRE(0);                // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24 (6 or higher helps detect ASK bursts)
  ELECHOUSE_cc1101.setSyncMode(0);           // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. (2 is optimal for ASK + RCSwitch)
  ELECHOUSE_cc1101.setFEC(0);                // Enable Forward Error Correction (FEC). 0 = Disable, 1 = Enable. (leave at 0 for RCSwitch ASK)
  ELECHOUSE_cc1101.setCCMode(0);             // set config for internal transmission mode.
  ELECHOUSE_cc1101.setPktFormat(0);          // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. (needed for RCSwitch, leave at 0)
  ELECHOUSE_cc1101.setAdrChk(0);             // Controls address check configuration of received packages. 0 = No address check. (leave at 0 for RCSwitch ASK)
  ELECHOUSE_cc1101.goSleep();
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
    // отправка 12 начальных импульсов 0-1
    for (int i = 0; i < 12; i++)
    {
      delayMicroseconds(AN_MOTORS_PULSE);
      digitalWrite(GD0_PIN_CC, HIGH);
      delayMicroseconds(AN_MOTORS_PULSE);
      digitalWrite(GD0_PIN_CC, LOW);
    }
    delayMicroseconds(AN_MOTORS_PULSE * 10);
    // отправка первой части кода
    for (int i = 32; i > 0; i--)
    {
      SendBit(bitRead(c1, i - 1), AN_MOTORS_PULSE);
    }
    // отправка второй части кода
    for (int i = 32; i > 0; i--)
    {
      SendBit(bitRead(c2, i - 1), AN_MOTORS_PULSE);
    }
    // отправка бит, которые означают батарею и флаг повтора
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
  // barrierCurrentLevel = digitalRead(RA_RX);
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
      anMotorsCaptured = true;
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
    cameCaptured = true;
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
    niceCaptured = true;
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
  if (radio_RF24.begin())
  {
    radio_RF24.powerUp();
    radio_RF24.setAutoAck(false);
    radio_RF24.stopListening();
    radio_RF24.setRetries(0, 0);
    radio_RF24.setPALevel(RF24_PA_MAX, true);
    radio_RF24.setDataRate(RF24_2MBPS);
    radio_RF24.setCRCLength(RF24_CRC_DISABLED);
    radio_RF24.startConstCarrier(RF24_PA_HIGH, START_CHANNEL);
  }
}

void stopRadioAttack()
{
  radio_RF24.stopConstCarrier();
  radio_RF24.powerDown();
}