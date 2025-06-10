#pragma once

#include <SPI.h>
#include <EEPROM.h>

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <DataTransmission.h>
#include <RCSwitch.h>
#include "signal_data.h"
#include "hardware/adc.h"
#include "RF24.h"

//Modes
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