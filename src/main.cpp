#include "functions.h"

DataTransmission communication(&radio_RF24, &ELECHOUSE_cc1101);

void setup()
{
  Serial.begin(115200);
  Serial.printf("Starting %s %s\n", APP_NAME, APP_VERSION);

  SPI.setSCK(6);
  SPI.setMOSI(7);
  SPI.setMISO(4);
  SPI.begin();

  analogReadResolution(12);
  batVoltage = readBatteryVoltage();
  EEPROM.begin(512);

  if (radio_RF24.begin())
  {
    Serial.println("RF24 initialized successfully");
  }
  else
  {
    Serial.println("RF24 initialization failed");
  }
  radio_RF24.powerDown();

  cc1101Init();
  if (ELECHOUSE_cc1101.getCC1101())
  {
    Serial.println("CC1101 initialized successfully");
  }
  else
  {
    Serial.println("CC1101 initialization failed");
  }

  communication.setRadioNRF24();
  communication.setSlaveMode();
}

void setup1()
{
}

// Loop for common tasks
void loop()
{
}

// Loop for communication tasks
void loop1()
{
  if (communication.checkConnection(1000))
  {
    Serial.println("Connection established");
  }
}
