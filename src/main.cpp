#include "functions.h"

DataTransmission communication(&radio_RF24, &ELECHOUSE_cc1101);

void setup()
{
  SPI.setSCK(6);
  SPI.setMOSI(7);
  SPI.setMISO(4);
  SPI.begin();

  analogReadResolution(12);
  batVoltage = readBatteryVoltage();
  EEPROM.begin(512);

  radio_RF24.powerDown();
  cc1101Init();

  communication.setRadioNRF24();
  communication.setSlaveMode();
}

void setup1()
{
}

void loop()
{
  if (communication.checkConnection(1000))
  {
    Serial.println("Connection established");
  }
}

void loop1()
{
}
