#include "functions.h"

void setup()
{
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

  communication.setSlaveMode();
  communication.init();

  DBG("Current mode: %d\n\n", currentMode);

  currentLedMode = LED_ON;

#ifdef DEBUG_eHack
  Serial.begin(9600);
#endif
}

void setup1()
{
}

// Loop for common tasks
void loop1()
{
  uint32_t now = millis();

  if (successfullyConnected)
  {
    switch (currentMode)
    {
    case IDLE:
    {
      if (!initializedIdle)
      {
        mySwitch.disableReceive();
        mySwitch.disableTransmit();
        mySwitch.resetAvailable();
        digitalWrite(GD0_PIN_CC, LOW);
        currentLedMode = LED_BLINK_SLOW;
        initialized = false;
        attackIsActive = false;
        initializedIdle = true;
      }
      break;
    }
    case HF_SPECTRUM:
    {
      static uint32_t spectrumTimer = 0;
      static bool waitingForSettle = false;

      if (!initialized)
      {
        pinMode(GD0_PIN_CC, INPUT);
        ELECHOUSE_cc1101.SetRx(raFrequencies[currentScanFreq]);
        radio_RF24.stopListening();
        currentLedMode = LED_BLINK_FAST;
        spectrumTimer = now;
        waitingForSettle = true;
        currentScanFreq = 0;
        initialized = true;
      }

      if (successfullyConnected && waitingForSettle)
      {
        if (now - spectrumTimer >= 1.5 * RSSI_STEP_MS)
        {
          currentRssi = ELECHOUSE_cc1101.getRssi();

          int data[2];
          data[0] = currentRssi;
          data[1] = currentScanFreq;

          radio_RF24.write(&data, sizeof(data));

          currentScanFreq = (currentScanFreq + 1) % raFreqCount;
          ELECHOUSE_cc1101.SetRx(raFrequencies[currentScanFreq]);
          spectrumTimer = now;
          waitingForSettle = true;
        }
      }

      break;
    }
    case HF_ACTIVITY:
    {
      static uint32_t lastStepMs = now;

      if (!initialized)
      {
        pinMode(GD0_PIN_CC, INPUT);
        ELECHOUSE_cc1101.SetRx(raFrequencies[currentFreqIndex]);
        radio_RF24.stopListening();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (successfullyConnected && now - lastStepMs >= RSSI_STEP_MS)
      {
        currentRssi = ELECHOUSE_cc1101.getRssi();
        radio_RF24.write(&currentRssi, sizeof(currentRssi));
        lastStepMs = now;
      }

      break;
    }
    case HF_SCAN:
    {
      if (!initialized)
      {
        pinMode(GD0_PIN_CC, INPUT);
        mySwitch.enableReceive(GD0_PIN_CC);
        ELECHOUSE_cc1101.SetRx(radioFrequency);
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
        pinMode(GD0_PIN_CC, INPUT);
        mySwitch.enableReceive(GD0_PIN_CC);
        ELECHOUSE_cc1101.SetRx(radioFrequency);
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
        pinMode(GD0_PIN_CC, OUTPUT);
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
        pinMode(GD0_PIN_CC, OUTPUT);
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
        DBG("Initializing UHF All Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }
      static int current_channel_index = 0;
      radioChannel = full_channels[current_channel_index];
      radio_RF24.setChannel(radioChannel);
      current_channel_index = (current_channel_index + 1) % (sizeof(full_channels) / sizeof(full_channels[0]));
      break;
    }
    case UHF_WIFI_JAMMER:
    {
      if (!initialized)
      {
        DBG("Initializing UHF WiFi Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }
      static int current_channel_index = 0;
      radioChannel = wifi_channels[current_channel_index];
      radio_RF24.setChannel(radioChannel);
      current_channel_index = (current_channel_index + 1) % (sizeof(wifi_channels) / sizeof(wifi_channels[0]));
      break;
    }
    case UHF_BT_JAMMER:
    {
      if (!initialized)
      {
        DBG("Initializing UHF Bluetooth Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }
      static int current_channel_index = 0;
      radioChannel = bluetooth_channels[current_channel_index];
      radio_RF24.setChannel(radioChannel);
      current_channel_index = (current_channel_index + 1) % (sizeof(bluetooth_channels) / sizeof(bluetooth_channels[0]));
      break;
    }
    case UHF_BLE_JAMMER:
    {
      if (!initialized)
      {
        DBG("Initializing UHF BLE Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }
      static int current_channel_index = 0;
      radioChannel = ble_channels[current_channel_index];
      radio_RF24.setChannel(radioChannel);
      current_channel_index = (current_channel_index + 1) % (sizeof(ble_channels) / sizeof(ble_channels[0]));
      break;
    }
    case UHF_USB_JAMMER:
    {
      if (!initialized)
      {
        DBG("Initializing UHF USB Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }
      static int current_channel_index = 0;
      radioChannel = usb_channels[current_channel_index];
      radio_RF24.setChannel(radioChannel);
      current_channel_index = (current_channel_index + 1) % (sizeof(usb_channels) / sizeof(usb_channels[0]));
      break;
    }
    case UHF_VIDEO_JAMMER:
    {
      if (!initialized)
      {
        DBG("Initializing UHF VIDEO Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }
      static int current_channel_index = 0;
      radioChannel = video_channels[current_channel_index];
      radio_RF24.setChannel(radioChannel);
      current_channel_index = (current_channel_index + 1) % (sizeof(video_channels) / sizeof(video_channels[0]));
      break;
    }
    case UHF_RC_JAMMER:
    {
      if (!initialized)
      {
        DBG("Initializing UHF RC Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }
      static int current_channel_index = 0;
      radioChannel = rc_channels[current_channel_index];
      radio_RF24.setChannel(radioChannel);
      current_channel_index = (current_channel_index + 1) % (sizeof(rc_channels) / sizeof(rc_channels[0]));
      break;
    }
    }
  }
}

// Loop for communication tasks
void loop()
{
  uint32_t now = millis();
  uint32_t onTime = 0;
  uint32_t offTime = 0;

  switch (currentLedMode)
  {
  case LED_ON:
    digitalWrite(LED_BUILTIN, HIGH);
    break;
  case LED_OFF:
    digitalWrite(LED_BUILTIN, LOW);
    break;
  case LED_BLINK_SLOW:
    onTime = 500;
    offTime = 500;
    break;
  case LED_BLINK_FAST:
    onTime = 100;
    offTime = 800;
    break;
  }

  if (onTime > 0 || offTime > 0)
  {
    if ((ledState && now - ledTimer >= onTime) || (!ledState && now - ledTimer >= offTime))
    {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      ledTimer = now;
    }
  }

  if (!successfullyConnected)
  {
    if (communication.receivePacket(recievedData, &recievedDataLen))
    {
      if (recievedData[0] == 'P' && recievedData[1] == 'I' && recievedData[2] == 'N' && recievedData[3] == 'G')
      {
        DBG("Slave: PING received. Sending PONG...\n");
        if (communication.sendPacket(pong, 32))
        {
          DBG("Slave: PONG sent successfully.\n");
          DBG("Connection established\n");
          successfullyConnected = true;
          currentLedMode = LED_BLINK_SLOW;
          batteryTimer = millis() - BATTERY_CHECK_INTERVAL;
        }
        else
        {
          DBG("Slave: PONG send failed.\n");
        }
      }
    }

    currentLedMode = LED_ON;
  }
  else
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
    case HF_ACTIVITY:
    case HF_SPECTRUM:
      break;

    default:
    {
      if (now - batteryTimer >= BATTERY_CHECK_INTERVAL)
      {
        uint8_t batteryVoltagePacket[32];
        batteryTimer = millis();
        batVoltage = readBatteryVoltage();
        DBG("Battery voltage: %.2fV\n", batVoltage);
        communication.buildPacket(COMMAND_BATTERY_VOLTAGE, (uint8_t *)&batVoltage, sizeof(batVoltage), batteryVoltagePacket);
        communication.sendPacket(batteryVoltagePacket, 32);
      }

      if (communication.receivePacket(recievedData, &recievedDataLen))
      {
        if (recievedData[0] == PROTOCOL_HEADER)
        {
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          DBG("Received frequency: %.2f MHz\n", radioFrequency);

          if (initialized)
          {
            switch (currentMode)
            {
            case HF_SPECTRUM:
            case HF_ACTIVITY:
            case HF_SCAN:
              ELECHOUSE_cc1101.SetRx(radioFrequency);
              break;
            case HF_REPLAY:
            case HF_JAMMER:
              ELECHOUSE_cc1101.SetTx(radioFrequency);
              break;
            }
          }
          initializedIdle = false;
          if (currentMode != IDLE)
          {
            communication.sendPacket(inited, 32);
          }
        }
        else if (recievedData[0] == 'P' && recievedData[1] == 'I' && recievedData[2] == 'N' && recievedData[3] == 'G')
        {
          communication.sendPacket(pong, 32);
        }
        else if (recievedData[0] == 'P' && recievedData[1] == 'O' && recievedData[2] == 'N' && recievedData[3] == 'G')
        {
          DBG("Slave: PONG received! Connection OK.\n");
          currentLedMode = LED_BLINK_SLOW;
          awaitingPong = false;
          successfullyConnected = true;
        }
      }

      if (awaitingPong && (now - pingSentTime > 1000))
      {
        DBG("Connection LOST (PONG timeout)!\n");
        successfullyConnected = false;
        awaitingPong = false;
      }

      if (!awaitingPong && (now - checkConnectionTimer > CONNECTION_DELAY))
      {
        if (communication.sendPacket(ping, 32))
        {
          DBG("Slave: PING sent.\n");
          awaitingPong = true;
          pingSentTime = now;
          checkConnectionTimer = now;
        }
        else
        {
          successfullyConnected = false;
        }
      }
      break;
    }
    }
  }
}