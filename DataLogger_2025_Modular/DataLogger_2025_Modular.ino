#include <Arduino_BMI270_BMM150.h>  // Rev2 IMU
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino_LPS22HB.h> // Barometric sensor (LPS22HB)
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <CircularBuffer.hpp>

#include "globals.h"        // extern globals
#include "display.h"        // OLED display logic
#include "powerMonitor.h"   // battery voltage & LED helpers
#include "button.h"         // ISR + wait for button helper
#include "imu.h"            // 9 Axis IMU
#include "baro.h"           // Barometer
#include "gps.h"            // GPS
#include "sdLogger.h"       // SD
#include "espLink.h"        // ESP32 link + JSON payload + ISR
#include "ble.h"            // BLE init + Event Handlers

// Function and Variable are now declared in the .h files

void setup() {
  Serial.begin(9600);    // USB serial for debug
  
  // Battery indication 
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_BAT1_VSENSE, INPUT);
  pinMode(PIN_BAT2_VSENSE, INPUT);
  analogReadResolution(12); 
  checkBattery();

  // BLE
  if (WITH_BLUETOOTH == 1) {
    initializeBLE();
  }

  // Button 
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButtonPress, FALLING); // Assuming active-low button

  // OLED Init
  u8g2.begin();
  displayTwoLines("OLED", "initialized!", u8g2_font_ncenB10_tr, 40, 25);
  delay(2000);

  // IMU Init (Rev2)
  displayTwoLines("Initializing", "IMU...", u8g2_font_ncenB10_tr, 25, 45);
  delay(2000);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    displayTwoLines("Failed to", "initialize IMU.", u8g2_font_ncenB10_tr, 35, 10);
    while (1);
  }
  
  delay(2000);  // Remove once we make sure an ISR cant interrupt it.
  Serial.println("IMU initialized successfully!");
  displayTwoLines("IMU initialized", "successfully!", u8g2_font_ncenB10_tr, 10, 20);
  delay(2000);

  displayTwoLines("Initializing", "BARO sensor...", u8g2_font_ncenB10_tr, 25, 10);
  delay(2000);
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor.");
    displayTwoLines("Failed to initialize", "pressure sensor.", u8g2_font_6x10_tr, 5, 20);
    while (1);
  }
  Serial.println("BARO sensor initialized!");
  displayTwoLines("BARO sensor", "initialized!", u8g2_font_ncenB10_tr, 15, 25);
  delay(2000);

  // Baro Init
  displayTwoLines("BARO sensor", "initialized!", u8g2_font_ncenB10_tr, 15, 25);
  delay(2000);


  //============ ESP Initialisation  ================
  Serial.println("Waiting for ESP32 + SIM7600 to initialize...");
  displayTwoLines("Waiting for", "ESP32...", u8g2_font_ncenB10_tr, 40, 25);
  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  pinMode(READY_LINE, INPUT);
  pinMode(ACK_LINE, INPUT);
  attachInterrupt(digitalPinToInterrupt(READY_LINE), onESPReady, RISING);

  WireESP.begin();

  espReady = digitalRead(READY_LINE);  // Check initial state
  
  while (!espReady) {
    Serial.println("ESP32 not ready. Retrying in 3s...");
    displayTwoLines("ESP32", "not ready...", u8g2_font_ncenB10_tr, 40, 25);
    delay(3000);
  }

  Serial.println("✅ ESP32 is ready. Proceeding...");
  displayTwoLines("✅ ESP32 Ready", "Proceeding ...", u8g2_font_ncenB08_tr, 0, 12);
  delay(2000);
  //============ ESP Initialisation End  ================

  // GPS module on Serial1 (D0 = RX, D1 = TX)
  //Serial.println("🔍 Starting GPS reader ..."); 
  displayTwoLines("Initializing", "GPS reader...", u8g2_font_ncenB10_tr, 25, 15);
  Serial1.begin(115200); 
  while (!Serial1);
  delay(2000);
  displayTwoLines("GPS started", "successfully!", u8g2_font_ncenB10_tr, 21, 20);
  delay(2000);
  // waiting to get the munimum gps sat count, max wait 30 sec
  waitForGPSLock();  // comment while debuging 


  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to initialize SD card.");
    displayTwoLines("Failed to ini-", "tialize SD card.", u8g2_font_6x10_tr, 25, 20);
    while (1);
  }
  Serial.println("SD card initialized successfully!");
  displayTwoLines("SD card initialized", "successfully!", u8g2_font_6x10_tr, 8, 25);
  delay(2000);

  updateGPSData();
  generateFilename();

  //============ IMU CALIBIRATION. =========//
  // Accelerometer & Gyroscope, stationary
  Serial.println("IMU Calibration 1 (Acc & Gyro): Press button");
  displayTwoLines("IMU Calibration 1:", "Press button", u8g2_font_ncenB10_tr, 10, 20);
  waitForButtonPressed();
  Serial.println("Press, once again, to end calibation 1");
  displayTwoLines("End?", "Press button", u8g2_font_ncenB10_tr, 10, 20);
  waitForButtonPressed();
  
  calibrateIMU();

  // Magnetometer, victory dance!
  Serial.println("IMU Calibration 2 (Magnetometer): Press button");
  displayTwoLines("Magnet. Calibration:", "Press button", u8g2_font_ncenB10_tr, 10, 20);
  waitForButtonPressed();
  do {
    if (retryCalibration == 1) {
      Serial.println("Calibration failed... Trying again");
      displayTwoLines("CALIB FAIL...", "Trying Again", u8g2_font_ncenB10_tr, 10, 20);
    }
    calibrateIMU2();
  } while (retryCalibration);

  Serial.println("IMU calibration complete.");  
  displayTwoLines("IMU calibration", "complete.", u8g2_font_ncenB10_tr, 10, 20);
  delay(2000);
  //============ IMU CALIBIRATION END. =========//

  displayTwoLines("Ready to", "start!", u8g2_font_ncenB10_tr, 35, 45);
  delay(2000);
  
  displayToScreen("Team 1", 30, 35);
  delay(2000);

  // Wait for button to start logging
  bool toggle = false;
  buttonInterruptFlag = false;
  while (!buttonInterruptFlag) {
    updateBatteryVoltage();

    if (battery1Voltage < BAT_VLOW || battery2Voltage < BAT_VLOW) {
      // Low battery: show warning and red LED only
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
      displayToScreen("Battery LOW!", 15, 40);
    } else {
      // Battery OK: rotate display info and blink green LED
      if (millis() - lastDisplaySwitch > 2000) {  // Every 2 seconds
        lastDisplaySwitch = millis();
        displayState = (displayState + 1) % 3;

        switch (displayState) {
          case 0: // Team number
            char buf[20];
            snprintf(buf, sizeof(buf), "Team %d", TEAM_NUMBER);
            displayToScreen(buf, 25, 35);
            break;

          case 1: { // Battery level
            char buf[20];
            snprintf(buf, sizeof(buf), "%.2f V, %.2f V", battery1Voltage, battery2Voltage);
            displayTwoLines("Battery level:", buf, u8g2_font_ncenB10_tr, 15, 50);
            break;
          }

          case 2: // promte the user to start logging
            displayTwoLines("Press the button", "to start logging", u8g2_font_ncenB10_tr, 10, 30);
            break;
        }

        toggle = !toggle;
        digitalWrite(PIN_LED_GREEN, HIGH);  // Blink green LED
        digitalWrite(PIN_LED_RED, LOW);       // Red LED off when voltage is fine
        digitalWrite(PIN_LED_BLUE, toggle);       // Red LED off when voltage is fine
      }
    }
  }

}

void loop() {
  now = millis();
  digitalWrite(LED_PIN, now % 1000 < 50);  // LED heartbeat every 1 sec
  //Serial.println(espReady);
  //Serial.println(digitalRead(READY_LINE));

  if (now - lastCollectTime >= loggingPeriod)  {
    lastCollectTime = now;
    


    updateLogging();
    updateIMUData();
    updateBarometerData();
    updateGPSData();
    log();
    //printIMUOffsetsAndReadings();
    // displayToScreen();
    // readNextLineFromSD();
    //delay(1000); // 1Hz logging rate will be removed later!
    checkBattery();
    
    // UI rotation: team, battery, GPS, log state, ACK status
    if (now - lastDisplaySwitch > 2000) {
      lastDisplaySwitch = now;
      displayState = (displayState + 1) % 5;  // now 5 states total

      switch (displayState) {
        case 0:
          displayToScreen("Team 1", 25, 35);
          break;

        case 1: {
          char buf[20];
          snprintf(buf, sizeof(buf), "%.2f V, %.2f V", battery1Voltage, battery2Voltage);
          displayTwoLines("Battery1,2 level:", buf, u8g2_font_ncenB10_tr, 15, 50);
          break;
        }

        case 2:
          displayTwoLines("Satellites:", String(SatCount).c_str(), u8g2_font_ncenB10_tr, 28, 60);
          break;

        case 3: {
          const char* logStateStr = (logState == LogState::IDLE) ? "IDLE" : "Logging";
          displayTwoLines("Logging State:", logStateStr, u8g2_font_ncenB10_tr, 10, 30);
          break;
        }

        case 4: {
          // Show ACK line status from ESP
          if(I2CError == true)
          {
            displayTwoLines("I2C Error:", "need Debug", u8g2_font_ncenB10_tr, 40, 25);
            break;  // for readability 
          }
          else{
            bool ackHigh = digitalRead(ACK_LINE) == HIGH;
            displayTwoLines("ACK Line:", ackHigh ? "✅ HIGH" : "LOW", u8g2_font_ncenB10_tr, 30, 30);
            //if (ackHigh) Serial.println("✅ ACK received from ESP");
            break;
          }
        }
      }
    }
  }
  // Optional: add a short delay to reduce CPU load if needed
 // delay(1000); // (uncomment if display flickers or CPU usage is high)
}