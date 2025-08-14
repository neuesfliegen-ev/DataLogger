#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <ArduinoBLE.h>
#include <CircularBuffer.hpp>

#define loggingPeriod 100 // logging Period in milli seconds. Logging Freq= 1/ loggingPeriod KHz

// === Project-wide macros ===
#define TEAM_NUMBER 1
#define MIN_SATS_REQUIRED 4
#define GPS_LOCK_TIMEOUT_MS 30000  // 30 seconds

// === ESP & SIM7600 constants ===
#define ESP32_ADDR 0x42
#define READY_LINE 4
#define ACK_LINE   5
#define LED_PIN    13
#define SDA_ESP    2
#define SCL_ESP    6
#define BUCKET_SIZE 5

// === BLE constants ===
#define WITH_BLUETOOTH 1      // (Mechanical -> 0 / BLE -> 1) button

// === Shared enums used by globals ===
enum class LogState { IDLE, ACTIVE };
enum Mode { REALTIME, BATCH };

// === I2C to ESP32 (second bus) ===
extern TwoWire WireESP;  // TwoWire WireESP(SDA_ESP, SCL_ESP);

// === GPS parser ===
extern TinyGPSPlus gps;

// === OLED (SH1106 I2C) ===
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;

// === SD Card Config ===
extern File dataFile;
extern char filename[13];  // 8 + 1 + 3 + null terminator = 13 -- arduino lib deals with 8.3 format! 8 letetrs and . then 3 letter as extention 
extern const int chipSelect;
extern const unsigned long WRITE_INTERVAL;   // Write to SD every 1 sec
extern const unsigned long FLUSH_INTERVAL;   // Flush SD every 5 sec

// === Global Sensor Variables ===
extern unsigned long timestamp;
extern float accX, accY, accZ;
extern float gyroX, gyroY, gyroZ;
extern float magX, magY, magZ;
extern float latitude, longitude, gpsAltitude, Speed;  // &speed in km/h
extern int SatCount;
extern float pressure, paltitude, temperature;
extern float roll, pitch, yaw;

// === Display Variables ===
extern unsigned long lastDisplaySwitch; // Keep track of the last time the display was updated
extern int displayState;                // Keep track of what to show: 0 = team, 1 = battery, 2 = satellites

// === Button Handling ===
constexpr int PIN_BUTTON = 3;           // Make sure this is an interrupt-capable pin
constexpr uint8_t DEBOUNCE_MS = 200;
extern volatile bool buttonInterruptFlag;
extern unsigned long lastEdge;
extern bool lastStableState;            // Assuming pull-up resistor

// === Logging state ===
extern LogState logState;
extern unsigned long lastWriteTime;
extern unsigned long lastFlushTime;
extern String dataBuffer; // Buffer for batching data

// ====== Calibration Variables ======
extern float accX_off, accY_off, accZ_off;
extern float gyroX_off, gyroY_off, gyroZ_off;
extern float magX_off, magY_off, magZ_off;
extern float preAccX, preAccY, preAccZ;
extern float preGyroX, preGyroY, preGyroZ;
extern float preMagX, preMagY, preMagZ;
extern bool retryCalibration;

// ====== BLE Variables ======
extern BLEService svc;                            // 19B10000-E8F2-537E-4F6C-D104768A1214
extern BLEByteCharacteristic triggerChar;         // 19B10001-E8F2-537E-4F6C-D104768A1214
extern BLEDevice central;
extern uint8_t v;
extern volatile bool triggerRequested;
extern volatile bool bleConnected;

// ====== Mode and Buffer Settings FOR ESP Sending ======
extern Mode currentMode;  // default mode
extern CircularBuffer<String, 5> sensorBuffer; // matches BUCKET_SIZE = 5

extern volatile bool espReady;
extern volatile bool I2CError;
extern bool awaitingAck;

//  ====== logging parameters ======
extern unsigned long now;
extern unsigned long lastCollectTime;
extern unsigned long LastI2CError;

// ====== Power monitor globals ======
extern float battery1Voltage;
extern float battery2Voltage;

// Voltage divider values
extern const float R1; // 1800Ohms 
extern const float R2;  // 510 Ohms

//=== Pin Assignment for batt indicators===
// constexpr tells the compiler that the value of a variabl can (and should) be evaluated at compile time
extern const uint8_t PIN_LED_RED;
extern const uint8_t PIN_LED_GREEN;
extern const uint8_t PIN_LED_BLUE;
extern const uint8_t PIN_BAT1_VSENSE;
extern const uint8_t PIN_BAT2_VSENSE;

// === Battery Indicator Values ===
extern const uint16_t ADC_MAX; // at 12-bit resolution
extern const float VREF; // Volts 
extern const float BAT_VLOW; // Volts at low battery