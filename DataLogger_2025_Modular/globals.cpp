#include <Arduino.h>
#include "globals.h"
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <ArduinoBLE.h>
#include <CircularBuffer.hpp>

// TwoWire WireESP(SDA_ESP, SCL_ESP);  // Second I2C bus on Nano BLE Sense Rev2
TwoWire WireESP(SDA_ESP, SCL_ESP);  // Second I2C bus on Nano BLE Sense Rev2

// Create GPS parser
TinyGPSPlus gps;

// === OLED Setup (SH1106 I2C) ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// === SD Card Config ===
File dataFile;
char filename[13];  // 8 + 1 + 3 + null terminator = 13 -- arduino lib deals with 8.3 format! 8 letetrs and . then 3 letter as extention 
const int chipSelect = 10;                          
const unsigned long WRITE_INTERVAL = 1000;           // Write to SD every 1 sec
const unsigned long FLUSH_INTERVAL = 10000;          // Flush SD every 5 sec

// === Global Sensor Variables ===
unsigned long timestamp;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float latitude, longitude, gpsAltitude, Speed = 0.0;  //&speed in km/h
int SatCount = 0;
float pressure, paltitude, temperature;
float roll, pitch, yaw = 0.0;

// === Display Variables ===
unsigned long lastDisplaySwitch = 0; // Keep track of the last time the display was updated
int displayState = 0;                // Keep track of what to show: 0 = team, 1 = battery, 2 = satellites

// === Button Handling ===
volatile bool buttonInterruptFlag = false;
unsigned long lastEdge = 0;
bool lastStableState = HIGH;             // Assuming pull-up resistor

//logging variables: 
// Logging state using enum
LogState logState = LogState::IDLE;
unsigned long lastWriteTime = 0;
unsigned long lastFlushTime = 0;
String dataBuffer = ""; // Buffer for batching data

// ====== Calibration Variables ======
float accX_off = 0, accY_off = 0, accZ_off = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float magX_off = 0, magY_off = 0, magZ_off = 0;
float preAccX, preAccY, preAccZ = 0;
float preGyroX, preGyroY, preGyroZ = 0;
float preMagX, preMagY, preMagZ;
bool retryCalibration = 0;

// ====== BLE Variables ======
/* A BLE UUID is a 128-bit value written in hexadecimal, like: 19B10000-E8F2-537E-4F6C-D104768A1214
   This format is standardized by the ITU (International Telecommunication Union).
   The idea is that the probability of two people picking the same UUID by accident is essentially zero.
*/
BLEService svc("19B10000-E8F2-537E-4F6C-D104768A1214");  // svc UUID — Universally Unique Identifier
BLEByteCharacteristic triggerChar(                       // triggerChar UUID, not the same: "0001"
	"19B10001-E8F2-537E-4F6C-D104768A1214",
	BLEWrite | BLEWriteWithoutResponse
);
BLEDevice central;                          // Will contrain information about my central (phone)
uint8_t v = 0;                              // The message I am sending from my central
volatile bool triggerRequested = false;     // volatile variables to be used in Even Handlers. An event handler can be interrupted by an ISR I guess, so keep (atomic) 1 byte size data type like "bool"
volatile bool bleConnected = false;

// ====== Mode and Buffer Settings FOR ESP Sending ======
Mode currentMode = REALTIME;  // default mode

CircularBuffer<String, BUCKET_SIZE> sensorBuffer;

volatile bool espReady = false;
volatile bool I2CError = false;
bool awaitingAck = false;

//  ====== logging parameters ======
unsigned long now = 0;
unsigned long lastCollectTime = 0;
unsigned long  LastI2CError = 0;

// ====== Power monitor globals ======
float battery1Voltage = 0.0f;
float battery2Voltage = 0.0f;

// Voltage divider values
const float R1 = 1786.0; // 1800Ohms 
const float R2 = 505.0;  // 510 Ohms

//=== Pin Assignment for batt indicators===
// constexpr tells the compiler that the value of a variabl can (and should) be evaluated at compile time
const uint8_t PIN_LED_RED = 8;
const uint8_t PIN_LED_GREEN = 9;
const uint8_t PIN_LED_BLUE = 7;
const uint8_t PIN_BAT1_VSENSE = A1;
const uint8_t PIN_BAT2_VSENSE = A2;

// === Battery Indicator Values ===
const uint16_t ADC_MAX = 4095; // at 12-bit resolution
const float VREF = 3.30F; // Volts 
const float BAT_VLOW = 3.50F; // Volts at low battery