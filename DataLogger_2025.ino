#include <Arduino_BMI270_BMM150.h>  // Rev2 IMU
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino_LPS22HB.h> // Barometric sensor (LPS22HB)
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

// team number 
#define IDataLoggerId 1
#define MIN_SATS_REQUIRED 4
#define GPS_LOCK_TIMEOUT_MS 30000  // 30 seconds
#define loggingPeriod 100 // logging Period in milli seconds. Logging Freq= 1/ loggingPeriod KHz

// === ESP & SIM7600 constants ===
#define ESP32_ADDR 0x42
#define READY_LINE 4
#define ACK_LINE   5
#define LED_PIN    13
#define SDA_ESP    2
#define SCL_ESP    6
#define BUCKET_SIZE 5

// === Calibration constants ===
#define CALIB_SAFETY_CHECK 1    // boolean
#define CALIB_DEBUG 0
#define CALIB_MIN_ACCURACY 60   // Recommended: 60. Maximum: 90

TwoWire WireESP(SDA_ESP, SCL_ESP);  // Second I2C bus on Nano BLE Sense Rev2

// Create GPS parser
TinyGPSPlus gps;
//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ARDUINOBLE

#include <ArduinoBLE.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "Datalogger1"
#define REMOTEXY_ACCESS_PASSWORD "NFC123"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 242 bytes
  { 255,4,0,20,0,235,0,19,0,0,0,68,97,116,97,108,111,103,103,101,
  114,49,0,25,1,200,84,1,1,12,0,129,65,2,58,10,64,17,68,97,
  116,97,32,76,111,103,103,101,114,0,67,47,34,100,10,5,135,26,16,129,
  166,37,21,4,64,2,79,114,105,101,110,116,97,116,105,111,110,0,1,86,
  48,21,21,0,95,31,0,67,121,4,11,8,77,2,26,1,129,85,25,24,
  7,64,2,83,116,97,116,117,115,58,0,129,85,72,23,4,64,95,80,117,
  115,104,32,66,117,116,116,111,110,0,12,163,23,29,5,194,30,26,49,0,
  50,0,52,0,52,0,12,163,43,29,5,192,30,26,49,0,50,0,12,7,
  21,29,6,199,30,26,49,0,50,0,51,0,52,0,53,0,54,0,55,0,
  56,0,57,0,129,2,15,44,4,64,2,84,101,97,109,32,78,117,109,98,
  101,114,32,83,101,108,101,99,116,111,114,0,129,157,17,44,4,64,2,70,
  108,105,103,104,116,32,78,117,109,98,101,114,32,83,101,108,101,99,116,111,
  114,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t PushButton; // =1 if button pressed, else =0
  uint8_t FlightNumber; // from 0 to 4
  uint8_t Orientation; // from 0 to 2
  uint8_t TeamNumber; // from 0 to 9

    // output variables
  char statusGui[16]; // string UTF8 end zero
  float DataLoggerId;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////


//Global parameters from gui
int FLIGHT_NUMBER;
int TEAM_NUMBER;
int Orientation;

// Voltage divider values
const float R1 = 1786.0; // 1800Ohms 
const float R2 = 505.0;  // 510 Ohms

//=== Pin Assignment for batt indicators===
// constexpr tells the compiler that the value of a variabl can (and should) be evaluated at compile time
constexpr uint8_t PIN_LED_RED = 8; 
constexpr uint8_t PIN_LED_GREEN = 9; 
constexpr uint8_t PIN_LED_BLUE = 7; 
constexpr uint8_t PIN_BAT1_VSENSE = A1;
constexpr uint8_t PIN_BAT2_VSENSE = A2;

// === Battery Indicator Values ===
constexpr uint16_t ADC_MAX = 4095; // at 12-bit resolution
constexpr float VREF = 3.30F; // Volts 
constexpr float BAT_VLOW = 3.50F; // Volts at low battery
float battery1Voltage = 0;
float battery2Voltage = 0;


// === OLED Setup (SH1106 I2C) ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// === SD Card Config ===
const int chipSelect = 10;
File dataFile;
char filename[13];  // 8 + 1 + 3 + null terminator = 13 -- arduino lib deals with 8.3 format! 8 letetrs and . then 3 letter as extention 


// === Global Sensor Variables ===
unsigned long timestamp;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float latitude, longitude, gpsAltitude, Speed = 0.0;  //&speed in km/h
int SatCount = 0; 
float pressure, paltitude, temperature; 

// === Attitude Angle Variables ===
bool firstIteration = true;
Kalman kalmanX, kalmanY, kalmanZ;
float roll = 0, pitch = 0, yaw = 0;
double rollFinal, pitchFinal, yawFinal; // Calculated angle using a Kalman filter
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double gyroXrate, gyroYrate, gyroZrate;
double dt;
unsigned long timer = 0;

// === Display Variables ===
unsigned long lastDisplaySwitch = 0; // Keep track of the last time the display was updated
int displayState = 0;                // Keep track of what to show: 0 = team, 1 = battery, 2 = satellites

// === Button Handling ===
const int PIN_BUTTON = 3;                // Make sure this is an interrupt-capable pin
constexpr uint8_t DEBOUNCE_MS = 200;

volatile bool buttonInterruptFlag = false;
unsigned long lastEdge = 0;
bool lastStableState = HIGH;            // Assuming pull-up resistor


//logging variables: 
// Logging state using enum
enum class LogState { IDLE, ACTIVE };
LogState logState = LogState::IDLE;

unsigned long lastWriteTime = 0;
unsigned long lastFlushTime = 0;
const unsigned long WRITE_INTERVAL = 1000;  // Write to SD every 1 sec
const unsigned long FLUSH_INTERVAL = 10000;  // Flush SD every 5 sec
String dataBuffer = ""; // Buffer for batching data

// ====== Calibration Variables ======
float accX_off  = 0, accY_off  = 0, accZ_off  = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float magX_off  = 0, magY_off  = 0, magZ_off  = 0;
float preAccX, preAccY, preAccZ = 0;
float preGyroX, preGyroY, preGyroZ = 0;
float preMagX, preMagY, preMagZ;
bool retryCalibration = 0;

// ====== Mode and Buffer Settings FOR ESP Sending ======
enum Mode { REALTIME, BATCH };
Mode currentMode = REALTIME;  // default mode

CircularBuffer<String, BUCKET_SIZE> sensorBuffer;

volatile bool espReady = false;
volatile bool I2CError = false;
bool awaitingAck = false;

//  ====== logging parameters ======
unsigned long now = 0;
unsigned long lastCollectTime = 0; 
unsigned long  LastI2CError = 0;

// === Function Declarations ===
void displayToScreen(const char str[], u8g2_uint_t x, u8g2_uint_t y);
void displayTwoLines(const char line1[], const char line2[], const uint8_t* font, u8g2_uint_t x1, u8g2_uint_t x2);
void updateIMUData();
void updateBarometerData();
String generateDataLine();
void logToSD();
void calibrateIMU(); 
void calibrateIMU2();
bool fullSpanCalibration(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t);
void printIMUOffsetsAndReadings();
void attitudeInitialization();
void attitudeAngles();
void generateFilename(); 
void readNextLineFromSD();

// button logic, voltage divider, logging!
void onButtonPress();
bool buttonReleased();
void waitForButtonPressed();
void updateBatteryVoltage();
void checkBattery();
void startLog();
void stopLog();
void updateLogging(); // starts logging at button *release*
String buildJsonPayload(const String& jsonWrappedLine);
template <size_t N>
String buildJsonPayload(const CircularBuffer<String, N>& buffer);
void logToServer();
void log();
void waitForGPSLock(); 
void updateGPSData();
void onESPReady();


void setup() {
  Serial.begin(9600);    // USB serial for debug
  RemoteXY_Init (); 

  // Battery indication 
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_BAT1_VSENSE, INPUT);
  pinMode(PIN_BAT2_VSENSE, INPUT);
  analogReadResolution(12); 
  checkBattery();

  // Button 
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButtonPress, FALLING); // Assuming active-low button

  // OLED Init
  u8g2.begin();
  displayTwoLines("OLED", "initialized!", u8g2_font_ncenB10_tr, 40, 25);
  RemoteXY_delay(2000);

  // IMU Init (Rev2)
  displayTwoLines("Initializing", "IMU...", u8g2_font_ncenB10_tr, 25, 45);
  RemoteXY_delay(2000);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    displayTwoLines("Failed to", "initialize IMU.", u8g2_font_ncenB10_tr, 35, 10);
    while (1);
  }
  
  RemoteXY_delay(2000);  // Remove once we make sure an ISR cant interrupt it.
  Serial.println("IMU initialized successfully!");
  displayTwoLines("IMU initialized", "successfully!", u8g2_font_ncenB10_tr, 10, 20);
  RemoteXY_delay(2000);

  displayTwoLines("Initializing", "BARO sensor...", u8g2_font_ncenB10_tr, 25, 10);
  RemoteXY_delay(2000);
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor.");
    displayTwoLines("Failed to initialize", "pressure sensor.", u8g2_font_6x10_tr, 5, 20);
    while (1);
  }
  Serial.println("BARO sensor initialized!");
  displayTwoLines("BARO sensor", "initialized!", u8g2_font_ncenB10_tr, 15, 25);
  RemoteXY_delay(2000);

  // Baro Init
  displayTwoLines("BARO sensor", "initialized!", u8g2_font_ncenB10_tr, 15, 25);
  RemoteXY_delay(2000);


  //============ ESP Initialisation  ================
  Serial.println("Waiting for ESP32 + SIM7600 to initialize...");
  displayTwoLines("Waiting for", "ESP32...", u8g2_font_ncenB10_tr, 40, 25);
  RemoteXY_delay(2000);

  pinMode(LED_PIN, OUTPUT);
  pinMode(READY_LINE, INPUT);
  pinMode(ACK_LINE, INPUT);
  attachInterrupt(digitalPinToInterrupt(READY_LINE), onESPReady, RISING);

  WireESP.begin();

  espReady = digitalRead(READY_LINE);  // Check initial state
  
  while (!espReady) {
    Serial.println("ESP32 not ready. Retrying in 3s...");
    displayTwoLines("ESP32", "not ready...", u8g2_font_ncenB10_tr, 40, 25);
    RemoteXY_delay(3000);
  }

  Serial.println("✅ ESP32 is ready. Proceeding...");
  displayTwoLines("✅ ESP32 Ready", "Proceeding ...", u8g2_font_ncenB08_tr, 0, 12);
  RemoteXY_delay(2000);
  //============ ESP Initialisation End  ================

  // GPS module on Serial1 (D0 = RX, D1 = TX)
  //Serial.println("🔍 Starting GPS reader ..."); 
  displayTwoLines("Initializing", "GPS reader...", u8g2_font_ncenB10_tr, 25, 15);
  Serial1.begin(115200); 
  while (!Serial1);
  RemoteXY_delay(2000);
  displayTwoLines("GPS started", "successfully!", u8g2_font_ncenB10_tr, 21, 20);
  RemoteXY_delay(2000);
  // waiting to get the munimum gps sat count, max wait 30 sec
  waitForGPSLock();  // comment while debuging 


  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to initialize SD card.");
    displayTwoLines("Failed to ini-", "tialize SD card.", u8g2_font_6x10_tr, 25, 20);
    while (1);
  }
  Serial.println("SD card initialized successfully!");
  displayTwoLines("SD card initialized", "successfully!", u8g2_font_6x10_tr, 8, 25);
  RemoteXY_delay(2000);

  updateGPSData();
  generateFilename();

  //============ IMU CALIBIRATION. =========//
  // Accelerometer & Gyroscope, stationary
  Serial.println("IMU Calibration 1 (Acc & Gyro): Press button");
  displayTwoLines("IMU Off Calibration:", "Press button", u8g2_font_ncenB10_tr, 10, 20);
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
  RemoteXY_delay(2000);
  //============ IMU CALIBIRATION END. =========//

  displayTwoLines("Ready to", "start!", u8g2_font_ncenB10_tr, 35, 45);
  RemoteXY_delay(2000);
  
  displayToScreen("Team 1", 30, 35);
  RemoteXY_delay(2000);

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
    attitudeInitialization();
    attitudeAngles();
    updateBarometerData();
    updateGPSData();
    log();
    //printIMUOffsetsAndReadings();
    // displayToScreen();
    // readNextLineFromSD();
    //RemoteXY_delay(1000); // 1Hz logging rate will be removed later!
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
  // Optional: add a short RemoteXY_delay to reduce CPU load if needed
 // RemoteXY_delay(1000); // (uncomment if display flickers or CPU usage is high)
}

// === IMU Data Update ===
void updateIMUData() {  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(preAccX, preAccY, preAccZ);
    accX = preAccX - accX_off;
    accY = preAccY - accY_off;
    accZ = preAccZ - accZ_off;
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(preGyroX, preGyroY, preGyroZ);
    gyroX = preGyroX - gyroX_off;
    gyroY = preGyroY - gyroY_off;
    gyroZ = preGyroZ - gyroZ_off;
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(preMagX, preMagY, preMagZ);
    magX = preMagX - magX_off;
    magY = preMagY - magY_off;
    magZ = preMagZ - magZ_off;
  }
}

// === Barometer Data Update ===
void updateBarometerData() {
  pressure = BARO.readPressure(); // in kPa
  paltitude = 44330 * (1 - pow(pressure / 101.325, 1 / 5.255));
}

// === OLED Display === For short messages
void displayToScreen(const char str[], u8g2_uint_t x, u8g2_uint_t y) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);   // u8g2_font_ncenB14_tr big size -- u8g2_font_ncenB10_tr IS OK // TOO SMALL u8g2_font_5x8_tr
  u8g2.setCursor(x, y);                 // Centered: (25, 35) -- Starting from the left: (0, 35)
  u8g2.print(str);
  u8g2.sendBuffer();
}

// === OLED Display === In two lines 
void displayTwoLines(const char line1[], const char line2[], const uint8_t* font, u8g2_uint_t x1, u8g2_uint_t x2) { // x1: x-axis position 1st line.  x2: x-axis position 2nd line.
  u8g2.clearBuffer();
  u8g2.setFont(font);
  u8g2.setCursor(x1, 25);  // Y1 = 25 for first line (adjust as needed)
  u8g2.print(line1);
  u8g2.setCursor(x2, 50);  // Y2 = 50 for second line (25+line height)
  u8g2.print(line2);
  u8g2.sendBuffer();
}

// === Read Next Line from SD ===
void readNextLineFromSD() {
  static int lineNumber = 0;
  dataFile = SD.open(filename);
  if (!dataFile) {
    Serial.println("Error opening log.csv for reading.");
    return;
  }

  int currentLine = 0;
  String line = "";

  while (dataFile.available()) {
    line = dataFile.readStringUntil('\n');
    if (currentLine == lineNumber) {
      Serial.print("Line ");
      Serial.print(lineNumber);
      Serial.print(": ");
      Serial.println(line);
      lineNumber++;
      break;
    }
    currentLine++;
  }

  dataFile.close();
}

// === Barometer Data Update ===
void updateGPSData() {
    while (Serial1.available()) {
      char c = Serial1.read();

      // Print raw NMEA sentence
      //Serial.write(c);

      // Feed to TinyGPS parser
      gps.encode(c);
    
      SatCount = gps.satellites.value();

  // Show satellite count every ~1s
  //  if (millis() % 1000 < 50) {
  //    Serial.print("\n Satellites count: ");
  //    Serial.println(gps.satellites.value());
  //    displayTwoLines("Satellites count: ", String(SatCount).c_str(), u8g2_font_ncenB10_tr, 5, 20);
  //  }

    if (gps.location.isUpdated()) {
      latitude     = gps.location.lat();
      longitude    = gps.location.lng();
      gpsAltitude  = gps.altitude.meters();
      Speed        = gps.speed.kmph();
      SatCount     = gps.satellites.value();

      // Serial.println("\n✅ Parsed GPS Data:");
      // Serial.print("Latitude: "); Serial.println(latitude, 6);
      // Serial.print("Longitude: "); Serial.println(longitude, 6);
      // Serial.print("Satellites: "); Serial.println(SatCount);
      // Serial.print("Speed (km/h): "); Serial.println(Speed);
      // Serial.print("Altitude (m): "); Serial.println(gpsAltitude);
      // Serial.println("----------------------");
    }
  }
}

// === Updates current battery voltage ===
void updateBatteryVoltage(){
  battery1Voltage = analogRead(PIN_BAT1_VSENSE) * VREF / ADC_MAX / (R1 /(R1 + R2)); //
  battery2Voltage = analogRead(PIN_BAT2_VSENSE) * VREF / ADC_MAX / (R1 /(R1 + R2)); //
  //Serial.println(analogRead(PIN_BAT1_VSENSE) );
  //Serial.println(battery1Voltage);
}

// === Checks battery charging need ===
void checkBattery(){
  updateBatteryVoltage();

  if(battery1Voltage < BAT_VLOW || battery2Voltage < BAT_VLOW){ //!!!check LED connection for HIGH/LOW assignment
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
  } else {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, HIGH);
  }
}

// // === Check if button is pressed and released ===
// bool buttonReleased() {
//   reading = digitalRead(PIN_BUTTON);
//   // Serial.println(lastStableState);
//   // Serial.println(lastEdge);

//   if (reading != lastStableState && (millis() - lastEdge) > DEBOUNCE_MS) {
//     lastEdge = millis();
//     if (lastStableState == LOW && reading == HIGH) {
//       lastStableState = reading;
//       Serial.println("Button Pressed. logging starting...");
//       return true; // button pressed
//     }
//     lastStableState = reading;
//   }  
//   return false;
// }
// interuupt servive routine to set the flag button pressed! if the press happened (falling edge) > 200 ms , button is active
void onButtonPress() { 
  if (millis() - lastEdge > DEBOUNCE_MS) {
    buttonInterruptFlag = true;
    lastEdge = millis();
  }
}

void waitForButtonPressed() {
  while (!buttonInterruptFlag && (RemoteXY.PushButton != 1)) {
    RemoteXY_Handler ();
    updateGuiInfo();
    RemoteXY_delay(5);
  }
  buttonInterruptFlag = false; 
}

String generateDataLine() {
  String line = "";
  line += String(millis()) + ",";
  line += String(accX) + ",";
  line += String(accY) + ",";
  line += String(accZ) + ",";
  line += String(gyroX) + ",";
  line += String(gyroY) + ",";
  line += String(gyroZ) + ",";
  line += String(magX) + ",";
  line += String(magY) + ",";
  line += String(magZ) + ",";
  line += String(latitude, 6) + ",";
  line += String(longitude, 6) + ",";
  line += String(gpsAltitude) + ",";
  line += String(Speed) + ",";
  line += String(SatCount) + ",";
//  line += String(rollFinal) + ",";
//  line += String(pitchFinal) + ",";
//  line += String(yawFinal) + ",";
//  line += String(pressure) + ",";
//  line += String(temperature) + ",";
//  line += String(paltitude); + ",";
  line += String(battery1Voltage); // + ",";  
//  line += String(battery2Voltage);
  return line;
}


void logToSD(const String& line) {
  // Collect data
  dataBuffer += line + "\n";

  // Write buffered data every 1 sed
  if (millis() - lastWriteTime >= WRITE_INTERVAL) {
    if (dataFile) {
      dataFile.print(dataBuffer);
      dataBuffer = "";  // Clear buffer after writing
    }
    lastWriteTime = millis();
  }

  // Flush every 5 mins
  if (millis() - lastFlushTime >= FLUSH_INTERVAL) {
    if (dataFile) {
      dataFile.flush();
    }
    lastFlushTime = millis();
  }
}

void startLog() {
  // Check if file exists by trying to open it in read mode
  if (SD.exists(filename)) {
    // File exists — open in append mode without writing header
    dataFile = SD.open(filename, O_APPEND);
    if (dataFile) {
      logState = LogState::ACTIVE;
      //Serial.println("Data logging Procceeded!");
      displayTwoLines("Data logging", "Proceed!", u8g2_font_ncenB10_tr, 15, 40);
    } else {
      //Serial.println("Error opening file for logging.");
      displayTwoLines("Error Proceeding", "logging", u8g2_font_ncenB10_tr, 15, 40);
      logState = LogState::IDLE;
    }
  } else {
    // File does not exist — open in write mode and write header
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,latitude,longitude,gpsAltitude,Speed,SatCount,roll,pitch,yaw,pressure,temperature,paltitude");
      logState = LogState::ACTIVE;
      //Serial.println("Started a new logging session");
      displayTwoLines("Started new", "log session", u8g2_font_ncenB10_tr, 15, 40);

    }else {
      //Serial.println("Error opening file for logging.");
      displayTwoLines("Error opening", "log file", u8g2_font_ncenB10_tr, 15, 40);
      logState = LogState::IDLE;
    }
  }
  RemoteXY_delay(2000); // 2 sec to read the message! 
}

// === Deactivates data logging status ===
void stopLog(){
  if(dataFile){
    dataFile.flush();
    dataFile.close();
    //Serial.println("File closed");
    //Serial.println("Stopped data logging");
    displayTwoLines("File closed", "log Stopped!", u8g2_font_ncenB10_tr, 15, 40);
    RemoteXY_delay(2000);

  }else{
    //Serial.println("File wasn't open");
    displayTwoLines("Error", "File wasn't open!", u8g2_font_ncenB10_tr, 15, 40);
    RemoteXY_delay(2000);

  }
  logState = LogState::IDLE;
}

// === Updates data logging ===
void updateLogging(){
  if (buttonInterruptFlag) {
    buttonInterruptFlag = false; // Reset the flag

    if (logState == LogState::IDLE) {
      startLog();
    } else if (logState == LogState::ACTIVE) {
      stopLog();
    }
  }
}

void generateFilename() {
  timestamp = millis();

 if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print("Using GPS for session log ");
    displayTwoLines("Using GPS", "for session log", u8g2_font_ncenB10_tr, 10, 10);
    RemoteXY_delay(2000);
    
    // Format: T<team><day><month><hour>.CSV → 8 chars before .CSV T0119083.csv team 01 day 19 month 08, hour (3 or 13)
    snprintf(filename, sizeof(filename), "T%02d%02d%02d%01d.CSV",
             TEAM_NUMBER,
             gps.date.day(),
             gps.date.month(),
             gps.time.minute() % 10);  // Only 1 digit for hour

    Serial.println(filename);
    
    displayTwoLines("session_ID:", filename, u8g2_font_ncenB10_tr, 15, 15);
    RemoteXY_delay(2000);

  } else {
    Serial.println("No GPS time available!");
    displayTwoLines("No GPS time", "available!", u8g2_font_ncenB10_tr, 10, 10);
    RemoteXY_delay(2000);

    displayTwoLines("Using timeStamp", "for session log", u8g2_font_ncenB10_tr, 10, 10);
    RemoteXY_delay(2000);
    
    unsigned long ts = timestamp % 10000;
    snprintf(filename, sizeof(filename), "T%02d_%04lu.CSV", TEAM_NUMBER, ts);

    displayTwoLines("session_ID:", filename, u8g2_font_ncenB10_tr, 15, 15);
    RemoteXY_delay(2000);
    Serial.println(filename);
  }
}


void waitForGPSLock() {
  unsigned long startTime = millis();
  bool locked = false;

  while (millis() - startTime < GPS_LOCK_TIMEOUT_MS) {
    updateGPSData();

    Serial.print("Satellites: ");
    Serial.println(SatCount);

    // Display satellite count
    char satMsg[30];
    snprintf(satMsg, sizeof(satMsg), "Sats: %d/%d", SatCount, MIN_SATS_REQUIRED);
    displayTwoLines("Waiting for GPS", satMsg, u8g2_font_ncenB10_tr, 15, 30);

    if (SatCount >= MIN_SATS_REQUIRED && gps.date.isValid() && gps.time.isValid()) {
      locked = true;
      updateGPSData();
      displayTwoLines("Waiting for GPS", satMsg, u8g2_font_ncenB10_tr, 15, 30);
      break;  // Good GPS fix, break early
    }

    RemoteXY_delay(1000);  // Check every 1 second
  }
  RemoteXY_delay(1000);  // Check every 1 second

  if (!locked) {
    // Timeout reached without lock
    Serial.print("Could not lock minimum satellites. Current sats: ");
    Serial.println(SatCount);

    char errMsg1[] = "GPS Lock Failed!";
    char errMsg2[30];
    snprintf(errMsg2, sizeof(errMsg2), "Sats: %d/%d", SatCount, MIN_SATS_REQUIRED);
    displayTwoLines(errMsg1, errMsg2, u8g2_font_ncenB10_tr, 10, 30);

    RemoteXY_delay(3000); // Show error for 3 seconds before continuing
  } else {
    Serial.println("GPS lock acquired!");
    displayTwoLines("GPS Lock", "acquired!", u8g2_font_ncenB10_tr, 30, 40);
    RemoteXY_delay(2000);
  }
}



// === IMU Calibration (Accelerometer and Gyroscope) ===
void calibrateIMU() {
  const int CALIB_SAMPLES = 100;
  float temp;
  float accXCalibBuffer[CALIB_SAMPLES];
  float accYCalibBuffer[CALIB_SAMPLES];  
  float accZCalibBuffer[CALIB_SAMPLES];
  float gyroXCalibBuffer[CALIB_SAMPLES];
  float gyroYCalibBuffer[CALIB_SAMPLES];
  float gyroZCalibBuffer[CALIB_SAMPLES];

  for (int i = 0;  i < CALIB_SAMPLES; i++) {
    // Waits until all data is ready for collection
    while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
      yield();
    }

    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Store the values 
    accXCalibBuffer[i] = accX;  accYCalibBuffer[i] = accY;  accZCalibBuffer[i] = accZ;  
    gyroXCalibBuffer[i] = gyroX; gyroYCalibBuffer[i] = gyroY;  gyroZCalibBuffer[i] = gyroZ; 
    RemoteXY_delay(11); // Accelerometer and gyrospcope output data rate is fixed at 99.84 Hz (10ms)
  }
    
  // Bubble Sorting to find the Median (for each sensor)
  for (int i = 0; i < CALIB_SAMPLES - 1; i++) {
    for (int j = 0; j < CALIB_SAMPLES - i - 1; j++) {
      // accX
      if (accXCalibBuffer[j] > accXCalibBuffer[j + 1]) {
        temp = accXCalibBuffer[j];
        accXCalibBuffer[j] = accXCalibBuffer[j + 1];
        accXCalibBuffer[j + 1] = temp;
      }
      // accY
      if (accYCalibBuffer[j] > accYCalibBuffer[j + 1]) {
        temp = accYCalibBuffer[j];
        accYCalibBuffer[j] = accYCalibBuffer[j + 1];
        accYCalibBuffer[j + 1] = temp;
      }
      // accZ
      if (accZCalibBuffer[j] > accZCalibBuffer[j + 1]) {
        temp = accZCalibBuffer[j];
        accZCalibBuffer[j] = accZCalibBuffer[j + 1];
        accZCalibBuffer[j + 1] = temp;
      }
      // gyroX
      if (gyroXCalibBuffer[j] > gyroXCalibBuffer[j + 1]) {
        temp = gyroXCalibBuffer[j];
        gyroXCalibBuffer[j] = gyroXCalibBuffer[j + 1];
        gyroXCalibBuffer[j + 1] = temp;
      }
      // gyroY
      if (gyroYCalibBuffer[j] > gyroYCalibBuffer[j + 1]) {
        temp = gyroYCalibBuffer[j];
        gyroYCalibBuffer[j] = gyroYCalibBuffer[j + 1];
        gyroYCalibBuffer[j + 1] = temp;
      }
      // gyroZ
      if (gyroZCalibBuffer[j] > gyroZCalibBuffer[j + 1]) {
        temp = gyroZCalibBuffer[j];
        gyroZCalibBuffer[j] = gyroZCalibBuffer[j + 1];
        gyroZCalibBuffer[j + 1] = temp;
      }
    }
  }

  // Calculate the offsets
  int middle = CALIB_SAMPLES / 2;
  accX_off = accXCalibBuffer[middle];  accY_off = accYCalibBuffer[middle];  accZ_off = accZCalibBuffer[middle] - 1;
  gyroX_off = gyroXCalibBuffer[middle];  gyroY_off = gyroYCalibBuffer[middle];  gyroZ_off = gyroZCalibBuffer[middle];

  if (CALIB_DEBUG) {
    printIMUOffsetsAndReadings();
  }
}

// === IMU Calibration (Magnetometer) ===
void calibrateIMU2() {
  // Positioning time: max 20'' per orientation. Total 6 x 20'' = 2'
  // Wait in each position to sample: 5''. Total 6 x 5'' = 30''
  // Magnetometer output data rate is fixed at 20 Hz.
  const int MAGNET_CALIB_SAMPLES = 3000;

  // Three int16_t buffers take 18 kB (7% of RAM) temporarily. (Ideally they'd be float buffers but the Thread's assigned stack gets exceeded)
  int16_t magnXCalibBuffer[MAGNET_CALIB_SAMPLES];
  int16_t magnYCalibBuffer[MAGNET_CALIB_SAMPLES];
  int16_t magnZCalibBuffer[MAGNET_CALIB_SAMPLES]; // Only if Six-Axis Calibration method is used.

  Serial.println("END Calibration? Press button");
  displayTwoLines("END Calibration?", "Press button", u8g2_font_ncenB10_tr, 10, 20);

  int j = 0;
  buttonInterruptFlag = false;
  while((!buttonInterruptFlag && (RemoteXY.PushButton != 1)) && j < MAGNET_CALIB_SAMPLES) {
    while (!IMU.magneticFieldAvailable()) {
      //yield();
      RemoteXY_Handler ();
      RemoteXY_delay(5);
      RemoteXY.PushButton = 0;
    }      
    
    RemoteXY_Handler ();

    IMU.readMagneticField(magX, magY, magZ);

    magnXCalibBuffer[j] = magX;  magnYCalibBuffer[j] = magY; magnZCalibBuffer[j] = magZ;
    j++;
  }

  // Find maximums and minimums. The other values may be necessary
  int16_t xMin = magnXCalibBuffer[0]; int16_t yMin = magnYCalibBuffer[0]; int16_t zMin = magnZCalibBuffer[0];
  int16_t xMax = xMin;                int16_t yMax = yMin;                int16_t zMax = zMin;

  for (int i = 0; i < j-1; ++i) {
      int16_t x = magnXCalibBuffer[i]; int16_t y = magnYCalibBuffer[i]; int16_t z = magnZCalibBuffer[i];

      if (x < xMin) {xMin = x;} else if (x > xMax) {xMax = x;}
      if (y < yMin) {yMin = y;} else if (y > yMax) {yMax = y;}
      if (z < zMin) {zMin = z;} else if (z > zMax) {zMax = z;}
  }

  // Hard-iron offsets
  float bx = 0.5f * (xMax + xMin);
  float by = 0.5f * (yMax + yMin);
  float bz = 0.5f * (zMax + zMin);

  magX_off = bx; magY_off = by; magZ_off = bz;

  // (DEBUGGING) Min & Max axis display
  if (CALIB_DEBUG) {
      Serial.print(F("xMax: ")); Serial.print(xMax); Serial.print(F(" // xMin: ")); Serial.println(xMin);
      Serial.print(F("yMax: ")); Serial.print(yMax); Serial.print(F(" // yMin: ")); Serial.println(yMin);
      Serial.print(F("zMax: ")); Serial.print(zMax); Serial.print(F(" // zMin: ")); Serial.println(zMin);
  }

  // Calibration reliability check-up
  if (CALIB_SAFETY_CHECK == 1) {
    // If calibration lasted less than ~5'' or there's a lack of positive || negative values recorded, then the calibration isn't reliable
    if (j < 150 || fullSpanCalibration(xMin, xMax, yMin, yMax, zMin, zMax)) {
      retryCalibration = 1;
    } else { retryCalibration = 0; }
  }
} // buffers go out of scope here, RAM reclaimed automatically

// Helper function. Checks that the recorded range sits on (ACCURACY)% of the expected span range. Otherwise, the calibration isn't reliable.
bool fullSpanCalibration(int16_t xMin, int16_t xMax, int16_t yMin, int16_t yMax, int16_t zMin, int16_t zMax) {
  if (((xMax - xMin) < CALIB_MIN_ACCURACY) || ((yMax - yMin) < CALIB_MIN_ACCURACY) || ((zMax - zMin) < CALIB_MIN_ACCURACY)) {
    return 1;
  } else {
    return 0;
  }
}

// (DEBUGGING) Helper function
void printIMUOffsetsAndReadings() {
  Serial.println("=== IMU Offsets ===");
  Serial.print("accX_off: "); Serial.print(accX_off);
  Serial.print(", accY_off: "); Serial.print(accY_off);
  Serial.print(", accZ_off: "); Serial.println(accZ_off);

  Serial.print("gyroX_off: "); Serial.print(gyroX_off);
  Serial.print(", gyroY_off: "); Serial.print(gyroY_off);
  Serial.print(", gyroZ_off: "); Serial.println(gyroZ_off);

  Serial.println("=== IMU Readings ===");
  Serial.print("accX: "); Serial.print(accX);
  Serial.print(", accY: "); Serial.print(accY);
  Serial.print(", accZ: "); Serial.println(accZ);

  Serial.print("gyroX: "); Serial.print(gyroX);
  Serial.print(", gyroY: "); Serial.print(gyroY);
  Serial.print(", gyroZ: "); Serial.println(gyroZ);
}

void onESPReady() {
  espReady = true;
}



// streamlit deals with json in realtime dashboards. so we must use it. 
//to decrease the payload we will not send the header. 
//only send the mode and the data as arraylist with the aggreed arangment.
//this function  takes whatever arraylsit or line/ string and encapulste it in json format this funcion has an overwrite with differnt sig
template <size_t N>
String buildJsonPayload(const CircularBuffer<String, N>& buffer) {
  String payload = "{\"mode\":\"" + modeToString() + "\",\"data\":[";
  for (int i = 0; i < buffer.size(); i++) {
    payload += buffer[i];
    if (i < buffer.size() - 1) payload += ",";
  }
  payload += "]}";
  return payload;
}
// covert the curent mode to string to be used in the payload
String modeToString() {
  switch (currentMode) {
    case REALTIME: return "REALTIME";
    case BATCH: return "BATCH";
    default: return "UNKNOWN";
  }
}

// For real-time mode (pass a pre-bracketed JSON string like "[" + line + "]")
// Will automatically create new node with filename if it doesn't already exist
String buildJsonPayload(const String& jsonWrappedLine) {
  return "{\"mode\":\"" + modeToString() + "\",\"data\":" + jsonWrappedLine + "}";
}


void sendToESP(const String& msg) {
  WireESP.beginTransmission(ESP32_ADDR);
  WireESP.write(msg.c_str(), msg.length());
  byte result = WireESP.endTransmission();  // byte data type of the error

  if (result != 0) {
    if(now - LastI2CError >= 5000) // try to send again every 5 sec
    {
      LastI2CError = now;
      I2CError = true;
      
      // Reset I2C
      WireESP.end();
      RemoteXY_delay(10);
      WireESP.begin();
      espReady = true; // make it true to re try sending data over I2c to avoid total dead lock witout stoping the sd card being logging
    
      // Display error info on OLED  /////******===========================Must be commented in real time==================================*****************================///////
      //switch (result) { 
      //  case 1:
      //    displayTwoLines("I2C Error:", "Buffer overflow", u8g2_font_ncenB10_tr, 40, 25);
      //    break;
      //  case 2:
      //    displayTwoLines("I2C Error:", "No device found", u8g2_font_ncenB10_tr, 40, 25);
      //    break;
      //  case 3:
      //    displayTwoLines("I2C Error:", "Data NACK", u8g2_font_ncenB10_tr, 40, 25);
      //    break;
      //  case 4:
      //    displayTwoLines("I2C Error:", "Bus error", u8g2_font_ncenB10_tr, 40, 25);
      //    break;
      //  default:
      //    displayTwoLines("I2C Error:", "Unknown code", u8g2_font_ncenB10_tr, 40, 25);
      //    break;
    //  }
    //  RemoteXY_delay(1000);  // Give the user time to see the message /////******===========================Must be commented in real time==================================*****************================///////
    }
  }
  else{
    I2CError = false;
    espReady = false; // reset the flag pls
  }
}

void logToServer(const String& line){
  //Serial.println(espReady);
  //Serial.println(digitalRead(READY_LINE));    
    if (currentMode == REALTIME && espReady) {
    sendToESP(buildJsonPayload("[" +line + "]"));
  }

  else if (currentMode == BATCH) {
    sensorBuffer.push("[" + line + "]");
    if (espReady) {
      sendToESP(buildJsonPayload(sensorBuffer));
      sensorBuffer.clear();
    }
  }

  // if (digitalRead(ACK_LINE) == HIGH) {
  //   Serial.println("✅ ACK received from ESP");
  // }
}

void log() {
  if (logState != LogState::ACTIVE) return;
  String line = generateDataLine(); 
  logToSD(line);
  logToServer(line);
}

void updateGuiInfo(){
FLIGHT_NUMBER = RemoteXY.FlightNumber;  // example
TEAM_NUMBER   = RemoteXY.TeamNumber;
Orientation  = RemoteXY.Orientation;
RemoteXY.DataLoggerId = IDataLoggerId;
}

void attitudeInitialization() {
  if (firstIteration) {
    timer = millis();
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    
    // tilt-compensated mag
    float mx_h = magZ * cosf(pitch) + magZ * sinf(pitch);
    float my_h = magX * sinf(roll) * sinf(pitch) + magY * cosf(roll) - magZ * sinf(roll) * cosf(pitch);

    yaw = atan2f(-my_h, mx_h) * RAD_TO_DEG;

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    kalmanZ.setAngle(yaw);
  
    gyroXangle = roll;
    gyroYangle = pitch;
    gyroZangle = yaw;

    firstIteration = false;
  }
}

void attitudeAngles() {
  dt = (double)(millis() - timer) / 1000;
  roll  = -atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  gyroXrate = -gyroX;
  gyroYrate = -gyroY;
  gyroZrate = gyroZ;

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && rollFinal > 90) || (roll > 90 && rollFinal < -90)) {
    kalmanX.setAngle(roll);
    rollFinal = roll;
    gyroXangle = roll;
  } else {rollFinal = kalmanX.getAngle(roll, gyroXrate, dt);} // Calculate the angle using a Kalman filter

  if (abs(rollFinal) > 90) {gyroYrate = -gyroYrate;} // Invert rate, so it fits the restriced accelerometer reading

  pitchFinal = kalmanY.getAngle(pitch, gyroYrate, dt);

  // yaw: tilt-compensation
  double tempRoll = rollFinal  * DEG_TO_RAD;
  double tempPitch = pitchFinal * DEG_TO_RAD;
  float mx_h = magX * cos(tempPitch) + magZ * sin(tempPitch);
  float my_h = magX * sin(tempRoll) * sin(tempPitch) + magY * cos(tempRoll) - magZ * sin(tempRoll) * cos(tempPitch);
  double yaw = atan2(-my_h, mx_h) * RAD_TO_DEG;
  //

  yawFinal = kalmanZ.getAngle(yaw, gyroZrate, dt);

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180) {gyroXangle = rollFinal;}
  if (gyroYangle < -180 || gyroYangle > 180) {gyroYangle = pitchFinal;}
  if (gyroZangle < -180 || gyroZangle > 180) {gyroZangle = yawFinal;}
}
