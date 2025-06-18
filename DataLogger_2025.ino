#include <Arduino_BMI270_BMM150.h>  // Rev2 IMU
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino_LPS22HB.h> // Barometric sensor (LPS22HB)
#include <TinyGPSPlus.h>
#include <Arduino.h>

// team number 
#define TEAM_NUMBER 1
#define MIN_SATS_REQUIRED 4
#define GPS_LOCK_TIMEOUT_MS 30000  // 30 seconds

// Create GPS parser
TinyGPSPlus gps;


// Voltage divider values
const float R1 = 1786.0; // 1800Ohms 
const float R2 = 505.0;  // 510 Ohms

//=== Pin Assignment ===
// constexpr tells the comoiler that the value of a variabl can (and should) be evaluated at compile time
constexpr uint8_t PIN_BUTTON = 3; 
constexpr uint8_t PIN_LED_RED = 8; 
constexpr uint8_t PIN_LED_GREEN = 9; 
constexpr uint8_t PIN_BAT_VSENSE = A1;

// === Battery Indicator Values ===
constexpr uint16_t ADC_MAX = 4095; // at 12-bit resolution
constexpr float VREF = 3.30F; // Volts 
constexpr float BAT_VLOW = 3.50F; // Volts at low battery
float batteryVoltage = 0;

// === Button and Logging ===
constexpr uint8_t DEBOUNCE_MS = 30;
constexpr uint8_t LOG_INTERVAL = 100; // millisecond interval to log data

// === OLED Setup (SH1106 I2C) ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// === SD Card Config ===
const int chipSelect = 10;
File dataFile;
String filename = "";

// === Global Sensor Variables ===
unsigned long timestamp;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float latitude, longitude, gpsAltitude, Speed = 0.0;  //&speed in km/h
int SatCount = 0; 
float pressure, paltitude, temperature; 
float roll, pitch, yaw = 0.0;

// === Calibration Variables ===
float accX_off  = 0, accY_off  = 0, accZ_off  = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;

// === Display Variables ===
unsigned long lastDisplaySwitch = 0; // Keep track of the last time the display was updated
int displayState = 0;                // Keep track of what to show: 0 = team, 1 = battery, 2 = satellites

// == button variables 
static uint32_t lastEdge = 0;
static bool lastStableState = HIGH;
bool reading = 0;

//logging variables: 
// Logging state using enum
enum class LogState { IDLE, ACTIVE };
LogState logState = LogState::IDLE;

unsigned long lastWriteTime = 0;
unsigned long lastFlushTime = 0;
const unsigned long WRITE_INTERVAL = 1000;  // Write to SD every 1 sec
const unsigned long FLUSH_INTERVAL = 10000;  // Flush SD every 5 sec
String dataBuffer = ""; // Buffer for batching data





// === Function Declarations ===
void displayToScreen(const char str[], u8g2_uint_t x, u8g2_uint_t y);
void displayTwoLines(const char line1[], const char line2[], const uint8_t* font, u8g2_uint_t x1, u8g2_uint_t x2);
void updateIMUData();
void calibrateIMU(); 
void updateBarometerData();
String generateDataLine();
void logToSD();

void readNextLineFromSD();

// button logic and voltage divider!
bool buttonReleased();
void updateBatteryVoltage();
void checkBattery();
void startLog();
void stopLog();
void updateLogging(); // starts logging at button *release*

void setup() {
  Serial.begin(9600);    // USB serial for debug
  
  // Battery indication 
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BAT_VSENSE, INPUT);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, HIGH);
  analogReadResolution(12); 
  checkBattery();

  // Button 
  pinMode(PIN_BUTTON, INPUT_PULLUP);

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
  calibrateIMU();
  Serial.println("IMU calibration complete.");  
  displayTwoLines("IMU calibration", "complete.", u8g2_font_ncenB10_tr, 10, 20);
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

  // SD Init
  displayTwoLines("BARO sensor", "initialized!", u8g2_font_ncenB10_tr, 15, 25);
  delay(2000);

  // GPS module on Serial1 (D0 = RX, D1 = TX)
  //Serial.println("🔍 Starting GPS reader ..."); 
  displayTwoLines("Initializing", "GPS reader...", u8g2_font_ncenB10_tr, 25, 15);
  Serial1.begin(115200); 
  while (!Serial1);
  delay(2000);
  displayTwoLines("GPS started", "successfully!", u8g2_font_ncenB10_tr, 21, 20);
  delay(2000);
  // waiting to get the munimum gps sat count, max wait 30 sec
  waitForGPSLock();


  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to initialize SD card.");
    displayTwoLines("Failed to ini-", "tialize SD card.", u8g2_font_6x10_tr, 25, 20);
    while (1);
  }
  Serial.println("SD card initialized successfully!");
  displayTwoLines("SD card initialized", "successfully!", u8g2_font_6x10_tr, 8, 25);
  delay(2000);


  updateGPSData();
  filename = generateFilename();

  displayTwoLines("Ready to", "start!", u8g2_font_ncenB10_tr, 35, 45);
  delay(2000);
  displayToScreen("Team 1", 30, 35);

  // // Remove old log file
  // if (SD.exists(filename)) {
  //   SD.remove(filename);
  //   Serial.println("Removed old log file.");
  //   delay(2000);
  // }


  // Wait for button to start logging
  bool toggle = false;
  while (!buttonReleased()) {
    updateBatteryVoltage();

    if (batteryVoltage <= BAT_VLOW) {
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
            snprintf(buf, sizeof(buf), "%.2f V", batteryVoltage);
            displayTwoLines("Battery level:", buf, u8g2_font_ncenB10_tr, 15, 50);
            break;
          }

          case 2: // promte the user to start logging
            displayTwoLines("Press the button", "to start logging", u8g2_font_ncenB10_tr, 10, 30);
            break;
        }

        toggle = !toggle;
        digitalWrite(PIN_LED_GREEN, toggle);  // Blink green LED
        digitalWrite(PIN_LED_RED, LOW);       // Red LED off when voltage is fine
      }
    }
  }
  startLog();  // Start logging after button press
}

void loop() {
  updateIMUData();
  updateBarometerData();
  updateGPSData();
  updateLogging();
  logToSD();
  
  // displayToScreen();
  // readNextLineFromSD();
  //delay(1000); // 1Hz logging rate will be removed later!
  checkBattery();
  
  // Looping through team number, battery voltage and GPS satellite
  if (millis() - lastDisplaySwitch > 2000) {    // Check if 2000 ms (2 seconds) have passed since last display change
    lastDisplaySwitch = millis();               // Reset the timer

    displayState = (displayState + 1) % 3;      // Move to next state (0 -> 1 -> 2 -> 0)

    switch (displayState) {
      case 0: {     // Display team
        displayToScreen("Team 1", 25, 35);      // Show "Team 1" centered on screen
        break;
      }

      case 1: {     // Display battery level
        char buf[20];                                                   // Create a text buffer
        snprintf(buf, sizeof(buf), "%.2f V", batteryVoltage);              // Format voltage as "3.70V"
        displayTwoLines("Battery level:", buf, u8g2_font_ncenB10_tr, 15, 50);
        break;
      }

      case 2: {     // Display GPS satellites count
        displayTwoLines("Satellites:", String(SatCount).c_str(), u8g2_font_ncenB10_tr, 28, 60);
        break;
      }
    }
  }

  // Optional: add a short delay to reduce CPU load if needed
 // delay(1000); // (uncomment if display flickers or CPU usage is high)
}

// === IMU Data Update ===
void updateIMUData() {
  float preAccX, preAccY, preAccZ;
  float preGyroX, preGyroY, preGyroZ;
  
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
    IMU.readMagneticField(magX, magY, magZ);
  }
}

// === IMU Calibration ===
void calibrateIMU() {
  const int CALIB_SAMPLES = 25;
  float temp;
  float accXCalibBuffer[CALIB_SAMPLE_COUNT];
  float accYCalibBuffer[CALIB_SAMPLE_COUNT];  
  float accZCalibBuffer[CALIB_SAMPLE_COUNT];
  float gyroXCalibBuffer[CALIB_SAMPLE_COUNT];
  float gyroYCalibBuffer[CALIB_SAMPLE_COUNT];
  float gyroZCalibBuffer[CALIB_SAMPLE_COUNT];

  for (int i = 0;  i < CALIB_SAMPLES; i++) {
    // Waits until all data is ready for collection
    while(!IMU.accelerationAvailable() || !IMU.gyroscocpeAvailable()));
    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Store the values 
    accXCalibBuffer[i] = accX;  accYCalibBuffer[i] = accY;  accZCalibBuffer[i] = accZ;  
    gyroXCalibBuffer[i] = gyroX; gyroYCalibBuffer[i] = gyroY;  gyroZCalibBuffer[i] = gyroZ; 

    // Accelerometer and gyrospcope output data rate is fixed at 99.84 Hz (10ms)
    delay(11);
  }
    
  // Bubble Sorting to find the Median (for each sensor)
  for (int i = 0; i < CALIB_SAMPLE_COUNT - 1; i++) {
    for (int j = 0; j < CALIB_SAMPLE_COUNT - i - 1; j++) {
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

   // Calculate the offsets
   int middle = CALIB_SAMPLES / 2;
   accX_off = accXCalibBuffer[middle];  accY_off = accYCalibBuffer[middle];  accZ_off = accZCalibBuffer[middle] - 1;
   gyroX_off = gyroXCalibBuffer[middle];  gyroY_off = gyroYCalibBuffer[middle];  gyroZ_off = gyroZCalibBuffer[middle];
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
      Serial.write(c);

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
  batteryVoltage = analogRead(PIN_BAT_VSENSE) * VREF / ADC_MAX / (R1 /(R1 + R2)); //
  //Serial.println(analogRead(PIN_BAT_VSENSE) );
  //Serial.println(batteryVoltage);

}

// === Checks battery charging need ===
void checkBattery(){
  updateBatteryVoltage();

  if(batteryVoltage <= BAT_VLOW){ //!!!check LED connection for HIGH/LOW assignment
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
  } else {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, HIGH);
  }
}

// === Check if button is pressed and released ===
bool buttonReleased() {
  reading = digitalRead(PIN_BUTTON);
  // Serial.println(lastStableState);
  // Serial.println(lastEdge);

  if (reading != lastStableState && (millis() - lastEdge) > DEBOUNCE_MS) {
    lastEdge = millis();
    if (lastStableState == LOW && reading == HIGH) {
      lastStableState = reading;
      Serial.println("Button Pressed. logging starting...");
      return true; // button pressed
    }
    lastStableState = reading;
  }  
  return false;
}

String generateDataLine() {
  String line = "";
  line += String(timestamp) + ",";
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
  line += String(roll) + ",";
  line += String(pitch) + ",";
  line += String(yaw) + ",";
  line += String(pressure) + ",";
  line += String(temperature) + ",";
  line += String(paltitude);
  line += String(paltitude);
  return line;
}


void logToSD() {
  if (logState != LogState::ACTIVE) return;

  // Collect data
  String line = generateDataLine(); 
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
      Serial.println("Data logging Procceeded!");
      displayTwoLines("Data logging", "Proceed!", u8g2_font_ncenB10_tr, 15, 40);
    } else {
    Serial.println("Error opening file for logging.");
    displayTwoLines("Error Proceeding", "logging", u8g2_font_ncenB10_tr, 15, 40);
    logState = LogState::IDLE;
    }
  } else {
    // File does not exist — open in write mode and write header
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,latitude,longitude,gpsAltitude,Speed,SatCount,roll,pitch,yaw,pressure,temperature,paltitude");
      logState = LogState::ACTIVE;
      Serial.println("Started a new logging session");
      displayTwoLines("Started new", "log session", u8g2_font_ncenB10_tr, 15, 40);

    }else {
      Serial.println("Error opening file for logging.");
      displayTwoLines("Error opening", "log file", u8g2_font_ncenB10_tr, 15, 40);
      logState = LogState::IDLE;
    }
  }
  delay(2000); // 2 sec to read the message! 
}

// === Deactivates data logging status ===
void stopLog(){
  if(dataFile){
    dataFile.flush();
    dataFile.close();
    Serial.println("File closed");
  }else{
    Serial.println("File wasn't open");
  }
  logState = LogState::IDLE;
  Serial.println("Stopped data logging");
}

// === Updates data logging ===
void updateLogging(){
  if(buttonReleased() && logState == LogState::IDLE){
    startLog();
  }else if(buttonReleased() && logState == LogState::ACTIVE){
    stopLog();
  }
}  

String generateFilename() {
  timestamp = millis();

  if (gps.date.isValid() && gps.time.isValid()) {
    // Compose date/time string: YYYYMMDD_HHMMSS
    char datetime[20];
    snprintf(datetime, sizeof(datetime), "%04d%02d%02d_%02d%02d%02d",
             gps.date.year(),
             gps.date.month(),
             gps.date.day(),
             gps.time.hour(),
             gps.time.minute(),
             gps.time.second());

    filename = "Team_" + String(TEAM_NUMBER) + "_" + String(datetime) + "_log.csv";

    Serial.print("✅ Using GPS datetime for log: ");
    Serial.println(filename);

    // Show "Using GPS time" and datetime in two lines for 2 seconds
    displayToScreen("Using GPS time!", 25, 35);
    delay(2000);
    
    displayTwoLines("session_ID:", datetime, u8g2_font_ncenB10_tr, 15, 15);
    delay(2000);

  } else {
    Serial.println("⚠️ No GPS time available!");
    filename = "Team_" + String(TEAM_NUMBER) + "_" + String(timestamp) + "_log.csv";

    // First message: "No GPS time!" alone for 2 seconds
    displayToScreen("No GPS time!", 25, 35);
    delay(2000);

    // Second message: "Using session id:" then session ID for 2 seconds
    char idBuf[30];
    snprintf(idBuf, sizeof(idBuf), "Session ID: %lu", timestamp);
    displayTwoLines("Using session id:", idBuf, u8g2_font_ncenB10_tr, 10, 10);
    delay(2000);
  }

  return filename;
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
      break;  // Good GPS fix, break early
    }

    delay(1000);  // Check every 1 second
  }

  if (!locked) {
    // Timeout reached without lock
    Serial.print("Could not lock minimum satellites. Current sats: ");
    Serial.println(SatCount);

    char errMsg1[] = "GPS Lock Failed!";
    char errMsg2[30];
    snprintf(errMsg2, sizeof(errMsg2), "Sats: %d/%d", SatCount, MIN_SATS_REQUIRED);
    displayTwoLines(errMsg1, errMsg2, u8g2_font_ncenB10_tr, 10, 30);

    delay(3000); // Show error for 3 seconds before continuing
  } else {
    Serial.println("GPS lock acquired!");
    displayTwoLines("GPS Lock", "acquired!", u8g2_font_ncenB10_tr, 30, 40);
    delay(2000);
  }
}
