#include <Arduino_BMI270_BMM150.h>  // Rev2 IMU
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino_LPS22HB.h> // Barometric sensor (LPS22HB)
#include <TinyGPSPlus.h>
#include <Arduino.h>

// Create GPS parser
TinyGPSPlus gps;

// === OLED Setup (SH1106 I2C) ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// === SD Card Config ===
const int chipSelect = 10;
File dataFile;
const char filename[] = "log.csv";

// === Global Sensor Variables ===
unsigned long timestamp;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float latitude, longitude, gpsAltitude, Speed = 0.0;  //&speed in km/h
int SatCount = 0;
float pressure, paltitude, temperature;
float roll, pitch, yaw = 0.0;

// === Function Declarations ===
void displayToScreen(const char str[], u8g2_uint_t x, u8g2_uint_t y);
void displayTwoLines(const char line1[], const char line2[], const uint8_t* font);
void updateIMUData();
void updateBarometerData();
void logToSD();
void readNextLineFromSD();

void setup() {
  Serial.begin(115200);    // USB serial for debug
  
  // OLED Init
  u8g2.begin();
  displayTwoLines("OLED", "initialized!", u8g2_font_ncenB10_tr);
  delay(2000);

  // GPS module on Serial1 (D0 = RX, D1 = TX)
  //Serial.println("🔍 Starting GPS reader ..."); 
  displayTwoLines("Starting GPS", "reader ...", u8g2_font_ncenB10_tr);
  delay(2000);
  Serial1.begin(115200); 
  while (!Serial1);
  displayTwoLines("GPS Started", "successfully!", u8g2_font_ncenB10_tr);
  delay(2000);


  // IMU Init (Rev2)
  displayTwoLines("Initializing", "IMU!", u8g2_font_ncenB10_tr);
  delay(2000);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    displayTwoLines("Failed to", "ininitialize IMU!", u8g2_font_ncenB10_tr);
    while (1);
  }
  Serial.println("IMU initialized successfully!");
  displayTwoLines("IMU initialized", "successfully!", u8g2_font_ncenB10_tr);
  delay(2000);


  displayTwoLines("Initializing", "BARO sensor", u8g2_font_ncenB10_tr);
  delay(2000);
  if (!BARO.begin()) {
      Serial.println("Failed to initialize pressure sensor!");
      displayTwoLines("Failed to initialize", "pressure sensor!", u8g2_font_ncenB10_tr);
      while (1);
  }
  Serial.println("BARO sensor initialized.");
  displayTwoLines("BARO sensor", "initialized.",u8g2_font_ncenB10_tr);
  delay(2000);

  // SD Init
  displayTwoLines("BARO sensor", "initialized.", u8g2_font_ncenB10_tr);
  delay(2000);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    displayTwoLines("SD card", "initialization failed!", u8g2_font_6x10_tr);
    while (1);
  }
  Serial.println("SD card initialized successfully!");
  displayTwoLines("SD card initialized", "successfully!", u8g2_font_6x10_tr);
  delay(2000);

  displayToScreen("Ready to start", 0, 35);
  delay(2000);
  displayToScreen("Team 1", 25, 35);

  // Remove old log file
  if (SD.exists(filename)) {
    SD.remove(filename);
    Serial.println("Removed old log file.");
    delay(2000);
  }

  // Create new log file with headers
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
  dataFile.println("timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,latitude,longitude,gpsAltitude,Speed,SatCount,roll,pitch,yaw,pressure,temperature,paltitude"); 
  dataFile.close();
  } else {
    Serial.println("Error creating new log file.");
    displayTwoLines("Error creating", "new log file.", u8g2_font_ncenB10_tr);
    delay(2000);
  }
}

void loop() {
  timestamp = millis();
  updateIMUData();
  updateBarometerData();
  updateGPSData();
  logToSD();
  // displayToScreen();
  //readNextLineFromSD();
  delay(1000); // 1Hz logging rate will be removed later!
}

// === IMU Data Update ===
void updateIMUData() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(magX, magY, magZ);
  }
}

// === Barometer Data Update ===
void updateBarometerData() {
  pressure = BARO.readPressure(); // in kPa
  paltitude = 44330 * (1 - pow(pressure / 101.325, 1 / 5.255));
}

// === Log Data to SD ===
void logToSD() {
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.print(timestamp); dataFile.print(",");
    dataFile.print(accX); dataFile.print(",");
    dataFile.print(accY); dataFile.print(",");
    dataFile.print(accZ); dataFile.print(",");
    dataFile.print(gyroX); dataFile.print(",");
    dataFile.print(gyroY); dataFile.print(",");
    dataFile.print(gyroZ); dataFile.print(",");
    dataFile.print(magX); dataFile.print(",");
    dataFile.print(magY); dataFile.print(",");
    dataFile.print(magZ); dataFile.print(",");
    dataFile.print(latitude, 6); dataFile.print(",");
    dataFile.print(longitude, 6); dataFile.print(",");
    dataFile.print(gpsAltitude); dataFile.print(",");
    dataFile.print(Speed); dataFile.print(",");
    dataFile.print(SatCount); dataFile.print(",");
    dataFile.print(roll); dataFile.print(",");
    dataFile.print(pitch); dataFile.print(",");
    dataFile.print(yaw); dataFile.print(",");
    dataFile.print(pressure); dataFile.print(",");
    dataFile.print(temperature); dataFile.print(",");
    dataFile.println(paltitude);
    dataFile.close();
  } else {
    Serial.println("Error writing to SD card.");
  }
}

// === OLED Display === For short messages
void displayToScreen(const char str[], u8g2_uint_t x, u8g2_uint_t y) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr); // u8g2_font_ncenB14_tr big size -- u8g2_font_ncenB10_tr IS OK // TOO SMALL u8g2_font_5x8_tr
  u8g2.setCursor(x, y); // Centered: (25, 35) -- Starting from the left: (0, 35)
  u8g2.print(str);
  u8g2.sendBuffer();
}

// === OLED Display === In two lines for 
void displayTwoLines(const char line1[], const char line2[], const uint8_t* font) {
  u8g2.clearBuffer();
  u8g2.setFont(font);
  u8g2.setCursor(0, 25);  // Y=25 for first line (adjust as needed)
  u8g2.print(line1);
  u8g2.setCursor(0, 50);  // Y=50 for second line (25+line height)
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
  
    SatCount     = gps.satellites.value();

  // Show satellite count every ~1s
  if (millis() % 1000 < 50) {
    Serial.print("\n Satellites visible: ");
    Serial.println(gps.satellites.value());
    displayTwoLines("Satellites visible: ", String(SatCount).c_str(), u8g2_font_ncenB10_tr);

  }

  if (gps.location.isUpdated()) {
    latitude     = gps.location.lat();
    longitude    = gps.location.lng();
    gpsAltitude  = gps.altitude.meters();
    Speed        = gps.speed.kmph();
    SatCount     = gps.satellites.value();

    Serial.println("\n✅ Parsed GPS Data:");
    Serial.print("Latitude: "); Serial.println(latitude, 6);
    Serial.print("Longitude: "); Serial.println(longitude, 6);
    Serial.print("Satellites: "); Serial.println(SatCount);
    Serial.print("Speed (km/h): "); Serial.println(Speed);
    Serial.print("Altitude (m): "); Serial.println(gpsAltitude);
    Serial.println("----------------------");
    }
  }
}
