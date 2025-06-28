#include <Arduino_BMI270_BMM150.h>  // Rev2 IMU
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino_LPS22HB.h> // Barometric sensor (LPS22HB)
#include <Arduino.h>

// team number 
#define TEAM_NUMBER 1


// Voltage divider values
const float R1 = 1786.0; // 1800Ohms 
const float R2 = 505.0;  // 510 Ohms

//=== Pin Assignment ===
// constexpr tells the comoiler that the value of a variabl can (and should) be evaluated at compile time
constexpr uint8_t PIN_BUTTON = 3; 

// === Global Sensor Variables ===
unsigned long timestamp;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float roll, pitch, yaw = 0.0;

// === Calibration Variables ===
float accX_off  = 0, accY_off  = 0, accZ_off  = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;

// === Display Variables ===
int first_writing = 0;
unsigned long lastDisplaySwitch = 0; // Keep track of the last time the display was updated

// == button variables 
static uint32_t lastEdge = 0;
static bool lastStableState = HIGH;
bool reading = 0;
bool buttonPressed = false;
bool calibFinished = false;
int flagg = 0;


// === Function Declarations ===
void updateIMUData();
void calibrateIMU(); 
void updateBarometerData();
void generateDataLine();

// button logic and voltage divider!
void buttonReleased();

void setup() {
  Serial.begin(9600);    // USB serial for debug
  
  analogReadResolution(12); 

  // Button 
  pinMode(PIN_BUTTON, INPUT_PULLUP);

 // IMU Init (Rev2)
  delay(2000);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }
  calibrateIMU();
  Serial.println("IMU calibration complete.");  
  delay(100);  // Remove once we make sure an ISR cant interrupt it.
  Serial.println("IMU initialized successfully!");
  delay(1000);

  delay(2000);
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor.");
    while (1);
  }

  Serial.println("Ready to start!!");
  delay(2000);

  // Wait for button to start logging
  Serial.println("Waiting for button to start the program...");
}

void loop() {
  while (buttonPressed) {
    updateIMUData();
    generateDataLine();
    buttonReleased();
  }

  buttonReleased();
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
  float accXCalibBuffer[CALIB_SAMPLES];
  float accYCalibBuffer[CALIB_SAMPLES];  
  float accZCalibBuffer[CALIB_SAMPLES];
  float gyroXCalibBuffer[CALIB_SAMPLES];
  float gyroYCalibBuffer[CALIB_SAMPLES];
  float gyroZCalibBuffer[CALIB_SAMPLES];

  for (int i = 0;  i < CALIB_SAMPLES; i++) {
    // Waits until all data is ready for collection
    while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
      delay(1);          // sleep to avoid a busy-loop. Actually I think this is also a busy loop
    }

    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Store the values 
    accXCalibBuffer[i] = accX;  accYCalibBuffer[i] = accY;  accZCalibBuffer[i] = accZ;  
    gyroXCalibBuffer[i] = gyroX; gyroYCalibBuffer[i] = gyroY;  gyroZCalibBuffer[i] = gyroZ; 

    // Accelerometer and gyrospcope output data rate is fixed at 99.84 Hz (10ms)
    delay(11);
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
}

// === Check if button is pressed and released ===
void buttonReleased() {
  // Unpressed = true. That's confusing.
  reading = digitalRead(PIN_BUTTON);

  if (reading == false) {
    buttonPressed = !buttonPressed;

    if (buttonPressed == true) {
      Serial.println("New Logging is starting!\n");
      first_writing = 0;
      delay(500);
    } else if (buttonPressed == false) {
      Serial.println("Stopped recording data.\n");
      delay(500);
    }
  }
}

void generateDataLine() {
  String line = "";

  if (first_writing == 0) {    
    Serial.println("magX,magY,magZ\n");
    line = "";
    first_writing++;
  }

  line += String(magX) + ",";
  line += String(magY) + ",";
  line += String(magZ);
  Serial.println(line);
  Serial.println("\n"); 
}