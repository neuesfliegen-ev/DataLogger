#include <Arduino_BMI270_BMM150.h>  // Rev2 IMU
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
// CHANGE THIS !!!
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define loggingPeriod 100     // logging Period in milli seconds. Logging Freq= 1/ loggingPeriod KHz
#define CALIB_SAFETY_CHECK 1  // boolean
#define CALIB_DEBUG 1
#define WITH_BLUETOOTH 1      // (Mechanical / BLE) button

// === Global Sensor Variables ===
unsigned long timestamp;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

// === Display Variables ===
int first_writing = 0;

// === Button Handling ===
const int PIN_BUTTON = 3;                // Make sure this is an interrupt-capable pin
constexpr uint8_t DEBOUNCE_MS = 200;

volatile bool buttonInterruptFlag = false;
unsigned long lastEdge = 0;
bool lastStableState = HIGH;            // Assuming pull-up resistor

// === Calibration Variables ===
float timeOffset;
float accX_off  = 0, accY_off  = 0, accZ_off  = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float magX_off  = 0, magY_off  = 0, magZ_off  = 0;
float preAccX, preAccY, preAccZ = 0;
float preGyroX, preGyroY, preGyroZ = 0;
float preMagX, preMagY, preMagZ;
bool retryCalibration = 0;

// === Attitude Angle Variables ===
bool firstIteration = true;
Kalman kalmanX, kalmanY, kalmanZ;
float roll = 0, pitch = 0, yaw = 0;
double rollFinal, pitchFinal, yawFinal; // Calculated angle using a Kalman filter
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double gyroXrate, gyroYrate, gyroZrate;
double dt;
unsigned long timer = 0;

//  ====== logging parameters ======
unsigned long now = 0;
unsigned long lastCollectTime = 0;

// ====== LE Bluetooth ======
/* A BLE UUID is a 128-bit value written in hexadecimal, like: 19B10000-E8F2-537E-4F6C-D104768A1214
   This format is standardized by the ITU (International Telecommunication Union).
   The idea is that the probability of two people picking the same UUID by accident is essentially zero. 
*/
BLEService svc("19B10000-E8F2-537E-4F6C-D104768A1214");                     // svc UUID — Universally Unique Identifier
BLEByteCharacteristic triggerChar( "19B10001-E8F2-537E-4F6C-D104768A1214",  // triggerChar UUID, not the same: "0001"
                                    BLEWrite | BLEWriteWithoutResponse
);
BLEDevice central;                          // Will contain information about my central (phone)
uint8_t v = 0;                              // The message I am sending from my central
volatile bool triggerRequested = false;     // volatile variables to be used in Event Handlers. An event handler can be interrupted by an ISR I guess, so keep (atomic) 1 byte size data type like "bool"
volatile bool bleConnected = false;

// === Function Declarations ===
void updateIMUData();
void generateDataLine();
void calibrateIMU(); 
void calibrateIMU2();
void waitForButtonPressed();
void onButtonPress();       // ISR
void onTriggerWritten(BLEDevice, BLECharacteristic);    // Event Handler (RTOS). BLEDevice and BLECharacteristics are the data types
bool fullSpanCalibration(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t);
void initializeBLE();
void newAttitudeInitialization();
void newAttitudeAngles();

void setup() {
  Serial.begin(9600);    // USB serial for debug
  analogReadResolution(12); 

  // BLE
  if (WITH_BLUETOOTH == 1) {
    initializeBLE();
  }

  // Button 
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButtonPress, FALLING); // Assuming active-low button

  delay(2000);
  // IMU Init (Rev2)
  Serial.println("Initializing IMU...");
  delay(2000);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }
  
  Serial.println("IMU initialized successfully!");
  delay(2000);

  //============ IMU CALIBIRATION. =========//
  Serial.println("Start IMU Offset Calibration? Press button");
  waitForButtonPressed();

  Serial.println("Press again to end calibation");
  waitForButtonPressed();

  calibrateIMU();

  //IMU MAG Calibiration
  Serial.println("Start IMU MAG Calibration? Press button");
  waitForButtonPressed();

  do {
    if (retryCalibration == 1) {
      Serial.println("Calibration failed... Trying again");
    }

    calibrateIMU2();
  } while (retryCalibration);

  Serial.println("IMU calibration complete.");  
  delay(2000);
  //============ IMU CALIBIRATION END. =========//

  Serial.println("Ready to start!");
  delay(2000);

  // Wait for button to start logging
  bool toggle = false;
  waitForButtonPressed();
  timeOffset = millis();
}

void loop() {
  now = millis();

  if (now - lastCollectTime >= loggingPeriod) {
    lastCollectTime = now;

    updateIMUData();
    newAttitudeInitialization();
    newAttitudeAngles();
    generateDataLine();
  }
  // Optional: add a short delay to reduce CPU load if needed
  // delay(1000); // (uncomment if display flickers or CPU usage is high)
}

// === BLE Inititialization ===
void initializeBLE() {
  while (!BLE.begin()) { yield(); }     // Make sure the Bluetooth module starts
  BLE.setLocalName("NFC25-Datalogger");
  BLE.setAdvertisedService(svc);        // This call tells ArduinoBLE: include this service’s UUID in the advertising packets. Why? So that a central (phone) can filter scans for devices that have a specific service without connecting first.
  svc.addCharacteristic(triggerChar);   // Actual data/command. One service can have multiple characteristics
  BLE.addService(svc);

  // Set Event Handlers, like ISR but not hardware driven, involved in the RTOS
  triggerChar.setEventHandler(BLEWritten, onTriggerWritten);
  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);

  BLE.advertise();                      // Starts advertising: sending out periodic BLE “I'm here” packets
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

/* Event Handler, RTOS functionality. Works like an ISR but not hardware driven.
   The parameters "central" and "characteristic" seem unused but are crucial */
void onTriggerWritten(BLEDevice central, BLECharacteristic characteristic) {
  v = 0;
  triggerChar.readValue(v);

  if (v == 0x01) {
    triggerRequested = true;
  }
}

// Event Handler, BLE
void onConnect(BLEDevice central) {
  bleConnected = true;      
  central = BLE.central();       
  // Now we can operate methods for this object Central (my phone).   
  // It updates because different centrals (phones) could connect at different stages of the program 
}

// Event Handler, BLE
void onDisconnect(BLEDevice central) {
  bleConnected = false;
  BLE.advertise();                   
}

void waitForButtonPressed() {
  if (WITH_BLUETOOTH == 1) {
    while (!triggerRequested) {
      yield();
      BLE.poll();   
      // let BLE stack run; otherwise it starves. This is because yield() lets the scheduler loop run, but the stack isnt integrated thereand it can't run
    }

    v = 0;
    triggerRequested = false;
    } else if (WITH_BLUETOOTH == 0) {
      while(!buttonInterruptFlag) {
        yield();
      }
      buttonInterruptFlag = false; 
    }
}

void generateDataLine() {
  String line = "";

  if (first_writing == 0) {    
    Serial.println("\ntime,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ");
    line = "";
    first_writing++;
  }
  line += String(millis() - timeOffset) + ",";
  line += String(accX) + ",";
  line += String(accY) + ",";
  line += String(accZ) + ",";
  line += String(gyroX) + ",";
  line += String(gyroY) + ",";
  line += String(gyroZ) + ",";
  line += String(magX) + ",";
  line += String(magY) + ",";
  line += String(magZ);
  line += " // ROLL = " + String(rollFinal);
  line += " // PITCH = " + String(pitchFinal);
  line += " // YAW = " + String(yawFinal);
  Serial.println(line);
}

// === IMU Calibration (Accelerometer and Gyroscope) ===
void calibrateIMU() {
  const int DATA_RATE = 11;
  const int CALIB_SAMPLES = 100;
  float temp;
  unsigned long timerNow = millis();
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
      BLE.poll();
    }

    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Store the values 
    accXCalibBuffer[i] = accX;  accYCalibBuffer[i] = accY;  accZCalibBuffer[i] = accZ;  
    gyroXCalibBuffer[i] = gyroX; gyroYCalibBuffer[i] = gyroY;  gyroZCalibBuffer[i] = gyroZ; 
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

  Serial.println("Press again to end calibration 2");

  int j = 0;
  buttonInterruptFlag = false;
  triggerRequested = false;
  while(!buttonInterruptFlag && !triggerRequested && j < MAGNET_CALIB_SAMPLES) {
    while (!IMU.magneticFieldAvailable()) {
      yield();
      BLE.poll();
    }
    
    IMU.readMagneticField(magX, magY, magZ);

    magnXCalibBuffer[j] = magX;  magnYCalibBuffer[j] = magY; magnZCalibBuffer[j] = magZ;
    j++;
  }

  // Find maximums and minimums. The other values will be necessary to double check correct initialization
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
    // If calibration lasted less than 15'' or there's a lack of positive || negative values recorded, then the calibration isn't reliable
    if (j < 150 || fullSpanCalibration(xMin, xMax, yMin, yMax, zMin, zMax)) {
      retryCalibration = 1;
    } else { retryCalibration = 0; }
  }
} // buffers go out of scope here, RAM reclaimed automatically

// This helper function checks that the recorded range sits on 50% of the expected span range. Otherwise, the calibration isn't reliable.
bool fullSpanCalibration(int16_t xMin, int16_t xMax, int16_t yMin, int16_t yMax, int16_t zMin, int16_t zMax) {
  if (((xMax - xMin) < 45) || ((yMax - yMin) < 50) || ((zMax - zMin) < 50)) {
    return 1;
  } else {
    return 0;
  }
}

void newAttitudeInitialization() {
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

void newAttitudeAngles() {
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