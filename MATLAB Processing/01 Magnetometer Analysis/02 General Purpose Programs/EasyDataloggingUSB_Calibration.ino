#include <Arduino_BMI270_BMM150.h>  // Rev2 IMU
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Arduino.h>

#define loggingPeriod 100     // logging Period in milli seconds. Logging Freq= 1/ loggingPeriod KHz
#define CALIB_SAFETY_CHECK 0  // boolean

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
float accX_off  = 0, accY_off  = 0, accZ_off  = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;
float magX_off  = 0, magY_off  = 0, magZ_off  = 0;
float preAccX, preAccY, preAccZ = 0;
float preGyroX, preGyroY, preGyroZ = 0;
float preMagX, preMagY, preMagZ;
bool retryCalibration = 0;

//  ====== logging parameters ======
unsigned long now = 0;
unsigned long lastCollectTime = 0; 

// === Function Declarations ===
void updateIMUData();
void generateDataLine();
void calibrateIMU(); 
void calibrateIMU2();
void waitForButtonPressed();
void onButtonPress();
bool fullSpanCalibration(float, float, float, float, float, float);

void setup() {
  Serial.begin(9600);    // USB serial for debug
  analogReadResolution(12); 

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
}

void loop() {
  now = millis();

  if (now - lastCollectTime >= loggingPeriod) {
    lastCollectTime = now;

    updateIMUData();
    generateDataLine();
  }
  // Optional: add a short delay to reduce CPU load if needed
  // delay(1000); // (uncomment if display flickers or CPU usage is high)
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

void waitForButtonPressed() {
  while(!buttonInterruptFlag) {
    yield();    // Nano BLE 33, RTOS native instructions. Doesn't starve the processor with delay(500) and ensures proper waiting of the button pressed (even though our current button doesnt have that problem).
  }

  buttonInterruptFlag = false; 
}

void generateDataLine() {
  String line = "";

  if (first_writing == 0) {    
    Serial.println("\nmagX,magY,magZ");
    line = "";
    first_writing++;
  }

  line += String(magX) + ",";
  line += String(magY) + ",";
  line += String(magZ);
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
  while(!buttonInterruptFlag && j < MAGNET_CALIB_SAMPLES) {
    while (!IMU.magneticFieldAvailable()) {
      yield();
    }

    IMU.readMagneticField(magX, magY, magZ);

    magnXCalibBuffer[j] = magX;  magnYCalibBuffer[j] = magY; magnZCalibBuffer[j] = magZ;
    j++;
  }
  Serial.println("DEBUG: Buffer filled OR Button pressed!");

  // Find maximums and minimums. The other values will be necessary to double check correct initialization
  float xMin = magnXCalibBuffer[0]; float yMin = magnYCalibBuffer[0]; float zMin = magnZCalibBuffer[0];
  float xMax = xMin;                float yMax = yMin;                float zMax = zMin;

  for (int i = 0; i < j-1; ++i) {
      float x = magnXCalibBuffer[i]; float y = magnYCalibBuffer[i]; float z = magnZCalibBuffer[i];

      if (x < xMin) {xMin = x;} else if (x > xMax) {xMax = x;}
      if (y < yMin) {yMin = y;} else if (y > yMax) {yMax = y;}
      if (z < zMin) {zMin = z;} else if (z > zMax) {zMax = z;}
  }

  // Hard-iron offsets
  float bx = 0.5f * (xMax + xMin);
  float by = 0.5f * (yMax + yMin);
  float bz = 0.5f * (zMax + zMin);

  magX_off = bx; magY_off = by; magZ_off = bz;

  // Calibration reliability check-up
  if (CALIB_SAFETY_CHECK == 1) {
    // If calibration lasted less than 30'' or there's a lack of positive || negative values recorded, then the calibration isn't reliable
    if (j < 600 || fullSpanCalibration(xMin, xMax, yMin, yMax, zMin, zMax)) {
      retryCalibration = 1;
    }
  }
} // buffers go out of scope here, RAM reclaimed automatically

// This helper function checks that the recorded range sits on 50% of the expected span range. Otherwise, the calibration isn't reliable.
bool fullSpanCalibration(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax) {
  if (((xMax - xMin) < 20) || ((yMax - yMin) < 50) || ((zMax - zMin) < 50)) {
    return 1;
  } else {
    return 0;
  }
}
