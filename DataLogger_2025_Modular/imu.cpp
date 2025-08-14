#include <Arduino.h>
#include "globals.h"
#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include "display.h"
#include "imu.h"

// === Calibration constants ===
#define CALIB_SAFETY_CHECK 1    // boolean
#define CALIB_DEBUG 0
#define CALIB_MIN_ACCURACY 60   // Recommended: 60. Maximum: 90

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

    for (int i = 0; i < CALIB_SAMPLES; i++) {
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
        delay(11); // Accelerometer and gyrospcope output data rate is fixed at 99.84 Hz (10ms)
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
    triggerRequested = false;
    while (!buttonInterruptFlag && !triggerRequested && j < MAGNET_CALIB_SAMPLES) {
        while (!IMU.magneticFieldAvailable()) {
            yield();
            BLE.poll();
        }

        IMU.readMagneticField(magX, magY, magZ);

        magnXCalibBuffer[j] = magX;  magnYCalibBuffer[j] = magY; magnZCalibBuffer[j] = magZ;
        j++;
    }

    // Find maximums and minimums. The other values may be necessary
    int16_t xMin = magnXCalibBuffer[0]; int16_t yMin = magnYCalibBuffer[0]; int16_t zMin = magnZCalibBuffer[0];
    int16_t xMax = xMin;                int16_t yMax = yMin;                int16_t zMax = zMin;

    for (int i = 0; i < j - 1; ++i) {
        int16_t x = magnXCalibBuffer[i]; int16_t y = magnYCalibBuffer[i]; int16_t z = magnZCalibBuffer[i];

        if (x < xMin) { xMin = x; }
        else if (x > xMax) { xMax = x; }
        if (y < yMin) { yMin = y; }
        else if (y > yMax) { yMax = y; }
        if (z < zMin) { zMin = z; }
        else if (z > zMax) { zMax = z; }
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
        }
        else { retryCalibration = 0; }
    }
} // buffers go out of scope here, RAM reclaimed automatically

// Helper function. Checks that the recorded range sits on (ACCURACY)% of the expected span range. Otherwise, the calibration isn't reliable.
bool fullSpanCalibration(int16_t xMin, int16_t xMax, int16_t yMin, int16_t yMax, int16_t zMin, int16_t zMax) {
    if (((xMax - xMin) < CALIB_MIN_ACCURACY) || ((yMax - yMin) < CALIB_MIN_ACCURACY) || ((zMax - zMin) < CALIB_MIN_ACCURACY)) {
        return 1;
    }
    else {
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
