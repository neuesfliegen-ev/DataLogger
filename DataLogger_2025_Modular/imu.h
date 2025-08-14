#pragma once
#include <Arduino.h>

// ---- Periodic update ----
void updateIMUData();

// ---- Calibration (Acc + Gyro, stationary) ----
void calibrateIMU();

// ---- Calibration (Magnetometer, “victory dance”) ----
void calibrateIMU2();

// ---- Range helper ----
bool fullSpanCalibration(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t);

// (DEBUGGING) Helper function
void printIMUOffsetsAndReadings();
