#include <Arduino.h>
#include "globals.h"
#include <Arduino_LPS22HB.h>
#include "baro.h"

// === Barometer Data Update ===
void updateBarometerData() {
	pressure = BARO.readPressure(); // in kPa
	paltitude = 44330 * (1 - pow(pressure / 101.325, 1 / 5.255));
}
