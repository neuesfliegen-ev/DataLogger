#include <Arduino.h>
#include "globals.h"
#include "powerMonitor.h"

// === Updates current battery voltage ===
void updateBatteryVoltage() {
    battery1Voltage = analogRead(PIN_BAT1_VSENSE) * VREF / ADC_MAX / (R1 / (R1 + R2)); //
    battery2Voltage = analogRead(PIN_BAT2_VSENSE) * VREF / ADC_MAX / (R1 / (R1 + R2)); //
    //Serial.println(analogRead(PIN_BAT1_VSENSE) );
    //Serial.println(battery1Voltage);
}

// === Checks battery charging need ===
void checkBattery() {
    updateBatteryVoltage();

    if (battery1Voltage < BAT_VLOW || battery2Voltage < BAT_VLOW) { //!!!check LED connection for HIGH/LOW assignment
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, HIGH);
    }
    else {
        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_LED_GREEN, HIGH);
    }
}
