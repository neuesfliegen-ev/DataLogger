#include <Arduino.h>
#include "globals.h"
#include <ArduinoBLE.h>
#include "button.h"

// If WITH_BLUETOOTH isn't defined in the including TU, default to 1 (no redefinition if already set)
#ifndef WITH_BLUETOOTH
#define WITH_BLUETOOTH 1
#endif

// Interrupt service routine to set the flag button pressed! if the press happened (falling edge) > 200 ms , button is active
void onButtonPress() {
    if (millis() - lastEdge > DEBOUNCE_MS) {
        buttonInterruptFlag = true;
        lastEdge = millis();
    }
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
    }
    else if (WITH_BLUETOOTH == 0) {
        while (!buttonInterruptFlag) {
            yield();
        }
        buttonInterruptFlag = false;
    }
}
