#include "Arduino.h"
#include "globals.h"
#include <ArduinoBLE.h>
#include "ble.h"

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
