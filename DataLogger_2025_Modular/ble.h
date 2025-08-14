#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>

// BLE init + event handlers
// Note: some user-trigger flow (button press ? waitForButtonPressed) lives in button.* per project §5/§12.

void initializeBLE();
void onTriggerWritten(BLEDevice central, BLECharacteristic characteristic);
void onConnect(BLEDevice central);
void onDisconnect(BLEDevice central);
