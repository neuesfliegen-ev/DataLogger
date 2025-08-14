#pragma once
#include <Arduino.h>

// === Logging Functions ===
String generateDataLine();
void logToSD(const String& line);
void startLog();
void stopLog();
void updateLogging();
void readNextLineFromSD();
void generateFilename();
