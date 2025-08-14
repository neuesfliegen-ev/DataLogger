#include <Arduino.h>
#include "globals.h"
#include <SD.h>
#include <U8g2lib.h>
#include "display.h"
#include "sdLogger.h"

String generateDataLine() {
    String line = "";
    line += String(millis()) + ",";
    line += String(accX) + ",";
    line += String(accY) + ",";
    line += String(accZ) + ",";
    line += String(gyroX) + ",";
    line += String(gyroY) + ",";
    line += String(gyroZ) + ",";
    line += String(magX) + ",";
    line += String(magY) + ",";
    line += String(magZ) + ",";
    line += String(latitude, 6) + ",";
    line += String(longitude, 6) + ",";
    line += String(gpsAltitude) + ",";
    line += String(Speed) + ",";
    line += String(SatCount) + ",";
    //  line += String(roll) + ",";
    //  line += String(pitch) + ",";
    //  line += String(yaw) + ",";
    //  line += String(pressure) + ",";
    //  line += String(temperature) + ",";
    //  line += String(paltitude); + ",";
    line += String(battery1Voltage); // + ",";  
    //  line += String(battery2Voltage);
    return line;
}

void startLog() {
    // Check if file exists by trying to open it in read mode
    if (SD.exists(filename)) {
        // File exists — open in append mode without writing header
        dataFile = SD.open(filename, O_APPEND);
        if (dataFile) {
            logState = LogState::ACTIVE;
            //Serial.println("Data logging Procceeded!");
            displayTwoLines("Data logging", "Proceed!", u8g2_font_ncenB10_tr, 15, 40);
        }
        else {
            //Serial.println("Error opening file for logging.");
            displayTwoLines("Error Proceeding", "logging", u8g2_font_ncenB10_tr, 15, 40);
            logState = LogState::IDLE;
        }
    }
    else {
        // File does not exist — open in write mode and write header
        dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile) {
            dataFile.println("timestamp,accX,accY,accZ,gyroX,gyroY,gyr...,Speed,SatCount,roll,pitch,yaw,pressure,temperature,paltitude");
            logState = LogState::ACTIVE;
            //Serial.println("Started a new logging session");
            displayTwoLines("Started new", "log session", u8g2_font_ncenB10_tr, 15, 40);

        }
        else {
            //Serial.println("Error opening file for logging.");
            displayTwoLines("Error opening", "log file", u8g2_font_ncenB10_tr, 15, 40);
            logState = LogState::IDLE;
        }
    }
    delay(2000); // 2 sec to read the message! 
}

// === Deactivates data logging status ===
void stopLog() {
    if (dataFile) {
        dataFile.flush();
        dataFile.close();
        //Serial.println("File closed");
        //Serial.println("Stopped data logging");
        displayTwoLines("File closed", "log Stopped!", u8g2_font_ncenB10_tr, 15, 40);
        delay(2000);

    }
    else {
        //Serial.println("File wasn't open");
        displayTwoLines("Error", "File wasn't open!", u8g2_font_ncenB10_tr, 15, 40);
        delay(2000);

    }
    logState = LogState::IDLE;
}

// === Updates data logging ===
void updateLogging() {
    if (buttonInterruptFlag) {
        buttonInterruptFlag = false; // Reset the flag

        if (logState == LogState::IDLE) {
            startLog();
        }
        else if (logState == LogState::ACTIVE) {
            stopLog();
        }
    }
}

// === Read Next Line from SD ===
void readNextLineFromSD() {
    static int lineNumber = 0;
    dataFile = SD.open(filename);
    if (!dataFile) {
        Serial.println("Error opening log.csv for reading.");
        return;
    }

    int currentLine = 0;
    String line = "";

    while (dataFile.available()) {
        line = dataFile.readStringUntil('\n');
        if (currentLine == lineNumber) {
            Serial.print("Line ");
            Serial.print(lineNumber);
            Serial.print(": ");
            Serial.println(line);
            lineNumber++;
            break;
        }
        currentLine++;
    }

    dataFile.close();
}

void logToSD(const String& line) {
    // Collect data
    dataBuffer += line + "\n";

    // Write buffered data every 1 sed
    if (millis() - lastWriteTime >= WRITE_INTERVAL) {
        if (dataFile) {
            dataFile.print(dataBuffer);
            dataBuffer = "";  // Clear buffer after writing
        }
        lastWriteTime = millis();
    }

    // Flush every 5 mins
    if (millis() - lastFlushTime >= FLUSH_INTERVAL) {
        if (dataFile) {
            dataFile.flush();
        }
        lastFlushTime = millis();
    }
}

void generateFilename() {
    timestamp = millis();

    if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print("Using GPS for session log ");
        displayTwoLines("Using GPS", "for session log", u8g2_font_ncenB10_tr, 10, 10);
        delay(2000);

        // Format: T<team><day><month><hour>.CSV ? 8 chars before .CSV T0119083.csv team 01 day 19 month 08, hour (3 or 13)
        snprintf(filename, sizeof(filename), "T%02d%02d%02d%01d.CSV",
            TEAM_NUMBER,
            gps.date.day(),
            gps.date.month(),
            gps.time.minute() % 10);  // Only 1 digit for hour

        Serial.println(filename);

    }
    else {
        Serial.println("No GPS time available!");
        displayTwoLines("No GPS time", "available!", u8g2_font_ncenB10_tr, 10, 10);
        delay(2000);

        displayTwoLines("Using timeStamp", "for session log", u8g2_font_ncenB10_tr, 10, 10);
        delay(2000);

        unsigned long ts = timestamp % 10000;
        snprintf(filename, sizeof(filename), "T%02d_%04lu.CSV", TEAM_NUMBER, ts);

        displayTwoLines("session_ID:", filename, u8g2_font_ncenB10_tr, 15, 15);
        delay(2000);
        Serial.println(filename);
    }
}
