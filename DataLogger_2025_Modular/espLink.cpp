#include "Arduino.h"
#include "globals.h"
#include <Wire.h>
#include "espLink.h"
#include "sdLogger.h"

// ISR
void onESPReady() {
    espReady = true;
}

// covert the curent mode to string to be used in the payload
String modeToString() {
    switch (currentMode) {
    case REALTIME: return "REALTIME";
    case BATCH: return "BATCH";
    default: return "UNKNOWN";
    }
}

// For real-time mode (pass a pre-bracketed JSON string like "[" + line + "]")
String buildJsonPayload(const String& jsonWrappedLine) {
    return "{\"mode\":\"" + modeToString() + "\",\"data\":" + jsonWrappedLine + "}";
}


void sendToESP(const String& msg) {
    WireESP.beginTransmission(ESP32_ADDR);
    WireESP.write(msg.c_str(), msg.length());
    byte result = WireESP.endTransmission();  // byte data type of the error

    if (result != 0) {
        if (now - LastI2CError >= 5000) // try to send again every 5 sec
        {
            LastI2CError = now;
            I2CError = true;

            // Reset I2C
            WireESP.end();
            delay(10);
            WireESP.begin(); // Re-initialize the I2C bus
            espReady = true; // make it true to re try sending data over I2c to avoid total dead lock witout stoping the sd card being logging

            // Display error info on OLED  /////******===========================Must be commented in real time==================================*****************================///////
            //switch (result) { 
            //  case 1:
            //    displayTwoLines("I2C Error:", "Buffer overflow", u8g2_font_ncenB10_tr, 40, 25);
            //    break;
            //  case 2:
            //    displayTwoLines("I2C Error:", "No device found", u8g2_font_ncenB10_tr, 40, 25);
            //    break;
            //  case 3:
            //    displayTwoLines("I2C Error:", "Data NACK", u8g2_font_ncenB10_tr, 40, 25);
            //    break;
            //  case 4:
            //    displayTwoLines("I2C Error:", "Bus error", u8g2_font_ncenB10_tr, 40, 25);
            //    break;
            //  default:
            //    displayTwoLines("I2C Error:", "Unknown code", u8g2_font_ncenB10_tr, 40, 25);
            //    break;
          //  }
          //  delay(1000);  // Give the user time to see the message /////******===========================Must be commented in real time==================================*****************================///////
        }
    }
    else {
        I2CError = false;
        espReady = false; // reset the flag pls
    }
}

void logToServer(const String& line) {
    //Serial.println(espReady);
    //Serial.println(digitalRead(READY_LINE));    
    if (currentMode == REALTIME && espReady) {
        sendToESP(buildJsonPayload("[" + line + "]"));
    }

    else if (currentMode == BATCH) {
        sensorBuffer.push("[" + line + "]");
        if (espReady) {
            sendToESP(buildJsonPayload(sensorBuffer));
            sensorBuffer.clear();
        }
    }

    // if (digitalRead(ACK_LINE) == HIGH) {
    //   Serial.println("? ACK received from ESP");
    // }
}

void log() {
    if (logState != LogState::ACTIVE) return;
    String line = generateDataLine();
    logToSD(line);
    logToServer(line);
}