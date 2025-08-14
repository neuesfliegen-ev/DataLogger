#include <Arduino.h>
#include "globals.h"
#include <TinyGPSPlus.h>
#include "display.h"
#include "gps.h"

void updateGPSData() {
    while (Serial1.available()) {
        char c = Serial1.read();

        // Print raw NMEA sentence
        //Serial.write(c);

        // Feed to TinyGPS parser
        gps.encode(c);

        SatCount = gps.satellites.value();

        // Show satellite count every ~1s
        //  if (millis() % 1000 < 50) {
        //    Serial.print("\n Satellites count: ");
        //    Serial.println(gps.satellites.value());
        //    displayTwoLines("Satellites count: ", String(SatCount).c_str(), u8g2_font_ncenB10_tr, 5, 20);
        //  }

        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            gpsAltitude = gps.altitude.meters();
            Speed = gps.speed.kmph();
            SatCount = gps.satellites.value();

            // Serial.println("\n? Parsed GPS Data:");
            // Serial.print("Latitude: "); Serial.println(latitude, 6);
            // Serial.print("Longitude: "); Serial.println(longitude, 6);
            // Serial.print("Satellites: "); Serial.println(SatCount);
            // Serial.print("Speed (km/h): "); Serial.println(Speed);
            // Serial.print("Altitude (m): "); Serial.println(gpsAltitude);
            // Serial.println("----------------------");
        }
    }
}

void waitForGPSLock() {
    unsigned long startTime = millis();
    bool locked = false;

    while (millis() - startTime < GPS_LOCK_TIMEOUT_MS) {
        updateGPSData();

        Serial.print("Satellites: ");
        Serial.println(SatCount);

        // Display satellite count
        char satMsg[30];
        snprintf(satMsg, sizeof(satMsg), "Sats: %d/%d", SatCount, MIN_SATS_REQUIRED);
        displayTwoLines("Waiting for GPS", satMsg, u8g2_font_ncenB10_tr, 15, 30);

        if (SatCount >= MIN_SATS_REQUIRED && gps.date.isValid() && gps.time.isValid()) {
            locked = true;
            break;
        }

        delay(500);  // Check every 500 ms
    }

    if (!locked) {
        Serial.print("Could not lock minimum satellites. Current sats: ");
        Serial.println(SatCount);

        char errMsg1[] = "GPS Lock Failed!";
        char errMsg2[30];
        snprintf(errMsg2, sizeof(errMsg2), "Sats: %d/%d", SatCount, MIN_SATS_REQUIRED);
        displayTwoLines(errMsg1, errMsg2, u8g2_font_ncenB10_tr, 10, 30);

        delay(3000); // Show error for 3 seconds before continuing
    }
    else {
        Serial.println("GPS lock acquired!");
        displayTwoLines("GPS Lock", "acquired!", u8g2_font_ncenB10_tr, 30, 40);
        delay(2000);
    }
}
