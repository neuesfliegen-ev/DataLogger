#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

// === OLED Display === For short messages
void displayToScreen(const char str[], u8g2_uint_t x, u8g2_uint_t y);

// === OLED Display === In two lines 
void displayTwoLines(const char line1[], const char line2[], const uint8_t* font,
    u8g2_uint_t x1, u8g2_uint_t x2);
