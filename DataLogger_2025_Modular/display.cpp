#include <Arduino.h>
#include "globals.h"
#include <U8g2lib.h>
#include "display.h"

// === OLED Display === For short messages
void displayToScreen(const char str[], u8g2_uint_t x, u8g2_uint_t y) {
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_ncenB14_tr);   // u8g2_font_ncenB14_tr big size -- u8g2_font_ncenB10_tr IS OK // TOO SMALL u8g2_font_5x8_tr
	u8g2.setCursor(x, y);                 // Centered: (25, 35) -- Starting from the left: (0, 35)
	u8g2.print(str);
	u8g2.sendBuffer();
}

// === OLED Display === In two lines 
void displayTwoLines(const char line1[], const char line2[], const uint8_t* font, u8g2_uint_t x1, u8g2_uint_t x2) { // x1: x-axis position 1st line.  x2: x-axis position 2nd line.
	u8g2.clearBuffer();
	u8g2.setFont(font);
	u8g2.setCursor(x1, 25);  // Y1 = 25 for first line (adjust as needed)
	u8g2.print(line1);
	u8g2.setCursor(x2, 50);  // Y2 = 50 for second line (25+line height)
	u8g2.print(line2);
	u8g2.sendBuffer();
}
