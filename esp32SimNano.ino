#include <Wire.h>

#define I2C_ADDRESS 0x08
#define SIM_BAUD 115200
#define SIM_TX 17  // TX to SIM7600 RX
#define SIM_RX 16  // RX from SIM7600 TX

HardwareSerial SIM7600(2);
String latestPayload = "";
String lastResponse = "";  // Global variable to store latest modem response
String lastSendStatus = "WAIT";  // "OK", "FAIL", or "WAIT"
volatile bool sendingInProgress = false;

bool simReady = false;

// ====== I2C RECEIVE CALLBACK ======
#define MAX_MESSAGE_SIZE 22000  // 780 chunk
char messageBuffer[MAX_MESSAGE_SIZE];
uint16_t messageIndex = 0;

void receiveEvent(int howMany) {
  if (sendingInProgress) {
    Serial.println("[I2C] Received data while busy. Ignoring...");
    // flush bytes anyway
    while (Wire.available()) Wire.read();
    return;
  }

  if (howMany < 1) return;

  uint8_t chunkId = Wire.read();  // first byte = chunk index or 255 end marker

  if (chunkId == 255) {
    // End of transmission
    messageBuffer[messageIndex] = '\0';  // null-terminate
    Serial.println("[I2C] ✅ Full message received:");
    Serial.println(messageBuffer);

    latestPayload = String(messageBuffer);  // copy to String for processing
    messageIndex = 0;  // reset buffer for next message
    return;
  }

  // Read chunk into messageBuffer
  while (Wire.available() && messageIndex < MAX_MESSAGE_SIZE - 1) {
    messageBuffer[messageIndex++] = Wire.read();
  }
}



// ====== I2C REQUEST CALLBACK ======
void requestEvent() {
  const char* msg;

  if (!simReady) {
    msg = "WAIT";  // SIM7600 not initialized yet
  } else if (sendingInProgress) {
    msg = "BUSY";  // Currently processing a send
  } else if (lastSendStatus == "OK") {
    msg = "READY_OK";  // Ready and last send succeeded
  } else if (lastSendStatus == "FAIL") {
    msg = "READY_FAIL";  // Ready but last send failed
  } else {
    msg = "READY";  // Ready but no previous send yet
  }

  Wire.write((const uint8_t*)msg, strlen(msg));
}




void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  initSIM7600();
}

void loop() {
  if (simReady && latestPayload.length() > 0) {
    Serial.println("[I2C] Received:");
    Serial.println(latestPayload);

    if (sendPayload(latestPayload)) {
      Serial.println("[SIM7600] Payload sent successfully.");
    } else {
      Serial.println("[SIM7600] Failed to send payload.");
    }

    latestPayload = "";
  }

  delay(500);
}

// ====== INIT SIM7600 ======
int initSIM7600() {
  SIM7600.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  Serial.println("[SIM7600] Waiting for module...");

  // Keep sending AT until OK
  while (!sendAT("AT", "OK", 1000)) {
    Serial.println("[SIM7600] No AT response. Retrying...");
    delay(1000);
  }

  Serial.println("[SIM7600] AT responded. Proceeding with initialization...");

  // --- ATE0 ---
  if (!sendAT("ATE0", "OK", 1000)) {
    Serial.println("[SIM7600] Failed to disable echo (ATE0).");
    return 1;
  }

  // --- NETOPEN ---
  if (!sendAT("AT+NETOPEN", "OK", 5000)) {
    if (lastResponse.indexOf("+IP ERROR:") != -1) {
      Serial.println("[SIM7600] Network already open. Continuing...");
    } else {
      Serial.println("[SIM7600] Failed to open network.");
      return 2;
    }
  }

  // --- HTTPINIT ---
  if (!sendAT("AT+HTTPINIT", "OK", 2000)) {
    Serial.println("[SIM7600] HTTPINIT failed. Attempting HTTPTERM recovery...");
    if (sendAT("AT+HTTPTERM", "OK", 2000)) {
      Serial.println("[SIM7600] HTTP session terminated. Retrying HTTPINIT...");
      if (!sendAT("AT+HTTPINIT", "OK", 2000)) {
        Serial.println("[SIM7600] HTTPINIT failed again.");
        return 3;
      }
    } else {
      Serial.println("[SIM7600] HTTPTERM failed. Cannot proceed.");
      return 3;
    }
  }

  // --- Set URL ---
  if (!sendAT("AT+HTTPPARA=\"URL\",\"http://shiro11.pythonanywhere.com/data\"", "OK", 2000)) {
    Serial.println("[SIM7600] Failed to set HTTP URL.");
    return 4;
  }

  // --- Set content type ---
  if (!sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 2000)) {
    Serial.println("[SIM7600] Failed to set content type.");
    return 4;
  }

  Serial.println("[SIM7600] Initialization successful.");
  simReady = true;
  lastSendStatus = "READY";  // Initial state, no data sent yet

  return 0;
}


// ====== SEND PAYLOAD ======
bool sendPayload(String payload) {
  sendingInProgress = true;

  if (!sendAT("AT+HTTPDATA=" + String(payload.length()) + ",10000", "DOWNLOAD", 5000)) {
    lastSendStatus = "FAIL";
    sendingInProgress = false;
    return false;
  }

  SIM7600.println(payload);

  if (!waitFor("OK", 3000)) {  // Give it some room
    Serial.println("[SIM7600] No OK after payload. Aborting.");
    lastSendStatus = "FAIL";
    sendingInProgress = false;
    return false;
  }

  if (sendAT("AT+HTTPACTION=1", "+HTTPACTION: 1,200", 5000)) {
    lastSendStatus = "OK";
  } else {
    lastSendStatus = "FAIL";
  }

  sendingInProgress = false;
  return lastSendStatus == "OK";
}


bool waitFor(String expected, int timeout) {
  unsigned long t0 = millis();
  String buffer = "";

  while (millis() - t0 < timeout) {
    while (SIM7600.available()) {
      char c = SIM7600.read();
      buffer += c;
      Serial.write(c);  // Optional: print for debugging
    }

    if (buffer.indexOf(expected) != -1) {
      return true;
    }
  }

  Serial.println("[SIM7600] Timeout waiting for (waitFor): " + expected);
  return false;
}


// ====== SEND AT & WAIT FOR TARGET ======
bool sendAT(String cmd, String expected, int timeout) {
  SIM7600.println(cmd);
  unsigned long t0 = millis();
  lastResponse = "";  // capture new response

  while (millis() - t0 < timeout) {
    while (SIM7600.available()) {
      char c = SIM7600.read();
      lastResponse += c;
      Serial.write(c);  // Optional: debug output
    }
    if (lastResponse.indexOf(expected) != -1) return true;
  }

  Serial.println("[SIM7600] Timeout waiting for: " + expected);
  return false;
}
