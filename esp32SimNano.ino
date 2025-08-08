#include <Wire.h>

#define I2C_ADDRESS    0x42
#define STATUS_LED     2 
#define READY_LINE     4   // ESP32 ➝ Nano
#define ACK_LINE       5   // ESP32 ➝ Nano
#define SIM_BAUD 115200
#define SIM_TX 17  // TX to SIM7600 RX
#define SIM_RX 16  // RX from SIM7600 TX
#define SEND_INTERVAL  200  // ms (time between I2C receive and ACK)

HardwareSerial SIM7600(2);

enum Mode { REALTIME, BATCH };
Mode currentMode = REALTIME;  // Default mode

String latestPayload = "";
String lastResponse = "";

bool dataHasBeenReceived = false;
bool simReady = false;

unsigned long now = 0;
unsigned long lastAttemptTime = 0;
unsigned long lastSIMCheckTime = 0;

// ====== I2C RECEIVE CALLBACK ======
void receiveEvent(int howMany) {
  if (howMany < 2) return;

  digitalWrite(READY_LINE, LOW);  // 🛑 Not ready while receiving

  latestPayload = "";
  while (Wire.available()) {
    latestPayload += (char)Wire.read();
  }

  dataHasBeenReceived = true;

  // 🔍 Look for mode in JSON payload
  if (latestPayload.indexOf("\"mode\":\"REALTIME\"") != -1) {
    currentMode = REALTIME;
    //Serial.println("🌀 Mode: REALTIME");
  } else if (latestPayload.indexOf("\"mode\":\"BATCH\"") != -1) {
    currentMode = BATCH;
    //Serial.println("🌀 Mode: BATCH");
  } else {
    //Serial.println("⚠️ Could not determine mode from payload.");
  }

  //Serial.println("✅ Payload received:");
  //Serial.println(latestPayload);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(READY_LINE, OUTPUT);
  pinMode(ACK_LINE, OUTPUT);

  digitalWrite(READY_LINE, LOW);
  digitalWrite(ACK_LINE, LOW);

  Wire.begin(I2C_ADDRESS);  // ESP32 as I2C Slave
  Wire.onReceive(receiveEvent);

  delay(2000);
  initSIM7600();
  Serial.println("✅ ESP32 I2C Slave ready");
}

void loop() {
  now = millis();

  // Blinking status LED (heartbeat)
  digitalWrite(STATUS_LED, now % 1000 < 50);

  if (dataHasBeenReceived && (now - lastAttemptTime >= SEND_INTERVAL)) {
    dataHasBeenReceived = false;

    //Serial.println("[ESP] 📨 Forwarding payload:");
    //Serial.println(latestPayload);

    // Simulated send to SIM7600 here
    // bool ok = sendPayload(latestPayload);
    bool ok =  sendPayload(latestPayload);  // Placeholder for real send

    if (ok) {
      //Serial.println("[ESP] ✅ Payload processed!");
      digitalWrite(ACK_LINE, HIGH);
    } else {
    //  Serial.println("[ESP] ❌ Send failed");
      digitalWrite(ACK_LINE, LOW);
    }

    lastAttemptTime = now;
    latestPayload = "";
  }

  // SIM status check every SEND_INTERVAL mm sec
  if (now - lastSIMCheckTime >= SEND_INTERVAL ) {
    simReady = checkSIMStatus();
    digitalWrite(READY_LINE, simReady ? HIGH : LOW);
    lastSIMCheckTime = now;
  }
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
  digitalWrite(READY_LINE, HIGH);  // Signal Nano that we're ready

  return 0;
}


// ====== SEND PAYLOAD ======
bool sendPayload(const String& payload) {
  if (!sendAT("AT+HTTPDATA=" + String(payload.length()) + ",10000", "DOWNLOAD", 3000)) return false;

  SIM7600.println(payload);
  SIM7600.flush();

  if (!waitFor("OK", 3000)) return false;
  
  if (sendAT("AT+HTTPACTION=1", "+HTTPACTION: 1,200", 5000)) return true;
  return false;
}


// ====== WAIT FOR TARGET ======

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

 // Serial.println("[SIM7600] Timeout waiting for (waitFor): " + expected);
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

  //Serial.println("[SIM7600] Timeout waiting for: " + expected);
  return false;
}

//====== Check if SIM still alive ======
bool checkSIMStatus() {
  return sendAT("AT", "OK", 1000);
}
