#pragma once
#include <Arduino.h>

// Forward for template body (used by the template)
String modeToString();

// === ESP ===
void onESPReady();  // ISR
void sendToESP(const String& msg);

// For REALTIME mode
String buildJsonPayload(const String& jsonWrappedLine);

/* For BATCH mode overload template must live in the header so it can be instantiated. 
streamlit deals with json in realtime dashboards.so we must use it.
to decrease the payload we will not send the header. 
only send the mode and the data as arraylist with the aggreed arangment.
this function  takes whatever arraylsit or line/ string and encapulste it in json format this funcion has an overwrite with differnt sig
*/
template <size_t N>
String buildJsonPayload(const CircularBuffer<String, N>& buffer) {
  String payload = "{\"mode\":\"" + modeToString() + "\",\"data\":[";
  for (int i = 0; i < buffer.size(); i++) {
    payload += buffer[i];
    if (i < buffer.size() - 1) payload += ",";
  }
  payload += "]}";
  return payload;
}

// SD + server send
void log();
void logToServer(const String& line);