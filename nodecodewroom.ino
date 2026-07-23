#include <WiFi.h>
#include <HTTPClient.h>

// ---- EDIT THESE ----
const char* WIFI_SSID = "superstuudio";
const char* WIFI_PASSWORD = "sepikoda";
const char* PI_IP = "192.168.1.213";   // <-- your Pi's IP from `hostname -I`
const int PI_PORT = 5000;
const char* NODE_ID = "checkpoint-1";  // change to checkpoint-2 on the other ESP32
// ---------------------

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 3000; // ms

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected! ESP32 IP address: ");
  Serial.println(WiFi.localIP());
}

void sendCheckpointEvent() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping send.");
    return;
  }

  HTTPClient http;
  String url = String("http://") + PI_IP + ":" + PI_PORT + "/checkpoint";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = String("{\"node_id\":\"") + NODE_ID +
                    "\",\"timestamp\":" + millis() + "}";

  int httpCode = http.POST(payload);

  Serial.print("POST -> ");
  Serial.print(url);
  Serial.print(" | payload: ");
  Serial.print(payload);
  Serial.print(" | response code: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    String response = http.getString();
    Serial.print("Server response: ");
    Serial.println(response);
  } else {
    Serial.print("POST failed, error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    sendCheckpointEvent();
  }
}
