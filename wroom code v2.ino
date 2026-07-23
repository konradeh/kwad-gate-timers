#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>

// ---- EDIT THESE ----
const char* WIFI_SSID = "superstuudio";
const char* WIFI_PASSWORD = "sepikoda";
const char* PI_IP = "192.168.1.213";   // <-- your Pi's IP from `hostname -I`
const int PI_PORT = 5000;
const char* NODE_ID = "checkpoint-1";
// ---------------------

// Calibrate these at the actual checkpoint.
const int8_t ENTER_RSSI = -62;
const int8_t EXIT_RSSI = -70;
const uint8_t REQUIRED_STRONG_SAMPLES = 3;
const unsigned long EVENT_COOLDOWN_MS = 2000;

const uint16_t BEACON_MAGIC = 0x4B47;
const uint8_t BEACON_VERSION = 1;

#pragma pack(push, 1)
struct DroneBeacon {
  uint16_t magic;
  uint8_t version;
  uint8_t drone_id;
  uint32_t boot_id;
  uint32_t sequence;
};
#pragma pack(pop)

struct ReceivedSample {
  uint8_t drone_id;
  uint32_t boot_id;
  uint32_t sequence;
  int8_t rssi;
  unsigned long received_at_ms;
};

portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool samplePending = false;
ReceivedSample pendingSample{};

bool droneInside = false;
uint8_t strongSampleCount = 0;
unsigned long lastEventMs = 0;
uint32_t lastBootId = 0;
uint32_t lastSequence = 0;
bool haveLastBeacon = false;

void onEspNowReceive(const esp_now_recv_info_t* info,
                     const uint8_t* data, int length) {
  if (info == nullptr || data == nullptr ||
      length != static_cast<int>(sizeof(DroneBeacon))) {
    return;
  }

  const auto* beacon = reinterpret_cast<const DroneBeacon*>(data);
  if (beacon->magic != BEACON_MAGIC ||
      beacon->version != BEACON_VERSION) {
    return;
  }

  const ReceivedSample sample{
      beacon->drone_id,
      beacon->boot_id,
      beacon->sequence,
      static_cast<int8_t>(info->rx_ctrl->rssi),
      millis()};

  portENTER_CRITICAL(&sampleMux);
  pendingSample = sample;
  samplePending = true;
  portEXIT_CRITICAL(&sampleMux);
}

void startEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: ESP-NOW initialization failed");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(onEspNowReceive);
  Serial.print("ESP-NOW listening on WiFi channel ");
  Serial.println(WiFi.channel());
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
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

  startEspNow();
}

void sendCheckpointEvent(const ReceivedSample& sample) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping send.");
    return;
  }

  HTTPClient http;
  String url = String("http://") + PI_IP + ":" + PI_PORT + "/checkpoint";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = String("{\"node_id\":\"") + NODE_ID +
                    "\",\"timestamp\":" + sample.received_at_ms +
                    ",\"drone_id\":" + sample.drone_id +
                    ",\"rssi\":" + sample.rssi +
                    ",\"sequence\":" + sample.sequence + "}";

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

void processSample(const ReceivedSample& sample) {
  if (haveLastBeacon &&
      sample.boot_id == lastBootId &&
      sample.sequence == lastSequence) {
    return;
  }

  haveLastBeacon = true;
  lastBootId = sample.boot_id;
  lastSequence = sample.sequence;

  Serial.printf("Drone %u  seq=%lu  RSSI=%d dBm\n",
                sample.drone_id,
                static_cast<unsigned long>(sample.sequence),
                static_cast<int>(sample.rssi));

  if (droneInside) {
    if (sample.rssi <= EXIT_RSSI) {
      droneInside = false;
      strongSampleCount = 0;
      Serial.println("Drone left checkpoint zone; detector re-armed.");
    }
    return;
  }

  if (sample.rssi < ENTER_RSSI) {
    strongSampleCount = 0;
    return;
  }

  if (strongSampleCount < REQUIRED_STRONG_SAMPLES) {
    strongSampleCount++;
  }

  const unsigned long now = millis();
  const bool cooldownFinished =
      lastEventMs == 0 || now - lastEventMs >= EVENT_COOLDOWN_MS;

  if (strongSampleCount >= REQUIRED_STRONG_SAMPLES && cooldownFinished) {
    droneInside = true;
    strongSampleCount = 0;
    lastEventMs = now;
    Serial.println("CHECKPOINT PASSED");
    sendCheckpointEvent(sample);
  }
}

void loop() {
  if (samplePending) {
    portENTER_CRITICAL(&sampleMux);
    const ReceivedSample sample = pendingSample;
    samplePending = false;
    portEXIT_CRITICAL(&sampleMux);

    processSample(sample);
  }

  delay(2);
}
