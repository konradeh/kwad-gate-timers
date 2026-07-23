#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>

// ---- NETWORK CONFIGURATION ----
const char* WIFI_SSID = "superstuudio";
const char* WIFI_PASSWORD = "sepikoda";
const char* PI_IP = "192.168.1.213";
const int PI_PORT = 5000;
const char* NODE_ID = "checkpoint-2";

// ---- GATE TIMING & TUNING PARAMETERS ----
// These are DEFAULTS only. They're overwritten at boot (and re-polled
// periodically) from the Pi's /api/settings/<NODE_ID> endpoint, so the
// live values can be changed from the site's Settings page without
// re-flashing. They stay non-const so fetchSettingsFromPi() can update
// them; if the Pi is unreachable, whatever value was last applied (or
// these defaults, on first boot) stays in effect.
int8_t ENTER_RSSI = -62;           // Threshold to begin tracking a pass
int8_t EXIT_RSSI = -72;            // Threshold considered outside gate
uint8_t REQUIRED_WEAK_SAMPLES = 5; // Weak samples required to close pass
unsigned long PASS_TIMEOUT_MS = 400; // Force-close pass if signal drops completely
unsigned long EVENT_COOLDOWN_MS = 2000; // Minimum time between valid passes per drone
unsigned long HEARTBEAT_INTERVAL_MS = 1000; // How often to tell the Pi this node is alive.
// Keep this >= ~1000ms. The debug panel already polls the Pi every 150ms
// on its own, so a faster heartbeat doesn't make the UI feel more live -
// it just multiplies TCP connection churn per node. At 100ms (10 req/s,
// each a fresh handshake, no keep-alive) a weak-signal board doesn't get
// enough time between requests to recover from packet loss, which is what
// was causing "read Timeout" / "connection refused" cascades.

const unsigned long SETTINGS_POLL_INTERVAL_MS = 10000; // How often to re-fetch settings from the Pi

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

struct CheckpointEvent {
  uint8_t drone_id;
  uint32_t sequence;
  int8_t rssi;
  unsigned long timestamp_ms;
};

struct DroneState {
  bool active = false;
  int8_t peakRssi = -128;
  unsigned long peakTimestamp = 0;
  uint32_t peakSequence = 0;
  uint8_t weakSampleCount = 0;
  unsigned long lastSampleMs = 0;
  unsigned long lastPassEmittedMs = 0;
  
  uint32_t lastBootId = 0;
  uint32_t lastSequence = 0;
  bool haveLastBeacon = false;
};

// Queues for inter-task communication
QueueHandle_t sampleQueue = nullptr;
QueueHandle_t httpQueue = nullptr;

// Per-drone tracking state array (supports drone IDs 0-255)
DroneState droneTrackers[256];

// Interrupt Service Routine for ESP-NOW packet reception
void IRAM_ATTR onEspNowReceive(const esp_now_recv_info_t* info,
                               const uint8_t* data, int length) {
  if (info == nullptr || data == nullptr || length != sizeof(DroneBeacon)) {
    return;
  }

  const auto* beacon = reinterpret_cast<const DroneBeacon*>(data);
  if (beacon->magic != BEACON_MAGIC || beacon->version != BEACON_VERSION) {
    return;
  }

  ReceivedSample sample{
      beacon->drone_id,
      beacon->boot_id,
      beacon->sequence,
      static_cast<int8_t>(info->rx_ctrl->rssi),
      millis()
  };

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(sampleQueue, &sample, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// Background FreeRTOS task handling network requests asynchronously
void httpTask(void* parameter) {
  CheckpointEvent event;
  while (true) {
    if (xQueueReceive(httpQueue, &event, portMAX_DELAY) == pdTRUE) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[HTTP Task] WiFi disconnected, dropping event.");
        continue;
      }

      HTTPClient http;
      String url = String("http://") + PI_IP + ":" + PI_PORT + "/checkpoint";

      http.begin(url);
      http.setConnectTimeout(3000);
      http.setTimeout(3000);
      http.addHeader("Content-Type", "application/json");

      String payload = String("{\"node_id\":\"") + NODE_ID +
                        "\",\"timestamp\":" + event.timestamp_ms +
                        ",\"drone_id\":" + event.drone_id +
                        ",\"rssi\":" + event.rssi +
                        ",\"sequence\":" + event.sequence + "}";

      int httpCode = http.POST(payload);
      if (httpCode > 0) {
        Serial.printf("[HTTP Task] POST Success (%d): %s\n", httpCode, payload.c_str());
      } else {
        Serial.printf("[HTTP Task] POST Failed: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    }
  }
}

void finalizePass(uint8_t droneId, DroneState& state) {
  const unsigned long now = millis();
  
  if (now - state.lastPassEmittedMs >= EVENT_COOLDOWN_MS) {
    state.lastPassEmittedMs = now;

    Serial.printf("\n>>> GATE PASSED | Drone ID: %u | Peak RSSI: %d dBm at %lu ms <<<\n\n",
                  droneId, state.peakRssi, state.peakTimestamp);

    CheckpointEvent event{
        droneId,
        state.peakSequence,
        state.peakRssi,
        state.peakTimestamp
    };

    if (xQueueSend(httpQueue, &event, 0) != pdTRUE) {
      Serial.println("WARNING: HTTP Event Queue full! Event dropped.");
    }
  } else {
    Serial.printf("Pass for Drone %u suppressed by cooldown buffer.\n", droneId);
  }

  // Reset pass tracking state
  state.active = false;
  state.weakSampleCount = 0;
  state.peakRssi = -128;
}

void processSample(const ReceivedSample& sample) {
  DroneState& state = droneTrackers[sample.drone_id];

  // Sequence deduplication
  if (state.haveLastBeacon &&
      sample.boot_id == state.lastBootId &&
      sample.sequence == state.lastSequence) {
    return;
  }

  state.haveLastBeacon = true;
  state.lastBootId = sample.boot_id;
  state.lastSequence = sample.sequence;

  const unsigned long now = sample.received_at_ms;

  if (!state.active) {
    // Check if drone enters gate sensitivity field
    if (sample.rssi >= ENTER_RSSI) {
      state.active = true;
      state.peakRssi = sample.rssi;
      state.peakTimestamp = sample.received_at_ms;
      state.peakSequence = sample.sequence;
      state.weakSampleCount = 0;
      state.lastSampleMs = now;
      Serial.printf("[Entry] Drone %u entering zone (RSSI: %d dBm)\n", sample.drone_id, sample.rssi);
    }
  } else {
    // Tracking active session: check for new peak
    if (sample.rssi > state.peakRssi) {
      state.peakRssi = sample.rssi;
      state.peakTimestamp = sample.received_at_ms;
      state.peakSequence = sample.sequence;
    }

    // Accumulate exit conditions
    if (sample.rssi <= EXIT_RSSI) {
      state.weakSampleCount++;
    } else {
      state.weakSampleCount = 0; // Signal recovered, reset exit counter
    }

    state.lastSampleMs = now;

    // Trigger pass event if exited
    if (state.weakSampleCount >= REQUIRED_WEAK_SAMPLES) {
      Serial.printf("[Exit] Drone %u left zone via signal fade.\n", sample.drone_id);
      finalizePass(sample.drone_id, state);
    }
  }
}

void checkActiveTimeouts() {
  const unsigned long now = millis();
  for (int id = 0; id < 256; id++) {
    DroneState& state = droneTrackers[id];
    if (state.active && (now - state.lastSampleMs > PASS_TIMEOUT_MS)) {
      Serial.printf("[Timeout] Drone %u left zone via timeout.\n", id);
      finalizePass(id, state);
    }
  }
}

// Background task that pings the Pi's /api/heartbeat endpoint on a fixed
// interval, independent of drone activity, so the debug panel can tell
// this node is alive even when no drone has passed recently.
void heartbeatTask(void* parameter) {
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      String url = String("http://") + PI_IP + ":" + PI_PORT + "/api/heartbeat";

      http.begin(url);
      http.setConnectTimeout(3000);
      http.setTimeout(3000);
      http.addHeader("Content-Type", "application/json");

      String payload = String("{\"node_id\":\"") + NODE_ID +
                        "\",\"wifi_rssi\":" + WiFi.RSSI() + "}";

      int httpCode = http.POST(payload);
      if (httpCode > 0) {
        Serial.printf("[Heartbeat] OK (%d)\n", httpCode);
      } else {
        Serial.printf("[Heartbeat] Failed: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    } else {
      Serial.println("[Heartbeat] WiFi disconnected, skipping.");
    }

    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
  }
}

// Sentinel returned by extractJsonNumber() when a key isn't present in the
// response, chosen to be outside any real value this firmware ever uses.
const long JSON_KEY_NOT_FOUND = -2147483647L;

// Minimal hand-rolled parser for the small flat JSON object the Pi returns
// from /api/settings/<node_id>, e.g. {"enter_rssi":-62,"exit_rssi":-72,...}.
// Avoids pulling in ArduinoJson for a fixed, known schema.
long extractJsonNumber(const String& json, const char* key) {
  String pattern = String("\"") + key + "\":";
  int idx = json.indexOf(pattern);
  if (idx < 0) return JSON_KEY_NOT_FOUND;

  idx += pattern.length();
  while (idx < (int)json.length() && json[idx] == ' ') idx++;

  int start = idx;
  if (idx < (int)json.length() && json[idx] == '-') idx++;
  while (idx < (int)json.length() && isDigit(json[idx])) idx++;

  if (idx == start || (idx == start + 1 && json[start] == '-')) return JSON_KEY_NOT_FOUND;
  return json.substring(start, idx).toInt();
}

// Fetches this node's gate-timing settings from the Pi and applies any
// values present in the response. Silently keeps the current values (last
// applied, or the compiled-in defaults on first boot) if the Pi can't be
// reached or a field is missing - a node should never brick itself just
// because the Pi is briefly unreachable.
void fetchSettingsFromPi() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String("http://") + PI_IP + ":" + PI_PORT + "/api/settings/" + NODE_ID;
  http.begin(url);
  http.setConnectTimeout(3000);
  http.setTimeout(3000);

  int httpCode = http.GET();
  if (httpCode == 200) {
    String body = http.getString();
    long v;

    v = extractJsonNumber(body, "enter_rssi");
    if (v != JSON_KEY_NOT_FOUND) ENTER_RSSI = (int8_t)v;

    v = extractJsonNumber(body, "exit_rssi");
    if (v != JSON_KEY_NOT_FOUND) EXIT_RSSI = (int8_t)v;

    v = extractJsonNumber(body, "required_weak_samples");
    if (v != JSON_KEY_NOT_FOUND) REQUIRED_WEAK_SAMPLES = (uint8_t)v;

    v = extractJsonNumber(body, "pass_timeout_ms");
    if (v != JSON_KEY_NOT_FOUND) PASS_TIMEOUT_MS = (unsigned long)v;

    v = extractJsonNumber(body, "event_cooldown_ms");
    if (v != JSON_KEY_NOT_FOUND) EVENT_COOLDOWN_MS = (unsigned long)v;

    v = extractJsonNumber(body, "heartbeat_interval_ms");
    if (v != JSON_KEY_NOT_FOUND) HEARTBEAT_INTERVAL_MS = (unsigned long)v;

    Serial.printf("[Settings] Applied: ENTER=%d EXIT=%d WEAK=%u PASS_TO=%lu COOLDOWN=%lu HB=%lu\n",
                  ENTER_RSSI, EXIT_RSSI, REQUIRED_WEAK_SAMPLES,
                  PASS_TIMEOUT_MS, EVENT_COOLDOWN_MS, HEARTBEAT_INTERVAL_MS);
  } else {
    Serial.printf("[Settings] Fetch failed (%d), keeping current values.\n", httpCode);
  }
  http.end();
}

// Background task that re-polls settings periodically so changes made on
// the site's Settings page take effect live, without re-flashing.
void settingsTask(void* parameter) {
  while (true) {
    fetchSettingsFromPi();
    vTaskDelay(pdMS_TO_TICKS(SETTINGS_POLL_INTERVAL_MS));
  }
}

void startEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: ESP-NOW initialization failed");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(onEspNowReceive);
  Serial.printf("ESP-NOW listening on WiFi channel %d\n", WiFi.channel());
}

// Connects to WiFi, retrying with a fresh WiFi.begin() call every 15s
// instead of hanging forever on a single attempt. Prints the numeric
// WiFi.status() code on failure so a stuck node can be diagnosed from
// Serial: 1 = SSID not found (out of range or typo), 4 = wrong password.
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const unsigned long attemptStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - attemptStart < 15000) {
      delay(400);
      Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.printf("\nWiFi connect timed out (status=%d). Retrying...\n", WiFi.status());
      WiFi.disconnect();
      delay(1000);
    }
  }

  Serial.println();
  Serial.print("Connected! ESP32 Gateway IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Pre-allocate FreeRTOS Queues
  sampleQueue = xQueueCreate(64, sizeof(ReceivedSample));
  httpQueue = xQueueCreate(16, sizeof(CheckpointEvent));

  // Spawn HTTP background worker task on Core 0
  xTaskCreatePinnedToCore(
      httpTask,
      "HTTP_Task",
      4096,
      nullptr,
      1,
      nullptr,
      0
  );

  // Spawn periodic heartbeat task on Core 0
  xTaskCreatePinnedToCore(
      heartbeatTask,
      "Heartbeat_Task",
      4096,
      nullptr,
      1,
      nullptr,
      0
  );

  // Spawn periodic settings-refresh task on Core 0
  xTaskCreatePinnedToCore(
      settingsTask,
      "Settings_Task",
      4096,
      nullptr,
      1,
      nullptr,
      0
  );

  connectWiFi();
  fetchSettingsFromPi(); // apply live settings once before the first pass can be detected
  startEspNow();
}

void loop() {
  ReceivedSample sample;

  // Process all incoming radio samples in queue
  while (xQueueReceive(sampleQueue, &sample, 0) == pdTRUE) {
    processSample(sample);
  }

  // Check if any tracked pass has timed out
  checkActiveTimeouts();

  delay(2);
}