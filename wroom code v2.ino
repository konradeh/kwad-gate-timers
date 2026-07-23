#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Bump this whenever this file changes and reflash every board that needs
// it. Reported on every heartbeat so the site's debug panel can show which
// firmware version each physical node is actually running - no more
// guessing which boards still need a reflash by diffing behavior.
const char* FW_VERSION = "1.2.0";

// ---- NETWORK CONFIGURATION ----
const char* WIFI_SSID = "superstuudio";
const char* WIFI_PASSWORD = "sepikoda";
const char* PI_IP = "192.168.1.213";
const int PI_PORT = 5000;
const char* NODE_ID = "checkpoint-3";

// ---- GATE TIMING & TUNING PARAMETERS ----
// These are DEFAULTS only. They're fetched from the Pi once at boot, then
// kept in sync via every heartbeat response, so live values can be changed
// from the site's Settings page without re-flashing. They stay non-const
// so applySettingsFromJson() can update them; if the Pi is unreachable,
// whatever value was last applied (or these defaults, on first boot)
// stays in effect.
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
  bool active = false;          // true while "inside" the gate zone, since the entry event fired
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

// Most recent ESP-NOW sample from ANY drone, updated on every packet
// regardless of pass-detection state. heartbeatTask reports this so the
// site can show live proximity ("which node is the drone closest to right
// now"), not just discrete pass events.
uint8_t lastDroneId = 0;
int8_t lastDroneRssi = -128;
unsigned long lastDroneSampleMs = 0; // 0 = never received a sample

// Last time WiFi was confirmed connected, used by the reconnect watchdog
// to decide when a soft reconnect has clearly failed and a hard cycle is
// needed. Written from the WiFi event handler and the watchdog task.
volatile unsigned long lastWiFiConnectedMs = 0;

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

// Fires the /checkpoint event. Called the moment a drone ENTERS the gate
// zone (crosses ENTER_RSSI), not when it leaves - lap timing wants the
// entry timestamp, which is consistent regardless of how fast the drone
// is moving or how long its signal takes to fade afterward.
void triggerPassEvent(uint8_t droneId, int8_t rssi, uint32_t sequence,
                       unsigned long timestampMs, DroneState& state) {
  const unsigned long now = millis();

  if (now - state.lastPassEmittedMs >= EVENT_COOLDOWN_MS) {
    state.lastPassEmittedMs = now;

    Serial.printf("\n>>> GATE PASSED | Drone ID: %u | Entry RSSI: %d dBm at %lu ms <<<\n\n",
                  droneId, rssi, timestampMs);

    CheckpointEvent event{droneId, sequence, rssi, timestampMs};

    if (xQueueSend(httpQueue, &event, 0) != pdTRUE) {
      Serial.println("WARNING: HTTP Event Queue full! Event dropped.");
    }
  } else {
    Serial.printf("Pass for Drone %u suppressed by cooldown buffer.\n", droneId);
  }
}

void processSample(const ReceivedSample& sample) {
  DroneState& state = droneTrackers[sample.drone_id];

  // Sequence deduplication
  if (state.haveLastBeacon &&
      sample.boot_id == state.lastBootId &&
      sample.sequence == state.lastSequence) {
    return;
  }

  // Record every sample as "live proximity" data, independent of whether
  // it crosses the ENTER_RSSI/EXIT_RSSI pass-detection thresholds below.
  // This is what lets the site show a continuous "how close is the drone"
  // reading instead of only reporting once a full gate-pass completes.
  lastDroneId = sample.drone_id;
  lastDroneRssi = sample.rssi;
  lastDroneSampleMs = sample.received_at_ms;

  state.haveLastBeacon = true;
  state.lastBootId = sample.boot_id;
  state.lastSequence = sample.sequence;

  const unsigned long now = sample.received_at_ms;

  if (!state.active) {
    // Drone enters the gate zone: fire the pass event right away instead
    // of waiting to see when it leaves.
    if (sample.rssi >= ENTER_RSSI) {
      state.active = true;
      state.weakSampleCount = 0;
      state.lastSampleMs = now;
      Serial.printf("[Entry] Drone %u entering zone (RSSI: %d dBm)\n", sample.drone_id, sample.rssi);
      triggerPassEvent(sample.drone_id, sample.rssi, sample.sequence, sample.received_at_ms, state);
    }
  } else {
    // Already inside the zone: just watch for the exit conditions that
    // re-arm this drone for its next pass. No event fires here.
    if (sample.rssi <= EXIT_RSSI) {
      state.weakSampleCount++;
    } else {
      state.weakSampleCount = 0; // Signal recovered, reset exit counter
    }

    state.lastSampleMs = now;

    if (state.weakSampleCount >= REQUIRED_WEAK_SAMPLES) {
      Serial.printf("[Exit] Drone %u left zone via signal fade; re-armed for next pass.\n", sample.drone_id);
      state.active = false;
      state.weakSampleCount = 0;
    }
  }
}

void checkActiveTimeouts() {
  const unsigned long now = millis();
  for (int id = 0; id < 256; id++) {
    DroneState& state = droneTrackers[id];
    if (state.active && (now - state.lastSampleMs > PASS_TIMEOUT_MS)) {
      Serial.printf("[Timeout] Drone %u left zone via timeout; re-armed.\n", id);
      state.active = false;
      state.weakSampleCount = 0;
    }
  }
}

// Sentinel returned by extractJsonNumber() when a key isn't present in the
// response, chosen to be outside any real value this firmware ever uses.
const long JSON_KEY_NOT_FOUND = -2147483647L;

// Minimal hand-rolled parser for the small flat JSON object the Pi returns
// from /api/settings/<node_id> and /api/heartbeat, e.g.
// {"status":"ok","enter_rssi":-62,"exit_rssi":-72,...}. Avoids pulling in
// ArduinoJson for a fixed, known schema.
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

// Applies any settings fields present in a JSON response body to the live
// globals. Missing fields are left untouched, so a partial/stale response
// never blanks out a value that just wasn't included.
void applySettingsFromJson(const String& body) {
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
}

// One-time settings fetch used only at boot, before the first pass can be
// detected. Ongoing updates arrive piggybacked on the heartbeat response
// instead (see heartbeatTask) rather than from a second periodic request -
// two concurrent HTTP tasks contending for one ESP32's single WiFi radio
// was causing periodic multi-second stalls on weak-signal boards.
void fetchSettingsFromPi() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String("http://") + PI_IP + ":" + PI_PORT + "/api/settings/" + NODE_ID;
  http.begin(url);
  http.setConnectTimeout(3000);
  http.setTimeout(3000);

  int httpCode = http.GET();
  if (httpCode == 200) {
    applySettingsFromJson(http.getString());
    Serial.printf("[Settings] Applied: ENTER=%d EXIT=%d WEAK=%u PASS_TO=%lu COOLDOWN=%lu HB=%lu\n",
                  ENTER_RSSI, EXIT_RSSI, REQUIRED_WEAK_SAMPLES,
                  PASS_TIMEOUT_MS, EVENT_COOLDOWN_MS, HEARTBEAT_INTERVAL_MS);
  } else {
    Serial.printf("[Settings] Fetch failed (%d), keeping current values.\n", httpCode);
  }
  http.end();
}

// Background task that pings the Pi's /api/heartbeat endpoint on a fixed
// interval, independent of drone activity, so the debug panel can tell
// this node is alive even when no drone has passed recently. The Pi's
// response also carries this node's current settings, so this single
// request keeps both the "alive" signal and live config in sync - no
// separate settings-polling task needed.
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
                        "\",\"fw_version\":\"" + FW_VERSION +
                        "\",\"wifi_rssi\":" + WiFi.RSSI();

      // Only include live drone-proximity fields once a sample has
      // actually been received - otherwise there's nothing meaningful to
      // report yet (lastDroneSampleMs stays 0 until the first ESP-NOW
      // packet arrives).
      if (lastDroneSampleMs != 0) {
        payload += String(",\"drone_id\":") + lastDroneId +
                   ",\"drone_rssi\":" + lastDroneRssi +
                   ",\"drone_age_ms\":" + (millis() - lastDroneSampleMs);
      }
      payload += "}";

      int httpCode = http.POST(payload);
      if (httpCode > 0) {
        applySettingsFromJson(http.getString());
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

// Fires on every WiFi state change. Two jobs:
//  1. On disconnect, kick off a reconnect IMMEDIATELY instead of waiting
//     for the next watchdog poll - this is what was missing before, where
//     a weak-signal board would drop and just sit offline for minutes.
//  2. Log the numeric disconnect reason code, which tells you WHY it
//     dropped so a recurring problem can be pinned to a real cause:
//        200 = beacon timeout (weak signal / drifting out of range)
//        201 = no AP found (AP gone, or channel changed)
//        8   = assoc leave / 4 = assoc expire (the router kicked it,
//              often because the signal got too weak)
//        15  = 4-way handshake timeout / 2 = auth expire (auth issue)
void applyWiFiRobustness(); // forward declaration; defined just below

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      lastWiFiConnectedMs = millis();
      // Re-apply on every (re)association - TX power and power-save can
      // reset when the link re-establishes, and this handler is the one
      // path every reconnect (soft or hard) passes through.
      applyWiFiRobustness();
      Serial.print("[WiFi] Connected, IP: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[WiFi] Dropped (reason %d). Reconnecting...\n",
                    info.wifi_sta_disconnected.reason);
      WiFi.reconnect();
      break;
    default:
      break;
  }
}

// Applies the settings that make an ESP32 hold a marginal WiFi link:
// disable modem power-save (the radio must never nap between beacons or a
// weak-signal board silently misses the AP), and push TX power near max to
// improve the uplink budget. Call after WiFi.mode() has started the driver.
void applyWiFiRobustness() {
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  // 80 = 20 dBm (units are 0.25 dBm). Near max without pushing into the
  // range where current spikes can brown out a board on a weak USB supply.
  esp_wifi_set_max_tx_power(80);
}

// Connects to WiFi, retrying with a fresh WiFi.begin() call every 15s
// instead of hanging forever on a single attempt. Prints the numeric
// WiFi.status() code on failure so a stuck node can be diagnosed from
// Serial: 1 = SSID not found (out of range or typo), 4 = wrong password.
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWiFiEvent);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  applyWiFiRobustness();

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

  lastWiFiConnectedMs = millis();
  Serial.println();
  Serial.print("Connected! ESP32 Gateway IP: ");
  Serial.println(WiFi.localIP());
}

// Backstop for onWiFiEvent's fast reconnect: if the link is STILL down
// several seconds later (the soft WiFi.reconnect() got stuck, which is
// exactly how a board ends up "offline for 7 minutes"), force a full
// disconnect/begin cycle. Between the event handler and this watchdog, a
// dropped node recovers in seconds instead of staying dead until reboot.
void wifiWatchdogTask(void* parameter) {
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      lastWiFiConnectedMs = millis();
    } else if (millis() - lastWiFiConnectedMs > 5000) {
      Serial.println("[WiFi Watchdog] Down >5s, forcing full reconnect.");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      applyWiFiRobustness();
      lastWiFiConnectedMs = millis(); // give this cycle time before forcing another
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.print("Firmware version: ");
  Serial.println(FW_VERSION);
  Serial.print("Node ID: ");
  Serial.println(NODE_ID);

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

  connectWiFi();
  fetchSettingsFromPi(); // apply live settings once before the first pass can be detected
  startEspNow();

  // Spawn the WiFi reconnect watchdog AFTER the initial blocking connect,
  // so it doesn't fight connectWiFi()'s own retry loop during startup.
  xTaskCreatePinnedToCore(
      wifiWatchdogTask,
      "WiFi_Watchdog",
      4096,
      nullptr,
      1,
      nullptr,
      0
  );
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