#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>

// ---- NETWORK CONFIGURATION ----
const char* WIFI_SSID = "superstuudio";
const char* WIFI_PASSWORD = "sepikoda";
const char* PI_IP = "192.168.1.213";
const int PI_PORT = 5000;
const char* NODE_ID = "checkpoint-1";

// ---- GATE TIMING & TUNING PARAMETERS ----
const int8_t ENTER_RSSI = -62;           // Threshold to begin tracking a pass
const int8_t EXIT_RSSI = -72;            // Threshold considered outside gate
const uint8_t REQUIRED_WEAK_SAMPLES = 5; // Weak samples required to close pass
const unsigned long PASS_TIMEOUT_MS = 400; // Force-close pass if signal drops completely
const unsigned long EVENT_COOLDOWN_MS = 2000; // Minimum time between valid passes per drone
const unsigned long HEARTBEAT_INTERVAL_MS = 5000; // How often to tell the Pi this node is alive

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
  Serial.print("Connected! ESP32 Gateway IP: ");
  Serial.println(WiFi.localIP());

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