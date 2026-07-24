#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Bump this whenever this file changes and reflash the board. This file
// never talks to the Pi over HTTP (ESP-NOW broadcast only), so unlike the
// checkpoint firmware this version can't be reported automatically -
// check it via the serial monitor if you need to confirm what's flashed.
const char* FW_VERSION = "1.1.0";

// Must match the WiFi network used by the receiver gateway.
const char* CHECKPOINT_WIFI_SSID = "superstuudio";

const uint8_t DRONE_ID = 1;
const unsigned long BEACON_INTERVAL_MS = 10; // High frequency (100 Hz) for high-speed gate crossing

// Beacon transmit power, in quarter-dBm (valid range 8-84, i.e. 2-21 dBm).
// This is the COARSE control over how large each gate's detection zone is:
// lower power makes RSSI drop off over a much shorter distance, which keeps
// gates from triggering from far away. That matters most airborne, where
// clear line-of-sight keeps the signal strong far past the gate.
// Pair this with the per-node "Enter RSSI" on the Settings page, which is
// the FINE control and can be tuned live without reflashing anything.
//   32 = 8 dBm (previous, large zone)   20 = 5 dBm (tighter)   8 = 2 dBm (tightest)
const int8_t TX_POWER_QUARTER_DBM = 20;

const uint16_t BEACON_MAGIC = 0x4B47;
const uint8_t BEACON_VERSION = 1;
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#pragma pack(push, 1)
struct DroneBeacon {
  uint16_t magic;
  uint8_t version;
  uint8_t drone_id;
  uint32_t boot_id;
  uint32_t sequence;
};
#pragma pack(pop)

uint8_t radioChannel = 0;
uint32_t bootId = 0;
uint32_t sequenceNumber = 0;
unsigned long lastBeaconMs = 0;

uint8_t findCheckpointChannel() {
  Serial.print("Scanning for ");
  Serial.println(CHECKPOINT_WIFI_SSID);

  while (true) {
    const int networkCount = WiFi.scanNetworks(false, true);
    for (int index = 0; index < networkCount; index++) {
      if (WiFi.SSID(index) == CHECKPOINT_WIFI_SSID) {
        const uint8_t channel = static_cast<uint8_t>(WiFi.channel(index));
        WiFi.scanDelete();
        return channel;
      }
    }

    WiFi.scanDelete();
    Serial.println("Checkpoint WiFi not found; retrying...");
    delay(2000);
  }
}

void addBroadcastPeer() {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = radioChannel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("FATAL: Could not add ESP-NOW broadcast peer");
    while (true) {
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.print("Firmware version: ");
  Serial.println(FW_VERSION);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  radioChannel = findCheckpointChannel();
  esp_wifi_set_channel(radioChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(TX_POWER_QUARTER_DBM);

  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: ESP-NOW initialization failed");
    while (true) {
      delay(1000);
    }
  }

  addBroadcastPeer();
  bootId = esp_random();

  Serial.print("XIAO Beacon ready. ESP-NOW channel: ");
  Serial.println(radioChannel);
}

void loop() {
  const unsigned long now = millis();
  if (now - lastBeaconMs < BEACON_INTERVAL_MS) {
    delay(1);
    return;
  }

  lastBeaconMs = now;
  const DroneBeacon beacon{
      BEACON_MAGIC,
      BEACON_VERSION,
      DRONE_ID,
      bootId,
      sequenceNumber++
  };

  const esp_err_t result = esp_now_send(
      BROADCAST_MAC,
      reinterpret_cast<const uint8_t*>(&beacon),
      sizeof(beacon)
  );

  if (result != ESP_OK) {
    Serial.printf("ESP-NOW send failed: %d\n", static_cast<int>(result));
  }
}