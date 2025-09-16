// Scanner + Data Logger (ESP32)
// - Scans Eddystone-TLM using NimBLE-Arduino
// - Stores readings in per-MAC linked lists (dynamic memory using new/delete)
// - Every minute: sends CSV lines over Serial2, ends with "END\n"
// - Waits for "ACK\n" to free memory, otherwise keeps data for retry
// - LED2: blinking normally, SOLID when stored readings exceed threshold

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <FastLED.h>
#include "NimBLEEddystoneTLM.h"

// ===== Config =====
#define SERIAL_BAUD      115200
#define SERIAL2_BAUD     115200
#define SERIAL2_TX_PIN   17   // adjust as needed
#define SERIAL2_RX_PIN   16

#define LED_PIN          4     // WS2812 LED pin
#define NUM_LEDS         1     // Only one LED
#define LED_BRIGHTNESS   255   // Full brightness
#define LED_TYPE         WS2812
#define COLOR_ORDER      GRB

CRGB leds[NUM_LEDS];

// ===== LED behavior =====
uint32_t ledBlinkInterval = 500;          // ms for blink toggle
unsigned long lastBlink = 0;
bool blinkState = false;

#define STORE_INTERVAL_MS 60000UL           // 1 minute
#define MEMORY_HEAVY_THRESHOLD 200         // if total stored readings > this -> LED solid

const unsigned long ACK_NACK_BLINK_MS = 3000UL; // how long ACK/NACK blink (ms)

// ===== Data structures =====
struct SensorData {
  float temperature;
  uint16_t voltage;
  uint32_t uptime;
  uint32_t advCount;
  int rssi;
  unsigned long timestamp; // millis of record
  SensorData* next;
};

struct SensorNode {
  String mac;
  SensorData* dataHead;
  SensorNode* next;
  SensorNode() : mac(""), dataHead(nullptr), next(nullptr) {}
};

static SensorNode* sensorList = nullptr;

// ===== NimBLE Scan handle =====
NimBLEScan* pBLEScan;

// ===== Utility functions =====
SensorNode* findOrCreateSensor(const String& mac) {
  for (SensorNode* n = sensorList; n; n = n->next) {
    if (n->mac == mac) return n;
  }
  SensorNode* node = new SensorNode();
  node->mac = mac;
  node->dataHead = nullptr;
  node->next = sensorList;
  sensorList = node;
  return node;
}

void addSensorData(SensorNode* sensor, float temp, uint16_t volt,
                   uint32_t uptime, uint32_t advCount, int rssi) {
  SensorData* d = new SensorData();
  d->temperature = temp;
  d->voltage = volt;
  d->uptime = uptime;
  d->advCount = advCount;
  d->rssi = rssi;
  d->timestamp = millis();
  d->next = sensor->dataHead;
  sensor->dataHead = d;
}

size_t totalStoredReadings() {
  size_t cnt = 0;
  for (SensorNode* n = sensorList; n; n = n->next) {
    for (SensorData* d = n->dataHead; d; d = d->next) ++cnt;
  }
  return cnt;
}

// Clear data for all sensors (free memory)
void clearAllData() {
  for (SensorNode* n = sensorList; n; n = n->next) {
    SensorData* d = n->dataHead;
    while (d) {
      SensorData* td = d;
      d = d->next;
      delete td;
    }
    n->dataHead = nullptr;
  }
}

// ===== Serial2 send logic =====
// Sends all data lines via Serial2 in CSV format:
// mac,temp,volt,uptime,advcount,rssi\n
// After sending lines, send "END\n" then wait for "ACK\n" or "NACK\n"
bool sendAllDataAndWaitAck(uint32_t timeoutMs = 2000) {
  // Send lines
  for (SensorNode* n = sensorList; n; n = n->next) {
    SensorData* d = n->dataHead;
    for (SensorData* cur = d; cur; cur = cur->next) {
      Serial2.printf("%s,%.2f,%u,%u,%u,%d\n",
                     n->mac.c_str(),
                     cur->temperature,
                     cur->voltage,
                     cur->uptime,
                     cur->advCount,
                     cur->rssi);
      delay(5); // small spacing to avoid serial overruns (adjust if needed)
    }
  }
  // End marker
  Serial2.print("END\n");

  // Wait for ACK/NACK
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\n') {
        resp.trim();
        if (resp == "ACK") return true;
        if (resp == "NACK") return false;
        resp = "";
      } else {
        resp += c;
      }
    }
    delay(5);
  }
  // timeout -> treat as NACK (no ack)
  return false;
}

// ===== NimBLE callback =====
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    NimBLEUUID eddyUUID = (uint16_t)0xFEAA;
    if (!advertisedDevice->haveServiceUUID() || !advertisedDevice->getServiceUUID().equals(eddyUUID)) return;
    std::string serviceData = advertisedDevice->getServiceData(eddyUUID);
    if (serviceData.empty() || (uint8_t)serviceData[0] != 0x20) return; // not TLM

    NimBLEEddystoneTLM tlm;
    tlm.setData(reinterpret_cast<const uint8_t*>(serviceData.data()), serviceData.length());

    // manual temp parsing (8.8 fixed)
    int rawTemp = ((uint8_t)serviceData[4] << 8) | (uint8_t)serviceData[5];
    float calcTemp = rawTemp / 256.0f;

    String mac = advertisedDevice->getAddress().toString().c_str();
    SensorNode* node = findOrCreateSensor(mac);
    addSensorData(node, calcTemp, tlm.getVolt(), tlm.getTime(), tlm.getCount(), advertisedDevice->getRSSI());
  }
};
static ScanCallbacks scanCallbacks;

// ===== LED handling =====
enum LedMode {
  LED_OFF,
  LED_SCAN,
  LED_SEND,
  LED_ACK,
  LED_NACK,
  LED_MEMORY  // solid red - highest priority
};

LedMode currentLedMode = LED_OFF;
unsigned long modeExpiry = 0;   // if non-zero, mode expires at this millis()
CRGB lastShownColor = CRGB::Black;

void setLedMode(LedMode mode, uint32_t durationMs = 0) {
  // Always update mode and reset blink state/timers
  currentLedMode = mode;
  blinkState = false;
  lastBlink = millis();
  if (durationMs > 0) modeExpiry = millis() + durationMs;
  else modeExpiry = 0;
}

void setLedModeIfDifferent(LedMode mode, uint32_t durationMs = 0) {
  // Avoid resetting timers if same mode is already active and no expiry change
  if (currentLedMode != mode || (durationMs > 0 && modeExpiry == 0)) {
    setLedMode(mode, durationMs);
  }
}

void updateLED() {
  unsigned long now = millis();
  CRGB color = CRGB::Black;

  // If modeExpiry set and expired, clear it (so the main loop can choose new permanent mode)
  if (modeExpiry != 0 && now >= modeExpiry) {
    modeExpiry = 0;
    // keep currentLedMode as-is for this tick; main loop will switch to SCAN/OFF as appropriate
  }

  // Build color depending on mode
  switch (currentLedMode) {
    case LED_MEMORY:
      // Solid red (no blinking)
      color = CRGB::Red;
      break;

    case LED_OFF:
      color = CRGB::Black;
      break;

    case LED_SCAN:
      if (now - lastBlink >= ledBlinkInterval) {
        lastBlink = now;
        blinkState = !blinkState;
      }
      color = blinkState ? CRGB::Green : CRGB::Black;
      break;

    case LED_SEND:
      if (now - lastBlink >= ledBlinkInterval) {
        lastBlink = now;
        blinkState = !blinkState;
      }
      color = blinkState ? CRGB::Blue : CRGB::Black;
      break;

    case LED_ACK:
      if (now - lastBlink >= ledBlinkInterval) {
        lastBlink = now;
        blinkState = !blinkState;
      }
      color = blinkState ? CRGB::Purple : CRGB::Black;
      break;

    case LED_NACK:
      if (now - lastBlink >= ledBlinkInterval) {
        lastBlink = now;
        blinkState = !blinkState;
      }
      color = blinkState ? CRGB::Red : CRGB::Black;
      break;

    default:
      color = CRGB::Black;
      break;
  }

  // Only update the LED strip if color changed (reduces update churn)
  if (!(color == lastShownColor)) {
    leds[0] = color;
    FastLED.show();
    lastShownColor = color;
  }
}

// ===== Setup & loop =====
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  leds[0] = CRGB::Black;
  FastLED.show();

  Serial.println("\n🚀 BLE Scanner + Data Logger (Scanner ESP)");
  NimBLEDevice::init("ESP32-Scanner");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(&scanCallbacks, true); // allow duplicates
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->start(0, false, false);

  lastSendTime = millis();
}

void loop() {
  // Evaluate memory-heavy override (highest priority)
  size_t stored = totalStoredReadings();
  bool memoryExceeded = (stored > MEMORY_HEAVY_THRESHOLD);

  if (memoryExceeded) {
    // force solid red immediately (don't call setLedMode repeatedly if already set)
    setLedModeIfDifferent(LED_MEMORY);
  } else {
    // If a transient mode (ACK/NACK) is active (modeExpiry != 0 and not yet expired),
    // do not override it. Otherwise choose scanning or off.
    if (modeExpiry == 0) {
      // No transient active -> let scanning/idle be reflected
      if (pBLEScan->isScanning()) {
        setLedModeIfDifferent(LED_SCAN);
      } else {
        setLedModeIfDifferent(LED_OFF);
      }
    }
    // If modeExpiry != 0 and not expired, keep current mode until expiry
  }

  // Periodic send
  if (millis() - lastSendTime >= STORE_INTERVAL_MS) {
    lastSendTime = millis();
    Serial.println("Sending stored data via Serial2 ...");

    // Put LED into sending blink (blue)
    setLedMode(LED_SEND); // permanent until we set ACK/NACK transient

    bool ok = sendAllDataAndWaitAck(3000); // 3s timeout for ACK
    if (ok) {
      Serial.println("ACK received -> clearing stored data");
      clearAllData();
      // Blink ACK (purple) for configured duration, then return to scanning/off
      setLedMode(LED_ACK, ACK_NACK_BLINK_MS);
    } else {
      Serial.println("No ACK / NACK -> will retry next minute. Data retained.");
      // Blink NACK (red) for configured duration, then return to scanning/off
      setLedMode(LED_NACK, ACK_NACK_BLINK_MS);
    }
  }

  // Update LED (handles blink timing and modeExpiry)
  updateLED();

  vTaskDelay(20 / portTICK_PERIOD_MS);  // Small delay to allow BLE scan updates
}
