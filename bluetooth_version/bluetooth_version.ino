/*
 * Sense — Potentiostat + BLE Server for ESP32-S3
 * ────────────────────────────────────────────────────────────────
 * Pin mapping:
 *   GPIO1   (PWM)   → DAC-like output to potentiostat circuit
 *   GPIO34  (ADC)   → Current measurement
 *   GPIO35  (ADC)   → Voltage readback
 *
 * BLE commands (sent as text lines ending in \n):
 *   POST /ca    → run Chronoamperometry
 *                 streams every data point live as:
 *                   {"i":0,"t":0.204,"c":1.23}\n
 *                 then sends summary JSON when complete:
 *                   {"status":"ok","type":"ca","points":50,...}\n
 *   POST /abort → abort a running scan mid-way
 *
 * Serial monitor output during scan:
 *   [CA] i=0  t=0.204s  current=1.230 uA  (phase=eq)
 *   [CA] i=1  t=0.408s  current=1.245 uA  (phase=eq)
 *   ...
 *   [CA] step voltage applied
 *   [CA] i=5  t=2.204s  current=3.120 uA  (phase=meas)
 *   ...
 *   [CA] done  points=50  peak=4.21uA  final=2.87uA
 * ────────────────────────────────────────────────────────────────
 */

#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── BLE settings ─────────────────────────────────────────────────
const char* BT_DEVICE_NAME = "Sense-ESP32";

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic* pTxCharacteristic = nullptr;
bool   deviceConnected  = false;
String bleCommandBuffer = "";

// ── Pin definitions ──────────────────────────────────────────────
#define DAC_PIN     A1
#define CURRENT_PIN A0
#define VOLTAGE_PIN A2

// ── PWM output ───────────────────────────────────────────────────
void dacOut(int val10bit) {
  ledcWrite(DAC_PIN, constrain(val10bit, 0, 1023));
}

// ── CA parameters  (match original 'P' command fields) ───────────
float Veq          = 0.00;  // equilibration voltage (V)
int   teq          = 2;     // equilibration time (s)   — int like original
float Vstart       = 0.00;  // step voltage = V_start after equilibration
int   CATime       = 10;    // measurement duration (s)
int   CADataPoints = 50;    // total points across BOTH phases

// Derived DAC integers (recomputed before each scan)
int V_eq    = 0;
int V_start = 0;

// ── CA storage ───────────────────────────────────────────────────
#define CA_MAX_POINTS 1000
float         CAIData[CA_MAX_POINTS];   // current (µA-scale)
float         CAtData[CA_MAX_POINTS];   // time (s from CAStartTime)
unsigned long CAPointNo      = 0;       // total points collected so far
bool          caReady        = false;
bool          caMeasuring    = false;
bool          bAborted       = false;

// Summary values computed after scan
float caPeakCurrent  = 0.0;
float caFinalCurrent = 0.0;

// ── Timing ───────────────────────────────────────────────────────
unsigned long CAStartTime;
unsigned long CATotalTime;       // (teq + CATime) * 1000  ms
unsigned long MyCAPointWaitTime;

// ── BLE helpers ──────────────────────────────────────────────────
void btSendLine(const String& s) {
  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue((s + "\n").c_str());
    pTxCharacteristic->notify();
    delay(5);   // give the stack a moment between notifications
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override    { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

class RXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rx = pCharacteristic->getValue().c_str();
    if (rx.length() > 0) bleCommandBuffer += rx;
  }
};

// ── Stream one data point over BLE + Serial ───────────────────────
//   Format: {"i":N,"t":T.TTT,"c":C.CCC}
//   i = global point index (0-based, never resets mid-scan)
//   t = seconds since CAStartTime
//   c = current in µA-scale
//   phase = "eq" or "meas" (Serial only, keeps BLE payload small)
void emitPoint(unsigned long idx, float t, float current, const char* phase) {
  // ── Serial ─────────────────────────────────────────────────────
  Serial.print("[CA] i=");     Serial.print(idx);
  Serial.print("  t=");        Serial.print(t, 3);  Serial.print("s");
  Serial.print("  current=");  Serial.print(current, 3); Serial.print(" uA");
  Serial.print("  (phase=");   Serial.print(phase); Serial.println(")");

  // ── BLE ────────────────────────────────────────────────────────
  // Use a compact JSON so it fits well within the ~20-byte MTU
  // (this is ~30 chars; BLE stack will fragment automatically,
  //  and the browser reassembles via the newline-buffer)
  String msg = "{\"i\":";
  msg += idx;
  msg += ",\"t\":";
  msg += String(t, 3);
  msg += ",\"c\":";
  msg += String(current, 3);
  msg += "}";
  btSendLine(msg);
}

// ── Chronoamperometry — faithful to original two-phase logic ─────
//
// Phase 1 (equilibration):  dacOut(V_eq), collect points 0..(n_eq-1)
//                            over teq seconds
// Phase 2 (measurement):    dacOut(V_start), collect points n_eq..(CADataPoints-1)
//                            over CATime seconds
//
// Both phases share a single CAStartTime origin and CAPointNo counter,
// exactly matching the original code.  Points are spaced evenly across
// CATotalTime = (teq + CATime) seconds.
//
void runCA() {
  caMeasuring = true;
  caReady     = false;
  bAborted    = false;
  CAPointNo   = 0;

  // Recompute DAC integers from float parameters
  V_eq    = 512 + (int)(Veq    * 312);
  V_start = 512 + (int)(Vstart * 312);

  CATotalTime = (unsigned long)((teq + CATime) * 1000);  // ms

  caPeakCurrent = -9999.0;

  // ── Apply equilibration voltage ──────────────────────────────
  dacOut(V_eq);
  CAStartTime = millis();
  unsigned long phaseEndTime = CAStartTime + (unsigned long)(teq * 1000);

  Serial.println("[CA] starting — equilibration phase");

  // ── Phase 1: equilibration ────────────────────────────────────
  // Collect data points while millis() < CAStartTime + teq*1000.
  // CAPointNo increments first (matching original), then we wait
  // for the scheduled sample time before reading the ADC.
  while ((millis() < phaseEndTime) && !bAborted) {
    delay(1);
    CAPointNo++;

    // Check for abort over BLE
    if (bleCommandBuffer.indexOf("POST /abort") >= 0) bAborted = true;

    MyCAPointWaitTime = CAStartTime + CAPointNo * CATotalTime / CADataPoints;
    while ((millis() < MyCAPointWaitTime) && !bAborted) delay(1);

    if (!bAborted && CAPointNo < CA_MAX_POINTS) {
      float raw     = analogRead(CURRENT_PIN) * 3.3 / 4096;
      float current = 100.0 * (raw - 1.65);

      CAtData[CAPointNo] = (millis() - CAStartTime) / 1000.0;
      CAIData[CAPointNo] = current;

      emitPoint(CAPointNo, CAtData[CAPointNo], current, "eq");

      if (current > caPeakCurrent) caPeakCurrent = current;
    }
  }

  // ── Apply step voltage ────────────────────────────────────────
  if (!bAborted) {
    dacOut(V_start);
    Serial.println("[CA] step voltage applied — measurement phase");
  }

  // ── Phase 2: measurement ──────────────────────────────────────
  unsigned long scanEndTime = CAStartTime + CATotalTime;

  while ((millis() < scanEndTime) && !bAborted) {
    delay(1);
    CAPointNo++;

    if (bleCommandBuffer.indexOf("POST /abort") >= 0) bAborted = true;

    MyCAPointWaitTime = CAStartTime + CAPointNo * CATotalTime / CADataPoints;
    while ((millis() < MyCAPointWaitTime) && !bAborted) delay(1);

    if (!bAborted && CAPointNo < CA_MAX_POINTS) {
      float raw     = analogRead(CURRENT_PIN) * 3.3 / 4096;
      float current = 100.0 * (raw - 1.65);

      CAtData[CAPointNo] = (millis() - CAStartTime) / 1000.0;
      CAIData[CAPointNo] = current;

      emitPoint(CAPointNo, CAtData[CAPointNo], current, "meas");

      if (current > caPeakCurrent) caPeakCurrent = current;
    }
  }

  // ── Return to equilibration voltage ──────────────────────────
  dacOut(V_eq);

  caFinalCurrent = (CAPointNo > 0) ? CAIData[CAPointNo] : 0.0;

  Serial.print("[CA] done  points="); Serial.print(CAPointNo);
  Serial.print("  peak=");  Serial.print(caPeakCurrent, 2);  Serial.print("uA");
  Serial.print("  final="); Serial.print(caFinalCurrent, 2); Serial.println("uA");

  caReady     = true;
  caMeasuring = false;
}

// ── BLE: POST /ca ─────────────────────────────────────────────────
void handleCA() {
  if (caMeasuring) {
    btSendLine("{\"error\":\"busy\"}");
    return;
  }

  // Signal scan start to the browser so it can open the live chart
  btSendLine("{\"type\":\"ca_start\",\"total_points\":" + String(CADataPoints) +
             ",\"teq\":" + String(teq) +
             ",\"duration_s\":" + String(CATime) + "}");

  runCA();

  // Send summary when complete (or aborted)
  StaticJsonDocument<256> doc;
  doc["status"]      = bAborted ? "aborted" : "ok";
  doc["type"]        = "ca";
  doc["points"]      = (int)CAPointNo;
  doc["duration_s"]  = CATime;
  doc["peak_uA"]     = round(caPeakCurrent  * 100) / 100.0;
  doc["final_uA"]    = round(caFinalCurrent * 100) / 100.0;

  String json;
  serializeJson(doc, json);
  btSendLine(json);
}

// ── BLE: POST /abort ──────────────────────────────────────────────
void handleAbort() {
  if (caMeasuring) {
    bAborted = true;
    btSendLine("{\"status\":\"aborting\"}");
  } else {
    btSendLine("{\"status\":\"not_running\"}");
  }
}

// ── BLE command router ────────────────────────────────────────────
void handleBluetooth() {
  int newlineIndex;
  while ((newlineIndex = bleCommandBuffer.indexOf('\n')) >= 0) {
    String cmd = bleCommandBuffer.substring(0, newlineIndex);
    bleCommandBuffer.remove(0, newlineIndex + 1);
    cmd.trim();
    if (cmd.length() == 0) continue;

    if      (cmd == "POST /ca")    handleCA();
    else if (cmd == "POST /abort") handleAbort();
    else if (cmd.startsWith("OPTIONS")) btSendLine("OK");
    else    btSendLine("{\"error\":\"unknown_command\"}");
  }
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  ledcAttach(DAC_PIN, 20000, 10);
  Serial.begin(115200);
  Serial.println("Sense ESP32-S3 booting...");
  analogReadResolution(12);

  BLEDevice::init(BT_DEVICE_NAME);
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new RXCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE ready — Sense-ESP32");
  Serial.println("Commands: POST /ca | POST /abort");
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  handleBluetooth();
}
