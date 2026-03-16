/*
 * Sense — Combined Potentiostat + BLE Server for ESP32-S3
 * ────────────────────────────────────────────────────────────────
 * Pin mapping:
 *   GPIO1   (PWM)   → DAC-like output to potentiostat circuit
 *   GPIO34  (ADC)   → Current measurement (replaces A1)
 *   GPIO35  (ADC)   → Voltage readback    (replaces A2)
 *
 * NOTE:
 * - ESP32-S3 does not support BluetoothSerial (Classic BT SPP),
 *   so this version uses BLE instead.
 * - ESP32-S3 also does not use dacWrite() like original ESP32,
 *   so PWM is used instead for analog-like output.
 *
 * BLE commands (same routes, sent as text lines ending in \n):
 *   GET /data
 *   POST /run
 *   GET /raw
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
bool deviceConnected = false;
String bleCommandBuffer = "";

// ── Pin definitions ──────────────────────────────────────────────
#define DAC_PIN     1     // PWM-capable pin on XIAO ESP32S3
#define CURRENT_PIN 34    // ADC - current measurement (was A1)
#define VOLTAGE_PIN 35    // ADC - voltage readback    (was A2)

// ── Helper: PWM output instead of dacWrite() ────────────────────
void dacOut(int val10bit) {
  int pwm = constrain(val10bit, 0, 1023);
  ledcWrite(DAC_PIN, pwm);
}

// ── Potentiostat parameters (same as original) ───────────────────
int dacVolt = 0;
float temp = 0;
int CycleNumber = 1;
int HalfCycle = 0;
bool bAborted = false;
float Veq = 0;
int V_eq = 512 + Veq * 312;
float teq = 0;
unsigned long myTime, myTime2, myTime3;
int EAppplyAfter = 0;
float Vafter = 0;
int V_after = 512 + Vafter * 312;
int CATime = 10;
int CADataPoints = 10;
unsigned long CAStartTime, CATotalTime, MyCAPointWaitTime;
float CAIData[1000];
float CAtData[1000];
unsigned long CAPointNo;

float StepE = 2;
int i_StepE = 2;
float PulseAmplitude = 10;
int i_PulseAmplitude = 10;
unsigned long i_PulsePeriod = 200;
unsigned long i_PulseWidth = 50;
unsigned long i_SamplingPeriod = 2;
int HalfCycle_DPV = 0;
int NoEReadBack = 0;

float current_base = 0;
float current = 0;
float volt = 0;

float Vstart = -1.20;
float Vstop  = +1.00;
float Srate  = 0.100;

int V_start = 512 + Vstart * 312;
int V_stop  = 512 + Vstop  * 312;
int Cycle_Number = 1;
int t = 0;

// ── Peak detection & calibration ─────────────────────────────────
const float UREA_PEAK_MIN     = +0.10;
const float UREA_PEAK_MAX     = +0.35;
const float GLUCOSE_PEAK_MIN  = +0.40;
const float GLUCOSE_PEAK_MAX  = +0.70;

// Calibration: concentration (mg/dL) = slope * peakCurrent + intercept
const float UREA_SLOPE        = 8.50;
const float UREA_INTERCEPT    = 2.00;
const float GLUCOSE_SLOPE     = 2.80;
const float GLUCOSE_INTERCEPT = 10.00;

// ── DPV scan storage ─────────────────────────────────────────────
struct DataPoint {
  float voltage;
  float current;
};

DataPoint dpvData[2000];
int dpvCount = 0;
float lastUrea    = 0;
float lastGlucose = 0;
bool measurementReady = false;
bool measuring = false;

// ── BLE helpers ──────────────────────────────────────────────────
void btSendLine(const String& s) {
  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue((s + "\n").c_str());
    pTxCharacteristic->notify();
    delay(5);
  }
}

void btSend(const String& s) {
  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue(s.c_str());
    pTxCharacteristic->notify();
    delay(5);
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

class RXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rx = pCharacteristic->getValue().c_str();
    if (rx.length() > 0) {
      bleCommandBuffer += rx;
    }
  }
};

// ── Find peak current in a voltage window ────────────────────────
float findPeak(float vMin, float vMax) {
  float peak = -9999;
  for (int i = 0; i < dpvCount; i++) {
    if (dpvData[i].voltage >= vMin && dpvData[i].voltage <= vMax) {
      if (dpvData[i].current > peak) peak = dpvData[i].current;
    }
  }
  return (peak == -9999) ? 0 : peak;
}

float peakToConc(float peak, float slope, float intercept) {
  return max(0.0f, slope * peak + intercept);
}

// ── Run a DPV measurement and extract concentrations ─────────────
void runDPV() {
  measuring = true;
  dpvCount  = 0;

  // ── Set DPV parameters ────────────────────────────────────────
  Vstart = -0.20;
  Vstop = +0.80;
  V_start = 512 + Vstart * 312;
  V_stop  = 512 + Vstop  * 312;
  Cycle_Number = 1;
  Srate = 0.100;
  Veq = 0.00;
  teq = 2;
  V_eq = 512 + Veq * 312;
  Vafter = 0.00;
  V_after = 512 + Vafter * 312;
  EAppplyAfter = 0;
  HalfCycle_DPV = 1;
  NoEReadBack = 1;
  StepE = 5;
  i_StepE = (StepE / 3300 * 1024) + 0.5;
  PulseAmplitude = 25;
  i_PulseAmplitude = (PulseAmplitude / 3300 * 1024) + 0.5;
  i_PulsePeriod    = 200;
  i_PulseWidth     = 50;
  i_SamplingPeriod = 2;

  // ── Equilibration ─────────────────────────────────────────────
  dacOut(V_eq);
  myTime = millis() + teq * 1000;
  while (millis() < myTime) delay(1);

  // ── Forward DPV scan ──────────────────────────────────────────
  for (dacVolt = V_start; dacVolt <= V_stop; dacVolt = dacVolt + i_StepE) {
    if (bAborted) break;

    dacOut(dacVolt);

    myTime = millis() + i_PulsePeriod - i_PulseWidth - i_SamplingPeriod;
    while (millis() < myTime) delay(1);

    current_base = analogRead(CURRENT_PIN) * 3.3 / 4096;

    myTime += i_SamplingPeriod;
    while (millis() < myTime) delay(1);

    dacOut(dacVolt + i_PulseAmplitude);

    myTime += i_PulseWidth - i_SamplingPeriod;
    while (millis() < myTime) delay(1);

    float cur = analogRead(CURRENT_PIN) * 3.3 / 4096;
    float cur_final = 100 * (cur - current_base);

    float v = (float)(dacVolt - 512) / 1024 * 3.3;

    if (dpvCount < 2000) {
      dpvData[dpvCount++] = {v, cur_final};
    }

    myTime += i_SamplingPeriod;
    while (millis() < myTime) delay(1);
  }

  // ── Extract concentrations ────────────────────────────────────
  float ureaPeak    = findPeak(UREA_PEAK_MIN, UREA_PEAK_MAX);
  float glucosePeak = findPeak(GLUCOSE_PEAK_MIN, GLUCOSE_PEAK_MAX);

  lastUrea    = round(peakToConc(ureaPeak, UREA_SLOPE, UREA_INTERCEPT) * 10) / 10.0;
  lastGlucose = round(peakToConc(glucosePeak, GLUCOSE_SLOPE, GLUCOSE_INTERCEPT) * 10) / 10.0;

  measurementReady = true;
  measuring = false;
}

// ── BLE: GET /data ───────────────────────────────────────────────
void handleData() {
  if (!measurementReady) runDPV();

  StaticJsonDocument<128> doc;
  doc["urea"]    = lastUrea;
  doc["glucose"] = lastGlucose;
  doc["points"]  = dpvCount;

  String json;
  serializeJson(doc, json);
  btSendLine(json);
}

// ── BLE: POST /run ───────────────────────────────────────────────
void handleRun() {
  if (measuring) {
    btSendLine("{\"error\":\"busy\"}");
    return;
  }

  runDPV();

  StaticJsonDocument<128> doc;
  doc["status"]  = "ok";
  doc["urea"]    = lastUrea;
  doc["glucose"] = lastGlucose;

  String json;
  serializeJson(doc, json);
  btSendLine(json);
}

// ── BLE: GET /raw ────────────────────────────────────────────────
void handleRaw() {
  btSend("voltage_V,current_uA\n");
  for (int i = 0; i < dpvCount; i++) {
    btSend(String(dpvData[i].voltage, 4) + "," +
           String(dpvData[i].current, 4) + "\n");
  }
}

// ── BLE command handler ──────────────────────────────────────────
void handleBluetooth() {
  int newlineIndex;

  while ((newlineIndex = bleCommandBuffer.indexOf('\n')) >= 0) {
    String cmd = bleCommandBuffer.substring(0, newlineIndex);
    bleCommandBuffer.remove(0, newlineIndex + 1);

    cmd.trim();
    if (cmd.length() == 0) continue;

    if (cmd == "GET /data") {
      handleData();
    } else if (cmd == "POST /run") {
      handleRun();
    } else if (cmd == "GET /raw") {
      handleRaw();
    } else if (cmd == "OPTIONS /data" ||
               cmd == "OPTIONS /run"  ||
               cmd == "OPTIONS /raw") {
      btSendLine("OK");
    } else {
      btSendLine("{\"error\":\"unknown_command\"}");
    }
  }
}

// ── Serial command handler (keeps original Arduino serial API) ───
void handleSerial() {
  if (Serial.available() > 0) {
    String myString = Serial.readString();
    char ch = myString.charAt(0);

    if (ch == 'P') {
      myString = myString.substring(2, 100);
      int n;
      String s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18;
      #define NEXT(sx) n=myString.indexOf(","); sx=myString.substring(0,n); myString.remove(0,n+1);
      NEXT(s1) NEXT(s2) NEXT(s3) NEXT(s4) NEXT(s5) NEXT(s6) NEXT(s7) NEXT(s8) NEXT(s9)
      NEXT(s10) NEXT(s11) NEXT(s12) NEXT(s13) NEXT(s14) NEXT(s15) NEXT(s16) NEXT(s17)
      n = myString.indexOf(",");
      s18 = myString.substring(0, n);

      Vstart = s1.toFloat();
      Vstop = s2.toFloat();
      V_start = 512 + Vstart * 312;
      V_stop = 512 + Vstop * 312;
      Cycle_Number = s3.toInt();
      Srate = s4.toFloat();
      Veq = s5.toFloat();
      teq = s6.toInt();
      V_eq = 512 + Veq * 312;
      Vafter = s7.toFloat();
      V_after = 512 + Vafter * 312;
      EAppplyAfter = s8.toInt();
      HalfCycle = s9.toInt();
      CATime = s10.toInt();
      CADataPoints = s11.toInt();
      StepE = s12.toFloat();
      i_StepE = (StepE / 3300 * 1024) + 0.5;
      PulseAmplitude = s13.toFloat();
      i_PulseAmplitude = (PulseAmplitude / 3300 * 1024) + 0.5;
      i_PulsePeriod = s14.toInt();
      i_PulseWidth = s15.toInt();
      i_SamplingPeriod = s16.toInt();
      HalfCycle_DPV = s17.toInt();
      NoEReadBack = s18.toInt();
    }

    if (ch == 'S') {
      bAborted = false;
      if (HalfCycle == 1) Cycle_Number = 1;

      dacOut(V_eq);
      myTime = millis() + teq * 1000;
      while (millis() < myTime && !bAborted) {
        delay(1);
        if (Serial.available()) {
          myString = Serial.readString();
          if (myString.charAt(0) == '!') bAborted = true;
        }
      }

      for (CycleNumber = 1; CycleNumber <= Cycle_Number; CycleNumber++) {
        if (!bAborted) {
          delay(10);
          Serial.println("*");
          myTime = millis();

          for (dacVolt = V_start; dacVolt <= V_stop; dacVolt++) {
            if (!bAborted) {
              dacOut(dacVolt);
              float cur = analogRead(CURRENT_PIN) * 3.3 / 4096;

              if (NoEReadBack == 1) {
                temp = (dacVolt - 512) / 1024.0 * 3.3;
                Serial.print(temp, 4);
              } else {
                float v = analogRead(VOLTAGE_PIN) * (-3.3) / 4096;
                Serial.print(1 * (v + 1.65), 4);
              }

              Serial.print(",");
              Serial.print(100 * (cur - 1.65), 4);
              Serial.print(",");
              Serial.println();

              if (Serial.available()) {
                myString = Serial.readString();
                if (myString.charAt(0) == '!') bAborted = true;
              }

              if (!bAborted) {
                temp = dacVolt - V_start + 1;
                temp = 1000 * temp / 1024 * 3.3 / Srate;
                myTime2 = myTime + temp + 0.5;
                while (millis() < myTime2 && !bAborted) {
                  delay(1);
                  if (Serial.available()) {
                    myString = Serial.readString();
                    if (myString.charAt(0) == '!') bAborted = true;
                  }
                }
              }
            }
          }

          if (!bAborted && HalfCycle == 0) {
            myTime = millis();
            for (dacVolt = V_stop; dacVolt >= V_start; dacVolt--) {
              if (!bAborted) {
                dacOut(dacVolt);
                float cur = analogRead(CURRENT_PIN) * 3.3 / 4096;

                if (NoEReadBack == 1) {
                  temp = (dacVolt - 512) / 1024.0 * 3.3;
                  Serial.print(temp, 4);
                } else {
                  float v = analogRead(VOLTAGE_PIN) * (-3.3) / 4096;
                  Serial.print(1 * (v + 1.65), 4);
                }

                Serial.print(",");
                Serial.print(100 * (cur - 1.65), 4);
                Serial.print(",");
                Serial.println();

                if (Serial.available()) {
                  myString = Serial.readString();
                  if (myString.charAt(0) == '!') bAborted = true;
                }

                if (!bAborted) {
                  temp = V_stop - dacVolt + 1;
                  temp = 1000 * temp / 1024 * 3.3 / Srate;
                  myTime2 = myTime + temp + 0.5;
                  while (millis() < myTime2 && !bAborted) {
                    delay(1);
                    if (Serial.available()) {
                      myString = Serial.readString();
                      if (myString.charAt(0) == '!') bAborted = true;
                    }
                  }
                }
              }
            }
          }
        }
      }

      if (!bAborted && EAppplyAfter == 1) dacOut(V_after);
      delay(10);
      Serial.println('@');
    }

    if (ch == 'C') {
      bAborted = false;
      CAPointNo = 0;
      dacOut(V_eq);
      CATotalTime = teq + CATime;
      CAStartTime = millis();
      myTime = CAStartTime + teq * 1000;

      while (millis() < myTime && !bAborted) {
        delay(1);
        CAPointNo++;

        if (Serial.available()) {
          myString = Serial.readString();
          if (myString.charAt(0) == '!') bAborted = true;
        }

        MyCAPointWaitTime = CAStartTime + CAPointNo * (CATotalTime * 1000) / CADataPoints;
        while (millis() < MyCAPointWaitTime && !bAborted) delay(1);

        if (!bAborted) {
          float cur = analogRead(CURRENT_PIN) * 3.3 / 4096;
          CAIData[CAPointNo] = 100 * (cur - 1.65);
          CAtData[CAPointNo] = (millis() - CAStartTime) / 1000.0;
        }
      }

      if (!bAborted) dacOut(V_start);
      myTime = CAStartTime + CATotalTime * 1000;

      while (millis() < myTime && !bAborted) {
        delay(1);
        CAPointNo++;

        if (Serial.available()) {
          myString = Serial.readString();
          if (myString.charAt(0) == '!') bAborted = true;
        }

        MyCAPointWaitTime = CAStartTime + CAPointNo * (CATotalTime * 1000) / CADataPoints;
        while (millis() < MyCAPointWaitTime && !bAborted) delay(1);

        if (!bAborted) {
          float cur = analogRead(CURRENT_PIN) * 3.3 / 4096;
          CAIData[CAPointNo] = 100 * (cur - 1.65);
          CAtData[CAPointNo] = (millis() - CAStartTime) / 1000.0;
        }
      }

      for (int i = 1; i <= (int)CAPointNo; i++) {
        Serial.print(CAtData[i], 3);
        Serial.print(",");
        Serial.print(CAIData[i], 3);
        Serial.print(",");
        Serial.println();
      }

      delay(10);
      Serial.println("*");
      delay(10);
      Serial.println('%');
    }

    if (ch == 'D') {
      bAborted = false;
      if (HalfCycle_DPV == 1) Cycle_Number = 1;

      dacOut(V_eq);
      myTime = millis() + teq * 1000;
      while (millis() < myTime && !bAborted) {
        delay(1);
        if (Serial.available()) {
          myString = Serial.readString();
          if (myString.charAt(0) == '!') bAborted = true;
        }
      }

      for (CycleNumber = 1; CycleNumber <= Cycle_Number; CycleNumber++) {
        if (!bAborted) {
          delay(10);
          Serial.println("*");

          for (dacVolt = V_start; dacVolt <= V_stop; dacVolt = dacVolt + i_StepE) {
            if (!bAborted) {
              dacOut(dacVolt);

              myTime = millis() + i_PulsePeriod - i_PulseWidth - i_SamplingPeriod;
              while (millis() < myTime && !bAborted) {
                delay(1);
                if (Serial.available()) {
                  myString = Serial.readString();
                  if (myString.charAt(0) == '!') bAborted = true;
                }
              }

              if (!bAborted) current_base = analogRead(CURRENT_PIN) * 3.3 / 4096;

              if (Serial.available()) {
                myString = Serial.readString();
                if (myString.charAt(0) == '!') bAborted = true;
              }

              myTime += i_SamplingPeriod;
              while (millis() < myTime && !bAborted) {
                delay(1);
                if (Serial.available()) {
                  myString = Serial.readString();
                  if (myString.charAt(0) == '!') bAborted = true;
                }
              }

              if (!bAborted) {
                dacOut(dacVolt + i_PulseAmplitude);
                myTime += i_PulseWidth - i_SamplingPeriod;
                while (millis() < myTime && !bAborted) {
                  delay(1);
                  if (Serial.available()) {
                    myString = Serial.readString();
                    if (myString.charAt(0) == '!') bAborted = true;
                  }
                }
              }

              if (!bAborted) {
                float cur = analogRead(CURRENT_PIN) * 3.3 / 4096;
                float cur_final = 100 * (cur - current_base);

                if (NoEReadBack == 1) {
                  temp = (dacVolt - 512) / 1024.0 * 3.3;
                  Serial.print(temp, 4);
                } else {
                  float v = analogRead(VOLTAGE_PIN) * (-3.3) / 4096;
                  Serial.print(1 * (v + 1.65), 4);
                }

                Serial.print(",");
                Serial.print(cur_final);
                Serial.print(",");
                Serial.println();

                if (Serial.available()) {
                  myString = Serial.readString();
                  if (myString.charAt(0) == '!') bAborted = true;
                }
              }

              myTime += i_SamplingPeriod;
              while (millis() < myTime && !bAborted) {
                delay(1);
                if (Serial.available()) {
                  myString = Serial.readString();
                  if (myString.charAt(0) == '!') bAborted = true;
                }
              }
            }
          }

          if (!bAborted && HalfCycle_DPV == 0) {
            for (dacVolt = V_stop; dacVolt >= V_start; dacVolt = dacVolt - i_StepE) {
              if (!bAborted) {
                dacOut(dacVolt);

                myTime = millis() + i_PulsePeriod - i_PulseWidth - i_SamplingPeriod;
                while (millis() < myTime && !bAborted) {
                  delay(1);
                  if (Serial.available()) {
                    myString = Serial.readString();
                    if (myString.charAt(0) == '!') bAborted = true;
                  }
                }

                if (!bAborted) current_base = analogRead(CURRENT_PIN) * 3.3 / 4096;

                if (Serial.available()) {
                  myString = Serial.readString();
                  if (myString.charAt(0) == '!') bAborted = true;
                }

                myTime += i_SamplingPeriod;
                while (millis() < myTime && !bAborted) {
                  delay(1);
                  if (Serial.available()) {
                    myString = Serial.readString();
                    if (myString.charAt(0) == '!') bAborted = true;
                  }
                }

                if (!bAborted) {
                  dacOut(dacVolt - i_PulseAmplitude);
                  myTime += i_PulseWidth - i_SamplingPeriod;
                  while (millis() < myTime && !bAborted) {
                    delay(1);
                    if (Serial.available()) {
                      myString = Serial.readString();
                      if (myString.charAt(0) == '!') bAborted = true;
                    }
                  }
                }

                if (!bAborted) {
                  float cur = analogRead(CURRENT_PIN) * 3.3 / 4096;
                  float cur_final = 100 * (cur - current_base);

                  if (NoEReadBack == 1) {
                    temp = (dacVolt - 512) / 1024.0 * 3.3;
                    Serial.print(temp, 4);
                  } else {
                    float v = analogRead(VOLTAGE_PIN) * (-3.3) / 4096;
                    Serial.print(1 * (v + 1.65), 4);
                  }

                  Serial.print(",");
                  Serial.print(cur_final);
                  Serial.print(",");
                  Serial.println();

                  if (Serial.available()) {
                    myString = Serial.readString();
                    if (myString.charAt(0) == '!') bAborted = true;
                  }
                }

                myTime += i_SamplingPeriod;
                while (millis() < myTime && !bAborted) {
                  delay(1);
                  if (Serial.available()) {
                    myString = Serial.readString();
                    if (myString.charAt(0) == '!') bAborted = true;
                  }
                }
              }
            }
          }
        }
      }

      if (!bAborted && EAppplyAfter == 1) dacOut(V_after);
      delay(10);
      Serial.println('$');
    }

    delay(10);
    Serial.println('#');
  }
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  ledcAttach(DAC_PIN, 20000, 10);

  Serial.begin(115200);
  Serial.println("ESP32-S3 booting...");
  Serial.println("Serial is working");
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

  Serial.println("BLE started");
  Serial.println("Device name: Sense-ESP32");
  Serial.println("Send BLE commands:");
  Serial.println("GET /data");
  Serial.println("POST /run");
  Serial.println("GET /raw");
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  handleBluetooth();
  handleSerial();
}