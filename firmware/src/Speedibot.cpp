// ============================================================================
// SpeediBot – Firmware (Refactored for Readability / Portfolio Presentation)
// Target MCU: ESP32-S3-DevKitC (Arduino Framework)
// Author: Aaron Jackson
// Description:
//   Final senior design firmware controlling an autonomous line-following robot
//   with encoder-based PID motor control, QTR reflectance sensors, and optional
//   BLE telemetry for real-time tuning.
//
//   This version is refactored for clarity and presentation in a public GitHub
//   portfolio repository.
// ============================================================================

// Core Arduino framework
#include <Arduino.h>

// Project-specific configuration (pins, constants, BLE UUIDs, etc.)
#include "config.h"

// Framework libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <QTRSensors.h>

// ============================== Globals ====================================
QTRSensors qtr;
uint16_t qtrValues[NUM_SENSORS] = {0};

volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;

float rpm1 = 0.0f, rpm2 = 0.0f;
float pwm1 = 0.0f, pwm2 = 0.0f;
float integral_error1 = 0.0f, integral_error2 = 0.0f;

float targetSpeed_mps = 0.0f;
float targetRPM       = 0.0f;
bool deviceConnected  = false;
bool isMoving         = false;

unsigned long lastLoopMs     = 0;
unsigned long robotStartTime = 0;
float speedSum = 0.0f;
unsigned long sampleCount = 0;

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

// ============================== Helper Functions ===========================

inline void motorForward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

inline void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  pwm1 = pwm2 = 0.0f;
  integral_error1 = integral_error2 = 0.0f;
  isMoving = false;
}

inline void motorSetRaw(uint8_t a, uint8_t b) {
  ledcWrite(PWM_CH_A, a);
  ledcWrite(PWM_CH_B, b);
}

void updateMotorOutputs(float correction) {
  int valA = constrain((int)(pwm1 - correction), 0, 255);
  int valB = constrain((int)(pwm2 + correction), 0, 255);
  motorSetRaw(valA, valB);
  motorForward();
}

int readLinePosition(bool& allOnBlack) {
  int weightedSum = 0, sum = 0;
  allOnBlack = true;
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    int s = digitalRead(SENSOR_PINS[i]);
    if (s != BLACK_THRESHOLD) allOnBlack = false;
    int active = (s == BLACK_THRESHOLD) ? 1 : 0;
    weightedSum += SENSOR_WEIGHTS[i] * active;
    sum += active;
  }
  return (sum > 0) ? (weightedSum / sum) : 0;
}

void computeRPM() {
  static int lastCount1 = 0, lastCount2 = 0;
  int d1 = encoderCount1 - lastCount1;
  int d2 = encoderCount2 - lastCount2;
  lastCount1 = encoderCount1;
  lastCount2 = encoderCount2;
  const float dt_min = (float)LOOP_PERIOD_MS / 60000.0f;
  rpm1 = (d1 / (float)COUNTS_PER_REV) / dt_min;
  rpm2 = (d2 / (float)COUNTS_PER_REV) / dt_min;
}

static inline void clampIntegral(float& i1, float& i2, float rpm1, float rpm2) {
  const float lim = (rpm1 < 150.0f && rpm2 < 150.0f) ? INT_LIM_SLOW : INT_LIM_FAST;
  i1 = constrain(i1, -lim, lim);
  i2 = constrain(i2, -lim, lim);
}

// ============================== BLE Callbacks ==============================

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { deviceConnected = true; }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    std::string v = ch->getValue();
    if (v.empty()) return;
    std::string u(v);
    for (auto& c : u) c = toupper((unsigned char)c);

    if (u.rfind("SPEED=", 0) == 0) {
      targetSpeed_mps = atof(v.substr(6).c_str());
      targetRPM = (targetSpeed_mps > 0.0f)
        ? (targetSpeed_mps * 60.0f) / (PI * WHEEL_DIAMETER_M)
        : 0.0f;
      isMoving = (targetSpeed_mps > 0.0f);
      speedSum = 0.0f; sampleCount = 0;
      Serial.printf("[BLE] SPEED=%.2f m/s (Target RPM=%.2f)\n", targetSpeed_mps, targetRPM);
      return;
    }
    if (u == "START") { isMoving = true; Serial.println("[BLE] START"); return; }
    if (u == "STOP")  { motorStop();   Serial.println("[BLE] STOP");  return; }
  }
};

void setupBLE() {
  Serial.println("Initializing BLE...");
  BLEDevice::init("SpeediBot");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* svc = pServer->createService(SERVICE_UUID);
  pCharacteristic = svc->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristic->setCallbacks(new MyCallbacks());
  svc->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::getAdvertising()->start();
  Serial.println("BLE Advertising Started!");
}

// ============================== ISRs =======================================
void IRAM_ATTR encoder1ISR() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) encoderCount1++;
  else encoderCount1--;
}
void IRAM_ATTR encoder2ISR() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) encoderCount2++;
  else encoderCount2--;
}

// ============================== Setup ======================================
void setup() {
  Serial.begin(115200);
  Serial.println("Calibrating QTR sensors...");
  for (uint16_t i = 0; i < 400; ++i) { qtr.calibrate(); delay(5); }
  Serial.println("Calibration complete.");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(STOP_LED, OUTPUT); digitalWrite(STOP_LED, LOW);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), encoder2ISR, CHANGE);

  ledcSetup(PWM_CH_A, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(PWM_CH_B, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(ENA, PWM_CH_A);
  ledcAttachPin(ENB, PWM_CH_B);

  setupBLE();

  lastLoopMs = millis();
  robotStartTime = millis();
}

// ============================== Main Loop ==================================
void loop() {
  bool allOnBlack = false;
  int linePos = readLinePosition(allOnBlack);
  if (allOnBlack) {
    Serial.println("STOP LINE DETECTED - Stopping motors.");
    motorStop(); digitalWrite(STOP_LED, HIGH);
    if (sampleCount > 0) Serial.printf("Average Speed: %.2f m/s\n", speedSum / sampleCount);
    targetSpeed_mps = 0.0f; targetRPM = 0.0f; delay(50); return;
  }
  if (!deviceConnected || targetSpeed_mps <= 0.0f) {
    motorStop(); digitalWrite(STOP_LED, LOW); delay(10); return;
  }
  unsigned long now = millis();
  if (now - lastLoopMs < LOOP_PERIOD_MS) return;
  lastLoopMs = now;

  computeRPM();
  const float wheel_circ = PI * WHEEL_DIAMETER_M;
  const float spd1 = (rpm1 * wheel_circ) / 60.0f;
  const float spd2 = (rpm2 * wheel_circ) / 60.0f;
  const float spd_avg = 0.5f * (spd1 + spd2);
  speedSum += spd_avg; sampleCount++;

  const float err1 = targetRPM - rpm1;
  const float err2 = targetRPM - rpm2;
  integral_error1 += err1; integral_error2 += err2;
  clampIntegral(integral_error1, integral_error2, rpm1, rpm2);
  pwm1 = KP_DEFAULT * err1 + KI_DEFAULT * integral_error1;
  pwm2 = KP_DEFAULT * err2 + KI_DEFAULT * integral_error2;

  float correction = LINE_K * (float)linePos;
  updateMotorOutputs(correction);

  Serial.printf("pos:%d rpm:%.1f/%.1f spd:%.2f pwm:%.0f/%.0f\n",
    linePos, rpm1, rpm2, spd_avg, pwm1, pwm2);
}

