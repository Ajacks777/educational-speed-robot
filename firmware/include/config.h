// ============================================================================
// config.h – SpeediBot Firmware Constants & Pin Definitions
// ============================================================================

#pragma once
#include <Arduino.h>

// ----------------------------- Pin Mapping ---------------------------------
#define IN1 42
#define IN2 41
#define IN3 40
#define IN4 39
#define ENA 38
#define ENB 37

#define ENC1_A 36
#define ENC1_B 35
#define ENC2_A 45
#define ENC2_B 48

// Optional LED (stop condition)
#define STOP_LED 2

// QTR Sensors (5 analog)
static const uint8_t NUM_SENSORS = 5;
static const uint8_t SENSOR_PINS[NUM_SENSORS] = {4, 5, 6, 7, 8};
static const int SENSOR_WEIGHTS[NUM_SENSORS]  = {-2, -1, 0, 1, 2};
static const int BLACK_THRESHOLD = HIGH;

// ----------------------------- Hardware Specs ------------------------------
static const float WHEEL_DIAMETER_M = 0.0635f;    // 63.5 mm
static const uint16_t COUNTS_PER_REV = 320;       // Encoder CPR
static const uint32_t LOOP_PERIOD_MS = 20;        // Control loop timing

// ----------------------------- PWM Settings --------------------------------
static const int PWM_CH_A   = 0;
static const int PWM_CH_B   = 1;
static const int PWM_FREQ_HZ = 5000;
static const int PWM_BITS    = 8;

// ----------------------------- Control Gains -------------------------------
static const float KP_DEFAULT = 0.45f;
static const float KI_DEFAULT = 0.15f;
static const float LINE_K     = 10.0f;

// Integral limits
static const float INT_LIM_SLOW = 600.0f;
static const float INT_LIM_FAST = 1000.0f;

// ----------------------------- BLE UUIDs -----------------------------------
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "87654321-4321-6789-4321-fedcba987654"
