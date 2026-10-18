#include <Arduino.h>
#include "config.h"
#include "wind_sensor.h"
#include "data_log.h"

#ifdef WIND_SENSOR

// wind pulses
volatile uint32_t gPulseCount = 0;
volatile uint32_t gLastPulseMicros = 0;

float gNowWindMS = 0.0f;

// Wind Sensor (Pulse-based Anemometer)
namespace WindConfig {
  static constexpr int   PULSE_PIN = 3;                 // GPIO pin
  static constexpr float PPS_TO_MS = 1.75f / 20.0f;     // Calibration: 20pps = 1.75m/s
  static constexpr uint32_t PPS_WINDOW_MS = 1000;       // Calculate every 1 second
  static constexpr uint32_t DEBOUNCE_US = 2000;         // 2ms debounce
}

void clearWindToday(void) {
  gTodaySumOfBucketAvgs = 0.0f;
  gTodayBucketCount = 0;
  gTodayMax = 0.0f;
}

void clearWindSamples(void) {
  gBucketWindSum = 0.0f;
  gBucketWindMax1 = 0.0f;
  gBucketWindMax2 = 0.0f;
  gBucketSamples = 0;
  gBucketPulseCount = 0;
  gBucketPulseElapsedMs = 0;
}

void updateWind(float avgWind, float maxWind) {
  if (isfinite(avgWind)) {
    gTodaySumOfBucketAvgs += avgWind;
    gTodayBucketCount++;
  }
  if (isfinite(maxWind)) {
    if (maxWind > gTodayMax) gTodayMax = maxWind;
  }
}

float computeWind(void) {
  float avgWindMs = 0.0f;
  if (gBucketPulseElapsedMs > 0) {
    float ppsAvg = (float)gBucketPulseCount * 1000.0f / (float)gBucketPulseElapsedMs;
    avgWindMs = ppsAvg * WindConfig::PPS_TO_MS;
  } else if (gBucketSamples > 0) {
    avgWindMs = gBucketWindSum / (float)gBucketSamples;
  }
  return avgWindMs;
}

float computeMaxWind(float avgWindMs) {
  float maxWindMs = gBucketWindMax1;
  if (maxWindMs < avgWindMs) maxWindMs = avgWindMs;
  return maxWindMs;
}

float calcWindPps(void) {
  float pps = (WindConfig::PPS_TO_MS > 0.0f) ? (gNowWindMS / WindConfig::PPS_TO_MS) : 0.0f;
  return pps;
}


void updateWindPPS(uint32_t msNow) {
  noInterrupts();
  uint32_t elapsedMs = msNow - gLastPpsMillis;
  if (elapsedMs >= WindConfig::PPS_WINDOW_MS) {

    uint32_t delta = gPulseCount;
    gPulseCount = 0;

    float pps = (delta * 1000.0f / (float)elapsedMs);
    float ms = pps * WindConfig::PPS_TO_MS;

    gNowWindMS = ms;
    if (gNowWindMS > gBucketWindMax1) {
      gBucketWindMax2 = gBucketWindMax1;
      gBucketWindMax1 = gNowWindMS;
    } else if (gNowWindMS > gBucketWindMax2) {
      gBucketWindMax2 = gNowWindMS;
    }

    gBucketPulseCount += delta;
    gBucketPulseElapsedMs += elapsedMs;

    // Sample wind into bucket immediately after calculation
    gBucketWindSum += gNowWindMS;
    gBucketSamples++;

    gLastPpsMillis = msNow;
  }
  interrupts();
}

// ------------------- ISR -------------------
void IRAM_ATTR onPulse(void) {
  uint32_t now = micros();
  uint32_t elapsed = now - gLastPulseMicros;

  // Debounce: ignore pulses closer than DEBOUNCE_US microseconds
  if (elapsed >= WindConfig::DEBOUNCE_US) {
    gLastPulseMicros = now;
    gPulseCount++;
  }
}

void setupWindPin(void) {
  // Set up pulse pin but don't attach interrupt yet to avoid counting noise during WiFi init
  pinMode(WindConfig::PULSE_PIN, INPUT_PULLUP);
}

void initWindSensor(void) {
  // Attach pulse interrupt after all initalization to avoid counting noise during boot
  gLastPulseMicros = micros();
  gPulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(WindConfig::PULSE_PIN), onPulse, FALLING);
  Serial.println("Attach Wind Pulse interrupt");
}

#endif

