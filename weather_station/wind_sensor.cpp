#include <Arduino.h>
#include "config.h"
#include "wind_sensor.h"

//#define SIMULATE_WIND   200     // 5 pulses/sec

#ifdef WIND_SENSOR

#include <Wire.h>
#include <SparkFun_TMAG5273_Arduino_Library.h>

float    gBucketWindSum = 0.0f;
float    gBucketWindMax1 = 0.0f; // highest observed in bucket
float    gBucketWindMax2 = 0.0f; // second-highest observed in bucket
uint32_t gBucketSamples = 0;
uint32_t gBucketPulseCount = 0;
uint32_t gBucketPulseElapsedMs = 0;

uint32_t gTodayBucketCount = 0;
float    gTodayMax = 0.0f;
uint32_t gLastPpsMillis = 0;
float    gTodaySumOfBucketAvgs = 0.0f;

// wind pulses
volatile uint32_t gPulseCount = 0;
volatile uint32_t gLastPulseMicros = 0;

float gNowWindMS = 0.0f;

//
//  Wind Direction global variables (bucket vs. daily accumulators)
//
float gBucketWindSumX = 0.0f;
float gBucketWindSumY = 0.0f;
uint32_t gBucketWindDirCount = 0;

float gTodayWindSumX = 0.0f;
float gTodayWindSumY = 0.0f;
uint32_t gTodayWindDirCount = 0;

uint16_t gWindAngle = 0;

// Wind Sensor (Pulse-based Anemometer)
namespace WindConfig {
  static constexpr int   PULSE_PIN = 0;                 // GPIO pin
  static constexpr float PPS_TO_MS = 1.75f / 20.0f;     // Calibration: 20pps = 1.75m/s
  static constexpr uint32_t PPS_WINDOW_MS = 1000;       // Calculate every 1 second
}

#define DEBOUNCE_US     2000      // Cannot be a constant in flash!

void clearWindToday(void) {
  gTodaySumOfBucketAvgs = 0.0f;
  gTodayBucketCount = 0;
  gTodayMax = 0.0f;
  gTodayWindSumX = 0.0f;
  gTodayWindSumY = 0.0f;
  gTodayWindDirCount = 0;
}

void clearWindSamples(void) {
  gBucketWindSum = 0.0f;
  gBucketWindMax1 = 0.0f;
  gBucketWindMax2 = 0.0f;
  gBucketSamples = 0;
  gBucketPulseCount = 0;
  gBucketPulseElapsedMs = 0;

  // Reset per-bucket direction accumulators
  gBucketWindSumX = 0.0f;
  gBucketWindSumY = 0.0f;
  gBucketWindDirCount = 0;
}

void updateWind(float avgWind, float maxWind, uint16_t windAngle) {
  if (isfinite(avgWind)) {
    gTodaySumOfBucketAvgs += avgWind;
    gTodayBucketCount++;
    // Accumulate daily vector average weighted by bucket wind speed
    float dirRad = radians(windAngle);
    gTodayWindSumX += avgWind * sin(dirRad);
    gTodayWindSumY += avgWind * cos(dirRad);
    gTodayWindDirCount++;
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

// ------------------- ISR -------------------
void IRAM_ATTR onPulse(void) {
  uint32_t now = micros();
  uint32_t elapsed = now - gLastPulseMicros;

  // Debounce: ignore pulses closer than DEBOUNCE_US microseconds
  if (elapsed >= DEBOUNCE_US) {
    gLastPulseMicros = now;
    gPulseCount++;
  }
}

#ifdef SIMULATE_WIND

uint16_t wind_dir;        // 0-359 degrees
uint32_t wind_period;     // period in ms

uint32_t lastSimPulse = 0;

void showWindParms(void) {
  Serial.printf("period=%d dir=%u\n", wind_period, wind_dir);
}

// Helper function to split a String by a delimiter
int splitString(String input, char delimiter, String results[], int maxResults) {
  int count = 0;
  int prevIndex = 0;
  int currentIndex = input.indexOf(delimiter);

  while (currentIndex != -1 && count < maxResults) {
    results[count++] = input.substring(prevIndex, currentIndex);
    prevIndex = currentIndex + 1;
    currentIndex = input.indexOf(delimiter, prevIndex);
  }

  // Grab the remaining piece after the last delimiter
  if (count < maxResults) {
    results[count++] = input.substring(prevIndex);
  }

  return count; // Returns the total number of parts found
}

const int VARIANCE = 100;
static int buf_index = 0;
#define BUF_SIZE  80
char line[BUF_SIZE];

void parseLine(const char* buf) {
  String line = buf;
  String parts[10];
  int count = splitString(line, ',', parts, 10);
  wind_period = parts[0].toInt();
  wind_dir = parts[1].toInt();
  showWindParms();
}

//
//  simulateWind sensor from period,dir data from the Serial console
//  Generates "virtual" interrupts and provides wind vane sensor not detected
//
void simulateWind(uint32_t msNow) {
  if (Serial.available()) {
    char ch;
    if (Serial.readBytes(&ch, 1)) {
      line[buf_index++] = ch;
      if (ch == '\n') {
        parseLine(line);
        buf_index = 0;
      }
    }
  }
  uint32_t elapsed = msNow - lastSimPulse;
  if (elapsed >= wind_period) {
      // call interrupt handler directly to simulate pulse!
      onPulse();
      lastSimPulse += wind_period;
      gWindAngle = wind_dir;
  }
}

#endif // SIMULATE_WIND

void updateWindPPS(uint32_t msNow) {
#ifdef SIMULATE_WIND
  simulateWind(msNow);
#endif
  uint32_t elapsedMs = msNow - gLastPpsMillis;
  if (elapsedMs >= WindConfig::PPS_WINDOW_MS) {

#ifndef SIMULATE_WIND
    gWindAngle = getWindAngle();
#endif
    noInterrupts();

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

    // Accumulate 1-second wind vector into current bucket
    accumWindDir(gNowWindMS, gWindAngle);

    gLastPpsMillis += WindConfig::PPS_WINDOW_MS;
    interrupts();
  }
}

void setupWindPin(void) {
  // Set up pulse pin but don't attach interrupt yet to avoid counting noise during WiFi init
  pinMode(WindConfig::PULSE_PIN, INPUT_PULLUP);
}

void initWindSensor(void) {

  // Initialize Wind Vane (direction) sensor
  initWindVane();

  // Attach pulse interrupt after all initalization to avoid counting noise during boot
  gLastPulseMicros = micros();
  gPulseCount = 0;
#ifdef SIMULATE_WIND
  lastSimPulse = millis();  
  // Set default wind speed and direction
  wind_period = SIMULATE_WIND;
  wind_dir = 0;
  showWindParms();
#endif
  attachInterrupt(digitalPinToInterrupt(WindConfig::PULSE_PIN), onPulse, FALLING);
  Serial.println("Attach Wind Pulse interrupt");
}


TMAG5273 sensor; // Create sensor object

// Default I2C address for TMAG5273 is usually 0x22 or 0x35 depending on variant
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

bool initWindVane(uint8_t i2cAddress) {

  // Initialize the sensor
  if (sensor.begin(i2cAddress, Wire) == false) {
    Serial.println("TMAG5273 sensor not found. Check wiring!");
    return false;
  }

  // Configure sensor for angle calculation using X and Y axes
  sensor.setMagneticChannel(TMAG5273_XYX_ENABLE); // Enable X and Y channels
  sensor.setAngleEn(TMAG5273_XY_ANGLE_CALCULATION); // Enable CORDIC calculation on XY
  
  // Set conversion averaging to smooth out noise (Optional but recommended)
  sensor.setConvAvg(TMAG5273_X32_CONVERSION);
  
  Serial.println("TMAG5273 Initialized successfully.");
  return true;
}

uint16_t getWindAngle(void) {
  // Read the angle calculated by the internal hardware engine
  float rawAngle = sensor.getAngleResult();

  // Enforce 0-359 integer range
  int angleDegrees = (int) rawAngle;
  if (angleDegrees < 0) angleDegrees += 360; // Handle any negative anomalies

  return (uint16_t) angleDegrees % 360U;
}

void accumWindDir(float currentSpeed, uint16_t currentDirDeg) {
  float dirRad = radians(currentDirDeg);
  float xComp = currentSpeed * sin(dirRad);
  float yComp = currentSpeed * cos(dirRad);

  gBucketWindSumX += xComp;
  gBucketWindSumY += yComp;
  ++gBucketWindDirCount;
}

static uint16_t vectorToDegrees(float sumX, float sumY, uint32_t count) {
  if (count == 0 || (sumX == 0.0f && sumY == 0.0f)) {
    return gWindAngle;
  }
  float avgX = sumX / count;
  float avgY = sumY / count;

  float avgDirRad = atan2(avgX, avgY);
  float avgDirDeg = degrees(avgDirRad);
  if (avgDirDeg < 0.0f) {
    avgDirDeg += 360.0f;
  }
  return ((uint16_t)roundf(avgDirDeg)) % 360U;
}

// Compute average Wind Direction for the current bucket
uint16_t computeWindDir(void) {
  return vectorToDegrees(gBucketWindSumX, gBucketWindSumY, gBucketWindDirCount);
}

// Compute average Wind Direction across today's completed buckets
uint16_t computeTodayWindDir(void) {
  return vectorToDegrees(gTodayWindSumX, gTodayWindSumY, gTodayWindDirCount);
}

#endif
