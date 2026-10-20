#include <Arduino.h>
#include "config.h"
#include "wind_sensor.h"

#define SIMULATE_WIND   100     // 10 pulses/sec

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

// Wind Sensor (Pulse-based Anemometer)
namespace WindConfig {
  static constexpr int   PULSE_PIN = 0;                 // GPIO pin
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
      lastSimPulse = msNow;
  }
}

#endif // SIMULATE_WIND

void updateWindPPS(uint32_t msNow) {
#ifdef SIMULATE_WIND
  simulateWind(msNow);
#endif
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

void setupWindPin(void) {
  // Set up pulse pin but don't attach interrupt yet to avoid counting noise during WiFi init
  pinMode(WindConfig::PULSE_PIN, INPUT_PULLUP);
}

void initWindSensor(void) {
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
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL; //

bool initWindVane(uint8_t i2cAddress) {

  // Initialize the sensor
  if (sensor.begin(i2cAddress, Wire) == false) { //
    Serial.println("TMAG5273 sensor not found. Check wiring!");
    return false;
  }

  // Configure sensor for angle calculation using X and Y axes
  sensor.setMagneticChannel(TMAG5273_XYX_ENABLE); // Enable X and Y channels
  sensor.setAngleEn(TMAG5273_XY_ANGLE_CALCULATION); // Enable CORDIC calculation on XY
  
  // Set conversion averaging to smooth out noise (Optional but recommended)
  sensor.setConvAvg(TMAG5273_X32_CONVERSION); //
  
  Serial.println("TMAG5273 Initialized successfully.");
  return true;
}

uint16_t getWindAngle(void) {
  // Read the angle calculated by the internal hardware engine
  float rawAngle = sensor.getAngleResult(); //

  // Enforce 0-359 integer range
  int angleDegrees = (int) rawAngle;
  if (angleDegrees < 0) angleDegrees += 360; // Handle any negative anomalies

  return (uint16_t) angleDegrees % 360U;
}

float sumX = 0.0;
float sumY = 0.0;
float sumSpeed = 0.0;
int sampleCount = 0;

float windSumX = 0.0f;
float windSumY = 0.0f;;
float windSumSpeed = 0.0f;
uint32_t windSampleCount = 0;

void accumWindDir(float currentSpeed, uint16_t currentDirDeg) {

  // --- 2. Convert wind direction from degrees to radians ---
  // Note: Meteorological wind direction is where the wind is COMING FROM.
  // Using standard math: angle in radians
  float dirRad = radians(currentDirDeg);

  // Calculate X and Y vector components
  // X = speed * sin(dir) or cos depending on convention; 
  // Standard meteorological vector: X = -speed * sin(dir), Y = -speed * cos(dir)
  // Or standard math unit vector:
  float xComp = currentSpeed * sin(dirRad);
  float yComp = currentSpeed * cos(dirRad);

  // --- 3. Accumulate components ---
  windSumX += xComp;
  windSumY += yComp;
  windSumSpeed += currentSpeed;
  windSampleCount++;
}

void calcAveWindDir(void) {
  float avgX = sumX / windSampleCount;
  float avgY = sumY / windSampleCount;
  float avgSpeedScalar = windSumSpeed / windSampleCount; // Simple scalar speed average (optional)
  
  // Calculate vector average speed and direction from mean X and Y
  float avgVectorSpeed = sqrt((avgX * avgX) + (avgY * avgY));
  
  // atan2(avgX, avgY) gives direction coming from (in radians), convert to degrees [0, 360)
  float avgDirRad = atan2(avgX, avgY);
  float avgDirDeg = degrees(avgDirRad);
  
  if (avgDirDeg < 0) {
    avgDirDeg += 360.0;
  }

  // Print results to Serial Monitor
  Serial.print("Avg Vector Speed: ");
  Serial.print(avgVectorSpeed);
  Serial.print(" | Avg Direction: ");
  Serial.println(avgDirDeg);

  // Reset accumulators
  windSumX = 0.0;
  windSumY = 0.0;
  windSumSpeed = 0.0;
  windSampleCount = 0;
}

#endif

