#include <Arduino.h>
#include "config.h"
#include <stdint.h>
#include <math.h>
#include "air_quality.h"

#ifdef AIR_QUAL_SENSOR

float gBucketPM1Sum = 0.0f;
float gBucketPM25Sum = 0.0f;
float gBucketPM10Sum = 0.0f;
uint32_t gBucketPmSamples = 0;
float gTodayPM1Sum = 0.0f;
uint32_t gTodayPM1Count = 0;
float gTodayPM1Max = NAN;
float gTodayPM25Sum = 0.0f;
uint32_t gTodayPM25Count = 0;
float gTodayPM25Max = NAN;
float gTodayPM10Sum = 0.0f;
uint32_t gTodayPM10Count = 0;
float gTodayPM10Max = NAN;

// PMS5003 latest readings
uint8_t gPmsFrameBuffer[32];
int gPmsFrameIndex = 0;
bool gPmsOk = false;
float gPM1 = NAN;
float gPM25 = NAN;
float gPM10 = NAN;

static uint32_t gLastAirQualPoll = 0;

// PMS5003 Air Quality Sensor (UART)
namespace PMS5003Config
{
  static constexpr bool ENABLE = true;
  static constexpr int RX_PIN = 20;
  static constexpr int TX_PIN = 21;                  // Not used, but required for Serial2.begin()
  static constexpr uint32_t POLL_INTERVAL_MS = 2000; // Poll every 2 seconds
}

// ------------------- UART FOR PMS5003 -------------------
// ESP32-C3 doesn't have Serial2 predefined, create custom HardwareSerial
HardwareSerial Serial2(1); // Use UART1 for PMS5003


// AQI (Air Quality Index) Calculation Standard
// Choose which AQI standard to use:
//   0 = EPA (United States Environmental Protection Agency)
//       - Scale: 0-500
//       - Categories: Good (0-50), Moderate (51-100), Unhealthy for Sensitive (101-150),
//                     Unhealthy (151-200), Very Unhealthy (201-300), Hazardous (301+)
//       - PM2.5 breakpoints: 0-12, 12.1-35.4, 35.5-55.4, 55.5-150.4, 150.5-250.4, 250.5+
//       - PM10 breakpoints: 0-54, 55-154, 155-254, 255-354, 355-424, 425+
//
//   1 = Australian AQI
//       - Scale: 0-200+
//       - Categories: Very Good (0-33), Good (34-66), Fair (67-99),
//                     Poor (100-149), Very Poor (150-200), Hazardous (200+)
//       - PM2.5 breakpoints: 0-25, 26-50, 51-100, 101-150, 151-200, 200+
//       - PM10 breakpoints: 0-50, 51-100, 101-200, 201-300, 301-400, 400+
//
// IMPORTANT: If you change this, also update the color thresholds in web/app.js (setAQIPill function)
#define AQI_STANDARD 1 // 0=EPA, 1=Australian

void clearAirQualToday(void)
{
  gTodayPM1Sum = 0.0f;
  gTodayPM1Count = 0;
  gTodayPM1Max = NAN;
  gTodayPM25Sum = 0.0f;
  gTodayPM25Count = 0;
  gTodayPM25Max = NAN;
  gTodayPM10Sum = 0.0f;
  gTodayPM10Count = 0;
  gTodayPM10Max = NAN;
}

void updateAirQuality(float avgPM1, float avgPM25, float avgPM10)
{
  // PM accumulation (average only, no min/max)
  if (isfinite(avgPM1))
  {
    gTodayPM1Sum += avgPM1;
    gTodayPM1Count++;
    if (!isfinite(gTodayPM1Max) || avgPM1 > gTodayPM1Max)
      gTodayPM1Max = avgPM1;
  }
  if (isfinite(avgPM25))
  {
    gTodayPM25Sum += avgPM25;
    gTodayPM25Count++;
    if (!isfinite(gTodayPM25Max) || avgPM25 > gTodayPM25Max)
      gTodayPM25Max = avgPM25;
  }
  if (isfinite(avgPM10))
  {
    gTodayPM10Sum += avgPM10;
    gTodayPM10Count++;
    if (!isfinite(gTodayPM10Max) || avgPM10 > gTodayPM10Max)
      gTodayPM10Max = avgPM10;
  }
}

// ------------------- AQI CALCULATION -------------------

#if AQI_STANDARD == 0
// EPA (US) AQI Standard
// Calculate EPA AQI for PM2.5 (24-hour average, but we use current reading)
int calculateAQI_PM25(float pm25)
{
  if (!isfinite(pm25) || pm25 < 0)
    return -1;

  // EPA AQI breakpoints for PM2.5 (μg/m³)
  if (pm25 <= 12.0)
    return map((int)(pm25 * 10), 0, 120, 0, 50);
  else if (pm25 <= 35.4)
    return map((int)(pm25 * 10), 121, 354, 51, 100);
  else if (pm25 <= 55.4)
    return map((int)(pm25 * 10), 355, 554, 101, 150);
  else if (pm25 <= 150.4)
    return map((int)(pm25 * 10), 555, 1504, 151, 200);
  else if (pm25 <= 250.4)
    return map((int)(pm25 * 10), 1505, 2504, 201, 300);
  else if (pm25 <= 500.4)
    return map((int)(pm25 * 10), 2505, 5004, 301, 500);
  else
    return 500;
}

// Calculate EPA AQI for PM10 (24-hour average, but we use current reading)
int calculateAQI_PM10(float pm10)
{
  if (!isfinite(pm10) || pm10 < 0)
    return -1;

  // EPA AQI breakpoints for PM10 (μg/m³)
  if (pm10 <= 54)
    return map((int)pm10, 0, 54, 0, 50);
  else if (pm10 <= 154)
    return map((int)pm10, 55, 154, 51, 100);
  else if (pm10 <= 254)
    return map((int)pm10, 155, 254, 101, 150);
  else if (pm10 <= 354)
    return map((int)pm10, 255, 354, 151, 200);
  else if (pm10 <= 424)
    return map((int)pm10, 355, 424, 201, 300);
  else if (pm10 <= 604)
    return map((int)pm10, 425, 604, 301, 500);
  else
    return 500;
}

// Get EPA AQI category string
const char *getAQICategory(int aqi)
{
  if (aqi < 0)
    return "N/A";
  else if (aqi <= 50)
    return "Good";
  else if (aqi <= 100)
    return "Moderate";
  else if (aqi <= 150)
    return "Unhealthy for Sensitive";
  else if (aqi <= 200)
    return "Unhealthy";
  else if (aqi <= 300)
    return "Very Unhealthy";
  else
    return "Hazardous";
}

#elif AQI_STANDARD == 1
// Australian AQI Standard
// Calculate Australian AQI for PM2.5 (24-hour average, but we use current reading)
int calculateAQI_PM25(float pm25)
{
  if (!isfinite(pm25) || pm25 < 0)
    return -1;

  // Australian AQI breakpoints for PM2.5 (μg/m³)
  // Scale: 0-200+ with categories: Very Good, Good, Fair, Poor, Very Poor, Hazardous
  if (pm25 <= 25.0)
    return map((int)(pm25 * 10), 0, 250, 0, 33);
  else if (pm25 <= 50.0)
    return map((int)(pm25 * 10), 251, 500, 34, 66);
  else if (pm25 <= 100.0)
    return map((int)(pm25 * 10), 501, 1000, 67, 99);
  else if (pm25 <= 150.0)
    return map((int)(pm25 * 10), 1001, 1500, 100, 149);
  else if (pm25 <= 200.0)
    return map((int)(pm25 * 10), 1501, 2000, 150, 200);
  else
    return 200; // Hazardous
}

// Calculate Australian AQI for PM10 (24-hour average, but we use current reading)
int calculateAQI_PM10(float pm10)
{
  if (!isfinite(pm10) || pm10 < 0)
    return -1;

  // Australian AQI breakpoints for PM10 (μg/m³)
  if (pm10 <= 50.0)
    return map((int)(pm10 * 10), 0, 500, 0, 33);
  else if (pm10 <= 100.0)
    return map((int)(pm10 * 10), 501, 1000, 34, 66);
  else if (pm10 <= 200.0)
    return map((int)(pm10 * 10), 1001, 2000, 67, 99);
  else if (pm10 <= 300.0)
    return map((int)(pm10 * 10), 2001, 3000, 100, 149);
  else if (pm10 <= 400.0)
    return map((int)(pm10 * 10), 3001, 4000, 150, 200);
  else
    return 200; // Hazardous
}

// Get Australian AQI category string
const char *getAQICategory(int aqi)
{
  if (aqi < 0)
    return "N/A";
  else if (aqi <= 33)
    return "Very Good";
  else if (aqi <= 66)
    return "Good";
  else if (aqi <= 99)
    return "Fair";
  else if (aqi <= 149)
    return "Poor";
  else if (aqi <= 200)
    return "Very Poor";
  else
    return "Hazardous";
}

#else
#error "Invalid AQI_STANDARD. Must be 0 (EPA) or 1 (Australian)"
#endif

void resetAirQualify(void)
{
  gBucketPM1Sum = 0.0f;
  gBucketPM25Sum = 0.0f;
  gBucketPM10Sum = 0.0f;
  gBucketPmSamples = 0;
}

// ------------------- PMS5003 ------------------
-
void pollAirQuality(uint32_t msNow)
{
  if (!PMS5003Config::ENABLE)
    return;

  gLastAirQualPoll = msNow;

  while (Serial2.available())
  {
    uint8_t byte = Serial2.read();

    // Frame synchronization - looking for header 0x42 0x4d
    if (gPmsFrameIndex == 0 && byte != PMS_HEADER_1)
      continue;
    if (gPmsFrameIndex == 1 && byte != PMS_HEADER_2)
    {
      gPmsFrameIndex = 0;
      continue;
    }

    gPmsFrameBuffer[gPmsFrameIndex++] = byte;

    if (gPmsFrameIndex == PMS_FRAME_SIZE)
    {
      // Validate checksum (sum of bytes 0-29)
      uint16_t checksum = 0;
      for (int i = 0; i < 30; i++)
      {
        checksum += gPmsFrameBuffer[i];
      }
      uint16_t frameChecksum = ((uint16_t)gPmsFrameBuffer[30] << 8) | gPmsFrameBuffer[31];

      if (checksum == frameChecksum)
      {
        // Extract PM values (CF=1 readings, proven working)
        // Bytes 4-5: PM1.0, 6-7: PM2.5, 8-9: PM10
        uint16_t pm1_raw = ((uint16_t)gPmsFrameBuffer[4] << 8) | gPmsFrameBuffer[5];
        uint16_t pm25_raw = ((uint16_t)gPmsFrameBuffer[6] << 8) | gPmsFrameBuffer[7];
        uint16_t pm10_raw = ((uint16_t)gPmsFrameBuffer[8] << 8) | gPmsFrameBuffer[9];

        gPM1 = (float)pm1_raw;
        gPM25 = (float)pm25_raw;
        gPM10 = (float)pm10_raw;

        // Accumulate for per-bucket averages
        if (timeIsValid(gCurrentBucketStart))
        {
          gBucketPM1Sum += gPM1;
          gBucketPM25Sum += gPM25;
          gBucketPM10Sum += gPM10;
          gBucketPmSamples++;
        }
      }

      gPmsFrameIndex = 0;
    }
  }
}

void pollAirQualIfNeeded(uint32_t msNow)
{
  if (!PMS5003Config::ENABLE)
    return;
  if (msNow - gLastAirQualPoll < PMS5003Config::POLL_INTERVAL_MS)
    return;
  pollAirQuality(msNow);
}

void initAirQuality(uint32_t msNow)
{
  // Initialize PMS5003 on Serial2
  if (PMS5003Config::ENABLE)
  {
    Serial2.begin(9600, SERIAL_8N1, PMS5003Config::RX_PIN, PMS5003Config::TX_PIN);
    gPmsOk = true; // Sensor ready (no handshake needed)
    Serial.println("PMS5003 UART initialized");
    pollAirQuality(msNow);
  }
}

#endif // AIR_QUAL_SENSOR
