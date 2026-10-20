#include <math.h>       // for NAN
#include "config.h"
#include "env_sensors.h"

#ifdef TEMP_SENSOR
float gTempC = NAN;
float gBucketTempSum = 0.0f;
float gTodayTempSum = 0.0f;
uint32_t gTodayTempCount = 0;
float gTodayTempMin = NAN;
float gTodayTempMax = NAN;

String getEnvTemp(int fmt) {
  float degC = gTempC;
  if (fmt == 0) {
    return String(degC);
  }
  if (fmt > 0) {
    return String(degC, fmt);
  }
  float degF = (degC * 9.0f/5.0f) + 32.0f;
  return String(degF, -fmt);
}
#endif

#ifdef HUMID_SENSOR
float gHumRH = NAN;
float gBucketHumSum = 0.0f;
float gTodayHumSum = 0.0f;
uint32_t gTodayHumCount = 0;
float gTodayHumMin = NAN;
float gTodayHumMax = NAN;

String getEnvHumidity(int places) {
  return String(gHumRH, places);
}
#endif

#ifdef PRESS_SENSOR
float gPressurePa = NAN;
float gBucketPressSumPa = 0.0f;
float gTodayPressSum = 0.0f;
uint32_t gTodayPressCount = 0;
float gTodayPressMin = NAN;
float gTodayPressMax = NAN;

String getEnvPressure(int places) {
  return String(gPressurePa, places);
}
#endif

uint32_t gBucketEnvSamples = 0;

#define POLL_INTERVAL_MS  2000      // Read every 2 seconds
#define ALTITUDE_METERS   580.0f    // Station altitude for MSLP calculation (0 = sea level)

#ifdef BME280_SENSOR

#include <Adafruit_BME280.h>

// BME280 Environmental Sensor (I2C)
static Adafruit_BME280 bme;
bool gBmeOk = false;

#endif

#ifdef BMP280_SENSOR

#include <Adafruit_BMP280.h>

// BMP280 temperature/pressure sensor
static Adafruit_BMP280 bmp;
bool gBmpOk = false;

#endif

#ifdef AHT20_SENSOR

#include <Adafruit_AHTX0.h>

// AHT20 temperature/humidity sensor
static Adafruit_AHTX0 aht;
bool gAhtOk = false;

#endif // AHT20_SENSOR

static uint32_t gLastEnvPoll = 0;

// ------------------- BME280 -------------------
// Convert station pressure to Mean Sea Level Pressure (MSLP)
// Using the ICAO Standard Barometric Formula with measured temperature
float convertToMSLP(float stationPressurePa, float tempC, float altitudeM)
{
  if (!isfinite(stationPressurePa) || !isfinite(tempC))
    return stationPressurePa;

  // If altitude is essentially zero, return station pressure
  if (altitudeM > -0.1f && altitudeM < 0.1f)
    return stationPressurePa;

  const float L = 0.0065f;                 // Standard lapse rate (K/m)
  const float T_station = tempC + 273.15f; // Station temperature in Kelvin
  const float exponent = 5.25588f;         // g / (R * L)

  // Use the station temperature directly in the ratio
  float ratio = 1.0f + (L * altitudeM / T_station);
  float mslpPa = stationPressurePa * pow(ratio, exponent);

  return mslpPa;
}

void clearEnvToday(void)
{
#ifdef TEMP_SENSOR
  gTodayTempSum = 0.0f;
  gTodayTempCount = 0;
  gTodayTempMin = NAN;
  gTodayTempMax = NAN;
#endif
#ifdef HUMID_SENSOR
  gTodayHumSum = 0.0f;
  gTodayHumCount = 0;
  gTodayHumMin = NAN;
  gTodayHumMax = NAN;
#endif
#ifdef PRESS_SENSOR
  gTodayPressSum = 0.0f;
  gTodayPressCount = 0;
  gTodayPressMin = NAN;
  gTodayPressMax = NAN;
#endif
}

void updateEnvMinMax(float avgTempC, float avgHumRH, float avgPressHpa)
{
#ifdef TEMP_SENSOR
  if (isfinite(avgTempC))
  {
    if (!isfinite(gTodayTempMin) || avgTempC < gTodayTempMin)
      gTodayTempMin = avgTempC;
    if (!isfinite(gTodayTempMax) || avgTempC > gTodayTempMax)
      gTodayTempMax = avgTempC;
    gTodayTempSum += avgTempC;
    gTodayTempCount++;
  }
#endif
#ifdef HUMID_SENSOR
  if (isfinite(avgHumRH))
  {
    if (!isfinite(gTodayHumMin) || avgHumRH < gTodayHumMin)
      gTodayHumMin = avgHumRH;
    if (!isfinite(gTodayHumMax) || avgHumRH > gTodayHumMax)
      gTodayHumMax = avgHumRH;
    gTodayHumSum += avgHumRH;
    gTodayHumCount++;
  }
#endif
#ifdef PRESS_SENSOR
  if (isfinite(avgPressHpa))
  {
    if (!isfinite(gTodayPressMin) || avgPressHpa < gTodayPressMin)
      gTodayPressMin = avgPressHpa;
    if (!isfinite(gTodayPressMax) || avgPressHpa > gTodayPressMax)
      gTodayPressMax = avgPressHpa;
    gTodayPressSum += avgPressHpa;
    gTodayPressCount++;
  }
#endif
}

void clearEnvSamples(void)
{
#ifdef TEMP_SENSOR
  gBucketTempSum = 0.0f;
#endif
#ifdef HUMID_SENSOR
  gBucketHumSum = 0.0f;
#endif
#ifdef PRESS_SENSOR
  gBucketPressSumPa = 0.0f;
#endif
  gBucketEnvSamples = 0;
}

void pollEnv(void)
{
#ifdef BME280_SENSOR
  if (!gBmeOk) {
#ifdef TEMP_SENSOR
    gTempC = bme.readTemperature();
#endif
#ifdef HUMID_SENSOR
    gHumRH = bme.readHumidity();
#endif
#ifdef PRESS_SENSOR
    float rawPressurePa = bme.readPressure();
    // Convert to Mean Sea Level Pressure using station altitude
    gPressurePa = convertToMSLP(rawPressurePa, gTempC, ALTITUDE_METERS);
#endif
  }
#endif // BME280_SENSOR

#ifdef BMP280_SENSOR
  if (gBmpOk) {
#ifdef TEMP_SENSOR
    gTempC = bmp.readTemperature();
#endif
#ifdef PRESS_SENSOR
    float rawPressurePa = bmp.readPressure();
    // Convert to Mean Sea Level Pressure using station altitude
    gPressurePa = convertToMSLP(rawPressurePa, gTempC, ALTITUDE_METERS);
#endif
  }
#endif

#ifdef AHT20_SENSOR
  if (gAhtOk) {
  sensors_event_t humidity, temp;

  // Retrieve fresh data
  aht.getEvent(&humidity, &temp); 

#ifdef TEMP_SENSOR
    gTempC = temp.temperature;
#endif
#ifdef HUMID_SENSOR
    gHumRH = humidity.relative_humidity;
#endif
  }
#endif // AHT20_SENSOR

  // Accumulate for per-bucket averages
#ifdef TEMP_SENSOR
  gBucketTempSum += gTempC;
#endif
#ifdef HUMID_SENSOR
  gBucketHumSum += gHumRH;
#endif
#ifdef PRESS_SENSOR
  gBucketPressSumPa += gPressurePa;
#endif
  ++gBucketEnvSamples;
}

bool pollEnvIfNeeded(uint32_t msNow)
{
  if (msNow - gLastEnvPoll >= POLL_INTERVAL_MS) {
    pollEnv();
    gLastEnvPoll += POLL_INTERVAL_MS;
    return true;
  }
  return false;
}

bool initEnvSensors(uint32_t msNow)
{
  Wire.begin(D8, D9);   // Use default SDA, SCL pins
#ifdef BME280_SENSOR
  gBmeOk = bme.begin(BME280_ADDRESS);
  if (!gBmeOk) {
    Serial.println("ERROR init BME280 PRIMARY");
    gBmeOk = bme.begin(BME280_ADDRESS_ALTERNATE);
    if (!gBmeOk) {
      Serial.println("ERROR init BME280 FALLBACK");
    }
  }
  if (gBmeOk)
  {
    bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X2,
        Adafruit_BME280::SAMPLING_X16,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_X16,
        Adafruit_BME280::STANDBY_MS_500);
    gLastEnvPoll = msNow;
    pollEnv();
  }
  return gBmeOk;
#elif defined(BMP280_SENSOR)
  gBmpOk = bmp.begin(BMP280_ADDRESS);
  if (!gBmpOk) {
    Serial.println("ERROR init BMP280 SENSOR");
  }
  if (gBmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
#ifdef AHT20_SENSOR
    gAhtOk = aht.begin();
    if (!gAhtOk) {
      Serial.println("ERROR init AHT20 SENSOR");
    }
#endif
    gLastEnvPoll = msNow;
    pollEnv();
  }
  return gBmpOk;
#endif
}
